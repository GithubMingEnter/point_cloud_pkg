#include "laser_safe_region.h"

void LaserSafeRegion::Init(ros::NodeHandle &nh)
{
    nh_ = nh;
    base_link_frame_=std::string("base_link");
    odom_frame_=std::string("odom");
    laser_frame_=std::string("base_scan");
    laser_topic_=std::string("/scan");

    vis_ptr_.reset(new vis::displayRviz(nh_));
    vis_ptr_->enroll<vis::vMarker>("collision_arrow");
    vis_ptr_->enroll<vis::vMarker>("robot_center");
    vis_ptr_->enroll<geometry_msgs::PolygonStamped>("diff_poly");
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    polys_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("poly_output", 3);
    laser_pc_pub_=nh_.advertise<sensor_msgs::PointCloud2>("laser_pc",10);
    robot_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LaserSafeRegion::robotCallBack, this);
    // laser_sub_ = nh_.subscribe(laser_topic_,10,&)
    TF_base_to_laser_ptr_.reset(new Tfs);
    tf2_ros::Buffer laser_tf2Buffer;
    //获取机器人base_link到激光雷达坐标变换
    try
    {
        /* code */
        *(TF_base_to_laser_ptr_) = laser_tf2Buffer.
                            lookupTransform(base_link_frame_,laser_frame_,
                            ros::Time::now(),ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        spdlog::warn("fail to listen the transform of {}->{}. Error message: {}", base_link_frame_, laser_frame_, ex.what());
        return;
    }
    base_to_laser_mat_ = tf2::transformToEigen(*(TF_base_to_laser_ptr_)).matrix().cast<float>();
    tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(3.0));
    tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    laser_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_,laser_topic_,10);
    laser_filter_=std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*laser_filter_sub_,*tf_,odom_frame_,10,nh_);
    laser_filter_->registerCallback(boost::bind(&LaserSafeRegion::laserCallBack,this,_1));
    pose_helper_ = std::make_unique<GetPoseHelper>(tf_.get(),odom_frame_,base_link_frame_);

}
void LaserSafeRegion::run()
{
    ros::Rate loop_rate(11);
    while (ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LaserSafeRegion::pointcloudGeneration(double mu, double sigma, int _width , int _height)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> random_x(mu, sigma), random_y(mu, sigma);
    cloud.width = _width;
    cloud.height = _height;
    cloud.points.resize(cloud.width * cloud.height);
    for (int i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].x = random_x(gen);
        cloud.points[i].y = random_y(gen);
        cloud.points[i].z = 0;
        cloud.points[i].r = 255 * (i + 10) / (cloud.points.size() + 10);
        cloud.points[i].g = 0;
        cloud.points[i].b = 0;
        cloud_lists.emplace_back(Vec2d(cloud.points[i].x, cloud.points[i].y));
        std::cout << "x=" << cloud.points[i].x;
    }
}
void LaserSafeRegion::laserCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{
    Mat4f odom_to_base_mat;
    laser_nearest_time_=laser_msg->header.stamp;
    spdlog::info("enter laser callback");
    if(laser_msg->ranges.size()==0)
    {
        spdlog::info("no msg");
        return ;
    }
    // if(!pose_helper_->getSelfPose(odom_to_base_mat,laser_msg->header.stamp))
    // {
    //     spdlog::info("no tf odom_to_base_mat");
    //     return ;
    // }
    tf2_ros::Buffer tf_odom_base_buf;
    Tfs odom_tfs;
    try
    {
      odom_tfs=tf_odom_base_buf.lookupTransform(odom_frame_,base_link_frame_,ros::Time::now(), ros::Duration(3.0));
    }
    catch(tf2::TransformException e)
    {
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    }
    getMatPose(odom_to_base_mat,odom_tfs);
    sensor_msgs::LaserScan laser_scan = *laser_msg;
    for (unsigned int j = 0; j < laser_scan.ranges.size(); j++)
    {
        if (std::isnan(laser_scan.ranges[j]))
        {
            laser_scan.ranges[j] = laser_scan.range_max;
        }
    }
    // convert laser scan to point cloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> latest_laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 laser_cloud;
    // projector_.projectLaser(laser_scan,laser_cloud);
    
    // pcl::fromROSMsg(laser_cloud,*latest_laser_cloud);
    laserScanToPC(laser_msg,*latest_laser_cloud);
    const Mat4f odom_to_laser_mat = odom_to_base_mat*base_to_laser_mat_;
    pcl::transformPointCloud(*latest_laser_cloud,*latest_laser_cloud,odom_to_laser_mat);
    sensor_msgs::PointCloud2 laser_pc;

    pcl::toROSMsg(*latest_laser_cloud,laser_pc);
    laser_pc.header.frame_id = "map";
    laser_pc.header.stamp=ros::Time::now();

    spdlog::info("pub laser laser cloud");
    laser_pc_pub_.publish(laser_pc);
    // projector_()


}
void LaserSafeRegion::robotCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("robotCallback");
    robot_center = Vec2d(msg->pose.position.x, msg->pose.position.y);
    if (cloud_lists.empty())
    {
        ROS_WARN("NO POINT CLOUD");
        return;
    }
    // sdqp problem
    int m = 2;
    Vec2d z = Vec2d::Zero();
    Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
    Vec2d c = Vec2d::Zero();
    Eigen::MatrixXd A; // TODO
    A.resize(cloud_lists.size(), 2);
    for (int i = 0; i < A.rows(); i++)
    {
        A.row(i) = (robot_center - cloud_lists[i]).transpose();
    }
    Eigen::VectorXd b = -1.0 * Eigen::VectorXd::Ones(cloud_lists.size());
    double cost = sdqp::sdqp<2>(Q, c, A, b, z);
    Vec2d x;
    ROS_INFO_STREAM("___--------------____");
    if (cost != INFINITY)
    {
        x = z / z.squaredNorm() + robot_center;
        std::cout << "optimal z: " << z.transpose() << std::endl;
        std::cout << "final cost: " << cost << std::endl;
        std::cout << "collision vector: " << x.transpose() << std::endl;
        std::cout << "collision distance: " << x.norm() << std::endl;
    }
    else
    {
        ROS_WARN("SDQP has not feasiable solution. Robot collided with an obstacle !");
    }
    // robot center
    geometry_msgs::Point robot_pt;
    robot_pt.x = robot_center(0);
    robot_pt.y = robot_center(1);
    robot_pt.z = 0.0; // robot_center(2);
    vis_ptr_->vis_single_marker(robot_pt, "robot_center", Vec3d(0.1, 0.1, 0.3), vis::vMarker::CUBE, "map", vis::pink);

    // collision arrow
    if (cost != INFINITY)
    {
        geometry_msgs::Point p1, pe;
        p1.x = robot_center.x();
        p1.y = robot_center.y();
        p1.z = 0.0;
        pe.x = x.x();
        pe.y = x.y();
        pe.z = 0.0;
        vis_ptr_->vis_arrow(p1, pe, "collision_arrow", Vec3d(0.1, 0.1, 0.6),
                            vis::vMarker::ADD, "map", vis::orange); //,vis::blue

        Vec3d separate_point = Vec3d(x.x(), x.y(), 0); // 分割点
        Vec3d normal_vector = Vec3d(robot_center(0), robot_center(1), 0) - separate_point;
        double h = 6, w = 6;

        Vec3ds pts(4);
        pts[0].y() = pe.y - w;
        pts[0].z() = pe.z;
        pts[0].x() = placeVx(separate_point, normal_vector, pts[0].z(), pts[0].y());

        pts[1].y() = pe.y + w;
        pts[1].z() = pe.z;
        pts[1].x() = placeVx(separate_point, normal_vector, pts[1].z(), pts[1].y());

        pts[2].y() = pe.y + w;
        pts[2].z() = pe.z + h;
        pts[2].x() = placeVx(separate_point, normal_vector, pts[2].z(), pts[2].y());

        pts[3].y() = pe.y - w;
        pts[3].z() = pe.z + h;
        pts[3].x() = placeVx(separate_point, normal_vector, pts[3].z(), pts[3].y());

        vis_ptr_->vis_poly(pts, "diff_poly");
    }
    else
    {
        geometry_msgs::Point p1, p2;
        vis_ptr_->vis_arrow(p1, p2, "collision_arrow", Vec3d(0.1, 0.1, 0.6), vis::vMarker::DELETE);
    }
}
void LaserSafeRegion::laserScanToPC(sensor_msgs::LaserScan::ConstPtr laser_scan,pcl::PointCloud<pcl::PointXYZ>& pc)
{
    pc.clear();
    pcl::PointXYZ  new_pt;
    new_pt.z=0.0;
    double new_pointAngle;
    int laser_num=laser_scan->ranges.size();
    for(int i=0 ; i<laser_num;i++){
        new_pointAngle=laser_scan->angle_min + laser_scan->angle_increment*i;
        new_pt.x=laser_scan->ranges[i]*cos(new_pointAngle);
        new_pt.y=laser_scan->ranges[i]*sin(new_pointAngle);
        pc.emplace_back(new_pt);
    }
}
// 根据平面方程计算x的值
double LaserSafeRegion::placeVx(Vec3d sp, const Vec3d nv, double z, double y)
{
    static int ind;
    ++ind;
    // a(x-x0)+b(y-y0)+c(z-z0)=0
    double tx = nv.tail(2).transpose() * Vec2d(y - sp(1), z - sp(0));
    double x = sp(0) - tx / nv(0);
    std::cout << "i = " << ind << " "
                                  "x = "
              << x << " "
                      "y = "
              << y << " "
                      "z = "
              << z << std::endl;
    return x;
}
bool LaserSafeRegion::getMatPose(Mat4f& pose,const Tfs& tfs_pose)
{
        pose = Mat4f::Identity();
        pose.block<3, 1>(0, 3) << tfs_pose.transform.translation.x,
            tfs_pose.transform.translation.y,
            tfs_pose.transform.translation.z;

        pose.block<3, 3>(0, 0) = Eigen::Quaternionf(tfs_pose.transform.rotation.w,
                                                    tfs_pose.transform.rotation.x,
                                                    tfs_pose.transform.rotation.y,
                                                    tfs_pose.transform.rotation.z).toRotationMatrix();
        return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_safe_region");

    ros::NodeHandle nh;
    LaserSafeRegion cv;
    cv.Init(nh);
    cv.run();
    return 0;
}
