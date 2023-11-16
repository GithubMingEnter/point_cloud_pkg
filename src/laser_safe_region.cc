

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>


#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>


#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>




#include <iostream>
#include <vector>
#include <random>
#include <cmath>

#include "sdqp.hpp"
#include "vis_ros1.hpp"

// spdlog
#include <spdlog/spdlog.h>

using Vec2d = Eigen::Vector2d;

class LaserSafeRegion
{

public:
    void Init(ros::NodeHandle &nh)
    {
        nh_ = nh;
        vis_ptr_.reset(new vis::displayRviz(nh_));
        vis_ptr_->enroll<vis::vMarker>("collision_arrow");
        vis_ptr_->enroll<vis::vMarker>("robot_center");
        vis_ptr_->enroll<geometry_msgs::PolygonStamped>("diff_poly");
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
        polys_pub_=nh_.advertise<geometry_msgs::PolygonStamped>("poly_output",3);
        robot_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LaserSafeRegion::robotCallBack, this);
        // laser_sub_ = nh_.subscribe(laser_topic_,10,&)
        pointcloudGeneration(0, 3, 30);

        pcl::toROSMsg(cloud, output_point_msg);
        output_point_msg.header.frame_id = "map";
       


    }
    void run()
    {
         ros::Rate loop_rate(11);
        while (ros::ok())
        {
            output_point_msg.header.stamp = ros::Time::now();
            pcl_pub_.publish(output_point_msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    std::string laser_topic_;
    ros::Publisher pcl_pub_,polys_pub_;
    ros::Subscriber laser_sub_,robot_sub_;
    Vec2d robot_center;
    std::shared_ptr<vis::displayRviz> vis_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud; // random point cloud
    sensor_msgs::PointCloud2 output_point_msg;
    std::vector<Vec2d> cloud_lists;

    void pointcloudGeneration(double mu, double sigma, int _width = 10, int _height = 1)
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
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
    {

    }
    void robotCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
        robot_pt.x=robot_center(0);
        robot_pt.y=robot_center(1);
        robot_pt.z=0.0;//robot_center(2);
        vis_ptr_->vis_single_marker(robot_pt,"robot_center",Vec3d(0.1,0.1,0.3),vis::vMarker::CUBE,"map",vis::pink);
        


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
                                vis::vMarker::ADD,"map",vis::orange); //,vis::blue
            
            
            Vec3d separate_point=Vec3d(x.x(),x.y(),0);//分割点
            Vec3d normal_vector=Vec3d(robot_center(0),robot_center(1),0)-separate_point;
            double h=6,w=6;
            
            Vec3ds pts(4);
            pts[0].y()=pe.y-w;
            pts[0].z()=pe.z;
            pts[0].x()=placeVx(separate_point,normal_vector,pts[0].z(),pts[0].y());

            pts[1].y()=pe.y+w;
            pts[1].z()=pe.z;
            pts[1].x()=placeVx(separate_point,normal_vector,pts[1].z(),pts[1].y());

            pts[2].y()=pe.y+w;
            pts[2].z()=pe.z+h;
            pts[2].x()=placeVx(separate_point,normal_vector,pts[2].z(),pts[2].y());

            pts[3].y()=pe.y-w;
            pts[3].z()=pe.z+h;
            pts[3].x()=placeVx(separate_point,normal_vector,pts[3].z(),pts[3].y());

            vis_ptr_->vis_poly(pts,"diff_poly");

        }
        else
        {
            geometry_msgs::Point p1, p2;
            vis_ptr_->vis_arrow(p1, p2, "collision_arrow", Vec3d(0.1, 0.1, 0.6),vis::vMarker::DELETE);

        }
    }
    // 根据平面方程计算x的值
    double placeVx(Vec3d sp,const Vec3d nv,double z,double y)
    {
        static int ind;
        ++ind;
        //a(x-x0)+b(y-y0)+c(z-z0)=0
        double tx=nv.tail(2).transpose()*Vec2d(y-sp(1),z-sp(0));
        double x=sp(0)-tx/nv(0);
        std::cout<<"i = "<<ind<<" "
                "x = "<<x<<" "
                "y = "<<y<<" "
                "z = "<<z<<std::endl;
        return x;

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_safe_region");

    ros::NodeHandle nh;
    LaserSafeRegion cv;
    cv.Init(nh);
    cv.run();
    return 0;
}




























