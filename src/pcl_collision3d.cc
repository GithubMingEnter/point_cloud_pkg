#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

#include "../include/sdqp.hpp"

using Ev3 = Eigen::Vector3d;
ros::Publisher pcl_pub;
ros::Publisher robot_pub;
ros::Publisher arrow_pub;
Ev3 robot_center;
double robot_z = 0.0;

pcl::PointCloud<pcl::PointXYZRGB> cloud;
std::vector<Ev3> cloud_lists;
void point_cloudGeneration(double mu, double sigma, int _width = 10, int _height = 1)
{
    // TODO
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> random_x(mu, sigma), random_y(mu, sigma), random_z(mu, sigma);
    cloud.width = _width;
    cloud.height = _height;
    ROS_INFO_STREAM("___----------1----____");
    cloud.points.resize(cloud.width*cloud.height);
    for (int i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].x = random_x(gen);
        cloud.points[i].y = random_y(gen);
        cloud.points[i].z = random_z(gen);
        cloud.points[i].r = 255 * (i + 10) / (cloud.points.size() + 10);
        cloud.points[i].g = 0;
        cloud.points[i].b = 0;
        ROS_INFO_STREAM("___----------2----____");
        cloud_lists.emplace_back(Ev3(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));
        std::cout << "x=" << cloud.points[i].x;
    }
}

void robotCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    robot_center = Ev3(msg->pose.position.x, msg->pose.position.y, robot_z);
    if (cloud_lists.empty())
    {
        ROS_WARN("NO POINT CLOUD");
        return;
    }
    int m = 3; // dim
    Ev3 z = Ev3::Zero();
    Ev3 c = Ev3::Zero();
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();//x^T Q x
    Eigen::MatrixXd A;
    A.resize(cloud_lists.size(), 3);
    for (int i = 0; i < A.rows(); i++)
    {
        A.row(i) = (robot_center - cloud_lists[i]).transpose();
    }
    Eigen::VectorXd b = -1.0 * Eigen::VectorXd::Ones(cloud_lists.size());
    double cost = sdqp::sdqp<3>(Q, c, A, b, z);
    Ev3 x; // optimal variable
    ROS_INFO_STREAM("___--------------____");
    if (cost != INFINITY)
    {
        x = z / z.squaredNorm() + robot_center;
        std::cout << "optimal z: " << z.transpose() << std::endl;
        std::cout << "final cost: " << cost << std::endl;
        std::cout << "collision vector: " << x.transpose() << std::endl;
        std::cout << "collision distance: " << x.norm() << std::endl;
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "point_cloud_frame_id";
        arrow.header.stamp = msg->header.stamp;
        arrow.id = 0;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.ns = "collision_arrow";
        arrow.scale.x = 0.15;
        arrow.scale.y = 0.15;
        arrow.scale.z = 0.5;
        arrow.color.a = 1.0;
        arrow.color.r = 0.0;
        arrow.color.g = 0.9;
        arrow.color.b = 0.1;
        geometry_msgs::Point st, end;
        st.x = robot_center.x();
        st.y = robot_center.y();
        st.z = robot_z;
        end.x = x.x();
        end.y = x.y();
        end.z = x.z();
        arrow.points.emplace_back(st);
        arrow.points.emplace_back(end);
        arrow_pub.publish(arrow);
    }
    else
    {
        ROS_WARN("SDQP has not feasiable solution. Robot collided with an obstacle !");
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "point_cloud_frame_id";
        arrow.header.stamp = msg->header.stamp;
        arrow.id = 0;
        arrow.ns = "collisin_arrow";
        arrow.action = visualization_msgs::Marker::DELETE;
        arrow.color.a = 0.0;
        arrow_pub.publish(arrow);
    }
    // header, id type action ns pose scale
    visualization_msgs::Marker robot;
    robot.header.frame_id = "point_cloud_frame_id";
    robot.header.stamp = msg->header.stamp;
    robot.id = 0;
    robot.type = visualization_msgs::Marker::SPHERE;
    robot.action = visualization_msgs::Marker::ADD;
    robot.ns = "robot_center";
    robot.pose.position.x = robot_center.x();
    robot.pose.position.y = robot_center.y();
    robot.pose.position.z = robot_z;
    robot.pose.orientation.x = 0.0;
    robot.pose.orientation.y = 0.0;
    robot.pose.orientation.z = 0.0;
    robot.pose.orientation.w = 1.0;
    robot.scale.x = 0.2;
    robot.scale.y = 0.2;
    robot.scale.z = 0.2;
    robot.color.a = 1.0;
    robot.color.b = 0.0;
    robot.color.g = 1.0;
    robot.color.r = 0.0;
    robot_pub.publish(robot);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_create");

    ros::NodeHandle nh;
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

    robot_pub = nh.advertise<visualization_msgs::Marker>(
        "robot_center", 1, true);
    arrow_pub = nh.advertise<visualization_msgs::Marker>(
        "collision_arrow", 1, true);
    ros::Subscriber robot_sub = nh.subscribe("/move_base_simple/goal", 1, robotCallBack);
    point_cloudGeneration(0, 3, 30,1);
    sensor_msgs::PointCloud2 output_point_msg;

    pcl::toROSMsg(cloud, output_point_msg);
    ROS_INFO_STREAM("___----------4----____");
    output_point_msg.header.frame_id = "point_cloud_frame_id";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output_point_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
