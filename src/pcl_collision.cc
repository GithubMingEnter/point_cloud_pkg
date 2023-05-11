
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

using Ev2 = Eigen::Vector2d;
ros::Publisher pcl_pub;
ros::Publisher robot_pub;
ros::Publisher arrow_pub;
Ev2 robot_center;

pcl::PointCloud<pcl::PointXYZRGB> cloud;
std::vector<Ev2> cloud_lists;
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
        cloud_lists.emplace_back(Ev2(cloud.points[i].x, cloud.points[i].y));
        std::cout << "x=" << cloud.points[i].x;
    }
}

void robotCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    robot_center = Ev2(msg->pose.position.x, msg->pose.position.y);
    if (cloud_lists.empty())
    {
        ROS_WARN("NO POINT CLOUD");
        return;
    }
    // sdqp problem
    int m = 2;
    Ev2 z = Ev2::Zero();
    Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
    Ev2 c = Ev2::Zero();
    Eigen::MatrixXd A; // TODO
    A.resize(cloud_lists.size(), 2);
    for (int i = 0; i < A.rows(); i++)
    {
        A.row(i) = (robot_center - cloud_lists[i]).transpose();
    }
    Eigen::VectorXd b = -1.0 * Eigen::VectorXd::Ones(cloud_lists.size());
    double cost = sdqp::sdqp<2>(Q, c, A, b, z);
    Ev2 x;
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
    visualization_msgs::Marker robot;
    robot.header.frame_id = "point_cloud_frame_id";
    robot.header.stamp = msg->header.stamp;
    robot.id = 0;
    robot.type = visualization_msgs::Marker::SPHERE;
    robot.action = visualization_msgs::Marker::ADD;
    robot.ns = "robot_center";
    robot.pose.position.x = robot_center.x();
    robot.pose.position.y = robot_center.y();
    robot.pose.position.z = 0.0;
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

    // collision arrow
    if (cost != INFINITY)
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "point_cloud_frame_id";
        arrow.header.stamp = msg->header.stamp;
        arrow.id = 0;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.ns = "collisin_arrow";
        arrow.scale.x = 0.1;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.5;
        arrow.color.a = 1.0;
        arrow.color.b = 1.0;
        arrow.color.g = 0.0;
        arrow.color.r = 0.0;
        geometry_msgs::Point p1, p2;
        p1.x = robot_center.x();
        p1.y = robot_center.y();
        p1.z = 0.0;
        p2.x = x.x();
        p2.y = x.y();
        p2.z = 0.0;
        arrow.points.push_back(p1);
        arrow.points.push_back(p2);
        arrow_pub.publish(arrow);
    }
    else
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "point_cloud_frame_id";
        arrow.header.stamp = msg->header.stamp;
        arrow.id = 0;
        arrow.ns = "collisin_arrow";
        arrow.action = visualization_msgs::Marker::DELETE;
        arrow.color.a = 0.0;
        arrow_pub.publish(arrow);
    }
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
    pointcloudGeneration(0, 3, 30);
    sensor_msgs::PointCloud2 output_point_msg;
    pcl::toROSMsg(cloud, output_point_msg);
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