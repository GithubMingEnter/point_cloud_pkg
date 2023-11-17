#pragma once
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
#include<string>

#include "sdqp.hpp"
#include "vis_ros1.hpp"

// spdlog
#include <spdlog/spdlog.h>

using Vec2d = Eigen::Vector2d;
using Mat4f = Eigen::Matrix4f;
using Tfs = geometry_msgs::TransformStamped;
     class GetPoseHelper
{
public:
  GetPoseHelper(tf2_ros::Buffer* tf,
    const std::string& base_frame,
    const std::string& odom_frame)
  : tf_(tf), base_frame_(base_frame), odom_frame_(odom_frame)
  {
  };

  bool getSelfPose(Eigen::Matrix4f& pose, const ros::Time& t)
  {
    geometry_msgs::TransformStamped base_ident, odom_pose;
    base_ident.header.stamp = t;
    base_ident.header.frame_id = base_frame_;
    base_ident.transform.rotation.w = 1.0;

    try
    {
      odom_pose = tf_->transform(base_ident, odom_frame_);
    }
    catch(tf2::TransformException e)
    {
      // ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }

    // const double yaw = tf2::getYaw(odom_pose.transform.rotation);
    // transfer geometry_msgs::TransformStamped to Eigen::Matrix4f
    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) << odom_pose.transform.translation.x,
      odom_pose.transform.translation.y,
      odom_pose.transform.translation.z;
    pose.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_pose.transform.rotation.w,
        odom_pose.transform.rotation.x,
        odom_pose.transform.rotation.y,
        odom_pose.transform.rotation.z)
        .toRotationMatrix();

    return true;
  };

private:
  tf2_ros::Buffer* tf_;
  std::string base_frame_, odom_frame_;
};       
// class GetPoseHelper
// {
// private:
//     tf2_ros::Buffer *tf_;
//     // reference frame , self frame;
//     std::string ref_frame_, self_frame_;

// public:
//     GetPoseHelper(tf2_ros::Buffer *tf,
//                   const std::string &ref_frame,
//                   const std::string &self_frame)
//         : tf_(tf), ref_frame_(ref_frame), self_frame_(self_frame)
//     {
//     }
//     bool getSelfPose(Mat4f &pose, const ros::Time &t)
//     {
//         Tfs ref_tfs, self_tfs;
//         ref_tfs.header.stamp = t;
//         ref_tfs.header.frame_id = ref_frame_;
//         ref_tfs.transform.rotation.w = 1.0;
//         try
//         {
//             self_tfs = tf_->transform(ref_tfs, self_frame_);
//         }
//         catch (tf2::TransformException e)
//         {
            
//             ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
//             return false;
//         }

//         // pose = Mat4f::Identity();
//         // pose.block<3, 1>(0, 3) << self_tfs.transform.translation.x,
//         //     self_tfs.transform.translation.y,
//         //     self_tfs.transform.translation.z;

//         // pose.block<3, 3>(0, 0) = Eigen::Quaternionf(self_tfs.transform.rotation.w,
//         //                                             self_tfs.transform.rotation.x,
//         //                                             self_tfs.transform.rotation.y,
//         //                                             self_tfs.transform.rotation.z).toRotationMatrix();
//         return true;
//     }
// };

class LaserSafeRegion
{
public:
    ~LaserSafeRegion(){
        pose_helper_.reset();
    }
    void Init(ros::NodeHandle &nh);
    void run();

private:
    ros::NodeHandle nh_;
    std::string laser_topic_;
    std::string laser_frame_;

    std::string base_link_frame_;
    std::string odom_frame_;
    tf::TransformListener tfListener_;

    Tfs::Ptr TF_base_to_laser_ptr_;
    laser_geometry::LaserProjection projector_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tfL_;

    ros::Publisher pcl_pub_, polys_pub_;
    ros::Publisher laser_pc_pub_;
    ros::Subscriber laser_sub_, robot_sub_;
    ros::Time laser_nearest_time_;
    Vec2d robot_center;
    std::shared_ptr<vis::displayRviz> vis_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud; // random point cloud
    sensor_msgs::PointCloud2 output_point_msg;
    std::vector<Vec2d> cloud_lists;
    std::unique_ptr<GetPoseHelper> pose_helper_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_filter_sub_;

    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_filter_;

    Eigen::Matrix4f base_to_laser_mat_;
    void pointcloudGeneration(double mu, double sigma, int _width = 10, int _height = 1);
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
    void robotCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void laserScanToPC(sensor_msgs::LaserScan::ConstPtr laser_scan,pcl::PointCloud<pcl::PointXYZ>& pc);
    bool getMatPose(Mat4f& pose,const Tfs& tfs_pose);
    
    // 根据平面方程计算x的值
    double placeVx(Vec3d sp, const Vec3d nv, double z, double y);
};
