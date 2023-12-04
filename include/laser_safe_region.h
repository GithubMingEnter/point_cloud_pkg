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
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/conditional_removal.h>

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

using PointT=pcl::PointXYZ;
using PclT=pcl::PointCloud<PointT>;

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Matrix<char, 3, 1>;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4d = Eigen::Vector4d;
using Vec4f = Eigen::Vector4f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using Vec15d = Eigen::Matrix<double, 15, 15>;
using Vec18d = Eigen::Matrix<double, 18, 1>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat32d = Eigen::Matrix<double, 3, 2>;
using Mat3d = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Mat4d = Eigen::Matrix4d;
using Mat4f = Eigen::Matrix4f;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat96d = Eigen::Matrix<double, 9, 6>;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using Mat18d = Eigen::Matrix<double, 18, 18>;

using VecXd = Eigen::Matrix<double, -1, 1>;
using MatXd = Eigen::Matrix<double, -1, -1>;
using MatX18d = Eigen::Matrix<double, -1, 18>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;
class GetPoseHelper
{
private:
    tf2_ros::Buffer *tf_;
    // reference frame , self frame;
    std::string ref_frame_, self_frame_;

public:
    GetPoseHelper(tf2_ros::Buffer *tf,
                  const std::string &ref_frame,
                  const std::string &self_frame)
        : tf_(tf), ref_frame_(ref_frame), self_frame_(self_frame)
    {
    }
    bool getSelfPose(Mat4f &pose, const ros::Time &t)
    {
        Tfs to_tfs, self_tfs;
        self_tfs.header.stamp = t;
        self_tfs.header.frame_id = self_frame_;
        self_tfs.transform.rotation.w = 1.0;
        try
        {
            to_tfs = tf_->transform(self_tfs ,ref_frame_);// tfs frame_id
        }
        catch (tf2::TransformException e)
        {
            
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        pose = Mat4f::Identity();
        pose.block<3, 1>(0, 3) << to_tfs.transform.translation.x,
            to_tfs.transform.translation.y,
            to_tfs.transform.translation.z;

        pose.block<3, 3>(0, 0) = Eigen::Quaternionf(to_tfs.transform.rotation.w,
                                                    to_tfs.transform.rotation.x,
                                                    to_tfs.transform.rotation.y,
                                                    to_tfs.transform.rotation.z).toRotationMatrix();
        return true;
    }
};

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

    PclT::Ptr latest_laser_cloud_;
    PclT::Ptr cloud_prepro_;

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
    void preprocPointCloud(PclT::Ptr &point_cloud);
    // 根据平面方程计算x的值
    double placeVx(Vec3d sp, const Vec3d nv, double z, double y);
};
