#include<iostream>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_publisher_topic", 1000);
    ros::Rate rate(10);
    unsigned int num_points = 10;

    //建立pcl点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    // 点云初始化
    cloud.points.resize(num_points);

    //建立ros点云
    sensor_msgs::PointCloud2 output_msg;
    
    while(ros::ok()){
        // 调用ros获取时间的接口，获取系统时间，作为时间戳stamp
        output_msg.header.stamp=ros::Time::now();
        // 创建num_points个绿色的点
        for(int i=0;i<num_points;i++)
        {
            cloud.points[i].x=i;
            cloud.points[i].y=i;
            cloud.points[i].z=i;
            cloud.points[i].r=0;
            cloud.points[i].g=255;
            cloud.points[i].b=0;
        }
        //将pcl点云转换到ros消息对象
        pcl::toROSMsg(cloud, output_msg);
        // 发布的点云坐标系
        output_msg.header.frame_id="point_cloud_frame_id";
        pub.publish(output_msg);
        rate.sleep();
    }
    ros::spin();

    return 0;
}
