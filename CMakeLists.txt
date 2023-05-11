cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_publisher src/create_point_cloud_pub.cpp)
target_link_libraries(point_cloud_publisher ${PCL_LIBRARIES}  ${catkin_LIBRARIES} )

add_executable(point_cloud_transform src/pub_sub_pcl_topic_pkg.cpp)
target_link_libraries(point_cloud_transform ${PCL_LIBRARIES}  ${catkin_LIBRARIES} )

add_executable(pcl_collision_node src/pcl_collision.cc)
target_link_libraries(pcl_collision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_collision3d_node src/pcl_collision3d.cc)
target_link_libraries(pcl_collision3d_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})



