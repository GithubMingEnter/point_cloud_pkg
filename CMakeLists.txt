cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_pkg)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 14)  # 将 C++ 标准设置为 C++ 14
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # C++ 14 是强制要求，不会衰退至低版本
set(CMAKE_CXX_EXTENSIONS OFF)  # 禁止使用编译器特有扩展
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  message_filters
  laser_geometry
)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(spdlog REQUIRED)

catkin_package(

 )

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${spdlog_INCLUDE_DIRS}
  include
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

add_executable(laser_safe_region_node src/laser_safe_region.cc)
target_link_libraries(laser_safe_region_node
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
${spdlog_LIBRARIES}

)




