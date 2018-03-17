cmake_minimum_required(VERSION 2.8.3)
project(perception)


find_package(catkin REQUIRED COMPONENTS
  
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  tf
  perception_msgs
)

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)
find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(Get_pose_server src/Get_pose_server.cpp)
target_link_libraries(Get_pose_server ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(Get_pose_server perception_msgs) 

add_executable(Client src/Client.cpp)
target_link_libraries(Client ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(Client perception_msgs) 

