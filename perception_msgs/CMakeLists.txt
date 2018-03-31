cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

# Generate messages in the 'msg' folder
#add_message_files(FILES Foo.msg)

# Generate service in the 'srv' folder
add_service_files(FILES GetPose.srv)
add_service_files(FILES SquareDetection.srv)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)
