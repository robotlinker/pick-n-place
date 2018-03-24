cmake_minimum_required(VERSION 2.8.3)
project(recongnition_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometric_shapes shape_msgs message_generation)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
# Generate messages in the 'msg' folder
#add_message_files(FILES Foo.msg)

# Generate service in the 'srv' folder
add_service_files(
  FILES
  GetTargetPose.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs shape_msgs)

catkin_package(CATKIN_DEPENDS std_msgs shape_msgs message_runtime)
