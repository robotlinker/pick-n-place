#include "ros/ros.h"
#include "perception_msgs/SquareDetection.h"
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Square_detection_client");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception_msgs::SquareDetection>("Square_detection");
  perception_msgs::SquareDetection srv;
  srv.request.signal = 1;
  while(ros::ok){
    if (client.call(srv))
    {
      std::cout<<"Hello"<<std::endl;
      ROS_INFO("dx: %.4f", srv.response.Square_pose.pose.position.x);
      ROS_INFO("dy: %.4f", srv.response.Square_pose.pose.position.y);
      ROS_INFO("dz: %.4f", srv.response.Square_pose.pose.position.z);
      //ROS_INFO("rsp: %.4f", srv.response.dz);
      ROS_INFO("---------");
    }
    else
    {
      ROS_ERROR("Failed to call test");
      return 1;
    }
    ros::Duration(0.5).sleep();
  }
  return 0;
}
