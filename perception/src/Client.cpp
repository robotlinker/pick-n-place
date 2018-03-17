#include "ros/ros.h"
#include "perception/GetPose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GetPose_client");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<perception::GetPose>("Get_pose");
  perception::GetPose srv;
  srv.request.signal = 1;
  while(ros::ok){
    if (client.call(srv))
    {
      ROS_INFO("dx: %.4f", srv.response.dx);
      ROS_INFO("dy: %.4f", srv.response.dy);
      //ROS_INFO("rsp: %.4f", srv.response.dz);
      ROS_INFO("---------");
    }
    else
    {
      ROS_ERROR("Failed to call test");
      return 1;
    }
    ros::Duration(0.1).sleep();
  }
  return 0;
}
