#include "aubo_driver/aubo_driver.h"

#include <string>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_io");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<aubo_msgs::SetIO>("aubo_driver/set_io");
  aubo_msgs::SetIO srv;
  srv.request.fun = 1;
  srv.request.pin = 0;
  srv.request.state = 1;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to set IO");
    return 1;
  }

  return 0;
}
