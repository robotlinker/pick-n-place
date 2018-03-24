#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <recongnition/GetTargetPose.h>

int main(int argc, char **argv)
{

  geometry_msgs::Pose box_pose;

  // ros initialization
  ros::init(argc, argv, "target_recongnition_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("test the object recongnition service!");

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("target_pose", 1000);

  ros::ServiceClient target_recognition_client;

  // target recognition client (perception)
  target_recognition_client = nh.serviceClient<recongnition::GetTargetPose>("target_recognition");

  if(ros::ok() && !target_recognition_client.waitForExistence(ros::Duration(2.0f)))
  {
    ROS_INFO_STREAM("Waiting for service");
  }

  // creating shape for recognition
  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = 0.1; //cfg.BOX_SIZE.getX();
  shape.dimensions[1] = 0.1; //cfg.BOX_SIZE.getY();
  shape.dimensions[2] = 0.1; //cfg.BOX_SIZE.getZ();

  // creating request object
  recongnition::GetTargetPose srv;
  srv.request.shape = shape;
  srv.request.world_frame_id = "world";
  //srv.request.ar_tag_frame_id = cfg.AR_TAG_FRAME_ID;
  srv.request.ar_tag_frame_id = "1";

  geometry_msgs::Pose place_pose;
  //tf::poseTFToMsg(cfg.BOX_PLACE_TF,place_pose);
  srv.request.remove_at_poses.push_back(place_pose);

  while(ros::ok())
  {
    if(target_recognition_client.call(srv))
    {
      if(srv.response.succeeded)
      {
        box_pose = srv.response.target_pose;
        ROS_INFO_STREAM("target recognition succeeded");
      }
      else
      {
        ROS_ERROR_STREAM("target recognition failed");
        exit(0);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Service call for target recognition failed with response '"<<
          (srv.response.succeeded ?"SUCCESS":"FAILURE")
              <<"', exiting");
      exit(0);
    }

    // updating box marker for visualization in rviz
    //visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
    //cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
    //cfg.MARKER_MESSAGE.pose = box_pose;
    //cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z - 0.5f*cfg.BOX_SIZE.z();

    //show_box(true);

    //pose_pub(box_pose);

   return 0;
  }
}
