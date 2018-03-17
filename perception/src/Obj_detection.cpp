#include <ros/ros.h>
//
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub_cloud;
ros::Publisher pub_position;

// Twk
double trans_x;
double trans_y;
double trans_z;
double q_w;
double q_x;
double q_y;
double q_z;

// Workspace plane filter
float x_start = 0.15;
float x_end = 0.75;
float y_start = -0.45;
float y_end = 0.45;
float z_start = 0.008;
float z_end = 0.15;

// Object distribution filter
double x_dis = 0.45;
double y_dis = 0;

void 
PCL (const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
  // Transform sensor msg to pcl type
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*source_cloud);
  
  // Transformation
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  
  transform.translation() << trans_x, trans_y, trans_z; 
  transform.rotate (Eigen::Quaternionf(q_w, q_x, q_y, q_z));
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud_output (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (transformed_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_start, x_end);
  pass.filter (*filter_cloud);
  pass.setInputCloud (filter_cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_start, y_end);
  pass.filter (*filter_cloud);
  pass.setInputCloud (filter_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_start, z_end);
  pass.filter (*filter_cloud_output);

  // Outliers removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(filter_cloud_output);
  outrem.setRadiusSearch(0.015);
  outrem.setMinNeighborsInRadius (70);
  outrem.filter (*filter_cloud_output);

  // Cluster segmentation
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (filter_cloud_output);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (filter_cloud_output);
  ec.extract (cluster_indices);

  // Object center calculation
  geometry_msgs::Pose part_position;

  geometry_msgs::PoseArray output;
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "WAIST";
   
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      part_position.position.x += filter_cloud_output->points[*pit].x;
      part_position.position.y += filter_cloud_output->points[*pit].y;
      part_position.position.z += filter_cloud_output->points[*pit].z;
      cloud_cluster->points.push_back (filter_cloud_output->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
  
    part_position.position.x /= cloud_cluster->points.size();
    part_position.position.y /= cloud_cluster->points.size();
    part_position.position.z /= cloud_cluster->points.size();
    
    // Object recognization
    if(part_position.position.x < x_dis && part_position.position.y < y_dis)
      part_position.orientation.x = 0.1;
    if(part_position.position.x < x_dis && part_position.position.y >= y_dis)
      part_position.orientation.x = 0.2;
    if(part_position.position.x >= x_dis && part_position.position.y < y_dis)
      part_position.orientation.x = 0.3;
    if(part_position.position.x >= x_dis && part_position.position.y >= y_dis)
      part_position.orientation.x = 0.4;
    
    output.poses.push_back(part_position);
    j++;
  }

  // Sort position
  geometry_msgs::PoseArray tmp = output;
  for( size_t i = 0; i < output.poses.size()-1; i++ )
  {
    int k = i;
    for( size_t j = i+1; j < output.poses.size(); j++ )
    if( output.poses[k].orientation.x > output.poses[j].orientation.x )
      k = j;
    tmp.poses[i] = output.poses[i];
    output.poses[i] = output.poses[k];
    output.poses[k] = tmp.poses[i];
  }

  // Publish the data
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*filter_cloud_output, output_cloud);
  output_cloud.header.frame_id = "WAIST";
  pub_cloud.publish (output_cloud);
  pub_position.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Object_detection");
  ros::NodeHandle nh;

  // Twk
  tf::TransformListener listener;
  tf::StampedTransform Twk;
  listener.waitForTransform("/WAIST","/kinect2_rgb_optical_frame", ros::Time(), ros::Duration(1.0));
  try
  {
    listener.lookupTransform("/WAIST","/kinect2_rgb_optical_frame", ros::Time(), Twk);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  trans_x = Twk.getOrigin().getX();
  trans_y = Twk.getOrigin().getY();
  trans_z = Twk.getOrigin().getZ();
  q_w = Twk.getRotation().getW();
  q_x = Twk.getRotation().getX();
  q_y = Twk.getRotation().getY();
  q_z = Twk.getRotation().getZ();
  
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/hd/points", 1, PCL);

  // Create a ROS publisher for the output point cloud
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/global_object_cloud", 1);
  pub_position = nh.advertise<geometry_msgs::PoseArray> ("/global_object_position", 1);

  // Spin
  ros::spin ();
}
