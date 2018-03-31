#include "ros/ros.h"
#include "perception_msgs/GetPose.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>
#include <sstream>
#include <tf/transform_broadcaster.h>

#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PCLPointCloud2.h>

#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/bind.hpp>


using namespace cv;
using namespace std;

Point Center;
geometry_msgs::Pose output;
ros::Publisher pub;
Mat src;
double Angle;
float P2M = 6.5e-4;


// Function declarations
void drawAxis(Mat&, Point, Point, Scalar, const float);
double getOrientation(const vector<Point> &, Mat&);

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
  double angle;
  double hypotenuse;
  angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
  hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
  // Here we lengthen the arrow by a factor of scale
  q.x = (int) (p.x - scale * hypotenuse * cos(angle));
  q.y = (int) (p.y - scale * hypotenuse * sin(angle));
  line(img, p, q, colour, 3, CV_AA);
}

double getOrientation(const vector<Point> &pts, Mat &img)
{
  //Construct a buffer used by the pca analysis
  int sz = static_cast<int>(pts.size());
  Mat data_pts = Mat(sz, 2, CV_64FC1);
  for (int i = 0; i < data_pts.rows; ++i)
  {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }
  //Perform PCA analysis
  PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
  //Store the center of the object
  Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                    static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  //Store the eigenvalues and eigenvectors
  vector<Point2d> eigen_vecs(2);
  vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i)
  {
    eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                            pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }
  // Draw the principal components
   
  Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
  //drawAxis(img, cntr, p1, Scalar(0, 0, 255), 1);
  //drawAxis(img, cntr, p2, Scalar(0, 255, 0), 1);
  circle(img, cntr, 5, Scalar(255, 0, 0), -1);
  double angle = atan2(eigen_vecs[0].x, eigen_vecs[0].y); // orientation in radians
  Center = cntr;
  
  return angle;
}

void imageCb (const sensor_msgs::ImageConstPtr& msg)
{
  // Load image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  src = cv_ptr->image;
}

bool GetPose(perception_msgs::GetPose::Request  &req,
             perception_msgs::GetPose::Response &res)
{
  if( req.signal == 0 )
  {
    res.dx = 0;
    res.dy = 0;
    res.dz = 0;
    res.flag = 0;
  }
  else
  { 
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    // Convert image to binary
    Mat bw;
    threshold(gray, bw, 150, 255, 1);
    // Find all the contours in the thresholded image
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    findContours(bw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    
    // Sort areas
    vector<vector<Point> > filt;
    for (size_t i = 0; i < contours.size(); ++i)
    {
      // Calculate the area of each contour
      double area = contourArea(contours[i]);
        
      // Ignore contours that are too small or too large
      if (area < 1e4 || 1e5 < area) continue;

      filt.push_back(contours[i]);
      Scalar color = Scalar(0, 0, 255);
      drawContours( src, contours, i, color, 2, 8, hierarchy, 0, Point());
    }  
    
    // Publish output
    if(filt.size() != 0){
      for( size_t i = 0; i < filt.size(); i++ ){
        getOrientation(filt[i], src);  
      }
      Angle = getOrientation(filt[filt.size()-req.signal], src);
      output.position.x = ( Center.x - 320) * P2M;
      output.position.y = -( Center.y - 240) * P2M;
      output.position.z = Angle * 180 / CV_PI - 180;

      res.dx = output.position.x;
      res.dy = output.position.y;
      res.dz = output.position.z;
      res.flag = 1;
    }
    else{
      res.dx = 0;
      res.dy = 0;
      res.dz = 0;
      res.flag = 0;
    }
    //imwrite( "test.jpg", src);
    cv_bridge::CvImage pub_img;
    pub_img.encoding = sensor_msgs::image_encodings::BGR8;
    pub_img.image = src;
    pub.publish(pub_img.toImageMsg());
    
    double position_x;
    double position_y;
    double position_z;
    position_x = res.dx;
    position_y = res.dy;
    position_z = res.dz;
    
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(position_x, position_y, position_z) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker_1", "world"));

  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Get_pose_server");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/depthsense/image_raw", 1, imageCb);
  pub = nh.advertise<sensor_msgs::Image>("img_proc_rslt", 50);

  ros::ServiceServer service = nh.advertiseService("Get_pose", GetPose);

  ros::spin();

  return 0;
}
