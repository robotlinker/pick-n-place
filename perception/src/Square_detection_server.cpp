#include "ros/ros.h"
#include "perception_msgs/SquareDetection.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/image_encodings.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <math.h>
#include <string.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <boost/make_shared.hpp>


using namespace cv;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef struct 
{
  int square_index;
  Point2f square_center;
}target_square;

Point Center;
geometry_msgs::Pose output;
ros::Publisher pub;
Mat src;
pcl::PCLPointCloud2 pcl_src;
double Angle;
float P2M = 6.5e-4;
int thresh = 1200;
int N = 1;
vector<vector<Point> > squares;
pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

// Function declarations
void drawAxis(Mat&, Point, Point, Scalar, const float);
double getOrientation(const vector<Point> &, Mat&);

//////////////////////////////////////////////////////////////////////////////
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = squares.size()-1; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];

        int n = (int)squares[i].size();

        polylines(image, &p, &n, 1, true, Scalar(255,0,0), 3, LINE_AA);
    }
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
target_square findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    // blur will enhance edge detection
    Mat timg;
    image.copyTo(timg);
    Mat img_mei;
    image.copyTo(img_mei);

    medianBlur(img_mei, timg, 9);
    Mat gray0(timg.size(), CV_8U), gray;

    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 5, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1), 3);
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 && fabs(contourArea(Mat(approx))) < 10000 &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
    target_square Tsquare;
    
    Tsquare.square_index = 0;
    float radius;
    minEnclosingCircle( squares[Tsquare.square_index], Tsquare.square_center, radius);
    return Tsquare;
}


//////////////////////////////////////////////////////////////////////////////
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

double getOrientation(const Mat& image, vector<vector<Point> >& squares)
{
  for( size_t i = 0; i < squares.size(); i++ )
  {
    const Point* p = &squares[i][0];
    double length1, length2;

    int n = (int)squares[i].size();

    Point p1_ = squares[i][0];
    Point p2_ = squares[i][1];
    Point p3_ = squares[i][2];
    Point p4_ = squares[i][3];
    Center = Point((p1_.x+p2_.x+p3_.x+p4_.x)/4, (p1_.y+p2_.y+p3_.y+p4_.y)/4);
    length1 = sqrt((p1_.x - p2_.x) * (p1_.x - p2_.x) + (p1_.y - p2_.y) * (p1_.y - p2_.y));
    length2 = sqrt((p2_.x - p3_.x) * (p2_.x - p3_.x) + (p2_.y - p3_.y) * (p2_.y - p3_.y));
    Angle = atan2((double)p1_.y - p2_.y, (double) p1_.x - p2_.x);

    circle(image, Center, 5, Scalar(255, 0, 0), -1);  
  }
  int l = 40;
  Point temp1 = Point((int)Center.x + cos(Angle*CV_PI/180 + CV_PI/2)*l, (int)Center.y + sin(Angle*CV_PI/180 + CV_PI/2)*l);
  Point temp2 = Point((int)Center.x + cos((Angle-90)*CV_PI/180 + CV_PI/2)*l, (int)Center.y + sin((Angle-90)*CV_PI/180 + CV_PI/2)*l);
  line(image, Center, temp1, Scalar(0, 0, 255), 3, CV_AA);
  line(image, Center, temp2, Scalar(0, 255, 0), 3, CV_AA);

  return Angle;
}

void imageCb (const sensor_msgs::ImageConstPtr& msg)
{
  // Load image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  src = cv_ptr->image; // 1920*1080
}

void point_cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
  pcl_conversions::toPCL(*msg, pcl_src);
  pcl::fromPCLPointCloud2(pcl_src, *sensor_cloud_ptr); // 512*424
}


bool SquareDetection(perception_msgs::SquareDetection::Request &req,
             perception_msgs::SquareDetection::Response &res)
{
  ros::Time t = ros::Time::now();
  geometry_msgs::Pose empty;
  if( req.signal == 0 )
  {
    res.Square_pose.pose = empty;
    res.flag = 0;   
  }
  else if( req.signal == 1 && src.rows != 0)
  { 
    Mat dst = src(Rect(540, 330, 500, 350));//src(Rect(120, 130, 200, 130));
    imwrite("test.jpg", dst);
    target_square Target_square = findSquares(dst, squares);

    // Publish output
    if(squares.size() != 0){

      Angle = getOrientation(dst, squares);
      drawSquares(dst, squares);

      Point tp1 = squares[squares.size()-1][0];
      Point tp2 = squares[squares.size()-1][1];
      Point tp3 = squares[squares.size()-1][2];
      Point tp4 = squares[squares.size()-1][3];
      Point Target_center = Point((int)(tp1.x+tp2.x+tp3.x+tp4.x)/4, (int)(tp1.y+tp2.y+tp3.y+tp4.y)/4);

      Target_center.x += 540;
      Target_center.y += 330;

      cout << Target_center << endl;

      Point pcl_Target_center;

      pcl_Target_center.x = (int) Target_center.x * pcl_src.width / src.cols;
      pcl_Target_center.y = (int) Target_center.y * pcl_src.height / src.rows;

      cout << pcl_Target_center << endl;
      cout << "===" << endl;

      float x_pc, y_pc, z_pc;

      x_pc = sensor_cloud_ptr->at(pcl_Target_center.x, pcl_Target_center.y).x;
      y_pc = sensor_cloud_ptr->at(pcl_Target_center.x, pcl_Target_center.y).y;
      z_pc = sensor_cloud_ptr->at(pcl_Target_center.x, pcl_Target_center.y).z;

      if(isnan(x_pc))
      {
        x_pc = -1;
        y_pc = -1;
        z_pc = -1;
      }
      

      static tf::TransformBroadcaster br;
      static tf::TransformListener listener;

      tf::Transform transform;
    
      transform.setOrigin( tf::Vector3(x_pc, y_pc, z_pc) );
      tf::Quaternion transform_orientation = tf::createQuaternionFromRPY(3.14, 0, Angle);
      transform.setRotation( transform_orientation );
      br.sendTransform(tf::StampedTransform(transform, t, "kinect2_ir_optical_frame", "target_box"));

      tf::StampedTransform getTransform;
      listener.waitForTransform("/world","/target_box", ros::Time(), ros::Duration(1.0));
      try{
        listener.lookupTransform("/world","/target_box", ros::Time(0), getTransform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      res.Square_pose.pose.position.x = getTransform.getOrigin().x();
      res.Square_pose.pose.position.y = getTransform.getOrigin().y();
      res.Square_pose.pose.position.z = getTransform.getOrigin().z();
      res.Square_pose.pose.orientation.x = getTransform.getRotation().x();
      res.Square_pose.pose.orientation.y = getTransform.getRotation().y();
      res.Square_pose.pose.orientation.z = getTransform.getRotation().z();
      res.Square_pose.pose.orientation.w = getTransform.getRotation().w();
      res.flag = 1;
    }
    else
    {
      res.Square_pose.pose = empty;
      res.flag = 2;
    }    

    cv_bridge::CvImage pub_img;
    pub_img.encoding = sensor_msgs::image_encodings::BGR8;
    pub_img.image = dst;
    pub.publish(pub_img.toImageMsg());
    src.release();
  }
  else
  {
    res.Square_pose.pose = empty;
    res.flag = 0;    
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Square_detection_server");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/kinect2/hd/image_color_rect", 1, imageCb);
  ros::Subscriber point_cloud_subscriber = nh.subscribe("/kinect2/sd/points",1, point_cloud_callback);
  pub = nh.advertise<sensor_msgs::Image>("img_proc_rslt", 50);

  ros::ServiceServer service = nh.advertiseService("Square_detection", SquareDetection);
  ros::spin();

  return 0;
}
