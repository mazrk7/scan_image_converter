//==============================================================================
// INCLUDES
//==============================================================================
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//==============================================================================
// CLASS DEFINITION
//==============================================================================
class ScanImageConverter
{
  public:
    ScanImageConverter(tf2_ros::Buffer& tf);
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);

  private:
    cv::Mat scanToImage(const sensor_msgs::LaserScan& scan) const;

    std::string base_frame_;

    tf2_ros::Buffer& tf_buffer_;

    ros::Subscriber laser_sub_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_flip_pub_;

    int size_x_;
    int size_y_;
};

//==============================================================================
// PRIVATE SCAN IMAGE CONVERTER METHODS (Utilities)
//==============================================================================
cv::Mat ScanImageConverter::scanToImage(const sensor_msgs::LaserScan &scan) const
{
  // Initialise empty image of depth 1 i.e. one channel
  cv::Mat image = cv::Mat::zeros(size_y_, size_x_, CV_8UC1);

  // Get appropriate transform
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(
            base_frame_,
            scan.header.frame_id,
            ros::Time(0),
            ros::Duration(0));
  }
  catch (const ros::Exception& ex)
  {
    // Returns empty image
    ROS_ERROR("Error during transform: %s", ex.what());
    return image;
  }

  // Convert each polar coordinate into a Cartesian coordinate
  int row, col;

  // Have map span a max scan range grid (m/pixel resolution)
  double x_resolution = scan.range_max/(size_x_*0.5);
  double y_resolution = scan.range_max/(size_y_*0.5);

  int origin_x = ceil(size_x_/2);
  int origin_y = ceil(size_y_/2);

  uchar* p = image.data;
  for (unsigned int i=0; i < scan.ranges.size(); ++i)
  {
    const double& range = scan.ranges.at(i);

    if (range > scan.range_min && range < scan.range_max)
    {
      geometry_msgs::PointStamped scan_point;
      geometry_msgs::PointStamped base_point;

      double angle = scan.angle_min + i*scan.angle_increment;

      scan_point.point.x = range*std::cos(angle);
      scan_point.point.y = range*std::sin(angle);
      scan_point.point.z = 0.0;

      tf2::doTransform(scan_point, base_point, transform);

      col = origin_x + floor(base_point.point.x/x_resolution);
      row = origin_y + floor(base_point.point.y/y_resolution);

      // Set to white
      p[size_x_*row + col] = 255;
    }
  }
            
  return image;
}

//==============================================================================
// PUBLIC SCAN IMAGE CONVERTER METHODS
//==============================================================================
ScanImageConverter::ScanImageConverter(tf2_ros::Buffer& tf) : tf_buffer_(tf)
{ 
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  nh_priv.param<std::string>("base_frame", base_frame_, std::string("base_link"));
  nh_priv.param<int>("image_size_x", size_x_, 320);
  nh_priv.param<int>("image_size_y", size_y_, 240);

  // Topic names
  std::string scan_topic;
  std::string image_topic;
  nh_priv.param<std::string>("scan_topic", scan_topic, std::string("base_scan"));
  nh_priv.param<std::string>("image_topic", image_topic, std::string("scan_image"));

  // Subscriber-publisher
  laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_topic.c_str(), 1, &ScanImageConverter::laserCallback, this);

  image_transport::ImageTransport it(nh);
  image_pub_ = it.advertise(image_topic, 10);
  // For a flipped image use-case
  image_flip_pub_ = it.advertise(std::string("scan_image/flipped"), 10);
}

void ScanImageConverter::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
  sensor_msgs::LaserScan scan_msg = *scan;  

  cv::Mat image = scanToImage(scan_msg);

  // Dilate and erode white for expanded border dimensions, using a 3x3 kernel
  // Morphological process also closes small holes between scanner readings
  cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);
  cv::erode(image, image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);

  // Publish the converted image
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(scan_msg.header, "mono8", image).toImageMsg();
  image_pub_.publish(img_msg);
  
  // Publish the flipped image
  cv::Mat flipped;
  cv::flip(image, flipped, 1);
  sensor_msgs::ImagePtr rot_msg = cv_bridge::CvImage(scan_msg.header, "mono8", flipped).toImageMsg();
  image_flip_pub_.publish(rot_msg);
}

//==============================================================================
// MAIN
//==============================================================================
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "scan_image_converter");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  ScanImageConverter converter(buffer);

  // Spin and leave work for callbacks
  ros::spin();

  return 0;
}
