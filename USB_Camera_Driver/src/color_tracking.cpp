// Include the ROS C++ APIs
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <unistd.h>

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  //cv::imshow("Color Tracking Window", cv_bridge::toCvShare(msg, "bgr8")->image);
  
  
  cv::Mat imgOriginal = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cv::Mat imgGRAY;
  cv::Mat imgCOLOR;
  
  cv::cvtColor(imgOriginal, imgGRAY, cv::COLOR_RGB2GRAY);//Convert the captured frame from RBG to GRAY
  cv::imshow("Gray Convert", imgGRAY);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  // Announce this program to the ROS master as a "node" called "hello_world_node"
  ros::init(argc, argv, "color_tracking");
  ros::NodeHandle n("~");

  image_transport::ImageTransport it(n);
  // Subscrive to input video feed
  image_transport::Subscriber image_sub_ = it.subscribe("/image_raw_zybo", 2, imageCb);
  
  
  // Broadcast a simple log message
  ROS_INFO_STREAM("Hello, world!");
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}
