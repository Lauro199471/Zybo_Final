// Include the ROS C++ APIs
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle n("~");
  image_transport::ImageTransport it(n);
  cv_bridge::CvImagePtr cv_ptr;
  image_transport::Publisher image_pub_;
  image_pub_ = it.advertise("/image_converter/output_video", 1);

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(205, 50), 10, CV_RGB(255,0,0));

  // Update GUI Window
  cv::imshow("Color Tracking Window", cv_ptr->image);
  cv::waitKey(1);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
  // Announce this program to the ROS master as a "node" called "hello_world_node"
  ros::init(argc, argv, "color_tracking");
  ros::NodeHandle n("~");
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub_;

   // Subscrive to input video feed
  image_sub_ = it.subscribe("/image_raw_zybo", 2, imageCb);

  cv::namedWindow("Color Tracking Window");

  // Start the node resource managers (communication, time, etc)
  ros::start();

  // Broadcast a simple log message
  ROS_INFO_STREAM("Hello, world!");
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}
