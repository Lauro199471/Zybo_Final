// Include the ROS C++ APIs
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <unistd.h>

using namespace cv;

cv::Mat imgOriginal;
cv::Mat frame_HSV;

// HSV(HUE Saturation Value) max range
//Hue (0 - 179)
//Saturation (0 - 255)
//Value (0 - 255)
int LowH = 0;
int HighH = 179;

int LowS = 0;
int HighS = 255;

int LowV = 0;
int HighV = 255;


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  imgOriginal = cv_bridge::toCvCopy(msg, "bgr8")->image;
  // Convert 'frame_raw' to a HSV Matrix
  cv::cvtColor(imgOriginal,frame_HSV,COLOR_BGR2HSV);

  //-- Detect the object based on HSV Range Values
  cv::Mat frameFilter;
  inRange(frame_HSV,
          Scalar(LowH, LowS, LowV),
          Scalar(HighH, HighS, HighV),
          frameFilter);

  //morphological opening (remove small objects from the foreground)
  // erode = destory. The erosion makes the object in white smaller.
  // dilate = To resize something. The dilatation makes the object in white bigger.
  erode(frameFilter, frameFilter, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate(frameFilter, frameFilter, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //morphological closing (fill small holes in the foreground)
  dilate(frameFilter, frameFilter, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(frameFilter, frameFilter, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //show the thresholded image
  imshow("Color Tracking", frameFilter);
  imshow("Raw", imgOriginal);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  cv::Mat imgGRAY;

  // Announce this program to the ROS master as a "node" called "hello_world_node"
  ros::init(argc, argv, "color_tracking");
  ros::NodeHandle n("~");

  image_transport::ImageTransport it(n);
  // Subscrive to input video feed
  image_transport::Subscriber image_sub_ = it.subscribe("/usb_cam/image_raw", 2, imageCb);
  
  // create a window called "Control"
  namedWindow("Control", CV_WINDOW_AUTOSIZE);
  // Create trackbars(sliders) in "Control" window for HSV
  cvCreateTrackbar("LowH", "Control", &LowH, 179);
  cvCreateTrackbar("HighH", "Control", &HighH, 179);

  cvCreateTrackbar("LowS", "Control", &LowS, 255);
  cvCreateTrackbar("HighS", "Control", &HighS, 255);

  cvCreateTrackbar("LowV", "Control", &LowV, 255);
  cvCreateTrackbar("HighV", "Control", &HighV, 255);

  // Broadcast a simple log message
  ROS_INFO_STREAM("Hello, world!");
  ros::spin();
  //==================================================
  //=                Main Loop                       =
  //==================================================
  ros::Rate r(1000); // 1 kHz
  while(ros::ok())
  {
    // Process ROS callbacks until receiving a SIGINT (ctrl-c)
    ros::spinOnce();
    r.sleep();
  }

  // Stop the node's resources
  ros::shutdown();
  return 0;
}
