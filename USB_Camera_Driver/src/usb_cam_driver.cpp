#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#define id_camera -1
#define FPS 60
#define kHz 1000
int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_cam_apada");
    ros::NodeHandle n("~");

    std::cout << "\n(Long Beach State:CECS461)Zybo USB Camera Driver Initiated...\n";

    // ROS image transport for publishing and subscribing the ROS images
    // in different compression
    image_transport::ImageTransport *it;
    it = new image_transport::ImageTransport(n);

    // cv::Mat <--> sensor_msgs/Image
    // Brige between OpenCV and ROS
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id = "image_1"; // Header used for Time Sync
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Image encoding ("mono8", "bgr8", etc.)

    // Setup ROS topic
    image_transport::Publisher pub;
    pub = it->advertise("/image_raw_zybo" , 1);

    // Setup Camera
    cv::Mat frame;
    cv::VideoCapture input_video;
    input_video.open(id_camera);
    input_video.set(CV_CAP_PROP_FPS,FPS);
    if(!input_video.isOpened())
    {
      ROS_ERROR("Couldnt open camera");
      ros::shutdown();
    }

    ros::Rate loop_rate(1 * kHz);
    while(ros::ok())
    {
      input_video.read(frame); // Get Frame
      out_msg.header.stamp = ros::Time::now(); // Get Time of when Frame was read
      out_msg.image = frame;   // Put Frame in MSG
      pub.publish(out_msg.toImageMsg()); // Publish MSG
      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
