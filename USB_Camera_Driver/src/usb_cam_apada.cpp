#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class CameraDriver
{
public:
  int id_camera;
  int FPS;
  bool show;
  cv::VideoCapture input_video;
  cv::Mat frame;

  // ROS <-> OpenCV
  cv_bridge::CvImage cvi;
  image_transport::ImageTransport *it;
  image_transport::Publisher pub;

  // Constructor
  CameraDriver()
  {
    ros::NodeHandle n("~");
    n.param("camera_index" , id_camera , -1);
    n.param("show", show , false);
    n.param("FPS" , FPS , 60);

    cvi.header.frame_id = "image";
    cvi.encoding = sensor_msgs::image_encodings::BGR8;

    it = new image_transport::ImageTransport(n);
    pub = it->advertise("/image_raw_zybo" , 1);

    if(id_camera == -1)
    {
      ROS_WARN("Camera's id was not recieved");
      ROS_WARN("I will open every camera that I find");
    }

    input_video.open(id_camera);
    input_video.set(CV_CAP_PROP_FPS,FPS);

    if(!input_video.isOpened())
    {
      ROS_ERROR("Couldnt open camera");
      ros::shutdown();
    }
    ros::Rate loop_rate(FPS);

    while(ros::ok())
    {
      input_video.read(frame);
      cvi.image = frame;
      cvi.header.stamp = ros::Time::now();
      pub.publish(cvi.toImageMsg());

      if(show)
      {
        cv::imshow("imgout.jpg",frame);
        if(cv::waitKey(1) > 0);
      }

      loop_rate.sleep();
    }

  }

};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "usb_cam_apada");


  std::cout << "Hello World" << std::endl;
  CameraDriver camera;


  return EXIT_SUCCESS;
}
