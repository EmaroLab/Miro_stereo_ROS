/**
 * \file scaleimage_left.cpp
 * \brief A node to scale and republish the Image from Miro's left eye camera. 
 * \authors Parag Khanna, John Thomas
 * \version 0.1
 * \date  11 February 2018
 * 
 * \param[in] ...
 * 
 * Subscribes to: /miro/rob01/platform/caml<BR>
 * 
 * Publishes to: /miro_scaledimage/left/image_raw<BR>
  
 * This node does scaling specifically for stereo vision purpose, giving the output image with only the stereo-overlap region. 
 * The node subscribes to the raw image from Miro and publishes via image_transport. The image_transport will automatically discover all
 * transport plugins built in your ROS system.Thus would give the output images in all topics corresponding to various transport plugins
 * available.
 */

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Image window for Opencv
/*static const std::string OPENCV_WINDOW = "Image window";*/
//can uncomment above and all other commands commented by "/*.....*/" for launching an open_cv window, useful for debugging.

//class for Republishing
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    //Subscribing to input image_topic and publishing to output image_topic//
    image_sub_ = it_.subscribe("/miro/rob01/platform/caml", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/miro_scaledimage/left/image_raw", 1);

    /*cv::namedWindow(OPENCV_WINDOW);*/
  }

  ~ImageConverter()
  {
     /*cv::destroyWindow(OPENCV_WINDOW);*/
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   	
	cv_ptr->image=cv_ptr->image(cv::Range(0,cv_ptr->image.rows),cv::Range(180,320));

     /*// Update GUI Window*/
   /* cv::imshow(OPENCV_WINDOW, cv_ptr->image);*/
    /*cv::waitKey(3);*/

   // Modified image to output_topic//
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scaleimage_left");
  ImageConverter ic;
  ros::spin();
  return 0;
}
