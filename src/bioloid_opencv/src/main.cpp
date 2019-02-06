#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_converter/image_converter.hpp>
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter1
{
  ros::NodeHandle nh_;
  bir::ImageConverter* ImgConv;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter1() : it_(nh_)
  {
    ImgConv = new bir::ImageConverter("rgb8");
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/typea/camera/image_raw", 1,
      &ImageConverter1::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter1()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat image = *(ImgConv)*(msg);
    // Draw an example circle on the video stream
    if (image.rows > 60 && image.cols > 60)
      cv::circle(image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(3);

    // Output modified video stream
    sensor_msgs::ImagePtr img_msg = *(ImgConv)*image;
    image_pub_.publish(img_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter1 ic;
  ros::spin();
  return 0;
}