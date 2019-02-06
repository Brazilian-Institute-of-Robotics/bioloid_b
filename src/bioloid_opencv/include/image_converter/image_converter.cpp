#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "image_converter.hpp"

bir::ImageConverter::ImageConverter(std::string encoding): _encoding(encoding) { }

bir::ImageConverter::~ImageConverter() { }

cv::Mat bir::ImageConverter::convertImage(const sensor_msgs::ImageConstPtr& imagem_ros){
    cv_bridge::CvImagePtr ptr_imagem_cv;
    try { ptr_imagem_cv = cv_bridge::toCvCopy(imagem_ros, _encoding); }
    catch(cv_bridge::Exception& erro) {
        ROS_ERROR("CV Bridge exception: %s", erro.what());
        cv::Mat empty_mat;
        return empty_mat;
    }

    return ptr_imagem_cv->image;
}

sensor_msgs::Image::Ptr bir::ImageConverter::convertImage(cv::Mat& openCV_image){
    cv_bridge::CvImage ros_image;
    ros_image.encoding = _encoding;
    ros_image.image = (openCV_image);
    
    return (ros_image.toImageMsg());
}

cv::Mat bir::ImageConverter::operator*(const sensor_msgs::ImageConstPtr& imagem_gazebo){
    return this->convertImage(imagem_gazebo);
}

sensor_msgs::Image::Ptr bir::ImageConverter::operator*(cv::Mat& imagem_openCV){
    return this->convertImage(imagem_openCV);
}

