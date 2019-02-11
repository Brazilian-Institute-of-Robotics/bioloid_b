#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace bir {
    class ImageConverter {
        public:
            /* 
                Create an ImageConverter object.
                Possible Convert ROS <-> OpenCV easily
                Params: encoding 
            */
            explicit ImageConverter(std::string encoding);
            virtual ~ImageConverter();
            /*
                Easy use of the library like a Transformation
                cv = Transformation * raw;
            */
            cv::Mat operator*(const sensor_msgs::ImageConstPtr& imagem_ros);
            /*
                Easy use of the library like a Transformation
                raw = Transformation * cv;
            */
            sensor_msgs::Image::Ptr operator*(cv::Mat& imagem_openCV);
        
        private:
            std::string _encoding;
            cv::Mat convertImage(const sensor_msgs::ImageConstPtr&);
            sensor_msgs::Image::Ptr convertImage(cv::Mat&);
    };
}
