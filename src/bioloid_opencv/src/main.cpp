// Codigo para estudo do OpenCV com ROS

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_converter/image_converter.hpp>
#include <color_detect/color_detect.hpp>
#include <aruco_identification/aruco_identification.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <geometry_msgs/Point.h>



static const std::string OPENCV_WINDOW = "Image window";

class OpenCV {

    ros::NodeHandle nh_;
    bir::ImageConverter* ImgConv;
    bir::Aruco* tagRead;
    bir::Aruco::marks mark;
    bir::ColorDetect* BlueDet;
    bir::ColorDetect* OranDet;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher goalPub;

    public:
        OpenCV(ros::NodeHandle& node) : it_(nh_), nh_(node) {
            ImgConv = new bir::ImageConverter("bgra8");
            tagRead = new bir::Aruco(16);
            std::vector<u_int8_t> min {80, 0, 0};
            std::vector<u_int8_t> max {120, 255, 255};
            BlueDet = new bir::ColorDetect(min , max);
            min = {1, 0, 0};
            max = {20, 255, 255};
            OranDet = new bir::ColorDetect(min, max);
            BlueDet->setArea(100);
            OranDet->setArea(500);

            // Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe("/typea/camera/image_raw", 1, &OpenCV::imageCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);
            goalPub = nh_.advertise<geometry_msgs::Point>("/typea/goal_point", 1);
            
            cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_FULLSCREEN);
      }

    ~OpenCV() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat image = *(ImgConv)*(msg);
        mark = (*tagRead)(image);
        (*BlueDet)(image);
        (*OranDet)(image);

        if(mark.size > 0){
            cv::aruco::drawDetectedMarkers(image, mark.corner, mark.id);
            std::vector<int>::iterator index = std::find(mark.id.begin(), mark.id.end(), 1);
            if(index != mark.id.end()){
                geometry_msgs::Point goal;
                goal.x = 5;
                goal.y = -5;
                goalPub.publish(goal);
            }
        }
        
        cv::Mat draw = BlueDet->drawContours(image, "Blue", cv::Scalar(255,128,0));
        draw = OranDet->drawContours(draw, "Orange", cv::Scalar(38,96,240));

        cv::imshow(OPENCV_WINDOW, draw);
        cv::waitKey(3);

        // Output modified video stream
        sensor_msgs::ImagePtr img_msg = *(ImgConv)*image;
        image_pub_.publish(img_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle node;
    OpenCV ic(node);

    ros::spin();

    return 0;
}