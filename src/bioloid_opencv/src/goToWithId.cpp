#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <image_converter/image_converter.hpp>
#include <aruco_identification/aruco_identification.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

void cameraCallback(const sensor_msgs::ImageConstPtr&);
ros::Publisher pubGoal;

int main(int argv, char* argc[]){
    ros::init(argv, argc, "goToWithId");
    ros::NodeHandle node;
    ros::Subscriber subCamera = node.subscribe("/typea/camera/image_raw", 1, cameraCallback);
    pubGoal = node.advertise<geometry_msgs::Point>("/typea/goal_point", 1);
    cv::namedWindow("Image", CV_WINDOW_NORMAL);

    ros::spin();
}

void cameraCallback(const sensor_msgs::ImageConstPtr& ros_img) {

    bir::ImageConverter converter("bgr8");
    bir::Aruco aruco(cv::aruco::DICT_ARUCO_ORIGINAL);
    bir::Aruco::marks mark;
    cv::Mat cv_img;
    geometry_msgs::Point goal;
    static int count;

    cv_img = converter * (ros_img);
    mark = aruco(cv_img);

    if (mark == 1) {
        count++;
        if(count > 15) {
            cv::putText(cv_img, "ID = 1 - Go to ( 4 , 3 )", cv::Point2i(cv_img.cols/4, cv_img.rows/3), cv::FONT_HERSHEY_SIMPLEX, 0.75, CV_RGB(125, 255, 0), 2);
            goal.x = 4;
            goal.y = 3;
            pubGoal.publish(goal);            
        }
    } else if (mark == 2) {
        count++;
        if(count > 15) {
            cv::putText(cv_img, "ID = 2 - Go to ( -2 , -2 )", cv::Point2i(cv_img.cols/4, cv_img.rows/3), cv::FONT_HERSHEY_SIMPLEX, 0.75, CV_RGB(255, 125, 0), 2);
            goal.x = 4;
            goal.y = 3;
            pubGoal.publish(goal);
        }
    } else if (mark == 0) {
        count++;
        if(count > 15) {
            cv::putText(cv_img, "ID = 0 - Go to ( -6 , -8 )", cv::Point2i(cv_img.cols/4, cv_img.rows/3), cv::FONT_HERSHEY_SIMPLEX, 0.75, CV_RGB(255, 125, 255), 2);
            goal.x = 4;
            goal.y = 3;
            pubGoal.publish(goal);
        }
    } else {
        count = 0;
    }

    cv::imshow("Image", cv_img);
    cv::waitKey(3);
}