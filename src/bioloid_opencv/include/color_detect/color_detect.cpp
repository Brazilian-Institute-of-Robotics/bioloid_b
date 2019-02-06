#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "color_detect.hpp"

bir::ColorDetect::ColorDetect(cv::Mat& image){
    cv::cvtColor(image, _image, CV_BGR2HSV);
    setRange(MIN, {0, 0, 0});
    setRange(MAX, {255, 255, 255} );
    _areaLimit = 300;
    _numberOfObjects = 0;
    _detected = false;
}

bir::ColorDetect::ColorDetect(cv::Mat& image, std::vector<u_int8_t> min_limit, std::vector<u_int8_t> max_limit) {
    cv::cvtColor(image, _image, CV_BGR2HSV);
    setRange(MIN, min_limit);
    setRange(MAX, max_limit);
    _areaLimit = 300;
    _numberOfObjects = 0;
    _detected = false;
}

void bir::ColorDetect::setRange(int option, std::vector<u_int8_t> limit){
    if(option == MIN){
        _minRange = cv::Scalar(limit[0], limit[1], limit[2]);
    } else if (option == MAX) {
        _maxRange = cv::Scalar(limit[0], limit[1], limit[2]);
    } else {
        ROS_ERROR("set Range options not defined");
    }
}

inline cv::Mat bir::ColorDetect::foundRange(){
    cv::Mat img_with_contours = _image.clone(); // img_with_contours have the same size that _image
    cv::inRange(_image, _minRange, _maxRange, img_with_contours);

    return img_with_contours;
}

inline std::vector<std::vector<cv::Point>> bir::ColorDetect::findContours(){
    cv::Mat img_with_contours = foundRange();

    std::vector<std::vector<cv::Point>> cnt;
    std::vector<cv::Vec4i> hie;
    cv::findContours(img_with_contours, cnt, hie, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,5));

    return cnt;
}

void bir::ColorDetect::findDetails(){
    clean();
    std::vector<std::vector<cv::Point>> cnt = findContours();
    for(int index = 0; index < cnt.size(); index++){
        if(cv::contourArea(cnt[index]) > _areaLimit){
            _detected = true;
            _numberOfObjects++;
        }
    }
}

inline void bir::ColorDetect::clean(){
    _detected = false;
    _numberOfObjects = 0;
}






