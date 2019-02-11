#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "color_detect.hpp"

bir::ColorDetect::ColorDetect() {
    setRange(MIN, {0, 0, 0});
    setRange(MAX, {255, 255, 255} );
    setArea(300);
    clean();
}

bir::ColorDetect::ColorDetect(std::vector<u_int8_t> min_limit, std::vector<u_int8_t> max_limit) {
    setRange(MIN, min_limit);
    setRange(MAX, max_limit);
    setArea(300);
    clean();
}

bir::ColorDetect::~ColorDetect() { 
    
}

void bir::ColorDetect::setRange(int option, std::vector<u_int8_t> limit){
    if(option == MIN){
        _minRange = cv::Scalar(limit[0], limit[1], limit[2]);
    } else if (option == MAX) {
        _maxRange = cv::Scalar(limit[0], limit[1], limit[2]);
    } else {
        ROS_ERROR("Range option unavailable.");
    }
}

inline void bir::ColorDetect::setArea(float area){
    if(area >= 0){
        _areaLimit = area;
    } else {
        ROS_ERROR("Area cannot be negative.");
    }
}

inline void bir::ColorDetect::setImage(cv::Mat& image) {
    cv::cvtColor(image, _image, cv::COLOR_BGR2HSV);
}

inline void bir::ColorDetect::foundRange(){
    cv::inRange(_image, _minRange, _maxRange, _image);
    // COLOR -> BLACK/WHITE
}

inline void bir::ColorDetect::findContours(){
    cv::dilate(_image, _image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))); 
    cv::erode(_image, _image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::findContours(_image, _cnt, _hie, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,5));
}

inline void bir::ColorDetect::clean() {
    _detected = false;
    _numberOfObjects = 0;
}

void bir::ColorDetect::findDetails() {
    for(int index = 0; index < _cnt.size(); index++){
        if(cv::contourArea(_cnt[index]) >= _areaLimit){
            _detected = true;
            _numberOfObjects++;
        }
    }
}

u_int16_t bir::ColorDetect::numberOfObjects(){
    return _numberOfObjects;
}

bool bir::ColorDetect::operator()(cv::Mat& image){
    
    if (image.empty()) { 
        ROS_ERROR("Image is Empty.");
        clean();
        return false; 
    }

    setImage(image);
    foundRange();
    findContours();
    clean();
    findDetails();

    return _detected;
}

cv::Mat bir::ColorDetect::drawContours(cv::Mat img){
    for(int index = 0; index < _numberOfObjects; index++){
        cv::drawContours(img, _cnt, index, (0,0, 200), 1.5, 8, _hie, 0, cv::Point(0,-5));
    }
    return img;
}

cv::Mat bir::ColorDetect::drawContours(cv::Mat img, std::string text, cv::Scalar color){
    for(int index = 0; index < _numberOfObjects; index++){
        cv::Rect perimeter = cv::boundingRect(_cnt[index]);
        cv::putText(img, text, cv::Point(perimeter.x + perimeter.width/2, perimeter.y - 10), cv::FONT_HERSHEY_PLAIN, 2, color);
        cv::drawContours(img, _cnt, index, color, 1.5, 8, _hie, 0, cv::Point(0,-5));
    }
    return img;
}





