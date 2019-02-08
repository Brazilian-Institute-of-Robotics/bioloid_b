#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <vector>
#include <iostream>

#include "aruco_identification.hpp"

bir::Aruco::Aruco(int dictionary) {
    if(dictionary > 16) throw ("Default Dictionary not Found");
    setPredefinedDictionary(dictionary);
    
    _parameters = cv::aruco::DetectorParameters::create();
}

bir::Aruco::~Aruco(){

}

void bir::Aruco::setPredefinedDictionary(int dictionary) {
    _dictionary = cv::aruco::getPredefinedDictionary(dictionary);
}

void bir::Aruco::setCustomDictionary(int number_of_marks, int number_of_bits) {
    _dictionary = cv::aruco::generateCustomDictionary(number_of_marks, number_of_bits);
}

void bir::Aruco::setParameters(cv::Ptr<cv::aruco::DetectorParameters> parameters){
    _parameters = parameters;
}

bir::Aruco::marks bir::Aruco::operator()(cv::Mat img) {
    marks markOutput;
    if (!img.empty()) {
        cv::aruco::detectMarkers(img, _dictionary, _corners, _ids, _parameters, _rejected); 
        markOutput.corner = _corners;
        markOutput.id = _ids;
        markOutput.rejected = _rejected;
        markOutput.size = _ids.size();

        _corners.clear();
        _ids.clear();
        _rejected.clear();
    }

    return markOutput;
}