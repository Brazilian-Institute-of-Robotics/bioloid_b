#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <vector>
int main() {
    cv::Mat img;
    try{
        img = cv::imread("/home/teo/Documentos/nelso/src/bioloid_opencv/src/mark_w_s.png");
        const cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create(); 
        cv::Mat inputImage; img.copyTo(inputImage);
        std::vector<int> markerIds; 
        std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates; 
        const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250); 
        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates); 
        cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);
        cv::imshow("read", inputImage);
    } catch (cv::Exception& erros) {
        static const char* error = erros.what();
        ROS_ERROR(error);
        return (-1);
    }
    cv::imshow("img", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}