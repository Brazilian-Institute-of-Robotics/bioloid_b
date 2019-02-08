#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <aruco_identification/aruco_identification.hpp>
#include <vector>

int main() {
    cv::Mat img;
    bir::Aruco tagRead_5X5(cv::aruco::DICT_4X4_250);
    bir::Aruco::marks mark;
    
    try{
        img = cv::imread("/home/teo/.gazebo/models/marker0/materials/textures/teste.png");
        cv::imshow("tag", img);
        cv::waitKey(0);
        mark = tagRead_5X5(img);
    } catch (cv::Exception& erros) {
        static const char* error = erros.what();
        ROS_ERROR(error);
        return (-1);
    }

    if(mark.size > 0){
        for (int index = 0; index < mark.size ; index++)
            std::cout << "ID Found: " << mark.id[index] << "\n";
    } else {
        std::cout << "ID Not Found\n";
    }

    cv::destroyAllWindows();

}