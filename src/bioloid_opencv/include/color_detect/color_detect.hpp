#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace bir {
    class ColorDetect {
        public:
            explicit ColorDetect(cv::Mat&);
            explicit ColorDetect(cv::Mat&, std::vector<u_int8_t> min_limit, std::vector<u_int8_t> max_limit);
            virtual ~ColorDetect();

            bool detected();
            void setRange(int, std::vector<u_int8_t>);
            enum {MIN, MAX};
            cv::Mat& operator=(cv::Mat&);
            void operator()();
            bool operator()(cv::Mat&);
            bool operator()(cv::Mat&, std::vector<u_int8_t> min_limit, std::vector<u_int8_t> max_limit);
        private:
            float _areaLimit;
            float _numberOfObjects;
            cv::Mat _image;
            cv::Scalar _minRange, _maxRange;
            cv::Mat foundRange();
            std::vector<std::vector<cv::Point>> findContours();
            void findDetails();
            void clean();
            bool _detected;

    };
}