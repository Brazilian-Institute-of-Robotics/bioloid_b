#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace bir {
    class ColorDetect {

        public:
            explicit ColorDetect();
            explicit ColorDetect(std::vector<u_int8_t> min_limit, std::vector<u_int8_t> max_limit);
            virtual ~ColorDetect();

            bool detected();
            bool operator()(cv::Mat&);
            void setRange(int, std::vector<u_int8_t>);
            void setArea(float);
            cv::Mat drawContours(cv::Mat img);
            cv::Mat drawContours(cv::Mat img, std::string, cv::Scalar);
            u_int16_t numberOfObjects();
            
            enum {MIN, MAX};

        private:
            float _areaLimit;
            u_int16_t _numberOfObjects;
            cv::Mat _image;
            cv::Scalar _minRange, _maxRange;
            std::vector<std::vector<cv::Point>> _cnt;
            std::vector<cv::Vec4i> _hie;
            void foundRange();
            void findContours();
            void findDetails();
            void setImage(cv::Mat&);
            void clean();
            bool _detected;

    };
}