#include <ros/ros.h>
#include <odom/accel.hpp>
#include <odom/odom.hpp>
int main(int argc, char* argv[]){
    ros::init(argc, argv, "acceleration_test");
    ros::NodeHandle node;
    bir::Acceleration accel(node, "/typea/");
    bir::Odom odom("/typea/", node);
    
    ros::spin();
}