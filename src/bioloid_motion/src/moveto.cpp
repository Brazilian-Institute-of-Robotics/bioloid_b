#include <ros/ros.h>
#include <moveto/moveto.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "moveTo");
    ros::NodeHandle Node;
    bir::MoveTo PostionControl(Node, "/typea/position", "/typea/cmd_vel", 0.05);
    ros::Timer timer = Node.createTimer(ros::Duration(1/20), boost::bind(&bir::MoveTo::run, &PostionControl));

    ros::spin();

}