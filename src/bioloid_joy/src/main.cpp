#include <ros/ros.h>
#include <libjoy/libjoy.hpp>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_interface");
    ros::NodeHandle node;
    ros::Publisher PubCmdVel = node.advertise<geometry_msgs::Twist>("cmd_vel", 2);
    bir::JoyController Controller(node,"");
    ros::ServiceClient unpauseClient = node.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    ros::Rate rate(50);
    ROS_INFO("Joy Node Started");
    bool paused = true;
    while(ros::ok()){
        if(Controller.get().button.start && paused) {
            paused = false;
            std_srvs::Empty msg;
            unpauseClient.call(msg);
        }
        geometry_msgs::Twist msg;
        float linear = 0.00, angular = 0.00;
        if(Controller.get().button.A) {
            linear = 0.5;
            if(Controller.get().axes.RT <= 0.00){
                linear += -(Controller.get().axes.RT)*1.5;
            }
        }
        msg.linear.x = linear;
        msg.angular.z = Controller.get().axes.horizontal_L_stick;
        PubCmdVel.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}