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

    bool sending = false;
    while(ros::ok()) {
        
        if(Controller.get().button.A || (Controller.get().axes.horizontal_L_stick > 0.1) || (Controller.get().axes.horizontal_L_stick < -0.1)) {
            float linear = 0.00, angular = 0.00;
            geometry_msgs::Twist msg;
            if(Controller.get().button.A) {
                linear = 0.5;
            }
            if(Controller.get().axes.RT <= 0.00) {
                linear += -(Controller.get().axes.RT)*1.5;
            }
            msg.linear.x = linear;
            msg.angular.z = Controller.get().axes.horizontal_L_stick;
            PubCmdVel.publish(msg);
            sending = true;

        } else if(sending) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = 0;
            PubCmdVel.publish(msg);
            sending = false;
        }

        ros::spinOnce();
        rate.sleep();
    }
}