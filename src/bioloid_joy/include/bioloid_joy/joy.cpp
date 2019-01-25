#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "joy.hpp"

JoyController::JoyController(ros::NodeHandle& node)
    :   _Node(node), 
        _namespace("/"), 
        _cmdVelTopicName("cmd_vel") 
{
    this->initJoyController();
}

JoyController::JoyController(ros::NodeHandle& node, std::string p_namespace)
    :   _Node(node), 
        _namespace(p_namespace), 
        _cmdVelTopicName("cmd_vel") 
{
    this->initJoyController();
}

JoyController::JoyController(ros::NodeHandle& node, std::string p_namespace, std::string p_cmdvel)
    :   _Node(node), 
        _namespace(p_namespace), 
        _cmdVelTopicName(p_cmdvel) 
{
    this->initJoyController();
}

inline void JoyController::initJoyController(){
    _joyTopicName = "/joy";
    _sendPublish = false;
    //? Init Subscriber
    _SubJoy = _Node.subscribe(_joyTopicName, 10, &JoyController::subJoyCallback, this);
    //? Init Publisher
    _PubCmdVel = _Node.advertise<geometry_msgs::Twist>(_cmdVelTopicName, 10);
}

void JoyController::setJoyTopicName(std::string new_name){
    if(!new_name.empty()) _joyTopicName = new_name;
    _SubJoy = _Node.subscribe(_joyTopicName, 10, &JoyController::subJoyCallback, this);
}

void JoyController::subJoyCallback(const sensor_msgs::JoyConstPtr& msg){
    _Joy.button.A   = msg->buttons[BUTTONS::A];
    _Joy.button.B   = msg->buttons[BUTTONS::B];
    _Joy.button.X   = msg->buttons[BUTTONS::X];
    _Joy.button.Y   = msg->buttons[BUTTONS::Y];
    _Joy.button.LB  = msg->buttons[BUTTONS::LB];
    _Joy.button.RB  = msg->buttons[BUTTONS::RB];
    _Joy.button.back = msg->buttons[BUTTONS::BACK];
    _Joy.button.start = msg->buttons[BUTTONS::START];
    _Joy.button.power = msg->buttons[BUTTONS::POWER];
    _Joy.button.L3 = msg->buttons[BUTTONS::L3];

    _Joy.axes.horizontal_crosskey = msg->axes[AXES::HORIZONTAL_CROSSKEY];
    _Joy.axes.horizontal_L_stick = msg->axes[AXES::HORIZONTAL_L_STICK];
    _Joy.axes.vertical_crosskey = msg->axes[AXES::VERTICAL_CROSSKEY];
    _Joy.axes.LT = msg->axes[AXES::LT];
    _Joy.axes.horizontal_R_stick = msg->axes[AXES::HORIZONTAL_R_STICK];
    _Joy.axes.vertical_R_stick = msg->axes[AXES::VERTICAL_R_STICK];
    _Joy.axes.RT = msg->axes[AXES::RT];
    _Joy.axes.vertical_L_stick = msg->axes[AXES::VERTICAL_L_STICK];

    _sendPublish = true;    
}