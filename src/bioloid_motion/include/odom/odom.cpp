#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "odom.hpp"
#include <string>
#include <iostream>

bir::Odom::Odom(std::string robotName, ros::NodeHandle Node): _robotName(robotName), _Node(Node){
    // Init Variables
    _velocity[LINEAR] = _velocity[ANGULAR] = 0;
    _position[X] = _position[Y] = _position[TH] = _lastAngle = 0;
    _Time[CURRENT] = ros::Time::now();
    _Time[LAST] = ros::Time::now();
    _enable = false;
    // Init Subscriber
    _OdomSub = _Node.subscribe((_robotName + "velocity"), 20, &Odom::callback, this);
    // Init Publisher
    _OdomPub = _Node.advertise<nav_msgs::Odometry>((_robotName + "odom"), 10);
    // Setup Transform
    _OdomTransform.header.frame_id  = ("odom");
    _OdomTransform.child_frame_id   = ("base_link");
    _OdomTransform.transform.translation.z = 0;
    // Setup Odometry Topic
    _OdomMsg.header.frame_id        = ("odom");
    _OdomMsg.child_frame_id         = ("base_link");
    _OdomMsg.pose.pose.position.z   = 0;
    _OdomMsg.twist.twist.linear.y = _OdomMsg.twist.twist.linear.z = _OdomMsg.twist.twist.angular.x = 
                                                                        _OdomMsg.twist.twist.angular.y = 0;
    ROS_INFO("Odometry Created"); 
}

void bir::Odom::callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    if(_enable){
        // Get data to compute
        _Time[CURRENT] = ros::Time::now();
        _velocity2Compute[CURRENT][LINEAR] = msg->twist.linear.x;
        _velocity2Compute[CURRENT][ANGULAR] = msg->twist.angular.z;
        // Compute
        this->compute();
        // Update and Publish the Results
        this->publish();
    } else {
        _velocity2Compute[LAST][LINEAR] = msg->twist.linear.x;
        _velocity2Compute[LAST][ANGULAR] = msg->twist.angular.z;
    
        _Time[LAST] = ros::Time::now();
        _enable = true;
    }
}

void bir::Odom::compute(){
    double deltaPosition[3];
    double deltaTime;
    // Compute Linear and Angular Velocity
    // Trapezoidal Rule for integration (x(t) + x(t-dt))*dt/2
    _velocity[X] = ((_velocity2Compute[CURRENT][LINEAR] * cos(_position[TH])) + (_velocity2Compute[LAST][LINEAR] * cos(_lastAngle)))/2;
    _velocity[Y] = ((_velocity2Compute[CURRENT][LINEAR] * sin(_position[TH])) + (_velocity2Compute[LAST][LINEAR] * sin(_lastAngle)))/2;
    _velocity[TH] = (_velocity2Compute[CURRENT][ANGULAR] + _velocity2Compute[LAST][ANGULAR])/2;
    // Compute Delta Time and Position Variation
    deltaTime = (_Time[CURRENT] - _Time[LAST]).toSec(); // Variation of Time in seconds
    deltaPosition[X] = _velocity[X] * deltaTime;
    deltaPosition[Y] = _velocity[Y] * deltaTime;
    deltaPosition[TH] = _velocity[TH] * deltaTime;
    _lastAngle = _position[TH];
    // Add Position Variation into Position
    _position[X] += deltaPosition[X];
    _position[Y] += deltaPosition[Y];
    _position[TH] += deltaPosition[TH];
    // Reset Variables to next call
    _velocity2Compute[LAST][LINEAR] = _velocity2Compute[CURRENT][LINEAR];
    _velocity2Compute[LAST][ANGULAR] = _velocity2Compute[CURRENT][ANGULAR];
    _Time[LAST] = _Time[CURRENT];
}

void bir::Odom::publish(){
    // Update Transform
    _OdomTransform.header.stamp = _Time[CURRENT];
    _OdomTransform.transform.translation.x = _position[X];
    _OdomTransform.transform.translation.y = _position[Y];
    _OdomTransform.transform.rotation = tf::createQuaternionMsgFromYaw(_position[TH]);
    // Update Odometry
    _OdomMsg.header.stamp = _Time[CURRENT];
    _OdomMsg.pose.pose.position.x = _position[X];
    _OdomMsg.pose.pose.position.y = _position[Y];
    _OdomMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,_position[TH]);
    _OdomMsg.twist.twist.linear.x = _velocity2Compute[LAST][LINEAR];
    _OdomMsg.twist.twist.angular.z = _velocity2Compute[LAST][ANGULAR];
    // Publish Odometry and TF  
    _Broadcaster.sendTransform(_OdomTransform);
    _OdomPub.publish(_OdomMsg);
}
