#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include "accel.hpp"

bir::Acceleration::Acceleration(ros::NodeHandle& Node, std::string p_namespace): _Node(Node), _namespace(p_namespace) {
    // Init Variables
    _linearAccel[CURRENT] = _linearAccel[LAST] = 0;
    _Time[CURRENT] = ros::Time::now();
    _Time[LAST] = ros::Time::now();
    _enable = false;
    // Init Subscriber
    _AccelSub = _Node.subscribe((_namespace + "imu"), 200, &Acceleration::callback, this);
    // Init Publisher
    _AccelPub = _Node.advertise<geometry_msgs::TwistStamped>((_namespace + "velocity"), 100);
    // Setup Odometry Topic
    _velMsg.header.frame_id        = (_namespace + "base_link");
    _velMsg.twist.linear.y = _velMsg.twist.linear.z = _velMsg.twist.angular.x = 
                                            _velMsg.twist.angular.y = _velMsg.twist.angular.z = 0;
    ROS_INFO("Converting IMU data to Velocity"); 
}

void bir::Acceleration::callback(const sensor_msgs::Imu::ConstPtr& msg){
    if(_enable){
        // Get data to compute
        _Time[CURRENT] = ros::Time::now();
        _linearAccel[CURRENT] = msg->linear_acceleration.x;
        // Compute
        this->compute();
        // Update and Publish the Results
        this->publish();
    } else {
        _linearAccel[LAST] = msg->linear_acceleration.x;        
        _Time[LAST] = ros::Time::now();
        _enable = true;
    }
}

void bir::Acceleration::compute(){
    double deltaVelocity;
    double deltaTime;
    double acceleration;
    // Compute Linear and Angular Velocity
    // Trapezoidal Rule for integration (x(t) + x(t-dt))*dt/2
    acceleration = ( (_linearAccel[CURRENT] + _linearAccel[LAST]) / 2 );
    // Compute Delta Time and Position Variation
    deltaTime = (_Time[CURRENT] - _Time[LAST]).toSec(); // Variation of Time in seconds
    deltaVelocity = acceleration * deltaTime;
    // Add Position Variation into Position
    _linearVelocity += deltaVelocity;
    // Reset Variables to next call
    _linearAccel[LAST] = _linearAccel[CURRENT];
    _Time[LAST] = _Time[CURRENT];
}

void bir::Acceleration::publish(){
    // Update Velocity Msg
    _velMsg.header.stamp = _Time[CURRENT];
    _velMsg.twist.linear.x = _linearVelocity;
    // Publish Velocity
    _AccelPub.publish(_velMsg);
}
