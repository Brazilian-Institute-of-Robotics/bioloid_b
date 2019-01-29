#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <pid/pid.hpp>

#include "moveto.hpp"

bir::MoveTo::MoveTo(ros::NodeHandle& node, std::string p_odom_topic, std::string p_cmd_vel_topic, double tolerance): _Node(node), _tolerance(tolerance) {
    // Subscriber Setting
    _odomRecived = false;
    _subOdom = _Node.subscribe(p_odom_topic, 20, &MoveTo::subOdomCallback, this);
    ros::Rate rate(2);
    while(ros::ok() && !_odomRecived){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Odom Topic Connected");
    // Goal Subscriber Setting
    _subGoal_pose = _Node.subscribe("/move_base_simple/goal", 1, &MoveTo::subGoalCallback_pose, this);
    _subGoal_point = _Node.subscribe("/typea/goal_point", 1, &MoveTo::subGoalCallback_point, this);
    // Publsiher Setting
    _pubCmdVel = _Node.advertise<geometry_msgs::Twist>(p_cmd_vel_topic, 20);
    // Class Config
    _angularVelocityPID = new PID(1, 0, 0, 10);
    _end = false;
}

void bir::MoveTo::subOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){ // Update _pose Variable
    if(!_odomRecived) {
        _odomRecived = true;
        _poseTarget[X] = msg->pose.pose.position.x;
        _poseTarget[Y] = msg->pose.pose.position.y;
    }
    
    _pose[X] = msg->pose.pose.position.x;
    _pose[Y] = msg->pose.pose.position.y;
    // Converting Quaternion to Euler (X,Y,Z,W) --> (ROW, PITCH, YAW)
    tf::Quaternion quartenion;
    quartenion.setX(msg->pose.pose.orientation.x);
    quartenion.setY(msg->pose.pose.orientation.y);
    quartenion.setZ(msg->pose.pose.orientation.z);
    quartenion.setW(msg->pose.pose.orientation.w);
    double row, pitch;
    tf::Matrix3x3(quartenion).getRPY(row, pitch, _pose[TH]);
}

void bir::MoveTo::subGoalCallback_pose(const geometry_msgs::PoseStamped::ConstPtr& pose){
    goTo(pose->pose.position.x, pose->pose.position.y);
    ROS_INFO("Goal Recived: (%f,%f)", _poseTarget[X], _poseTarget[Y]);
}

void bir::MoveTo::subGoalCallback_point(const geometry_msgs::Point::ConstPtr& pose){
    goTo(pose->x, pose->y);
    ROS_INFO("Goal Recived: (%f,%f)", _poseTarget[X], _poseTarget[Y]);
}

void bir::MoveTo::goTo(double x, double y){
    _poseTarget[X] = x;
    _poseTarget[Y] = y;
}

void bir::MoveTo::run(){
    geometry_msgs::Twist velocityCommand;
    velocityCommand.linear.x = 0.5;
    double distanceToTarget = sqrt(pow((_poseTarget[X] - _pose[X]),2) + pow((_poseTarget[Y] - _pose[Y]),2));
    if(distanceToTarget <= _tolerance) {
        _end = true;
        velocityCommand.angular.z = velocityCommand.linear.x = 0;
    } else{
        double desiredAngle = atan2((_poseTarget[Y] - _pose[Y]), (_poseTarget[X] - _pose[X]));
        _angularVelocityPID->setSetpoint(desiredAngle);
        velocityCommand.angular.z = _angularVelocityPID->run(_pose[TH]);
    }
    _pubCmdVel.publish(velocityCommand);
}