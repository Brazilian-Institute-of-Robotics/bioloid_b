#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <pid/pid.hpp>
#define is_negative(x) ((x < 0) ? true : false)
#include "moveto.hpp"

bir::MoveTo::MoveTo(ros::NodeHandle& node, std::string p_odom_topic, std::string p_cmd_vel_topic, double tolerance): _Node(node), _tolerance(tolerance) {
    // Subscriber Setting
    _odomRecived = false;
    ROS_WARN("Waiting for Odometry Topic.");
    _subOdom = _Node.subscribe(p_odom_topic, 1, &MoveTo::subOdomCallback, this);
    // Goal Subscriber Setting
    _goalPoseTopic = "/move_base_simple/goal";
    _goalPointTopic = "goal_point";
    _subGoal_pose = _Node.subscribe(_goalPoseTopic, 1, &MoveTo::subGoalCallback_pose, this);
    _subGoal_point = _Node.subscribe(_goalPointTopic, 1, &MoveTo::subGoalCallback_point, this);
    // Publsiher Setting
    _pubCmdVel = _Node.advertise<geometry_msgs::Twist>(p_cmd_vel_topic, 1);
    // Class Config
    _kP[LINEAR] = 0.6;
    _kI[LINEAR] = 0;
    _kI[LINEAR] = 0;
    
    _kP[ANGULAR] = 1;
    _kI[ANGULAR] = 0.00;
    _kD[ANGULAR] = 0.0;
    _angularVelocityPID = new PID(_kP[ANGULAR], _kI[ANGULAR], _kD[ANGULAR], 10);
    _linearVelocityPID = new PID(_kP[LINEAR], _kI[LINEAR], _kD[LINEAR], 10);
    _linearVelocityPID->setSetpoint(0.00);
    _end = true;
}

void bir::MoveTo::subOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){ // Update _pose Variable
    if(!_odomRecived) {
        _odomRecived = true;
        _poseTarget[X] = msg->pose.pose.position.x;
        _poseTarget[Y] = msg->pose.pose.position.y;
        
        ROS_INFO("Odom Topic Connected");
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

void bir::MoveTo::setGoalTopicName(GOAL goal, std::string new_name){
    if(goal == POINT) {
        _goalPointTopic = new_name;
    } else if(goal == POSE) {
        _goalPoseTopic = new_name;
    }
}
void bir::MoveTo::subGoalCallback_pose(const geometry_msgs::PoseStamped::ConstPtr& pose){
    goTo(pose->pose.position.x, pose->pose.position.y);
    ROS_INFO("Goal Recived: (%f,%f)", _poseTarget[X], _poseTarget[Y]);
}

void bir::MoveTo::subGoalCallback_point(const geometry_msgs::Point::ConstPtr& pose){
    goTo(pose->x, pose->y);
    ROS_INFO("Goal Recived: (%f,%f)", _poseTarget[X], _poseTarget[Y]);
}

void bir::MoveTo::setPIDConst(bool PID, double kP, double kI, double kD){
    _kP[PID] = kP; _kI[PID] = kI; _kD[PID] = kD;
}

void bir::MoveTo::setLimits(bool velocityType, double value){
    if(velocityType == ANGULAR) _maxAngularVelocity = value;
    else _maxLinearVelocity = value;
}

inline void bir::MoveTo::goTo(double x, double y){
    _poseTarget[X] = x;
    _poseTarget[Y] = y;
    _end = false;
}

bool bir::MoveTo::getEnd(){
    return _end;
}

bool bir::MoveTo::run() {
    geometry_msgs::Twist velocityCommand;
    double distanceToTarget = sqrt(pow((_poseTarget[X] - _pose[X]),2) + pow((_poseTarget[Y] - _pose[Y]),2));
    if(distanceToTarget < _tolerance){
        velocityCommand.angular.z = velocityCommand.linear.x = 0;
        _end = true;
    } else if(_odomRecived) {
        double desiredAngle = atan2((_poseTarget[Y] - _pose[Y]), (_poseTarget[X] - _pose[X]));
        // Linear Velocity
        double linearVelocity = fabs(_linearVelocityPID->run(distanceToTarget));
        if( ((desiredAngle - _pose[TH]) >= _pi) || ((desiredAngle - _pose[TH]) <= -_pi) ){
            linearVelocity = 0;
        } else if(linearVelocity >= _maxLinearVelocity) {
            linearVelocity = _maxLinearVelocity;
        }     
        velocityCommand.linear.x = linearVelocity;
        // Angular Velocity
        _angularVelocityPID->setSetpoint(desiredAngle);
        velocityCommand.angular.z = _angularVelocityPID->run(_pose[TH], true);
        velocityCommand.linear.z = velocityCommand.angular.z;
        if(fabs(velocityCommand.angular.z) > _maxAngularVelocity) {
            if(is_negative(velocityCommand.angular.z)) velocityCommand.angular.z = -_maxAngularVelocity;
            else velocityCommand.angular.z = _maxAngularVelocity;
        }
    }
   
    _pubCmdVel.publish(velocityCommand);
    if (_end) return true;
    return false;
}