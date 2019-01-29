/*******************************************************************************
* Copyright 2017 Mathieu Rondonneau
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
// Author: Mathieu Rondonneau
// Modified by: Etevaldo Cardoso

#ifndef BIOLOID_MOTION_INCLUDE_BIOLOID_BIOLOID_ROBOT_HPP_
#define BIOLOID_MOTION_INCLUDE_BIOLOID_BIOLOID_ROBOT_HPP_

#include <math.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <bioloid/bioloid_utils.hpp>

class Bioloid {

    public:
        Bioloid();
        ~Bioloid();

        void setWalkVelocity(double x, double y, double th);
        std::string getNamespace(void);
        void setAngles(joints_t angles);
        void setAnglesSlow(joints_t stop_angles, unsigned int delay);
        joints_t getAngles(void);
    
    private:
        // ROS Timers
        ros::Time _start_time;
        ros::Time _stop_time;

        bool init(void);

        // ROS NodeHandle
        ros::NodeHandle _Node;
        ros::NodeHandle _PrivNode;

        // ROS Topic Publishers
        ros::Publisher _pubCmdVel;
        std::map<std::string, ros::Publisher> _pubJointPositionCmdVel;

        // ROS Topic Subscribers
        ros::Subscriber _subJointStates;

        // ROS Messages
        sensor_msgs::JointState _joint_states_msg;
        std_msgs::Float64 _float64_msg;
        geometry_msgs::Twist _twist_msg;

        // Variables
        std::string _namespace;
        joints_t _joints;

        // Function prototypes
        void joint_callback(const sensor_msgs::JointStateConstPtr msg);
};      

#endif // BIOLOID_MOTION_INCLUDE_BIOLOID_BIOLOID_ROBOT_HPP_
