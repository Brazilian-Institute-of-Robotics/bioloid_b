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

#include <bioloid/bioloid_robot.hpp>

Bioloid::Bioloid():	_PrivNode("~"),	_Node("/typea/") {
  ROS_ASSERT( init() );
}

Bioloid::~Bioloid() {
}

std::string Bioloid::getNamespace(void) {
	return _namespace;
}

bool Bioloid::init(void) {
	ROS_ASSERT(_joints.empty());
	// initialize subscribers
	_subJointStates  = _Node.subscribe(_namespace + "joint_states", 1, &Bioloid::joint_callback, this);

	ROS_INFO_STREAM("+Waiting for joints to be populated...");

	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(1).sleep();
		if (_joints.size())
			break;
	}

	if (_joints.size() != NUMBER_OF_JOINTS) {
		ROS_INFO("%d motors connected.", (unsigned int)_joints.size());
		ROS_ERROR("Number of actuator required %d", NUMBER_OF_JOINTS);
		return false;
	}

	// initialize publishers

	ROS_INFO_STREAM("+Creating joint command publishers...");
	joints_t::iterator it;

	for (it = _joints.begin(); it != _joints.end(); ++it) {
        _pubJointPositionCmdVel.insert(std::pair<std::string, ros::Publisher>(it->first,_Node.advertise<std_msgs::Float64>(_namespace + it->first + "_position_controller/command", 10)));
        ROS_INFO_STREAM(" -joint: " << it->first);
	}

	_pubCmdVel = _Node.advertise<geometry_msgs::Twist>( _namespace + "cmd_vel", 10);

    return true;
}

void Bioloid::joint_callback(const sensor_msgs::JointStateConstPtr msg) {
	joints_t::iterator it;

	for (uint32_t i = 0; i < msg->name.size(); i++) {

		it = _joints.find(msg->name[i]);

		if (it == _joints.end()) {
			_joints.insert(std::pair<std::string,double>(msg->name[i], msg->position[i]));
		} else {
			it->second = msg->position[i];
		}
	}
}

void Bioloid::setWalkVelocity(double x, double y, double t) {
	_twist_msg.linear.x = x;
	_twist_msg.linear.y = y;
	_twist_msg.angular.z = t;
	_pubCmdVel.publish(_twist_msg);
}

joints_t Bioloid::getAngles(void) {
	ROS_ASSERT(_joints.size() == NUMBER_OF_JOINTS);
	return _joints;
}

void Bioloid::setAngles(joints_t angles) {
	joints_t::iterator it;

	for (it = angles.begin(); it != angles.end(); ++it) {
		_float64_msg.data = it->second;
		//ROS_INFO_STREAM("set_angle " << it->first << ": " << it->second);
		_pubJointPositionCmdVel[it->first].publish(_float64_msg);
	}
}

void Bioloid::setAnglesSlow(joints_t stop_angles, unsigned int delay) {
	ros::Time current_time;
	_start_time = ros::Time::now();
	_stop_time.sec = _start_time.sec + delay;
	joints_t new_joints;

	double ratio;

	while (ros::ok()) {
		current_time = ros::Time::now();
		if (current_time.sec > _stop_time.sec)
			break;

		ratio = (current_time.sec - _start_time.sec) / delay;
		new_joints = interpolate(stop_angles, _joints, ratio);
		setAngles(new_joints);

		ros::Rate(100).sleep();
	}
}

