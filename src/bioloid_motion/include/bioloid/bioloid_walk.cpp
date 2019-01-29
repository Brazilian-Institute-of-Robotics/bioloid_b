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

#include <cmath>
#include <iostream>
#include <bioloid/bioloid_walk.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

WalkJointFunction::WalkJointFunction():offset(0), scale(1), in_offset(0), in_scale(1), 
                                        fout_above(0), fout_below(0) 
{

}

WalkJointFunction::WalkJointFunction(const WalkJointFunction& function){
	offset = function.offset;
	scale = function.scale;
	in_offset = function.in_offset;
	in_scale = function.in_scale;
	fout_below = function.fout_below;
	fout_above = function.fout_above;
}

WalkJointFunction::~WalkJointFunction()
{

}

double WalkJointFunction::get_y(double x)
{
	double f = sin(in_offset + in_scale * x);
	f = offset + scale * f;
	if (fout_above)
		if (f > fout_above)
			f = fout_above;
	if (fout_below)
		if (f < fout_below)
			f = fout_below;

	return f;
}

void WalkJointFunction::clone(const WalkJointFunction* function)
{
	offset = function->offset;
	scale = function->scale;
	in_offset = function->in_offset;
	in_scale = function->in_scale;
	fout_below = function->fout_below;
	fout_above = function->fout_above;
}

void WalkJointFunction::mirror(void)
{
	offset *= -1;
	scale *= -1;
	double tmp = fout_below * -1;
	fout_below = fout_above * -1;
	fout_above = tmp;
}

void WalkJointFunction::print(void)
{
	ROS_INFO_STREAM(
	"y=" << offset << "+" << scale << "* sin(" << in_offset << "+" << in_scale << ")");
}

void WalkJointFunction::setOffset(double new_offset)
{
	offset = new_offset;
}

double WalkJointFunction::getOffset(void)
{
	return offset;
}

void WalkJointFunction::setInOffset(double new_in_offset)
{
	in_offset = new_in_offset;
}

void WalkJointFunction::setScale(double new_scale)
{
	scale = new_scale;
}

double WalkJointFunction::getScale(void)
{
	return scale;
}

void WalkJointFunction::setInScale(double new_in_scale)
{
	in_scale = new_in_scale;
}

double WalkJointFunction::getInScale(void)
{
	return in_scale;
}

void WalkJointFunction::setFoutBelow(double new_fout)
{
	fout_below = new_fout;
}

WalkJointFunction& WalkJointFunction::operator= (const WalkJointFunction& clone)
{
	scale = clone.scale;
	in_scale = clone.in_scale;
	offset = clone.offset;
	in_offset = clone.in_offset;
	fout_below = clone.fout_below;
	fout_above = clone.fout_above;

	return *this;
}


WalkFunction::WalkFunction():
	swing_scale(0.4),
	step_scale(0.3),
	step_offset(0.55),
	ankle_offset(0),
	vx_scale(0.5),
	vy_scale(0.5),
	vt_scale(0.4)
{
	generate();
}

WalkFunction::WalkFunction(
		double swing_scale, double step_scale, double step_offset,
		double ankle_offset, double vx_scale, double vy_scale, double vt_scale):
	swing_scale(swing_scale),
	step_scale(step_scale),
	step_offset(step_offset),
	ankle_offset(ankle_offset),
	vx_scale(vx_scale),
	vy_scale(vy_scale),
	vt_scale(vt_scale)
{
	generate();
}

WalkFunction::~WalkFunction()
{

}

void WalkFunction::generate(void)
{
	WalkJointFunction f1;
	WalkJointFunction f2;
	WalkJointFunction f3;
	WalkJointFunction f33;
	WalkJointFunction f4;
	WalkJointFunction f5;
	WalkJointFunction f6;
	WalkJointFunction f7;
	// ARMS
	WalkJointFunction f8;
	WalkJointFunction f9;
	WalkJointFunction f10;


	// f1 = thigh1=ankle1=L=R in phase
	f1.setInScale(pi());
	f1.setScale(-1 * swing_scale);
	f1.setFoutBelow(f1.getScale()/2);
	pfn.insert(std::pair<std::string, WalkJointFunction>("l_ankle_lateral_joint", f1));
	pfn.insert(std::pair<std::string, WalkJointFunction>("l_hip_lateral_joint", f1));

	ROS_INFO_STREAM("f1.scale: " << f1.getScale());
	ROS_INFO_STREAM("pfn[l_ankle_lateal_joint].scale: " << pfn["l_ankle_lateral_joint"].getScale());

	// f2 = mirror f1 in antiphase
	f2.clone(&f1);
	f2.mirror();
	afn.insert(std::pair<std::string,WalkJointFunction>("l_ankle_lateral_joint", f2));
	afn.insert(std::pair<std::string,WalkJointFunction>("l_hip_lateral_joint", f2));

	ROS_INFO_STREAM("pfn[l_ankle_lateral_joint].scale: " << pfn["l_ankle_lateral_joint"].getScale());
	ROS_INFO_STREAM("afn[l_ankle_lateral_joint].scale: " << afn["l_ankle_lateral_joint"].getScale());

	ROS_ASSERT(pfn["l_ankle_lateral_joint"].getScale() == -1*afn["l_ankle_lateral_joint"].getScale());

	// f3
	f3.setInScale(pi());
	f3.setScale(step_scale);
	f3.setOffset(step_offset);
	pfn.insert(std::pair<std::string,WalkJointFunction>("l_hip_swing_joint", f3));
	f33.clone(&f3);
	f33.mirror();
	f33.setOffset(f33.getOffset() + ankle_offset);
	pfn.insert(std::pair<std::string,WalkJointFunction>("l_ankle_swing_joint", f33));

	// f4
	f4.clone(&f3);
	f4.mirror();
	f4.setOffset(f4.getOffset() * 2);
	f4.setScale(f4.getScale() * 2);
	pfn.insert(std::pair<std::string,WalkJointFunction>("l_knee_joint", f4));

	// f5
	f5.clone(&f3);
	f5.setInScale(f5.getInScale() * 2);
	f5.setScale(0);
	afn.insert(std::pair<std::string,WalkJointFunction>("l_hip_swing_joint", f5));

	// f6
	f6.clone(&f3);
	f6.mirror();
	f6.setInScale(f6.getInScale() * 2);
	f6.setScale(f5.getScale());
	f6.setOffset(f6.getOffset() + ankle_offset);
	afn.insert(std::pair<std::string,WalkJointFunction>("l_ankle_swing_joint", f6));

	// f7
	f7.clone(&f4);
	f7.setScale(0);
	afn.insert(std::pair<std::string,WalkJointFunction>("l_knee_joint", f7));

	f8.setScale(0);
	f8.setOffset(0.8);
	pfn.insert(std::pair<std::string, WalkJointFunction>("l_shoulder_lateral_joint", f8));
	pfn.insert(std::pair<std::string, WalkJointFunction>("l_elbow_joint", f8));
	afn.insert(std::pair<std::string, WalkJointFunction>("l_shoulder_lateral_joint", f8));
	afn.insert(std::pair<std::string, WalkJointFunction>("l_elbow_joint", f8));

	f9.setInScale(pi());
	f9.setScale(0.8);
	f9.setOffset(1.5);
	pfn.insert(std::pair<std::string, WalkJointFunction>("l_shoulder_swing_joint", f9));
	f10.clone(&f9);
	f10.mirror();
	afn.insert(std::pair<std::string, WalkJointFunction>("l_shoulder_swing_joint", f10));

	generateRight();

	print();
}

// Obtain the joint angles for a given phase, position in cycle (x 0,1)) and velocity parameters
joints_t WalkFunction::getAngles(bool phase, double x, std::vector<double> velocity)
{

	ROS_ASSERT(pfn.size() == afn.size());

    joints_t angles;

    std::map<std::string, WalkJointFunction>::iterator itp;

    if (phase) {
    	for (itp = pfn.begin(); itp != pfn.end(); ++itp)
    		angles.insert(std::pair<std::string, double>(itp->first, itp->second.get_y(x)));
    } else {
    	for (itp = afn.begin(); itp != afn.end(); ++itp)
    		angles.insert(std::pair<std::string, double>(itp->first, itp->second.get_y(x)));
	}

    applyVelocity(angles, velocity, phase, x);

    return angles;
}

// Modify on the walk-on-spot joint angles to apply the velocity vector
void WalkFunction::applyVelocity(joints_t &angles,
		std::vector<double> velocity, bool phase, double x)
{
	ROS_ASSERT(velocity.size() == VELOCITY_VECTOR_SIZE);
	ROS_ASSERT(angles.size() == pfn.size());

    // VX
    double v = velocity[0] * vx_scale;
    double d = (x * 2 - 1) * v;
    if (phase) {
        angles["l_hip_swing_joint"] += d;
        angles["l_ankle_swing_joint"] += d;
        angles["r_hip_swing_joint"] += d;
        angles["r_ankle_swing_joint"] += d;
    } else {
        angles["l_hip_swing_joint"] -= d;
        angles["l_ankle_swing_joint"] -= d;
        angles["r_hip_swing_joint"] -= d;
        angles["r_ankle_swing_joint"] -= d;
    }

    //ROS_INFO_STREAM("vel: " << v << "delta: " << d);
    //ROS_INFO_STREAM("angles[l_ankle_swing_joint]" << angles["l_ankle_swing_joint"]);

    // VY
    v = velocity[1] * vy_scale;
    d = x * v;
    double d2 = (1 - x) * v;
    if (v >= 0) {
        if (phase) {
            angles["l_hip_lateral_joint"] -= d;
            angles["l_ankle_lateral_joint"] -= d;
            angles["r_hip_lateral_joint"] += d;
            angles["r_ankle_lateral_joint"] += d;
        } else {
            angles["l_hip_lateral_joint"] -= d2;
            angles["l_ankle_lateral_joint"] -= d2;
            angles["r_hip_lateral_joint"] += d2;
            angles["r_ankle_lateral_joint"] += d2;
        }
    } else {
        if (phase) {
            angles["l_hip_lateral_joint"] += d2;
            angles["l_ankle_lateral_joint"] += d2;
            angles["r_hip_lateral_joint"] -= d2;
            angles["r_ankle_lateral_joint"] -= d2;
        } else {
            angles["l_hip_lateral_joint"] += d;
            angles["l_ankle_lateral_joint"] += d;
            angles["r_hip_lateral_joint"] -= d;
            angles["r_ankle_lateral_joint"] -= d;
        }
    }

    // VT
    v = velocity[2] * vt_scale;
    d = x * v;
    d2 = (1 - x) * v;
    if (v >= 0) {
        if (phase) {
            angles["l_hip_twist_joint"] = -d;
            angles["r_hip_twist_joint"] = d;
        } else {
            angles["l_hip_twist_joint"] = -d2;
            angles["r_hip_twist_joint"] = d2;
        }
    } else {
        if (phase) {
            angles["l_hip_twist_joint"] = d2;
            angles["r_hip_twist_joint"] = -d2;
        } else {
            angles["l_hip_twist_joint"] = d;
            angles["r_hip_twist_joint"] = -d;
        }
    }
}

// Mirror CPG functions from left to right and antiphase right
void WalkFunction::generateRight(void)
{
	std::map<std::string, WalkJointFunction>::iterator itp;

	for (itp=pfn.begin(); itp!=pfn.end(); ++itp) {
		std::string j(itp->first);
		ROS_INFO_STREAM("generate right: " << j);
		j.erase(0, 2);
		pfn["r_"+j].clone(&afn["l_"+j]);
		if ((j != "shoulder_lateral_joint") &&
			(j != "elbow_joint"))
			pfn["r_"+j].mirror();
		afn["r_"+j].clone(&pfn["l_"+j]);
		if ((j != "shoulder_lateral_joint") &&
			(j != "elbow_joint"))
			afn["r_"+j].mirror();
	}
}

// Display CPG function
void WalkFunction::print(void)
{
	std::map<std::string, WalkJointFunction>::iterator itp;
	std::map<std::string, WalkJointFunction>::iterator ita = afn.begin();

	ROS_ASSERT(pfn.size() == afn.size());

	for (itp=pfn.begin(); itp!=pfn.end(); ++itp, ++ita ) {
		ROS_INFO_STREAM(itp->first << " p ");
		itp->second.print();
		ROS_INFO_STREAM(ita->first << " a ");
		ita->second.print();
	}
}

Walker::Walker(Bioloid *robot): _PrivNode("Walker"), _Robot(robot), _running(false), _name("Walker") {
	// Load rosparams
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(_name, _PrivNode, "loop_hz", _loop_hz);
	if (error) {
		ROS_WARN_STREAM_NAMED(_name, "Walker requires the following config in the yaml:");
		ROS_WARN_STREAM_NAMED(_name, "   loop_hz: <x> $  10 < x < 100");
	}

	rosparam_shortcuts::shutdownIfError(_name, error);

	_velocityTarget.resize(3,0.0);
	_velocityCurrent.resize(3,0.0);

	// initialize subscribers
	_subCmdVel  = _Node.subscribe(_Robot->getNamespace()+"cmd_vel", 1, &Walker::cmdVelCallback, this);
}

Walker::~Walker()
{

}

// If not there yet, go to initial walk position
void Walker::walkInit(void)
{
    joints_t::iterator it;

	_readyPos = _Function.getAngles(true, (double)0.0, _velocityCurrent);

    if (getDistanceToReady() > VELOCITY_WALKING) {
    	ROS_INFO_STREAM("Going to walk position...");
        //for (it = _readyPos.begin(); it != _readyPos.end(); ++it) {
        	//ROS_INFO_STREAM("_readyPos " << it->first << ": " << it->second);
        //}
        _Robot->setAnglesSlow(_readyPos, ANGLE_DELAY);
		ROS_INFO_STREAM("Done.");
    }
}

void Walker::walkStart(void)
{
	if (!_running) {
		ROS_INFO_STREAM("Starting...");
		// TODO: Too long for callback, move in thread.
		walkInit();
        _running = true;
	}
}

void Walker::walkStop(void)
{
    if (_running) {
        ROS_INFO_STREAM("Stopping...");
		_running = false;
    }
}

void Walker::setVelocity(double x, double y, double t)
{
	_velocityTarget = {x, y, t};

    if (x == 0 && y == 0 && t == 0)
    	walkStop();
    else
    	walkStart();
}

void Walker::cmdVelCallback(const geometry_msgs::TwistConstPtr msg)
{
	setVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

bool Walker::getWalking(void)
{
	ROS_ASSERT(_velocityCurrent.size() == VELOCITY_VECTOR_SIZE);

	for (unsigned int i = 0; i < VELOCITY_VECTOR_SIZE; i++) {
	    if (abs(_velocityCurrent[i]) > VELOCITY_WALKING)
	    	return true;
	}
	return false;
}

void Walker::velocityUpdate(std::vector<double> target, double n)
{
	joints_t::iterator it;
	double a = (double)3 / n;
	double b = (double)1 - a;

	ROS_ASSERT(target.size() == VELOCITY_VECTOR_SIZE);

	for (unsigned int i = 0; i < VELOCITY_VECTOR_SIZE; i++) {
		_velocityCurrent[i] = a * target[i] + b * _velocityCurrent[i];
	}
}

double Walker::getDistanceToReady(void)
{
	joints_t angles = _Robot->getAngles();

	return getDistance(_readyPos, angles);
}

// Main walking loop, smoothly update velocity vectors and apply corresponding angles
void Walker::update(void)
{

	// Global walk loop
	uint32_t n = 50;
	uint32_t p = true;
	uint32_t i = 0;
	double x = 0.0;
	joints_t angles;

	ros::Rate r(_loop_hz);

	ROS_INFO_STREAM_NAMED(_name, "Loop rate is set to: " << _loop_hz);

	_velocityCurrent = { 0.0, 0.0, 0.0};

	while (ros::ok()) {
		// Do not move if nothing to do and already at 0
		if (!_running && (i == 0)) {
			velocityUpdate(_velocityTarget, n);
		} else {
			// Process next 1/50th
			x = (double)i++ / n;
			//ROS_INFO_STREAM("|i=" << i - 1 << " |p=" << p << " |x=" << x << " |vc=" << _velocityCurrent[0] << " |vt=" << _velocityTarget[0]);
			angles = _Function.getAngles(p, x, _velocityCurrent);
			velocityUpdate(_velocityTarget, n);
			_Robot->setAngles(angles);

			// Next phase
			if (i > n) {
			    i = 0;
			    p = !p;
			}
		}

		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Walker");

	ROS_INFO_STREAM("Instantiating Bioloid Robot");
	Bioloid Robot;
	ros::Duration(1).sleep();

	Walker Walk(&Robot);
	ROS_INFO_STREAM("Bioloid Robot ready.");

	ROS_INFO_STREAM("Started walking thread");
	Walk.update();
	ROS_INFO_STREAM("Finished walking thread");

	return 0;
}
