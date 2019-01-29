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

#ifndef BIOLOID_MOTION_INCLUDE_BIOLOID_BIOLOID_WALK_HPP_
#define BIOLOID_MOTION_INCLUDE_BIOLOID_BIOLOID_WALK_HPP_

#include <bioloid/bioloid_utils.hpp>
#include <bioloid/bioloid_robot.hpp>

class WalkJointFunction {
    public:
        WalkJointFunction();
        WalkJointFunction(const WalkJointFunction& function);
        ~WalkJointFunction();

        double get_y(double x);
        void mirror(void);
        void clone(const WalkJointFunction* function);
        void print(void);

        void setOffset(double new_offset);
        double getOffset(void);
        void setInOffset(double new_in_offset);
        void setScale(double new_scale);
        double getScale(void);
        void setInScale(double new_in_scale);
        double getInScale(void);
        void setFoutBelow(double new_fout);
        WalkJointFunction& operator= (const WalkJointFunction& clone);
    private:
        double offset;
        double scale;
        double in_offset;
        double in_scale;

        double fout_above;
        double fout_below;
};

class WalkFunction {
    public:
        WalkFunction();
        WalkFunction(double swing_scale, double step_scale, double step_offset,
        		double ankle_offset, double vx_scale, double vy_scale, double vt_scale);
        ~WalkFunction();

        void generate(void);
        joints_t getAngles(bool phase, double x, std::vector<double> velocity);
        void applyVelocity(joints_t &angles, std::vector<double> velocity, bool phase, double x);

    private:
        void generateRight(void);
        void print(void);

        double swing_scale;
        double step_scale;
        double step_offset;
        double ankle_offset;
        double vx_scale;
        double vy_scale;
        double vt_scale;

        // Phase
        std::map<std::string, WalkJointFunction> pfn;
        // Anti-Phase
        std::map<std::string, WalkJointFunction> afn;
};

class Walker {
    public:
	    Walker(Bioloid *Robot);
	    ~Walker();
	    void update(void);

    private:
	    ros::NodeHandle _Node;
	    ros::NodeHandle _PrivNode;
	    Bioloid *_Robot;
	    bool _running;
	    std::vector<double> _velocityTarget;
	    std::vector<double> _velocityCurrent;
	    WalkFunction _Function;
	    joints_t _readyPos;
	    ros::Subscriber _subCmdVel;
	    void cmdVelCallback(const geometry_msgs::TwistConstPtr msg);
	    void walkStop(void);
	    void walkInit(void);
	    void walkStart(void);
	    void setVelocity(double x, double y, double t);
	    bool getWalking(void);
	    void velocityUpdate(std::vector<double> target, double n);
	    double getDistanceToReady(void);

	    // Name of this class
	    std::string _name;

	    // Frequency
	    int _loop_hz = 0;
};

#endif // BIOLOID_WALK_H_
