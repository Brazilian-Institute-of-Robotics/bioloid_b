#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <pid/pid.hpp>

namespace bir{
    class MoveTo {
        public:
            MoveTo(ros::NodeHandle&, std::string p_odom_topic, std::string p_cmd_vel_topic, double p_tolerance);
            /*  
                Function: goTo
                Type: void
                Params: Position Desired (X and Y)
                Return: Nothing
                Duty: Update _poseTarget
            */
            void goTo(double, double);          
            enum GOAL {POSE, POINT};
            /*  
                Function: setGoalTopicName
                Type: void
                Params: "GOAL" to choose with Point or Pose and "string" to the new topic name.
                Return: Nothing
                Duty: Update _goal{GOAL}Topic
            */            
            void setGoalTopicName(GOAL, std::string);
            /*  
                Function: setPIDConst
                Type: void
                Params: "bool" to choose with Angular or Linear PID, Kp, Ki, Kd, Interval
                Return: Nothing
                Duty: Set Angular or Linear PID's constants
            */            
            void setPIDConst(bool, double, double, double);
            /*  
                Function: setLimits
                Type: void
                Params: "bool" to choose with Angular or Linear PID, maximum value //!TODO Add minimum value.
                Return: Nothing
                Duty: Set Angular or Linear PID's maximum value
            */               
            void setLimits(bool, double);
            /*  
                Function: run
                Type: bool
                Params: Nothing
                Return: True if _poseTarget has been reached
                Duty: Run MoveTo Algorithm
                Required: Constant spin() to update _pose Variables.
                Advice: Use ros::Timer to call run. (boost::bind needed)
                Sample: ros::Timer timer = Node.createTimer(ros::Duration(1/{Frequency}), boost::bind(&bir::MoveTo::run, &{Class Instance}));
            */               
            bool run();
            enum {ANGULAR, LINEAR};
        private:
            ros::NodeHandle _Node;
            ros::Subscriber _subOdom;
            ros::Subscriber _subGoal_pose;
            ros::Subscriber _subGoal_point;
            ros::Publisher _pubCmdVel;
            std::string _goalPoseTopic;
            std::string _goalPointTopic;
            bool _odomRecived, _end;
            double _tolerance;
            double _poseTarget[2];
            double _pose[3]; // Pose with X, Y and Theta
            double _kP[2], _kI[2], _kD[2];
            enum {X, Y, TH};
            PID* _angularVelocityPID;
            PID* _linearVelocityPID;
            double _maxAngularVelocity = 1.5;
            double _maxLinearVelocity = 1.1;
            const float _pi = 3.141592;
            void subOdomCallback(const nav_msgs::Odometry::ConstPtr&);
            void subGoalCallback_point(const geometry_msgs::Point::ConstPtr&);
            void subGoalCallback_pose(const geometry_msgs::PoseStamped::ConstPtr&);
    };
}