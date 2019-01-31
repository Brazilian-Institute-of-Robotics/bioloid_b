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
            void goTo(double, double);
            enum GOAL {POSE, POINT};
            void setGoal(GOAL, std::string);
            void setPIDConst(bool, double, double, double);
            void setLimits(bool, double);
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
            void subOdomCallback(const nav_msgs::Odometry::ConstPtr&);
            void subGoalCallback_point(const geometry_msgs::Point::ConstPtr&);
            void subGoalCallback_pose(const geometry_msgs::PoseStamped::ConstPtr&);
    };
}