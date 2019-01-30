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
            void run();
        private:
            ros::NodeHandle _Node;
            ros::Subscriber _subOdom;
            ros::Subscriber _subGoal_pose;
            ros::Subscriber _subGoal_point;
            ros::Publisher _pubCmdVel;
            bool _odomRecived, _end;
            double _tolerance;
            double _poseTarget[2];
            double _pose[3]; // Pose with X, Y and Theta
            enum {X, Y, TH};
            PID* _angularVelocityPID;
            void subOdomCallback(const nav_msgs::Odometry::ConstPtr&);
            void subGoalCallback_point(const geometry_msgs::Point::ConstPtr&);
            void subGoalCallback_pose(const geometry_msgs::PoseStamped::ConstPtr&);
    };
}