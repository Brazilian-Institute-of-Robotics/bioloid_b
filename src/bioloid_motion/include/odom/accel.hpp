#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <string>

namespace bir{

    class Acceleration{
        private:
            ros::Subscriber _AccelSub; // Recieve the data from IMU
            ros::Publisher  _AccelPub; // Send the data as velocity msg
            ros::NodeHandle _Node;    // Node
            geometry_msgs::TwistStamped _velMsg;
            void publish();
            void compute();
            void callback(const sensor_msgs::Imu::ConstPtr& msg);
        public:
            //Acceleration(ros::NodeHandle&);
            Acceleration(ros::NodeHandle&, std::string p_namespace);
        private:
            std::string _namespace;
            double _linearAccel[2]; // Store current and last linear in x acceleration
            double _linearVelocity; // Store current linear velocity
            ros::Time _Time[2];
            bool _enable;
            enum {LINEAR, ANGULAR};
            enum {CURRENT, LAST};
    };
}