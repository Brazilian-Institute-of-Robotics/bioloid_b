#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>

class JoyController {
   
    public:
        explicit JoyController(ros::NodeHandle& );
        explicit JoyController(ros::NodeHandle& , std::string p_namespace);
        explicit JoyController(ros::NodeHandle& , std::string p_namespace, std::string p_cmdvel);
    
        enum COMMAND_TYPE {
            CMD_VEL
        };

        /*  
            Function: setJoyTopicName
            Type: void
            Params: std::string
            Return: Nothing
            Duty: Set the variable _joyTopicName, Variable whose indicate the topic of JoyController.
        */
        void setJoyTopicName(std::string);
    private:
        ros::NodeHandle _Node;
        ros::Subscriber _SubJoy;
        ros::Publisher  _PubCmdVel;
        std::string     _namespace;
        std::string     _cmdVelTopicName; //? Without namespace
        std::string     _joyTopicName;
        bool cmdVelCommand();
        bool _commandsFunction[];
        bool _sendPublish;

        typedef struct {
            bool A;    bool B;     bool X;     bool Y;
            bool LB;   bool RB;    bool back;  bool start;
            bool power; bool L3;   bool R3;
        } _Buttons;
        
        enum BUTTONS {
            A,
            B,
            X,
            Y,
            LB,
            RB,
            BACK,
            START,
            POWER,
            L3
        };

        typedef struct {
            double horizontal_L_stick;
            double vertical_L_stick;
            double LT;
            double horizontal_R_stick;
            double vertical_R_stick;
            double RT;
            double horizontal_crosskey;
            double vertical_crosskey;
        } _Axes;

        enum AXES {
            HORIZONTAL_L_STICK,
            VERTICAL_L_STICK,
            LT,
            HORIZONTAL_R_STICK,
            VERTICAL_R_STICK,
            RT,
            HORIZONTAL_CROSSKEY,
            VERTICAL_CROSSKEY            
        };

        union {
            _Buttons button;
            _Axes axes;
        } _Joy;
        /*  
            Function: subJoyCallback
            Type: void
            Params: const sensor_msgs::JoyConstPtr&
            Return: Nothing
            Duty: Recive data from Joy Topic and allow JoyController send msgs
        */
        void subJoyCallback(const sensor_msgs::JoyConstPtr&);
        /*  
            Function: initJoyController
            Type: void
            Params: void
            Return: Nothing
            Duty: Initialize all Joy default configs, Subscribers, Publishers ...
        */
        void initJoyController();
        /*  
            Function: sendCommands
            Type: bool
            Params: COMMAND_TYPE
            Return: (bool) With commands were send or if hadn't any commands to sended
            Duty: Transform _Joy status into Commands
        */
        bool sendCommands(COMMAND_TYPE);

};