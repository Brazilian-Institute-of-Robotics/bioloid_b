# Bioloid Type A

## Folders
- src/
  - bioloid_config/
  - bioloid_joy/
  - bioloid_motion/
  - bioloid_typea_description/
  
 ### Bioloid Config
  - Main Launch Files
  - ROS Controllers Configuration Files
  - Launch for Multi Robots Spawn
  
 ### Bioloid Joy
  - Launch File
  - Library Using C++ and Python
  - Script to Use Joy
  
 ### Bioloid Motion
  - Library MoveTo ( Go to Goal Script )
  - Library PID ( Used in MoveTo )
  - Walker Functions to Bioloid convert Twist msg in Joints Control
  
 ### Bioloid Type Description
  - URDF and Xacro Files.
  
  
## How to Use
### How to launch a single robot.
#### Without Joy
> roslaunch bioloid_config bioloid_launch.launch
#### With Joy
> roslaunch bioloid_config bioloid_launch.launch joy:=true
>> by default, joy deviced is named (js0) , with change is needed, replance at (rosed bioloid_joy bioloid_joy.launch ) or (rosed bioloid_config bioloid_launch) and add device arg with the real device name.

> Based on https://github.com/MathieuR
