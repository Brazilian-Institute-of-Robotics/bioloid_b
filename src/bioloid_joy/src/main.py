#!/usr/bin/env python
import rospy
from libjoy import JoyController
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("joy_controller_py")
    pubCmdVel = rospy.Publisher("/typea/cmd_vel", Twist, queue_size=10)
    Joy = JoyController()
    Rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        msg = Twist()
        if(Joy.get()['A']):
            msg.linear.x = 0.5
        
        msg.angular.z = 1.2*(Joy.get()['horizontal_L_stick'])
        pubCmdVel.publish(msg)
        Rate.sleep()