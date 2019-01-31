#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

outroGoalPublish = rospy.Publisher("/slender/goal_point", Point, queue_size=10)
_poseTarget = [0, 0]

def callback(msg):
    global _poseTarget
    _poseTarget[0] = msg.pose.pose.position.x
    _poseTarget[1] = msg.pose.pose.position.y

if __name__ == "__main__":
    rospy.init_node("get_the_nelson")
    nelsoPoseGetSubscriber = rospy.Subscriber("/nelso/position", Odometry, callback)
    rospy.logwarn("Program Init")
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        point = Point()
        point.x = _poseTarget[0]
        point.y = _poseTarget[1]
        outroGoalPublish.publish(point)
        rate.sleep()
        

