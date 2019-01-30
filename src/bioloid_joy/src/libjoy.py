#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyController:

    def __init__(self, namespace="/"):
        self._joyTopicName = "joy"
        self._namespace = namespace
        self._buttons = {'A' : 0, 'B' : 0, 'X' : 0, 'Y' : 0, 'LB' : 0, 'RB' : 0, 'back' : 0, 'start' : 0, 'power' : 0, 'L3' : 0, 'R3' : 0, 
        'horizontal_crosskey' : 0, 'horizontal_L_stick' : 0, 'vertical_crosskey' : 0, 'LT' : 0, 'horizontal_R_stick' : 0,  'vertical_R_stick' : 0,  'RT' : 0,  'vertical_L_stick' : 0 }
        self._subJoy = rospy.Subscriber(self._namespace + self._joyTopicName, Joy, self.subJoyCallback)
        self._dicButton = {
            "A" : 0 ,
            "B" : 1,
            "X" : 2,
            "Y" : 3,
            "LB" : 4,
            "RB" : 5,
            "back" : 6,
            "start" : 7,
            "power" : 8,
            "L3" : 9,
            "R3" : 10,
            "horizontal_L_stick" : 0,
            "vertical_L_stick" : 1,
            "LT" : 2,
            "horizontal_R_stick" : 3,
            "vertical_R_stick" : 4,
            "RT" : 5,
            "horizontal_crosskey" : 6,
            "vertical_crosskey" : 7
        }

    def setJoyTopicName(self, (topicName)):
        if not topicName:
            print(topicName + "is an invalid name.")
        else:
            self._joyTopicName = str(topicName)
            self._subJoy = rospy.Subscriber(self._namespace + self._joyTopicName, Joy, self.subJoyCallback)

    def get(self):
        return (self._buttons)

    def subJoyCallback(self, msg):
        # Send msg value to internal variable
        # Buttons
        self._buttons['A'] = msg.buttons[self._dicButton['A']]
        self._buttons['B'] = msg.buttons[self._dicButton['B']]
        self._buttons['X'] = msg.buttons[self._dicButton['X']]
        self._buttons['Y'] = msg.buttons[self._dicButton['Y']]
        self._buttons['LB'] = msg.buttons[self._dicButton['LB']]
        self._buttons['RB'] = msg.buttons[self._dicButton['RB']]
        self._buttons['back'] = msg.buttons[self._dicButton['back']]
        self._buttons['start'] = msg.buttons[self._dicButton['start']]
        self._buttons['power'] = msg.buttons[self._dicButton['power']]
        self._buttons['L3'] = msg.buttons[self._dicButton['L3']]
        self._buttons['R3'] = msg.buttons[self._dicButton['R3']]
        # Axes
        self._buttons['horizontal_crosskey'] = msg.axes[self._dicButton['horizontal_crosskey']]
        self._buttons['horizontal_L_stick'] = msg.axes[self._dicButton['horizontal_L_stick']]
        self._buttons['vertical_crosskey'] = msg.axes[self._dicButton['vertical_crosskey']]
        self._buttons['LT'] = msg.axes[self._dicButton['LT']]
        self._buttons['horizontal_R_stick'] = msg.axes[self._dicButton['horizontal_R_stick']]
        self._buttons['vertical_R_stick'] = msg.axes[self._dicButton['vertical_R_stick']]
        self._buttons['RT'] = msg.axes[self._dicButton['RT']]
        self._buttons['vertical_L_stick'] = msg.axes[self._dicButton['vertical_L_stick']]