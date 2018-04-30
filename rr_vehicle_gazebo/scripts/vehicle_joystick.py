#!/usr/bin/env python

## vehicle_joystick.py

#  Control the wheels of the University of Waterloo Robot Racing Vehicle

#  Subscribed Topics:
#      joy (sensor_msgs/Joy)
#          It contains the joystick input

#  Published Topics:
#      velocity_cmd (std_msgs/Float32)
#          It contains the vehicle's desired speed angle.
#      steering_cmd (std_msgs/Float32)
#          It contains the vehicle's desired velocity.

import math
import numpy
import threading

from math import pi

import rospy
import tf

from std_msgs.msg import Float64, Float32
from controller_manager_msgs.srv import ListControllers

import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class PS3Controller:
    def __init__(self):
        "PS3 controller node constructor"
        # tele-operation controls
        self._steer = 0                  # steering axis
        self._drive = 13                  # shift to Drive
        self._reverse = 12                # shift to Reverse
        self._disable = False
        self._run_suspend_button_state = 0
        self._max_velocity = 8 #m/s
        self._max_steering_angle = 0.524 #rad
        self._steering_publisher = rospy.Publisher('rr_vehicle/steering_cmd', Float32,queue_size = 1)
        self._velocity_publisher = rospy.Publisher('rr_vehicle/velocity_cmd', Float32,queue_size = 1)
        self._joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        
    def joy_callback(self, joy):
        "invoked every time a joystick message arrives" 

        if not(self._disable):
            # create the throttle and steering command
            self._steering_publisher.publish(joy.axes[self._steer])
            self._velocity_publisher.publish(self._max_velocity *( -joy.axes[self._drive] + joy.axes[self._reverse]))
        elif self._disable: 
            self._steering_publisher.publish(0.0)
            self._velocity_publisher.publish(0.0)

def main():
    rospy.init_node('vehicle_joystick.py', anonymous = True)
    controller = PS3Controller()
    rospy.loginfo('joystick vehicle controller starting')

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()