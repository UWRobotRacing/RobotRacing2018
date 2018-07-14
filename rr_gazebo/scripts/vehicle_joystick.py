#!/usr/bin/env python

## vehicle_joystick.py

#  Control the wheels of the University of Waterloo Robot Racing Vehicle

#  Subscribed Topics:
#      joy (sensor_msgs/Joy)
#          It contains the joystick input

#  Published Topics:
#      steering_cmd (geometry_msgs/Twist)
#          It contains the vehicle's desired velocity.

import math
import numpy
import threading

from math import *

import rospy

from controller_manager_msgs.srv import ListControllers

import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS3Controller:
    def __init__(self):
        "PS3 controller node constructor"
        # tele-operation controls
        self._steer = 0                  # steering axis
        self._drive = 5                  # shift to Drive
        self._reverse = 2                # shift to Reverse
        self._steering_angle = 0.0
        self._wheel_velocity = 0.0
        self._wheel_to_wheel_dist = rospy.get_param("simulated_odometry/wheel_to_wheel_dist")
        self._forward_change = False
        self._reverse_change = False
        self._disable = False
        self._run_suspend_button_state = 0
        self._max_velocity = 4 #m/s to make the car easier to control in simulation
        self._max_steering_angle = 0.524 #rad

        self._joy_cmd_publisher = rospy.Publisher('joy_cmd', Twist,queue_size = 1)
        self._joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        self._joy_cmd = Twist()
        self._joy_cmd_publisher.publish(self._joy_cmd)

    def joy_callback(self, joy):
        "invoked every time a joystick message arrives" 
        if not(self._disable):
            forward = 1.0
            reverse = 1.0
            if not(self._forward_change):
                if joy.axes[self._drive] != 0.0:
                    self._forward_change = True
            else:
                forward = joy.axes[self._drive]

            if not(self._reverse_change):
                if joy.axes[self._reverse] != 0.0:
                    self._reverse_change = True
            else:
                reverse = joy.axes[self._reverse]

            self._steering_angle = joy.axes[self._steer] * self._max_steering_angle
            self._wheel_velocity = self._max_velocity * (-1 * forward + reverse)
        elif self._disable: 
            self._wheel_velocity = 0.0
            self._steering_angle = 0.0

    def publish_command(self, event):
        #TODO(oluwatoni) find the COG and adjust for that?
        lf = self._wheel_to_wheel_dist / 2.0
        lr = lf
        beta = atan2(lr * tan(self._steering_angle), lf + lr)

        #since we don't have access to the current orientation of the car use 45 degrees as a placeholder
        #TODO(oluwatoni) change this?
        self._joy_cmd.linear.x = self._wheel_velocity * cos(0.7071 + beta)
        self._joy_cmd.linear.y = self._wheel_velocity * sin(0.7071 + beta)
        self._joy_cmd.angular.z = (self._wheel_velocity / lr) * sin(beta)

        self._joy_cmd_publisher.publish(self._joy_cmd)

def main():
    rospy.init_node('vehicle_joystick.py', anonymous = True)
    controller = PS3Controller()
    rospy.loginfo('joystick vehicle controller starting')
    rospy.Timer(rospy.Duration(0.033), controller.publish_command)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()