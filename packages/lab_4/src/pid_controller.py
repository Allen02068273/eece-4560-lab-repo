#!/usr/bin/env python3
# pid_controller.py by Nolan Allen, 7 December 2023

import sys
import rospy
from std_msgs.msg import Float32


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp # proportional
        self.ki = ki # integral
        self.kd = kd # derivative
        self.sum = 0
        self.initialize = True
        self.timestamp = 0
        self.pre_error = 0
    
    def get_accel(self, error):
        if (self.initialize):
            self.timestamp = rospy.get_time()
            self.pre_error = error
            self.initialize = False
        
        delta_error = error - self.pre_error
        delta_time = rospy.get_time() - self.timestamp
        self.sum += delta_error * delta_time
        p = self.kp * error
        i = self.ki * self.sum
        d = 0
        if (delta_time != 0):
            d = self.kd * (delta_error / delta_time)
        
        self.timestamp = rospy.get_time()
        self.pre_error = error
        
        return p + i + d


class ProcessMsgs:
    def __init__(self):
        rospy.Subscriber("error", Float32, self.set_accel)
        self.pub_accel = rospy.Publisher("control_input", Float32, queue_size=1)
        rospy.set_param('controller_ready', 'ready')
        self.controller = PIDController(3,1,4)
    
    def set_accel(self, msg):
        self.pub_accel.publish(self.controller.get_accel(msg.data))


if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("pid_controller", anonymous=True)
    node_code = ProcessMsgs()
    rospy.spin()

