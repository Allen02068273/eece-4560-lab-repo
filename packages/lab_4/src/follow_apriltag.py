#!/usr/bin/env python3
# follow_apriltag.py by Nolan Allen, 20 December 2023

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray


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
            self.pre_error = error
            self.initialize = False
        
        delta_error = error - self.pre_error
        delta_time = rospy.get_time() - self.timestamp
        self.sum += delta_error * delta_time
        p = self.kp * error
        i = self.ki * self.sum
        d = 0
        if (self.timestamp != 0):
            d = self.kd * (delta_error / delta_time)
        
        self.timestamp = rospy.get_time()
        self.pre_error = error
        
        return p + i + d
    
    def reset(self):
        self.sum = 0
        self.initialize = True
        self.timestamp = 0
        self.pre_error = 0



class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=1)
    
    def talk(self, msg):
        self.pub.publish(msg)


class Listener:
    def __init__(self):
        rospy.Subscriber('apriltag_detector_node/detections', AprilTagDetectionArray, self.callback)
        self.t = Talker()
        self.running = False
        self.tag_detected = False
        self.linear_controller = PIDController(0.7,0.2,0.2)
        self.rotational_controller = PIDController(1.5,0.2,0.2)
    
    def callback(self, msg):
        move_cmd = Twist2DStamped()
        # check for an apriltag
        if msg.detections == []:
            self.rotational_controller.reset()
            self.linear_controller.reset()
            move_cmd.v = 0
            move_cmd.omega = 0
            self.t.talk(move_cmd)
            if self.tag_detected:
                rospy.logwarn('no tag detected')
                self.tag_detected = False
            return
        if not self.tag_detected:
            rospy.logwarn('tag detected')
            self.tag_detected = True
        # calculate linear velocity with the linear PID controller
        transform = msg.detections[0].transform.translation
        error_input = transform.z - 0.1
        move_cmd.v = self.linear_controller.get_accel(error_input)
        # s = '\nInput: ' + str(transform.z) + '\nCommand: ' + str(move_cmd.v)
        # rospy.logwarn(s)
        # calculate rotational velocity with the rotational PID controller
        error_input = -transform.x / transform.z
        move_cmd.omega = self.rotational_controller.get_accel(error_input)
        # publish the calculated velocities
        self.t.talk(move_cmd)


if __name__ == '__main__':
    try:
        rospy.init_node('follow_apriltag', anonymous=True)
        l = Listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
