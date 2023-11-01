#!/usr/bin/env python3
# move_square.py by Nolan Allen and Rohan Goyal, 1 November 2023

import rospy
from duckietown_msgs.msg import Twist2DStamped


class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('joy_mapper_node/car_cmd', Twist2DStamped, queue_size=1)
    
    def talk(self, msg):
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('move_square', anonymous=True)
        t = Talker()
        rate = rospy.Rate(1) # 1hz
        move_cmd = Twist2DStamped()
        move_cmd.v = 1
        move_cmd.omega = 0
        t.talk(move_cmd)
        rate.sleep()
        move_cmd.v = 0
        t.talk(move_cmd)
    except rospy.ROSInterruptException:
        pass

