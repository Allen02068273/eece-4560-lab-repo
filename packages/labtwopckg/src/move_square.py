#!/usr/bin/env python3
# move_square.py by Nolan Allen and Rohan Goyal, 1 November 2023

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=1)
    
    def talk(self, msg):
        self.pub.publish(msg)


class Listener:
    def __init__(self):
        rospy.Subscriber('fsm_node/mode', FSMState, self.callback)
        rospy.logwarn('subscribed')
        self.t = Talker()
        rospy.logwarn('publisher set up')
    
    def callback(self, msg):
        rospy.logwarn('callback started')
        # check message for a mode switch
        if not msg.state == 'LANE_FOLLOWING':
            rospy.logwarn('LANE_FOLLOWING state not detected')
            return
        rospy.logwarn('LANE_FOLLOWING state detected')
        # move in a square
        rate = rospy.Rate(1) # 1hz
        move_cmd = Twist2DStamped()
        move_cmd.omega = 0
        while True:
            rospy.logwarn('loop started')
            # forward 1 meter
            move_cmd.v = 1
            self.t.talk(move_cmd)
            rospy.logwarn('published move forward')
            rate.sleep()
            # wait 5 seconds
            move_cmd.v = 0
            self.t.talk(move_cmd)
            for i in range(5):
                rate.sleep()
            # turn 90 degrees
            move_cmd.omega = 1
            self.t.talk(move_cmd)
            rate.sleep()
            # wait 5 seconds
            move_cmd.omega = 0
            self.t.talk(move_cmd)
            for i in range(5):
                rate.sleep()


if __name__ == '__main__':
    rospy.logwarn('program started')
    try:
        rospy.init_node('move_square', anonymous=True)
        rospy.logwarn('node started')
        l = Listener()
        rospy.spin()
        rospy.logwarn('after spin')
    except rospy.ROSInterruptException:
        pass
    rospy.logwarn('after try/except')

