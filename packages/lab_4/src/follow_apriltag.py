#!/usr/bin/env python3
# follow_apriltag.py by Nolan Allen, 20 December 2023

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
        self.t = Talker()
        self.running = False
    
    def callback(self, msg):
        # check message for a mode switch
        if not msg.state == 'LANE_FOLLOWING':
            return
        if self.running:
            return
        self.running = True
        # move in a square
        rate = rospy.Rate(1) # 1hz
        move_cmd = Twist2DStamped()
        move_cmd.omega = 0
        #while True:
        for i in range(4):
            rospy.logwarn(i)
            # forward 1 meter
            move_cmd.v = rospy.get_param('velocity')
            move_cmd.omega = rospy.get_param('lean')
            self.t.talk(move_cmd)
            rate.sleep()
            rate.sleep()
            rate.sleep()
            # wait 5 seconds
            move_cmd.v = 0
            move_cmd.omega = 0
            self.t.talk(move_cmd)
            for j in range(5):
                rate.sleep()
            # turn 90 degrees
            move_cmd.omega = rospy.get_param('omega')
            self.t.talk(move_cmd)
            rate.sleep()
            # wait 5 seconds
            move_cmd.omega = 0
            self.t.talk(move_cmd)
            for j in range(5):
                rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('move_square', anonymous=True)
        rospy.set_param('velocity', 0.35)
        rospy.set_param('omega', 0.7)
        rospy.set_param('lean', 0.02) # slight turn on forward movement
        l = Listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

