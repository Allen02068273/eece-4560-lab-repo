#!/usr/bin/env python3
# lane_finder.py by Nolan Allen, 9 November 2023

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class LaneFinder:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.processor_cb, queue_size=1, buff_size=2**24)
        self.pub_white = rospy.Publisher("/lane_lines_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("/lane_lines_yellow", Image, queue_size=10)
    
    def processor_cb(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # crop to the bottom half of the image
        img_cropped = cv_img[240:480, 0:640]
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        img_cropped = new_image[offset:, :]
        
        # convert image to HSV
        img_hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)
        
        # filter image into white and yellow
        img_white = cv2.inRange(img_hsv, (0, 0, 200), (255, 50, 255))
        img_yellow = cv2.inRange(img_hsv, (25, 100, 100), (35, 255, 255))
        
        # apply Canny edge detection
        low = 50
        high = 150
        edges = cv2.Canny(img_cropped, low, high)
        
        # dilate filtered images
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        img_white = cv2.dilate(img_white, kernel)
        img_yellow = cv2.dilate(img_yellow, kernel)
        img_yellow = cv2.dilate(img_yellow, kernel)
        
        # get white line outlines and publish
        lines = self.get_lines(edges, img_white)
        img = self.output_lines(img_cropped, lines)
        ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.pub_white.publish(ros_img)
        
        # get yellow line outlines and publish
        lines = self.get_lines(edges, img_yellow)
        img = self.output_lines(img_cropped, lines)
        ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.pub_yellow.publish(ros_img)
        
    def get_lines(self, edges, image_color):
        # apply Hough transform
        color_edges = cv2.bitwise_and(edges, image_color, mask=None)
        lines = cv2.HoughLinesP(color_edges, rho=1.0, theta=np.pi/180.0, threshold=20)
        return lines
        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output


if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lane_finder", anonymous=True)
    img_processed = LaneFinder()
    rospy.spin()

