#!/usr/bin/env python3.8

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import rospy
import time

path = "/home/david/Pictures/saves/"
padding_left = 50
padding_right = 50
padding_top = 50
padding_bottom = 50
# margin has to be less or equal to padding_left or padding_top
square_margin = 50

def split_images(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(path + "board.jpg", cv2Img)

        rows,cols = cv2Img.shape[:2]
        row_height = int((rows - padding_top - padding_bottom) / 8)
        col_width = int((cols - padding_left - padding_right) / 8)

        for i in range(0,8):
            for j in range(0,8):
                square = cv2Img[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
                cv2.imwrite(path + "square_" + str(i) + "_" + str(j)+ ".jpg", square)


    except CvBridgeError as e:
        print(e)

def split_depth(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

print("Python version: %s" % sys.version)
print("OpenCV version: %s" % cv2.__version__)

bridge = CvBridge()

rospy.init_node('aruco_detector')
# Define your image topic
image_topic = "/chessboard_image/color/image_raw"
depth_topic = "/chessboard_image/depth/image_raw"

rospy.Subscriber(image_topic, Image, split_images)
rospy.Subscriber(depth_topic, Image, split_depth)


# Spin until Ctrl+C
rospy.spin()