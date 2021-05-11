#!/usr/bin/env python3.8

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import rospy
import rospkg 
import time
from datetime import datetime

#path = "/home/david/Pictures/saves/"
padding_left = 50
padding_right = 50
padding_top = 50
padding_bottom = 50
# margin has to be less or equal to padding_left or padding_top
square_margin = 50

rospack = rospkg.RosPack()

path = rospack.get_path('chess_detector')
path = path + "/tmp/"

save_delay = 5.0
#save_delay = float("inf")
last_save_time = 0

def split_images(msg):
    global last_save_time
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if time.time() > last_save_time + save_delay:
            cv2.imwrite(path + "board-" + datetime.today().strftime('%Y%m%d-%H%M%S-%f') + ".jpg", cv2Img)

        rows,cols = cv2Img.shape[:2]
        row_height = int((rows - padding_top - padding_bottom) / 8)
        col_width = int((cols - padding_left - padding_right) / 8)

        for i in range(0,8):
            for j in range(0,8):
                square = cv2Img[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
                if time.time() > last_save_time + save_delay:
                    cv2.imwrite(path + "square-" + str(i) + "-" + str(j) + "-" + datetime.today().strftime('%Y%m%d-%H%M%S-%f') + ".jpg", square)

        if time.time() > last_save_time + save_delay:
            print("Images were saved!")
            last_save_time = time.time()


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