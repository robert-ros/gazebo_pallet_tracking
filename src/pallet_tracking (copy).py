#!/usr/bin/env python

# https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html
# https://codeloop.org/opencv-python-color-detection-example/

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import struct

pallet_center = 0

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):

    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    #rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_range = np.array([50,150,50])
        upper_range = np.array([255,255,255])

        color_mask = cv2.inRange(hsv, lower_range, upper_range)

        _, contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #cv2.drawContours(img, contours, -1, (0,0,0), 3)

        for cnt in contours:

            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            cv2.drawContours(img, [approx], 0, (0), 2)

            if len(approx) == 4:


                (x,y),radius = cv2.minEnclosingCircle(approx)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(img,center,radius,(0,255,0),2)

                cv2.circle(img, center , 2 , (0,0,0), -1)

                global pallet_center
                pallet_center = center

            else:
                print("Not detected")

        show_image(img)

    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))



def pixelTo3DPoint(cloud, u, v):

    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step
    row_step = cloud.row_step

    array_pos = v*row_step + u*point_step

    bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
    bytesY = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
    bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

    byte_format=struct.pack('4B', *bytesX)
    X = struct.unpack('f', byte_format)[0]

    byte_format=struct.pack('4B', *bytesY)
    Y = struct.unpack('f', byte_format)[0]

    byte_format=struct.pack('4B', *bytesZ)
    Z = struct.unpack('f', byte_format)[0]

    return [X, Y, Z]


def pointcloud2_callback(pointcloud2_msg):
    dummy = 0
    #global pallet_center 
    #x, y, z = pixelTo3DPoint(pointcloud2_msg, pallet_center[0], pallet_center[1])
    #print("-----")
    #print(x+0.015)
    #print(y)
    #print(z)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/robot/robot/rear_camera/rgb/image_raw", Image, image_callback)
sub_depth = rospy.Subscriber("/robot/robot/rear_camera/depth/points", PointCloud2, pointcloud2_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()



# import cv2
# import numpy as np

# img = cv2.imread('img.png') 
# hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# #Red color rangle  169, 100, 100 , 189, 255, 255
# lower_range = np.array([110,50,50])
# upper_range = np.array([130,255,255])

# mask = cv2.inRange(hsv, lower_range, upper_range)

# _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# for cnt in contours:
#     approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
#     cv2.drawContours(img, [approx], 0, (0), 5)


# cv2.imshow('image', img)
# cv2.imshow('mask', mask)
 
# cv2.waitKey(0)
# cv2.destroyAllWindows()