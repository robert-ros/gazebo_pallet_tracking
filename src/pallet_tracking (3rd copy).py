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

# Macros used in self.pallet_position variable
LEFT = 0
CENTER = 1
RIGH = 2
PX = 0 
PY = 1

class GazeboPalletTracking:

    def __init__(self):

        # Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
        rospy.init_node('opencv_example', anonymous=True)

        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Create img variable
        self.img = None

        # Pallet positions
        self.pallet_position = [[0,0], [0,0], [0,0]]

        # Initialize an OpenCV Window named "Image Window"
        cv2.namedWindow("Image Window", 1)

        # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
        sub_image = rospy.Subscriber("/robot/robot/rear_camera/rgb/image_raw", Image, self.image_callback)
        sub_depth = rospy.Subscriber("/robot/robot/rear_camera/depth/points", PointCloud2, self.pointcloud2_callback)

    def get_distance(self, p1, p2):

        a = p2[0] - p1[0]
        b = p2[1] - p1[1]

        return math.sqrt(pow(a,2) + pow(b,2))

    def find_last_square(self, contours):

        x_square = 0
        y_square = 0
        found_square = False

        max_height = 0

        for cnt in contours:

            approx = cv2.approxPolyDP(cnt, 0.05*cv2.arcLength(cnt, True), True)
            
            #Get the bigest square
            if len(approx) == 4:  

                x,y,w,h = cv2.boundingRect(approx)

                if h > max_height:
                    max_height = h
                    square = approx

        cv2.drawContours(self.img, [square], 0, (0), 2)
        found_square = True

        (x,y),radius = cv2.minEnclosingCircle(square)
        center = (int(x),int(y))
        radius = int(radius)

        # Draw a circle formed by 4 vertex of the square
        #cv2.circle(self.img,center,radius,(0,255,0),2)

        # Draw a point in the middle of the shape
        cv2.circle(self.img, center , 2 , (0,0,0), -1)

        x_square = int(x)
        y_square = int(y)

               

        return x_square, y_square, found_square


    def find_last_circle(self, contours):

        x_circle = 0
        y_circle = 0
        found_circle = False


        for cnt in contours:

            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            
            # First circle detected
            if len(approx) >= 5:

                cv2.drawContours(self.img, [approx], 0, (0), 2)
                found_circle = True

                (x,y),radius = cv2.minEnclosingCircle(approx)
                center = (int(x),int(y))
                radius = int(radius)

                # Draw a circle
                #cv2.circle(self.img,center,radius,(0,255,0),2)

                # Draw a point in the middle of the shape
                cv2.circle(self.img, center , 2 , (0,0,0), -1)
                
                x_circle = int(x)
                y_circle = int(y)

                break

        # contour_list = []

        # for contour in contours:
        #     approx = cv2.approxPolyDP(contour,0.1*cv2.arcLength(contour,True),True)
        #     area = cv2.contourArea(contour)
        #     # Filter based on length and area
        #     if (7 < len(approx) < 18) & (900 >area > 200):
        #         # print area
        #         contour_list.append(contour)

        # cv2.drawContours(self.img, contour_list,  -1, (255,0,0), 2)
 

        return x_circle, y_circle, found_circle


    # Define a callback for the Image message
    def image_callback(self, img_msg):

        # Try to convert the ROS Image message to a CV2 Image
        try:

            self.img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

            lower_range = np.array([50,150,50])
            upper_range = np.array([255,255,255])

            color_mask = cv2.inRange(hsv, lower_range, upper_range)

            _, contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            #cv2.drawContours(self.img, contours, -1, (0,0,0), 3)

            # Get the center of the square
            x1, y1, found1 = self.find_last_square(contours)
            if found1 == True:
                self.pallet_position[CENTER] = (x1, y1)    
            else:
                print("Square not found")

            # Get the center of the circle
            x2, y2, found2 =  self.find_last_circle(contours)
            if found2 == True:
                self.pallet_position[LEFT] = (x2, y2)
            else:
                print("Circle not found")


            # for cnt in contours:

            #     approx = cv2.approxPolyDP(cnt, 0.05*cv2.arcLength(cnt, True), True)
            #     cv2.drawContours(self.img, [approx], 0, (0), 2)

            #     # Last square detected
            #     if len(approx) == 4:

            #         (x,y),radius = cv2.minEnclosingCircle(approx)
            #         center = (int(x),int(y))
            #         radius = int(radius)

            #         # Draw a circle formed by 4 vertex of the square
            #         cv2.circle(self.img,center,radius,(0,255,0),2)
            
            #         # Draw a point in the middle of the shape
            #         cv2.circle(self.img, center , 2 , (0,0,0), -1)

            #         self.pallet_position[CENTER] = (int(x), int(y))

            #     # Last circle detected
            #     if len(approx) >= 5:

            #         (x,y),radius = cv2.minEnclosingCircle(approx)
            #         center = (int(x),int(y))
            #         radius = int(radius)

            #         # Draw a circle
            #         cv2.circle(self.img,center,radius,(0,255,0),2)

            #         # Draw a point in the middle of the shape
            #         cv2.circle(self.img, center , 2 , (0,0,0), -1)
                    
            #         self.pallet_position[LEFT] = (int(x), int(y))



            # Show the image in an OpenCV Window
            cv2.imshow("Image Window", self.img)
            cv2.waitKey(3)


        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))



    def pixelTo3DPoint(self, cloud, u, v):

        width = cloud.width
        height = cloud.height
        point_step = cloud.point_step
        row_step = cloud.row_step

        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        x = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        y = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        z = struct.unpack('f', byte_format)[0]

        return [x, y, z]


    def pointcloud2_callback(self, pointcloud2_msg):

        x1, y1, z1 = self.pixelTo3DPoint(pointcloud2_msg, self.pallet_position[LEFT][PX],
                                                          self.pallet_position[LEFT][PY])
        
        print("-------- Center")
        print(x1+0.015)
        print(y1)
        print(z1)


        x2, y2, z2 = self.pixelTo3DPoint(pointcloud2_msg, self.pallet_position[CENTER][PX],
                                                          self.pallet_position[CENTER][PY])
        
        print("-------- Side") 
        print(x2+0.015)
        print(y2)
        print(z2)

        x = x2 - x1
        z = z2 - z1
        print("------- OP")
        print(x)
        print(z)
        if x == 0:
            theta = 0
        else:
            theta = math.atan(z/x)

        print("------- Angle")
        print(theta*(180/math.pi))
        #Si el angulo es superior a 45 grados, dar un aviso diciendo que la medida puede dar errores




    def run(self):

        while not rospy.is_shutdown():

            rospy.spin()





if __name__ == '__main__':
    try:
        gazebo_pallet_tracking = GazeboPalletTracking()
        gazebo_pallet_tracking.run()
    except rospy.ROSInterruptException:
        pass