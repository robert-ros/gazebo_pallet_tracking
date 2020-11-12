#!/usr/bin/env python

# https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html
# https://codeloop.org/opencv-python-color-detection-example/

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
import tf.transformations

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

        # Read params
        self.read_rosparams()

        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Create img variable to show the results
        self.img = None

        # Varaibles to save the callback msgs
        self.image_msg = None
        self.pointcloud2_msg = None

        self.count = 0

        # Initialize an OpenCV Window named "Image Window"
        cv2.namedWindow("Image Window", 1)

        # Intialize a publisher to continuously publish the pallet pose
        self.pub_pallet = rospy.Publisher("pallet_detection/pose", PoseStamped, queue_size=1) 

        # Intialize a publisher to publish the pallet pose when a service server is called
        self.pub_pallet_triggered = rospy.Publisher("pallet_detection/pose_triggered", PoseStamped, queue_size=1)  

        # Initialize a service server to publish the pallet position once
        trigger_detection_srv = rospy.Service('pallet_detection/trigger', Trigger, self.trigger_pallet_detection)

        # Initialize a subscriber to read the camera
        sub_image = rospy.Subscriber(self.camera_topic, Image, self.image_callback,  queue_size=1)
        sub_depth = rospy.Subscriber(self.camera_depth_topic, PointCloud2, self.pointcloud2_callback,  queue_size=1)

        rospy.loginfo("Waiting for topics...")
        rospy.wait_for_message(self.camera_topic, Image)
        rospy.wait_for_message(self.camera_depth_topic, PointCloud2)
        rospy.loginfo("Node ready!")

    def read_rosparams(self):

        self.node_name = rospy.get_name()
        self.camera_topic= rospy.get_param(self.node_name + '/camera_topic','front_camera/rgb/image_raw')
        self.camera_depth_topic = rospy.get_param(self.node_name + '/camera_depth_topic','front_camera/depth/points')
        self.verbosity_level = rospy.get_param(self.node_name + '/verbosity_level', 1 )

        # Verbosity levels
        # Level 0: nothing to show
        # Level 1: shows the image captured in a window, print if pallet is detected or not
        # Level 2: print pallet position
        # Level 3: print all

    def image_callback(self, img_msg):

        self.image_msg = img_msg


    def pointcloud2_callback(self, cloud2_msg):

        self.pointcloud2_msg = cloud2_msg


    def trigger_pallet_detection(self, req):

        res = TriggerResponse()
        pallet_pose = PoseStamped
        pallet_found = False

        pallet_pose, pallet_found = self.pallet_detection()

        if pallet_found == True:
            self.pub_pallet_triggered.publish(pallet_pose)
            res.success = True
            res.message = "Pallet found"
        else:
            res.success = False
            res.message = "Pallet not found"

        return res


    def find_square(self, contours):

        x_square = 0
        y_square = 0
        found_square = False

        max_height = 0

        if len(contours) > 0:
      
            for cnt in contours:
                
                approx = cv2.approxPolyDP(cnt, 0.05*cv2.arcLength(cnt, True), True)
                
                #Get the bigest square
                if len(approx) == 4:  
                    found_square = True
                    x,y,w,h = cv2.boundingRect(approx)

                    if h > max_height:
                        max_height = h
                        square = approx

            if found_square == True:

                cv2.drawContours(self.img, [square], 0, (0), 2)
                
                (x,y),radius = cv2.minEnclosingCircle(square)
                center = (int(x),int(y))
                radius = int(radius)

                # Draw a circle formed by 4 vertex of the square
                #cv2.circle(self.img,center,radius,(0,255,0),2)

                # Draw a point in the middle of the shape
                cv2.circle(self.img, center , 2 , (0,0,0), -1)

                x_square = int(x)
                y_square = int(y)

        else:
            found_square = False

        return x_square, y_square, found_square


    def find_circle(self, contours):

        x_circle = 0
        y_circle = 0
        found_circle = False
        
        if len(contours) > 0:

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


        return x_circle, y_circle, found_circle


    def color_detection(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Green detection
        lower_range = np.array([50,150,50])
        upper_range = np.array([255,255,255])

        binary_img = cv2.inRange(hsv, lower_range, upper_range)

        return binary_img


    def contour_detection(self, img):

        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #cv2.drawContours(self.img, contours, -1, (0,0,0), 3)

        return contours


    def pixelTo3DPoint(self, cloud, pixel_position):

        u = pixel_position[0]
        v = pixel_position[1]

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


    def calculate_angle(self, point1, point2):

        x1 = point1[0]
        z1 = point1[1]

        x2 = point2[0]
        z2 = point2[1]

        x = x2 - x1
        z = z2 - z1

        if x == 0:
            theta = 0
        else:
            theta = math.atan(z/x)

        return theta


    def get_pallet_pose_stamped(self, x, y, z, theta):

        pallet_pose_stamped = PoseStamped()
        
        self.count+=1

        pallet_pose_stamped.header.seq = self.count
        pallet_pose_stamped.header.stamp = rospy.get_rostime()
        pallet_pose_stamped.header.frame_id = self.image_msg.header.frame_id

        # The coordinate axis changes, so the axes are not the same in the PoseStamped message
        pallet_pose_stamped.pose.position.x = z  # Depth distance between the center of the camera and the pallet
        pallet_pose_stamped.pose.position.y = x  # Horizontal distance between the center of the camera and the pallet
        pallet_pose_stamped.pose.position.z = y  # Vertical distance between the center of the camera and the pallet
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        pallet_pose_stamped.pose.orientation.x = quaternion[0]
        pallet_pose_stamped.pose.orientation.y = quaternion[1]
        pallet_pose_stamped.pose.orientation.z = quaternion[2]
        pallet_pose_stamped.pose.orientation.w = quaternion[3]
        
        # quaternions = (
        #     quaternion[0],
        #     quaternion[1],
        #     quaternion[2],
        #     quaternion[3]
        #     )

        # euler = tf.transformations.euler_from_quaternion(quaternions)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]

        # print("--------")
        # print("Roll:" + str(roll))
        # print("Pitch:" + str(pitch))
        # print("Yaw:" + str(self.rad2deg(yaw)))

        return pallet_pose_stamped

    def rad2deg(self, rad):

        return rad*(180/math.pi)

    def print_verbose(self, x1,y1,z1,x2,y2,z2,theta):
 
        print("-------- Center -------")
        print(x1+0.015)
        print(y1)
        print(z1)

        print("-------- Side -------- ") 
        print(x2+0.015)
        print(y2)
        print(z2)

        print("------- Operation ------")
        x = x2 - x1
        z = z2 - z1
        print(x)
        print(z)

        print("---- Deg Angle -----")
        print(self.rad2deg(theta))


    def print_result(self, x, y, z, theta):

        print("----- Pallet detected in -----")
        print("X: " + str(x+0.015) )
        print("Y: " + str(y) )
        print("Z: " + str(z) )
        print("Theta: " + str( self.rad2deg(theta) ))


    def pallet_detection(self):

        pallet_pose = PoseStamped()
        pallet_found = False

        try:

            # Convert OpenCV image to ROS Image
            img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
            self.img = img

            # Detect green color and get binary image
            binary_img = self.color_detection(img)

            # Detect contours of an image
            contours = self.contour_detection(binary_img)

            # Detect the center of the square
            px1, py1, found_square= self.find_square(contours)
              
            # Detect the center of the circle
            px2, py2, found_circle =  self.find_circle(contours)

            # Detect pallet distance and orientation
            if found_square == True and found_circle == True:
                
                pallet_center_pixel_pos = (px1,py1)  
                pallet_side_pixel_pos = (px2, py2)

                x1, y1, z1 = self.pixelTo3DPoint(self.pointcloud2_msg, pallet_center_pixel_pos)
                x2, y2, z2 = self.pixelTo3DPoint(self.pointcloud2_msg, pallet_side_pixel_pos)

                theta = self.calculate_angle([x1, z1], [x2, z2])                

                pallet_found = True
                pallet_pose = self.get_pallet_pose_stamped(x1,y2,z1, theta)

                if self.verbosity_level >= 3:
                    self.print_verbose(x1,y1,z1,x2,y2,z2, theta)
                
                if self.verbosity_level >= 2:
                    self.print_result(x1, y2, z1, theta)

            # Show the image in an OpenCV Window
            if self.verbosity_level >= 1:
                cv2.imshow("Image Window", self.img)
                cv2.waitKey(3)


        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))


        return pallet_pose, pallet_found


    def run(self):

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            
            pallet_pose, pallet_found = self.pallet_detection()

            if pallet_found == True:

                self.pub_pallet.publish(pallet_pose)
                
                if self.verbosity_level >= 1:
                    rospy.loginfo_throttle(5, "Gazebo pallet detected!")

            else:
                if self.verbosity_level >= 1:
                    rospy.loginfo_throttle(5, "Gazebo pallet not detected")

            r.sleep()



if __name__ == '__main__':
    try:
        gazebo_pallet_tracking = GazeboPalletTracking()
        gazebo_pallet_tracking.run()
    except rospy.ROSInterruptException:
        pass