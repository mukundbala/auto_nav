#!/usr/bin/env python


import rospy
import os
import cv2
import imutils
import time
import numpy as np
import math
import cmath
#import RPi.GPIO as GPIO
from PIL import Image
from std_msgs.msg import String
#from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from imutils.video import VideoStream
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

cX = 1000
cY = 1000


def stopbot():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    time.sleep(1)
    pub.publish(twist)

def get_odom_dir(msg):
    global yaw

    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
    global laser_range

    laser_range = np.array(msg.ranges)


def move_forward():
    global laser_range

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(5)

    twist = Twist()
    twist.linear.x = 0.05
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)

    while True:
        #rospy.loginfo("inside while loop")
        lr2 = (laser_range[range(0, 5, 1)]<0.3).nonzero()
        # print(lr2[0])
        if len(lr2[0]) > 0:
            #rospy.loginfo("exited loop")
            break

    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)
    # We kill the node because there is electrical inteference with our servo
    os.system("rosnode kill /turtlebot3_lds")
    rospy.loginfo("Finished moving")


class TargetDetector:

    def __init__(self, colourLower, colourUpper):
        
        self.lower = colourLower
        self.upper = colourUpper

        self.t0 = time.time()

        self.target_point = Point()

        self.bridge = CvBridge()
        self.pan()


    def pan(self):
        global cX, cY

        twist = Twist()
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        tilt = rospy.Publisher('tilt', String, queue_size=10)
        rate = rospy.Rate(10)

        # As long as the target is within this x range, we are more than likely to hit it
        while not -50 <= cX <= 50:
            # We wait for message instead of subscribe so as to only execute the callback only once
            self.image_sub = rospy.wait_for_message("/raspicam_node/image/compressed",CompressedImage)
            self.callback(self.image_sub)
            if cX == 1000:
                rospy.logwarn("Target not found")
            elif cX < 0:
                rospy.loginfo("Panning right")
                twist.angular.z = -0.1
                time.sleep(0.5)
                pub.publish(twist)
            elif cX > 0:
                rospy.loginfo("Panning left")
                twist.angular.z = 0.1
                time.sleep(0.5)
                pub.publish(twist)

        rospy.loginfo("Finished panning")
        twist.angular.z = 0
        time.sleep(0.5)
        pub.publish(twist)
        # Kill camera to reduce electrical inteference
        os.system("rosnode kill /raspicam_node")
        # Send tilt data to node on the pi
        angle = math.atan2(cY, 500)
        tilt.publish(str(angle))
        rospy.loginfo("Now firing")


    def callback(self, data):
        global cX, cY

        if data == None:
            rospy.logerr("Video not found")
            time.sleep(1000)

        #rospy.loginfo("Loaded video stream.")
        np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Resize the frame, blur it, and then convert it to the HSV
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Create a mask for the color so as to ignore everything else
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow("mask", mask), cv2.waitKey(1)

        # Find contours in the mask and get the center of it
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            # Find the biggest contour
            c = max(cnts, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            # Draw a rectangle in green around the target for visualisation
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            M = cv2.moments(c)
            # Use calibration constants to shif the origin to our perfect pan & tilt orientation
            cX = int(M["m10"] / M["m00"] - 285)
            cY = int(M["m01"] / M["m00"] - 100)
            print(str(cX) + ', ' + str(cY))
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


        # show the frame to our screen
        cv2.imshow("Frame", frame), cv2.waitKey(1)



if __name__ == '__main__':
    rospy.on_shutdown(stopbot)
    print("Enter color:")
    coloor = raw_input()
    if coloor == "red":
        colourLower = (0, 25, 0)
        colourUpper = (20, 255, 255)
    elif coloor == "green":
        colourLower = (40, 100, 0)
        colourUpper = (80, 255, 255)
    elif coloor == "blue":
        colourLower = (100, 80, 0)
        colourUpper = (140, 255, 255)
    else:
        rospy.loginfo("Color Invalid")
    rospy.init_node('target_acquisition', anonymous=False, disable_signals=True)
    rospy.loginfo("Node initialised")
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    time.sleep(2)
    
    try:
        move_forward()
        td = TargetDetector(colourLower, colourUpper)
        time.sleep(5)
        

    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down")
    rospy.spin()
