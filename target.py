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

    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # rate = rospy.Rate(5)

    # twist = Twist()
    # twist.linear.x = 0.05
    # twist.angular.z = 0.0
    # time.sleep(1)
    # pub.publish(twist)

    # while True:
    #     #rospy.loginfo("inside while loop")
    #     lr2 = (laser_range[range(0, 7, 1)]<0.2).nonzero()
    #     print(lr2[0])
    #     if len(lr2[0]) > 0:
    #         #rospy.loginfo("exited loop")
    #         break

    # twist.linear.x = 0
    # twist.angular.z = 0
    # pub.publish(twist)
    os.system("rosnode kill /turtlebot3_lds")
    rospy.loginfo("Finished moving")


class TargetDetector:

    def __init__(self, colourLower, colourUpper):
        
        self.lower = colourLower
        self.upper = colourUpper

        self.t0 = time.time()

        self.target_point = Point()

        #self.image_pub = rospy.Publisher("/target/image_target",Image,queue_size=1)
        #self.mask_pub = rospy.Publisher("/target/image_mask",Image,queue_size=1)
        #self.target_pub = rospy.Publisher("/target/point_target",Point,queue_size=1)
        self.bridge = CvBridge()
        self.pan()


    def pan(self):
        global cX

        twist = Twist()
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        tilt = rospy.Publisher('tilt', String, queue_size=10)
        rate = rospy.Rate(10)

        while not -50 <= cX <= 50:
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
        tilt.publish('True')


    def tilt(self):
        global cY

        tilt_angle = math.degrees(math.atan(cY / 1000))
        print(tilt_angle)
        GPIO.setmode(GPIO.BOARD)
        servo_pin = 12
        GPIO.setup(servo_pin, GPIO.OUT)
        p = GPIO.PWM(servo_pin, 50)
        actual = 2.5 + (70 * 10 /180)
        p.start(actual)
        p.ChangeDutyCycle(actual)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(21, GPIO.OUT)
        GPIO.output(21, GPIO.HIGH)
        time.sleep(6)
        GPIO.output(21, GPIO.LOW)
        GPIO.cleanup()
        

    def callback(self, data):
        global cX, cY

        if data == None:
            rospy.loginfo("Video not found")
            rospy.logerr("LOL")
            time.sleep(1000)

        #rospy.loginfo("Loaded video stream.")

        np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #frame = cv2.cvtColor(np.array(data), cv2.COLOR_RGB2BGR)


        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow("mask", mask), cv2.waitKey(1)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            M = cv2.moments(c)
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
        colourLower = (170, 100, 0)
        colourUpper = (180, 255, 255)
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
