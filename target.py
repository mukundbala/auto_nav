#!/usr/bin/env python


import rospy
import cv2
import imutils
import time
import numpy as np
import RPi.GPIO as GPIO
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


def get_odom_dir(msg):
	global yaw

	orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
	global laser_range

	laser_range = np.array([msg.ranges])
	laser_range[laser_range==0] = np.nan


def move_forward():
	global laser_range

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(5)

	twist.linear.x = 0.05
	twist.angular.z = 0.0
	pub.publish(twist)

	while True:
		lr2 = (laser_range[-10, 10, 1]<0.4).nonzero()
		if len(lr2) > 0:
			break

	twist.linear.x = 0
	twist.angular.z = 0
	pub.publish(twist)


def pan():
	global yaw, cX

    if cX - 150 == 0:
    	return

    else:
    	rot_angle = math.atan(cX / 100)

    twist = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    current_yaw = np.copy(yaw)
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    target_yaw = rot_angle
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo('Desired Yaw: ' + str(math.degrees(cmath.phase(c_target_yaw))))
    c_change = c_target_yaw / c_yaw
    c_change_dir = np.sign(c_change.imag)
    twist.linear.x = 0.0
    twist.angular.z = c_change_dir * 0.1
    pub.publish(twist)

    c_dir_diff = c_change_dir
    while(c_change_dir * c_dir_diff > 0):
        current_yaw = np.copy(yaw)
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        c_change = c_target_yaw / c_yaw
        c_dir_diff = np.sign(c_change.imag)
        rate.sleep()

    rospy.loginfo('End Yaw: ' + str(math.degrees(current_yaw)))
    twist.angular.z = 0.0
    pub.publish(twist)


def tilt():
	global cY

	if cY - 150 == 0:
		return
	else:
		tilt_angle = math.degrees(math.atan(cY / 100))

	print(tilt_angle)
	GPIO.setmode(GPIO.BOARD)
	servo_pin = 12
	GPIO.setup(servo_pin, GPIO.OUT)
	p = GPIO.PWM(servo_pin, 50)
	p.start(7.5)
	actual = 2.5 + (tilt_angle * 10 /180)
	p.ChangeDutyCycle(actual)
	p.stop()


def fire():
	motor_pin = 40
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(21, GPIO.OUT)
	GPIO.output(21, GPIO.HIGH)
	time.sleep(7)
	GPIO.output(21, GPIO.LOW)
	GPIO.cleanup()

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
		self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback, queue_size=1)


	def callback(self, data):
		global cX, cY

		if data == None:
			rospy.loginfo("Video not found")
			rospy.logerr("LOL")
			time.sleep(1000)

		rospy.loginfo("Loaded video stream.")

		
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
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
			print(cX + ', ' + cY)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 1, (0, 0, 255), -1)

		# show the frame to our screen
		cv2.imshow("Frame", frame), cv2.waitKey(1)



if __name__ == '__main__':
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
	rospy.init_node('target_acquisition', anonymous=False)
	rospy.loginfo("Node initialised")
	rospy.Subscriber('scan', LaserScan, get_laserscan)
	rospy.Subscriber('odom', Odometry, get_odom_dir)
	move_forward()
	td = TargetDetector(colourLower, colourUpper)
	pan()
	tilt()
	fire()

	rospy.spin()