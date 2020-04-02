#!/usr/bin/env python


import rospy
import cv2
import imutils
import time
import numpy as np
from PIL import Image
from std_msgs.msg import String
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from imutils.video import VideoStream
from collections import deque
from cv_bridge import CvBridge, CvBridgeError


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
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)

		# show the frame to our screen
		cv2.imshow("Frame", frame), cv2.waitKey(1)


	def main(self):
	
		rospy.spin()


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
	td = TargetDetector(colourLower, colourUpper)
	td.main()