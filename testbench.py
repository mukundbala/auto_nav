#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 20:54:17 2020

@author: huachen
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import RPi.GPIO as GPIO

laser_range = np.array([])
occdata = []
yaw = 0.0
rotate_speed = 0.3
linear_speed = 0.03
stop_distance = 1
occ_bins = [-1, 0, 100, 101]
front_angle = 5
front_angles = range(-front_angle,front_angle+1,1)

def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array([msg.ranges])

def pick_direction():
    global laser_range

    rospy.loginfo(['Moving forward'])
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)

    # start moving
    rospy.loginfo(['Start moving'])
    twist.linear.x = linear_speed
    twist.angular.z = 0.0
    # not sure if this is really necessary, but things seem to work more
    # reliably with this
    time.sleep(1)
    pub.publish(twist)


def mover():
    global laser_range

    rospy.init_node('mover', anonymous=True)

    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(5) # 5 Hz

    # find direction with the largest distance from the Lidar
    # rotate to that direction
    # start moving
    pick_direction()

    while not rospy.is_shutdown():
        # check distances in front of TurtleBot
        lr2 = laser_range[0,front_angles]
        # distances beyond the resolution of the Lidar are returned
        # as zero, so we need to exclude those values
        lr20 = (lr2!=0).nonzero()
        # find values less than stop_distance
        lr2i = (lr2[lr20]<float(stop_distance)).nonzero()

        # if the list is not empty
        if(len(lr2i[0])>0):
            rospy.loginfo(['Stop!'])
            # stop moving
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            time.sleep(1)
            pub.publish(twist)
            # fire servo
            rospy.loginfo(['Firing servo'])
            GPIO.setmode(GPIO.BOARD)
            servo_pin = 40
            GPIO.setup(servo_pin, GPIO.OUT)
            p = GPIO.PWM(servo_pin, 50)
            p.start(7.5)
            time.sleep(1)
            p.ChangeDutyCycle(5)
            time.sleep(1)
            p.stop()
            # fire solenoid
            rospy.loginfo(['Firing solenoid'])
            GPIO.setmode(GPIO.BOARD)
            solenoid_pin = 38
            GPIO.setup(solenoid_pin, GPIO.OUT)
            GPIO.output(solenoid_pin, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(solenoid_pin, GPIO.LOW)
            time.sleep(1)
            GPIO.cleanup()
            rate.sleep()
            pass

        rate.sleep()
        pass


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
