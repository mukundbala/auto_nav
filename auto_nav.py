#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
import cmath
import numpy as np
import time

laser_range = np.array([])
occdata = []
yaw = 0.0
rotate_speed = 0.1
linear_speed = 0.1
stop_distance = 0.5
occ_bins = [-1, 0, 100, 101]
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)


def get_odom_dir(msg):
    global yaw

    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array(msg.ranges)


def get_occupancy(msg):
    global occdata

    # create numpy array
    occdata = np.array(msg.data)
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)


def rotatebot(rot_angle):
    global yaw

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(1)
    pub.publish(twist)


def pick_direction():
    global laser_range

    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)

    if laser_range.size != 0:
        lr2i = np.argmax(laser_range)
    else:
        lr2i = 0

    rospy.loginfo(['Picked direction: ' + str(lr2i)])

    # rotate to that direction
    rotatebot(float(lr2i))

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

    # subscribe to odometry data
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, get_occupancy)

    rate = rospy.Rate(5) # 5 Hz

    # find direction with the largest distance from the Lidar
    # rotate to that direction
    # start moving
    pick_direction()

    while not rospy.is_shutdown():
        # check distances in front of TurtleBot
        lr2 = laser_range[front_angles]
        # distances beyond the resolution of the Lidar are returned
        # as zero, so we need to exclude those values
        lr20 = (lr2!=0).nonzero()
        # find values less than stop_distance
        lr2i = (lr2[lr20]<float(stop_distance)).nonzero()

        # if the list is not empty
        if(len(lr2i[0])>0):
            rospy.loginfo(['Stop!'])
            # find direction with the largest distance from the Lidar
            # rotate to that direction
            # start moving
            pick_direction()

        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
