#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def callback(msg):
	# create numpy array
	laser_range = np.array(msg.ranges)
	# replace 0's with nan
	laser_range[laser_range==0] = np.nan
	# find index with minimum value
	lr2i = np.nanargmin(laser_range)

	# log the info
        rospy.loginfo('Shortest distance at %i degrees', lr2i)


def scanner():
	# initialize node
	rospy.init_node('scanner', anonymous=True)

	# set the update rate to 1 Hz
	rate = rospy.Rate(1) # 1 Hz

	# subscribe to LaserScan data
	rospy.Subscriber('scan', LaserScan, callback)

	# wait until it is time to run again
	rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
