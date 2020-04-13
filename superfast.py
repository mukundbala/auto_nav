#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import math
import cmath
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import time
import cv2
import imutils
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

laser_range = np.array([])
occdata = np.array([])
yaw = 0.0
rotate_speed = 0.5
linear_speed = 0.1
stop_distance = 0.4
occ_bins = [-1, 0, 100, 101]
front_angle = 28
front_angles = range(-front_angle,front_angle+1,1)


class Point(object):

    def __init__(self, (x, y)):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point((self.x + other.x, self.y + other.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __sub__(self, other):
        return Point((self.x - other.x, self.y - other.y))

    def __mul__(self, scalar):
        return Point((int(self.x*scalar), int(self.y*scalar)))

    def midpoint(self, other):
        return Point(((self.x + other.x)/2, (self.y + other.y)/2))

    def length(self):
        return int((math.sqrt(self.x**2 + self.y**2)))

    def iswithin(self, other):
        return (other.x-3 <= self.x <= other.x+3) and (other.y-3 <= self.y <= other.y+3)


def get_odom_dir(msg):
    global yaw

    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array(msg.ranges)
    # replace 0's with nan's
    # could have replaced all values below msg.range_min, but the small values
    # that are not zero appear to be useful
    laser_range[laser_range==0] = np.nan


def callback(msg, tfBuffer):
    global odata, loc
    
    occdata = np.array([msg.data])
    occ_counts = np.histogram(occdata,occ_bins)
    oc2 = occdata + 1
    oc3 = (oc2>1).choose(oc2,255)
    oc4 = (oc3==1).choose(oc3,155)
    odata = np.uint8(oc4.reshape(msg.info.height,msg.info.width,order='F'))
    map_h = msg.info.height
    map_w = msg.info.width
    total_bins = msg.info.width * msg.info.height
    # find transform to convert map coordinates to base_link coordinates
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation

    map_res = msg.info.resolution
    map_origin = msg.info.origin.position
    grid_x = int((cur_pos.x - map_origin.x) / map_res)
    grid_y = int((cur_pos.y - map_origin.y) / map_res)
    loc = Point((grid_x, grid_y))



def findpoint():
    global odata, loc, q, start_time

    kernel = np.ones((2,2),np.uint8)
    q = list()
        
    img = np.ascontiguousarray(odata, dtype=np.uint8)
    
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, bina1 = cv2.threshold(img, 125, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(bina1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    _, nowhite = cv2.threshold(img, 255, 0, cv2.THRESH_BINARY)
    rgb1 = cv2.cvtColor(nowhite, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        cv2.drawContours(rgb1, contour, -1, (255,255,255),1)

    _, bina2 = cv2.threshold(img,200,255, cv2.THRESH_BINARY)
    _, contours2, _ = cv2.findContours(bina2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    rgb2 = cv2.cvtColor(bina2, cv2.COLOR_GRAY2BGR)

    for contour in contours2:
        cv2.drawContours(rgb2, contour, -1, (0,255,0),1)

    dil1 = cv2.dilate(rgb1, kernel, iterations=3)
    dil2 = cv2.dilate(rgb2, kernel, iterations=3)
    diff = cv2.absdiff(dil1, dil2)
    ero = cv2.erode(diff,kernel,iterations=3)
    img = cv2.cvtColor(ero, cv2.COLOR_BGR2GRAY)
    _, img = cv2.threshold(img, 148, 255, cv2.THRESH_BINARY)


    kern1 = np.array(([1, -1, -1], [-1, 1, -1], [-1,-1,-1]), dtype='int')
    kern2 = np.array(([-1, 1, -1], [-1, 1, -1], [-1,-1,-1]), dtype='int')
    kern3 = np.array(([-1, -1, 1], [-1, 1, -1], [-1,-1,-1]), dtype='int')
    kern4 = np.array(([-1, -1, -1], [1, 1, -1], [-1,-1,-1]), dtype='int')
    kern6 = np.array(([-1, -1, -1], [-1, 1, 1], [-1,-1,-1]), dtype='int')
    kern7 = np.array(([-1, -1, -1], [-1, 1, -1], [1,-1,-1]), dtype='int')
    kern8 = np.array(([-1, -1, -1], [-1, 1, -1], [-1,1,-1]), dtype='int')
    kern9 = np.array(([-1, -1, -1], [-1, 1, -1], [-1,-1,1]), dtype='int')

    out1 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern1)
    out2 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern2)
    out3 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern3)
    out4 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern4)
    out6 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern6)
    out7 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern7)
    out8 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern8)
    out9 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern9)

    out = cv2.bitwise_or(out1, out2)
    out = cv2.bitwise_or(out, out3)
    out = cv2.bitwise_or(out, out4)
    out = cv2.bitwise_or(out, out6)
    out = cv2.bitwise_or(out, out7)
    out = cv2.bitwise_or(out, out8)
    out = cv2.bitwise_or(out, out9)

    endpts = np.where(out == 255)

    if endpts == []:
        return False
    else: 
        return True
    

def movebase():

    findpoint()
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    wait = client.wait_for_server(rospy.Duration(5))

    if not wait:
        rospy.logerr("Cannot connect to movebase")

    while findpoint():
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0.5
        goal.target_pose.pose.orientation.w = 1.0
    
        client.send_goal(goal, done_cb)

        rospy.spin()


def done_cb(status, result):
    if status == 4:
        rospy.logerr('Mapping Complete')
        stopbot()
        with open("maptime.txt", "w") as f:
            f.write("Elapsed Time: " + str(time.time() - start_time))
        cv2.imwrite('mazemap.png',odata)
        print('Press CTRL+C to quit')
        rospy.signal_shutdown("Shutting Down")
        time.sleep(10000000000)


def stopbot():
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)


def mover():
    global laser_range, start_time
    global botpos

    rospy.init_node('mover', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)

    # subscribe to odometry data
    rospy.Subscriber('odom', Odometry, get_odom_dir)
    # subscribe to LaserScan data
    rospy.Subscriber('scan', LaserScan, get_laserscan)
    # subscribe to map occupancy data
   # rospy.Subscriber('map', OccupancyGrid, get_occupancy)
    # subscribe to map occupancy data for callback
    rospy.Subscriber('map', OccupancyGrid, callback, tfBuffer)

    rospy.on_shutdown(stopbot)

    rate = rospy.Rate(5) # 5 Hz

    time.sleep(2)

    # save start time to file
    start_time = time.time()
    # initialize variable to write elapsed time to file
    timeWritten = 0

    # find direction with the largest distance from the Lidar,
    # rotate to that direction, and start moving
    while not rospy.is_shutdown():
        movebase()
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
