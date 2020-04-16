#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import time
import cv2
import imutils


occdata = np.array([])



# Define Point object, basically vectors
class Point(object):

    def __init__(self, (x, y)):
        self.x = x
        self.y = y
        self.d = 1.0

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


# Callback function for subscribing to occupancy data
def callback(msg, tfBuffer):
    global odata, loc, map_res, map_origin
    
    occdata = np.array([msg.data])
    oc2 = occdata + 1
    oc3 = (oc2>1).choose(oc2,255)
    oc4 = (oc3==1).choose(oc3,155)
    odata = np.uint8(oc4.reshape(msg.info.height,msg.info.width,order='F'))
    map_h = msg.info.height
    map_w = msg.info.width
    # find transform to convert map coordinates to base_link coordinates
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    # Get current position and rotation
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation
    map_res = msg.info.resolution
    map_origin = msg.info.origin.position
    grid_x = int((cur_pos.x - map_origin.x) / map_res)
    grid_y = int((cur_pos.y - map_origin.y) / map_res)
    # Get current location in vector form
    loc = Point((grid_x, grid_y))


# Finds a point to move towards
def findpoint():
    global odata, loc, q, start_time

    # Kernel used for image dilation and erosion
    kernel = np.ones((2,2),np.uint8)
    # This will be the queue of points to move to
    q = list()
    
    # This converts the occupancy data into a proper image
    img = np.ascontiguousarray(odata, dtype=np.uint8)
    # Might not be neccessary, but this works
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Get the contour of the whole grid including all mapped and occupied points
    _, bina1 = cv2.threshold(img, 125, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(bina1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    _, nowhite = cv2.threshold(img, 255, 0, cv2.THRESH_BINARY)
    rgb1 = cv2.cvtColor(nowhite, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        cv2.drawContours(rgb1, contour, -1, (255,255,255),1)

    # Get the contour of only occupied points
    _, bina2 = cv2.threshold(img,200,255, cv2.THRESH_BINARY)
    _, contours2, _ = cv2.findContours(bina2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    rgb2 = cv2.cvtColor(bina2, cv2.COLOR_GRAY2BGR)

    for contour in contours2:
        cv2.drawContours(rgb2, contour, -1, (0,255,0),1)

    # Dilate image, get the difference between the 2 (ie boundary between unmapped and unoccupied area)
    # and erode the image
    # this is the rough area we want to move towards
    dil1 = cv2.dilate(rgb1, kernel, iterations=3)
    dil2 = cv2.dilate(rgb2, kernel, iterations=3)
    diff = cv2.absdiff(dil1, dil2)
    ero = cv2.erode(diff,kernel,iterations=3)
    img = cv2.cvtColor(ero, cv2.COLOR_BGR2GRAY)
    _, img = cv2.threshold(img, 148, 255, cv2.THRESH_BINARY)


    # Kernels for hit-miss function
    kern1 = np.array(([1, -1, -1], [-1, 1, -1], [-1,-1,-1]), dtype='int')
    kern2 = np.array(([-1, 1, -1], [-1, 1, -1], [-1,-1,-1]), dtype='int')
    kern3 = np.array(([-1, -1, 1], [-1, 1, -1], [-1,-1,-1]), dtype='int')
    kern4 = np.array(([-1, -1, -1], [1, 1, -1], [-1,-1,-1]), dtype='int')
    kern6 = np.array(([-1, -1, -1], [-1, 1, 1], [-1,-1,-1]), dtype='int')
    kern7 = np.array(([-1, -1, -1], [-1, 1, -1], [1,-1,-1]), dtype='int')
    kern8 = np.array(([-1, -1, -1], [-1, 1, -1], [-1,1,-1]), dtype='int')
    kern9 = np.array(([-1, -1, -1], [-1, 1, -1], [-1,-1,1]), dtype='int')

    # What we are doing here is finding the endpoints of the contour found above
    # These match the pixels to find those that are only connected to one other pixel all around
    out1 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern1)
    out2 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern2)
    out3 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern3)
    out4 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern4)
    out6 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern6)
    out7 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern7)
    out8 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern8)
    out9 = cv2.morphologyEx(img, cv2.MORPH_HITMISS, kern9)

    # Combine the results from each hitmiss
    out = cv2.bitwise_or(out1, out2)
    out = cv2.bitwise_or(out, out3)
    out = cv2.bitwise_or(out, out4)
    out = cv2.bitwise_or(out, out6)
    out = cv2.bitwise_or(out, out7)
    out = cv2.bitwise_or(out, out8)
    out = cv2.bitwise_or(out, out9)

    # Our endpoints of the contours is the pixels which are white
    endpts = np.where(out == 255)

    try:
        ptmin = Point((endpts[0][0], endpts[1][0]))
    # If there are no endpoints found, there can be two possibilities: the map is complete
    # or the endpoints just could not be found
    except IndexError as e:
        # We assume that the map cannot be completed within 30 seconds, so we ask for a restart
        if time.time() - start_time <= 30:
            rospy.logerr("Navigation could not be executed. Please restart turtlebot")
            rospy.signal_shutdown("Shutting down...")
        # If more than 30 seconds have passed, that means that the mapping is complete. 
        else:
            return

    try:
        ptmax = Point((endpts[0][1], endpts[1][1]))
    # Sometimes there is only 1 endpoint found, so we use that point as a target for the robot
    # to move towards and hopefully refresh the occupancy grid
    # This is where the code takes the longest, as the point may be out of the movable area, 
    # thus causing the robot to execute recovery actions. 
    except IndexError as e:
        rospy.logwarn('Only 1 endpoint found')
        dist = ptmin - loc
        norm = Point((-dist.y, dist.x))
        first = ptmin - norm*(5.0/norm.length())
        first2 = ptmin + norm*(5.0/norm.length())

        if bina1[first.x][first.y] == 255:
            q.append(first)
        else:
            q.append(first2)
    else:
        # We set the target point to be between the two endpoints
        ptmid = ptmax.midpoint(ptmin)
        ptdif = ptmax - ptmin
        perp = Point((-ptdif.y, ptdif.x))
        angle2s = math.atan2(-perp.y, perp.x)
        ptmid.d = angle2s
        q.append(ptmid)
    
    # Colour and show the image for visualisation    
    #img[loc.x][loc.y] = 255
    #cv2.imshow("final", img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


# This part uses the ros navigation stack to move our robot to the target location
def moveto(p):
    global map_res, map_origin

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    #rospy.loginfo("Waiting for server...")
    wait = client.wait_for_server(rospy.Duration(5.0))

    if not wait:
        rospy.logerr("Cannot connect to movebase")
        rospy.signal_shutdown("Server not available")

    rospy.loginfo("Started server")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = p.x * map_res + map_origin.x
    goal.target_pose.pose.position.y = p.y * map_res + map_origin.x
    goal.target_pose.pose.orientation.w = p.d
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Moving")
    return
    

# Main thread
def pick_direction():
    global laser_range, loc, q
    
    rate = rospy.Rate(10)

    moveee = True
    s = Point((0,0))
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # We stop the bot from moving before executing any commands
    botspeed = Twist()
    botspeed.linear.x = 0.0
    botspeed.angular.z = 0.0
    time.sleep(1)
    pub.publish(botspeed)
    findpoint()

    # Moves to each point on the queue successively. There is only 1 point in the queue at any time. 
    while q != []:
        #for i in q:
            #print('('str(i.x) + ' ' + str(i.y) + ')')
        s = q.pop()
        moveto(s)
        rospy.loginfo('Finding next target')
        findpoint()

    # Once the queue is empty, we have completed mapping. 
    rospy.logerr('Mapping completed in ' + str(time.time() - start_time))
    stopbot()
    with open("Group4MapTime.txt", "w") as f:
        f.write("Elapsed Time: " + str(time.time() - start_time))
    cv2.imwrite('Group4MazeMap.png',odata)
    rospy.signal_shutdown("Shutting down")
    

def stopbot():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)


# Initialises everything
def mover():
    global laser_range, start_time
    global botpos

    # Disable signals is set to true so that the script can be stopped at the end of mapping. 
    rospy.init_node('mover', anonymous=True, disable_signals=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)

    #rospy.Subscriber('odom', Odometry, get_odom_dir)
    #rospy.Subscriber('scan', LaserScan, get_laserscan)
    rospy.Subscriber('map', OccupancyGrid, callback, tfBuffer)
    rospy.on_shutdown(stopbot)

    rate = rospy.Rate(5)

    # Save start time to file
    start_time = time.time()
 
    while not rospy.is_shutdown():
        pick_direction()
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down")