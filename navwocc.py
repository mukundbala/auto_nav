#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
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
stop_distance = 0.25
occ_bins = [-1, 0, 100, 101]
front_angle = 30
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
        return (other.x-10 <= self.x <= other.x+10) and (other.y-10 <= self.y <= other.y+10)


dir4 = [Point((0, -1)), Point((0, 1)), Point((1, 0)), Point((-1, 0))]


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
    global odata, loc, q

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
    twoff = np.where(img == 255)
    tup = list(zip(twoff[0], twoff[1]))
    ptmin = Point(tup[0])
    ptmax = Point(tup[-1])
    ptdif = ptmax - ptmin
    ptmid = ptmax.midpoint(ptmin)
    perp = Point((-ptdif.y, ptdif.x))
    ptperp = ptmid - perp*(8.0/perp.length())
    ptperp2 = ptmid + perp*(8.0/perp.length())

    if bina1[ptperp.x][ptperp.y] == 255:
        q.append(ptperp)
        q.append(ptperp2)
        img[ptperp.x][ptperp.y] = 255
    else: 
        q.append(ptperp2)
        q.append(ptperp)
        img[ptperp2.x][ptperp2.y] = 255

    for i in q:
        print(i.x, i.y)

    img[loc.x][loc.y] = 255
    cv2.imshow("final", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def movebot(goal):
    global loc

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    dist = goal - loc
    s = dist.length()
    twist.linear.x = 0.15
    twist.angular.z = 0.0
    pub.publish(twist)
    time.sleep(s/10)
    
    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)


def rotatebot(rot_angle):
    global yaw

    rospy.on_shutdown(stopbot)

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(10)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = rot_angle
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
    pub.publish(twist)


def pick_direction():
    global laser_range, loc, q, yaw
    rate = rospy.Rate(10)

    moveee = True
    s = Point((0,0))
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # stop moving
    botspeed = Twist()
    botspeed.linear.x = 0.0
    botspeed.angular.z = 0.0
    time.sleep(1)
    pub.publish(botspeed)

    findpoint()

    while q != []:
        s = q.pop()
        
        while not loc.iswithin(s):
            dirr = s - loc
            angle2s = math.atan2(dirr.y, dirr.x)
            print("angle: ", angle2s)
            print("current: ", yaw)
            rotatebot(angle2s)
            print("DONE ROTATING")
            movebot(s)
            print("DONE MOVING")

        findpoint()

    botspeed.linear.x = 0
    botspeed.angular.z = 0
    pub.publish(botspeed)
    rospy.logwarn("Reached end")

    # start moving
    #rospy.loginfo(['Start moving'])
    #twist.linear.x = linear_speed
    #twist.angular.z = 0.0
    # not sure if this is really necessary, but things seem to work more
    # reliably with this
    #time.sleep(1)
    #pub.publish(twist)


def stopbot():
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)


def closure(mapdata):
    # This function checks if mapdata contains a closed contour. The function
    # assumes that the raw map data from SLAM has been modified so that
    # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
    # values go from 1 to 101.

    # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
    # closed contours have larger areas than arc length, while open contours have larger
    # arc length than area. But in my experience, open contours can have areas larger than
    # the arc length, but closed contours tend to have areas much larger than the arc length
    # So, we will check for contour closure by checking if any of the contours
    # have areas that are more than 10 times larger than the arc length
    # This value may need to be adjusted with more testing.
    ALTHRESH = 10
    # We will slightly fill in the contours to make them easier to detect
    DILATE_PIXELS = 3

    # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
    # and other positive values up to 101 (occupied)
    # so we will apply a threshold of 2 to create a binary image with the
    # occupied pixels set to 255 and everything else is set to 0
    # we will use OpenCV's threshold function for this
    ret,img2 = cv2.threshold(mapdata,2,255,0)
    # we will perform some erosion and dilation to fill out the contours a
    # little bit
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(DILATE_PIXELS,DILATE_PIXELS))
    # img3 = cv2.erode(img2,element)
    img4 = cv2.dilate(img2,element)
    # use OpenCV's findContours function to identify contours
    fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = fc[0]
    # find number of contours returned
    lc = len(contours)
    # create array to compute ratio of area to arc length
    cAL = np.zeros((lc,2))
    for i in range(lc):
        cAL[i,0] = cv2.contourArea(contours[i])
        cAL[i,1] = cv2.arcLength(contours[i], True)

    # closed contours tend to have a much higher area to arc length ratio,
    # so if there are no contours with high ratios, we can safely say
    # there are no closed contours
    cALratio = cAL[:,0]/cAL[:,1]
    # print(cALratio)
    if np.any(cALratio > ALTHRESH):
        return True
    else:
        return False


def mover():
    global laser_range
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

    # save start time to file
    start_time = time.time()
    # initialize variable to write elapsed time to file
    timeWritten = 0

    # find direction with the largest distance from the Lidar,
    # rotate to that direction, and start moving
    pick_direction()

    rospy.spin()

    '''while not rospy.is_shutdown():
                # create image from 2D array using PIL
            
                    if laser_range.size != 0:
                        # check distances in front of TurtleBot and find values less
                        # than stop_distance
                        lri = (laser_range[front_angles]<float(stop_distance)).nonzero()
                        rospy.loginfo('Distances: %s', str(lri)[1:-1])
                    else:
                        lri[0] = []
            
                    # if the list is not empty
                    if(len(lri[0])>0):
                        rospy.loginfo(['Stop!'])
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        pick_direction()
            
                    # check if SLAM map is complete
                    if timeWritten:
                        if closure(odata):
                            # map is complete, so save current time into file
                            with open("maptime.txt", "w") as f:
                                f.write("Elapsed Time: " + str(time.time() - start_time))
                            timeWritten = 1
                            # play a sound
                            soundhandle = SoundClient()
                            rospy.sleep(1)
                            soundhandle.stopAll()
                            soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
                            rospy.sleep(2)
                            # save the map
                            cv2.imwrite('mazemap.png',odata)
            
                    rate.sleep()
            '''


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
