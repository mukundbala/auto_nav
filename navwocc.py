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
stop_distance = 0.5
occ_bins = [-1, 0, 100, 101]
front_angle = 25
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

    try:
        ptmin = Point((endpts[0][0], endpts[1][0]))
    except IndexError as e:
        rospy.logerr('Mapping Complete')
        stopbot()
        with open("maptime.txt", "w") as f:
            f.write("Elapsed Time: " + str(time.time() - start_time))
        cv2.imwrite('mazemap.png',odata)
        print('Press CTRL+C to quit')
        time.sleep(10000000000)

    try:
        ptmax = Point((endpts[0][1], endpts[1][1]))
    except IndexError as e:
        rospy.logwarn('Only 1 endpoint found')
        dist = ptmin - loc
        norm = Point((-dist.y, dist.x))
        first = loc - norm*(10.0/norm.length())
        first2 = loc + norm*(10.0/norm.length())

        if bina1[first.x][first.y] == 255:
            q.append(first)
            q.append(ptmin)
        else:
            q.append(first2)
            q.append(ptmin)
    else:
        ptdif = ptmax - ptmin
        ptmid = ptmax.midpoint(ptmin)
        perp = Point((-ptdif.y, ptdif.x))
        ptperp = ptmid - perp*(10.0/perp.length())
        ptperp2 = ptmid + perp*(10.0/perp.length())
        dist = ptperp - loc
        norm = Point((-dist.y, dist.x))
        first = loc + norm*(10.0/norm.length())
        if bina1[first.x][first.y] != 255:
            first = loc - norm*(10.0/norm.length())
        dist2 = ptperp2 - loc
        norm2 = Point((-dist2.y, dist2.x))
        first2 = loc + norm2*(10.0/norm2.length())
        if bina1[first2.x][first2.y] != 255:
            first2 = loc - norm2*(10.0/norm2.length())

        if bina1[ptperp.x][ptperp.y] == 255:
            q.append(first)
            q.append(ptperp)
            q.append(ptperp2)
            #img[ptperp.x][ptperp.y] = 255
        else: 
            q.append(first2)
            q.append(ptperp2)
            q.append(ptperp)
            #img[ptperp2.x][ptperp2.y] = 255

    img[loc.x][loc.y] = 255
    rospy.loginfo('Current Position: (' + str(loc.x) + ', ' + str(loc.y) + ')')
    rospy.loginfo('Target Position: (' + str(q[0].x) + ', ' + str(q[0].y) + ')')
    #cv2.imshow("final", img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


def movebot(goal):
    global loc, laser_range

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = 0.15
    twist.angular.z = 0.0
    pub.publish(twist)
    lri = (laser_range[front_angles]<float(stop_distance)).nonzero()

    while len(lri[0])==0:
        if loc.iswithin(goal):
            break
        lri = (laser_range[front_angles]<float(stop_distance)).nonzero()
        rate.sleep()
    
    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    #twist.angular.z = 0.0
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
    #rospy.loginfo('Current Yaw: ' + str(math.degrees(current_yaw)))
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = rot_angle
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo('Desired Yaw: ' + str(math.degrees(cmath.phase(c_target_yaw))))
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
        #rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    rospy.loginfo('End Yaw: ' + str(math.degrees(current_yaw)))
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
        dirr = s - loc
        angle2s = math.atan2(dirr.y, dirr.x)
        rotatebot(angle2s)
        rospy.loginfo('Rotation Complete')
        movebot(s)
        rospy.loginfo('Move Complete')

        findpoint()

    botspeed.linear.x = 0
    botspeed.angular.z = 0
    pub.publish(botspeed)
    rospy.logwarn("Reached End")

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
    
    ALTHRESH = 10
    DILATE_PIXELS = 3

    ret,img2 = cv2.threshold(mapdata,2,255,0)
    
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(DILATE_PIXELS,DILATE_PIXELS))
    img4 = cv2.dilate(img2,element)
    fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = fc[0]
    lc = len(contours)
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

    # save start time to file
    start_time = time.time()
    # initialize variable to write elapsed time to file
    timeWritten = 0

    # find direction with the largest distance from the Lidar,
    # rotate to that direction, and start moving
    while not rospy.is_shutdown():
        pick_direction()
        rate.sleep()

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
