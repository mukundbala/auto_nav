#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import tf2_ros
from PIL import Image
from tf.transformations import euler_from_quaternion
import math

# constants
occ_bins = [-1, 0, 100, 101]

# create global variables
rotated = Image.fromarray(np.array(np.zeros((1,1))))


def callback(msg, tfBuffer):
    global rotated

    # create numpy array
    occdata = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    rospy.loginfo('Width: %i Height: %i',msg.info.width,msg.info.height)
    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # find transform to convert map coordinates to base_link coordinates
    # lookup_transform(target_frame, source_frame, time)
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation
    rospy.loginfo(['Trans: ' + str(cur_pos)])
    rospy.loginfo(['Rot: ' + str(cur_rot)])

    # get map resolution
    map_res = msg.info.resolution
    # get map origin struct has fields of x, y, and z
    map_origin = msg.info.origin.position
    # get map grid positions for x, y position
    grid_x = round((cur_pos.x - map_origin.x) / map_res)
    grid_y = round(((cur_pos.y - map_origin.y) / map_res))
    rospy.loginfo(['Grid Y: ' + str(grid_y) + ' Grid X: ' + str(grid_x)])

    # make occdata go from 0 instead of -1, reshape into 2D
    oc2 = occdata + 1
    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    odata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
    # set current robot location to 0
    odata[grid_x][grid_y] = 0
    # create image from 2D array using PIL
    img = Image.fromarray(odata.astype(np.uint8))
    # find center of image
    i_centerx = msg.info.width/2
    i_centery = msg.info.height/2
    # translate by curr_pos - centerxy to make sure the rotation is performed
    # with the robot at the center
    # using tips from:
    # https://stackabuse.com/affine-image-transformations-in-python-with-numpy-pillow-and-opencv/
    translation_m = np.array([[1, 0, (i_centerx-grid_y)],
                               [0, 1, (i_centery-grid_x)],
                               [0, 0, 1]])
    # Image.transform function requires the matrix to be inverted
    tm_inv = np.linalg.inv(translation_m)
    # translate the image so that the robot is at the center of the image
    img_transformed = img.transform((msg.info.height, msg.info.width),
                                    Image.AFFINE,
                                    data=tm_inv.flatten()[:6],
                                    resample=Image.NEAREST)

    # convert quaternion to Euler angles
    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.loginfo(['Yaw: R: ' + str(yaw) + ' D: ' + str(np.degrees(yaw))])

    # rotate by 180 degrees to invert map so that the forward direction is at the top of the image
    rotated = img_transformed.rotate(np.degrees(-yaw)+180)
    # we should now be able to access the map around the robot by converting
    # back to a numpy array: im2arr = np.array(rotated)

    # show image using grayscale map
    plt.imshow(rotated,cmap='gray')
    plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.00000000001)


def occupancy():
    # initialize node
    rospy.init_node('occupancy', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1.0)

    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, callback, tfBuffer)

    plt.ion()
    plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        occupancy()
    except  rospy.ROSInterruptException:
        pass
