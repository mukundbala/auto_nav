#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time


def movebase():
    global boole

    boole = True
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    wait = client.wait_for_server(rospy.Duration(5))

    if not wait:
        rospy.logerr("Cannot connect to movebase")

    while boole:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0.5
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal, done_cb)


def done_cb(status, result):
    global start_time, boole

    if status == 4:
        rospy.logerr('Mapping completed in ' + str(time.time() - start_time))
        stopbot()
        with open("maptime.txt", "w") as f:
            f.write("Elapsed Time: " + str(time.time() - start_time))
        cv2.imwrite('mazemap.png',odata)
        boole = False
        rospy.signal_shutdown('Shutting down...')

def move():
    global start_time

    rospy.init_node('superfastnav', anonymous=True, disable_signals=True)
    rate = rospy.Rate(5) # 5 Hz

    # save start time to file
    start_time = time.time()
    
    while not rospy.is_shutdown():
        movebase()
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
