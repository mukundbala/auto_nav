#!/usr/bin/env python


import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String

go = False
servo_pin = 12
motor_pin = 32

def shoot():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin, GPIO.OUT)
    p = GPIO.PWM(servo_pin, 50)
    actual = 2.5 + (70.0 * 10.0 / 180.0)
    p.start(6.5)
    p.ChangeDutyCycle(actual)
    time.sleep(20)
    #GPIO.setup(motor_pin, GPIO.OUT, initial=0)
    #GPIO.output(motor_pin, True)
    #time.sleep(8)
    #GPIO.output(motor_pin, False)
    #p.stop
    GPIO.cleanup()
    time.sleep(1000)
    rospy.signal_shutdown("Shutting down")


if __name__ == '__main__':
    rospy.init_node('pi_sub', anonymous=False, disable_signals=True)
    rospy.loginfo("Node Initialised")
    while not go:
        sub = rospy.wait_for_message('tilt', String)
        if sub.data == 'True':
            rospy.loginfo("Shooting")
            break
    shoot()
    rospy.spin()
