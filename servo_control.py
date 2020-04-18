#!/usr/bin/env python


import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String

go = False
servo_pin = 12
motor_pin = 32

def shoot(angle):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin, GPIO.OUT)
    p = GPIO.PWM(servo_pin, 50)
    # Angle is offset from calibrated angle
    x = 70 + angle
    # Convert to PWM
    actual = 2.5 + (x * 10.0 / 180.0)
    p.start(actual)
    p.ChangeDutyCycle(actual)
    time.sleep(1)
    GPIO.setup(motor_pin, GPIO.OUT, initial=0)
    GPIO.output(motor_pin, True)
    time.sleep(8)
    GPIO.output(motor_pin, False)
    time.sleep(1)
    p.stop()
    GPIO.cleanup()
    time.sleep(1000)
    rospy.signal_shutdown("Shutting down")


if __name__ == '__main__':
    rospy.init_node('pi_sub', anonymous=False, disable_signals=True)
    rospy.loginfo("Node Initialised")
    angle = 0
    while not go:
        sub = rospy.wait_for_message('tilt', String)
        if len(sub.data) > 0:
            angle = int(sub.data)
            rospy.loginfo("Shooting")
            go = True
            break
    shoot(angle)
    rospy.spin()
