#!/usr/bin/env python


import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String

go = False
servo_pin = 12
motor_pin = 40

def shoot():  
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin, GPIO.OUT)
    p = GPIO.PWM(servo_pin, 50)
    actual = 2.5 + (70 * 10 /180)
    p.start(actual)
    p.ChangeDutyCycle(actual)
    GPIO.setup(motor_pin, GPIO.OUT)
    GPIO.output(motor_pin, GPIO.HIGH)
    time.sleep(6)
    GPIO.output(motor_pin, GPIO.LOW)
    p.stop
    GPIO.cleanup()


if __name__ == '__main__':
    rospy.init_node('pi_sub', anonymous=False)
    rospy.loginfo("Node Initialised")
    while not go:
        sub = rospy.wait_for_message('tilt', String)
        if sub.data == 'True':
            rospy.loginfo("Shooting")
            break
        
    shoot()
    rospy.spin()