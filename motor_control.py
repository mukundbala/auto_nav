#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(21, GPIO.OUT)

GPIO.output(21, GPIO.HIGH)
time.sleep(10)
GPIO.output(21, GPIO.LOW)



##for reset the pin's statuses to their default states before 
##we're done with the program
GPIO.cleanup()
