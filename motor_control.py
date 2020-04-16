#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(32, GPIO.OUT, initial=0)

GPIO.output(32, True)
print('it is high now')
time.sleep(8)
GPIO.output(32, False)

GPIO.cleanup()
print('end')
