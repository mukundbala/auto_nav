#eg2310
# this code is used to control the servo

import time
import RPi.GPIO as GPIO

 
GPIO.setmode(GPIO.BCM)

servo_pin = 18

#set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

#initialize the servo to be controlled by pwm with 50 HZ frequency
p = GPIO.PWM(servo_pin, 50)

#set servo to 90 degrees as it's starting position
p.start(7.5)


try:
        while True:
                x = input("Key in a number or exit by ctrl-c: ")
#               while x != '-1':
                angle = 2.5 + (x * 10.0 / 180.0)
                p.ChangeDutyCycle(angle)
                time.sleep(1)
#                       x = input("Key in a number in degree or type '-1' to ex$

except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()
#!/usr/bin/env python
#servo_control output PWM signal from the GPIO pin 18
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

init = 18;

GPIO.setup(init, GPIO.OUT)

#initialize pwm object with 1 kHz frequency
x = input('Angle: ')

pwm = GPIO.PWM(init, x) #the 1000 here need to be read in value from the main
pwm.start(0)

print("start")

for i in range(0, 100, 1):
	pwm.ChangeDutyCycle(i)
	time.sleep(0.2)
else: 
	print("finished")


#end
pwm.stop()
GPIO.cleanup()
