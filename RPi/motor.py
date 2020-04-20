#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float64
import numpy as np
import time


def get_pwm(pwm_duty):
    sig = pwm_duty.data
    if sig >=0:
        pwm_A1R.ChangeDutyCycle(sig)
        pwm_A2R.ChangeDutyCycle(0)
    else:
        pwm_A1R.ChangeDutyCycle(0)
        pwm_A2R.ChangeDutyCycle(-sig)


if __name__ == '__main__':
    rospy.init_node('get_input_pwm', anonymous=True)
    pin1,pin2,pinEN = 19, 26, 13

    GPIO.setmode(GPIO.BCM)
    #wheel A
    GPIO.setup(pin1, GPIO.OUT) #PWM 1
    GPIO.setup(pin2, GPIO.OUT) #PWM 2
    GPIO.setup(pinEN, GPIO.OUT) #En


    pwm_A1R = GPIO.PWM(pin1, 100)
    pwm_A2R = GPIO.PWM(pin2, 100)

    pwm_A1R.start(0)
    pwm_A2R.start(0)

    try:
        GPIO.output(pinEN, 0)
        rospy.Subscriber('/PWM', Float64, get_pwm)
        rospy.spin()
        while not rospy.is_shutdown():
            pass


    except KeyboardInterrupt:
        print("keyboardinterrupt")
        pwm_A1R.stop()
        pwm_A2R.stop()
    finally:
        print("Done")
        GPIO.cleanup()

