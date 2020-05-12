#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time


class read_encoder():
    def __init__(self):
        self.pub = rospy.Publisher('/Encoder', Float64, queue_size=10)
        rospy.init_node('Encoder_pub', anonymous=True)

        self.Enc_A = 3
        self.Enc_B = 4
        # Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Enc_A, GPIO.IN)
        GPIO.setup(self.Enc_B, GPIO.IN)
        self.count = 0

        self.t0 = rospy.Time.now().to_sec()
        GPIO.add_event_detect(  self.Enc_A, 
                                GPIO.RISING, 
                                callback=self.read_encoder_rise, 
                                bouncetime=1)

        GPIO.add_event_detect(  self.Enc_A, 
                                GPIO.RISING, 
                                callback=self.read_encoder_fall, 
                                bouncetime=1)

        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - self.t0 >= 0.1:
                self.pub.publish(self.count)
                self.t0 = rospy.Time.now().to_sec()

    def read_encoder_rise(self, count):
        if GPIO.input(self.Enc_B) == 0:
            self.count += 1
        else:
            self.count -= 1


    def read_encoder_fall(self, count):
        if GPIO.input(self.Enc_B) == 0:
            self.count -= 1
        else:
            self.count += 1



if __name__ == "__main__":
    
    try:
        read_encoder()
    
    except KeyboardInterrupt:
        print("Encoder Finished")

    finally:
        GPIO.cleanup()

