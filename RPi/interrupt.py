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
        GPIO.setup(self.Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.count = 0

        self.t0 = rospy.Time.now().to_sec()
        GPIO.add_event_detect(  self.Enc_A, 
                                GPIO.BOTH, 
                                callback=self.read_encoder, 
                                bouncetime=1)

        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - self.t0 >= 0.1:
                self.pub.publish(self.count)
                self.t0 = rospy.Time.now().to_sec()

    def read_encoder(self, count):
        if GPIO.input(self.Enc_A) == 1:
            if GPIO.input(self.Enc_B) == 1:
                self.count += 1
            else:
                self.count -= 1
        else:
            if GPIO.input(self.Enc_B) == 1:
                self.count -= 1
            else:
                self.count += 1





if __name__ == "__main__":
    
    try:
        a = read_encoder()
        
    except KeyboardInterrupt:
        print("Encoder Finished")

    finally:
        GPIO.cleanup()