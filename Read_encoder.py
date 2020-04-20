#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time


if __name__ == "__main__":

    pub = rospy.Publisher('/Encoder', Float64, queue_size=10)
    rospy.init_node('Encoder_pub', anonymous=True)
    rate = rospy.Rate(20)

    Enc_A = 3
    Enc_B = 4
    # Setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Enc_A, GPIO.IN)
    GPIO.setup(Enc_B, GPIO.IN)

    try:
        counter = 0
        t0 = rospy.Time.now().to_sec()
        aLastState = GPIO.input(Enc_A)
        while not rospy.is_shutdown():
            aState = GPIO.input(Enc_A)
            if aState != aLastState :
                if GPIO.input(Enc_B) != aState :
                    counter += 1
                else:
                    counter -= 1
                aLastState = aState

            #print('Counter Value', counter)
            if rospy.Time.now().to_sec() - t0 >= 0.1:
                pub.publish(counter)
                t0 = rospy.Time.now().to_sec()
            #pub.publish(counter)


    except KeyboardInterrupt:
        print("Encoder Finished")

    finally:
        GPIO.cleanup()

