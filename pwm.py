#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class pwm_command():
    def __init__(self):
        rospy.init_node('pub_pwm_command', anonymous=True)
        self.pub = rospy.Publisher('/PWM', Float64, queue_size=10)
        self.rate = rospy.Rate(20)
        self.reference = 30       # deg / s
        self.t0 = rospy.Time.now().to_sec()
        self.error_sum = 0
        self.tolerance = 2.5
        self.gain = 0
        self.gain_last = 0
        self.error_sum_limit = 20000
        rospy.Subscriber('/Filtered_vel', Float64, self.get_filtered_vel)
        rospy.spin()
        
    def get_filtered_vel(self, omega):
        self.error = self.reference - omega.data
        print('Error:', self.error)
        self.error_sum += self.error
        
        if self.error_sum >= self.error_sum_limit:
            self.error_sum = self.error_sum_limit
        elif self.error_sum <= -self.error_sum_limit:
            self.error_sum = -self.error_sum_limit

        if abs(self.error) > self.tolerance:
            self.pid()
            if self.gain >=100:
                self.gain = 100
            elif self.gain <= -100:
                self.gain = -100
    
        self.pub.publish(self.gain)
        # self.pub.publish(10)
        if rospy.Time.now().to_sec() - self.t0 >10:
            print('')
            print('')
            print('')
            print('----------Start Track Zero Input !!---------')
            self.reference = 0

    def pid(self, kp=0.001, ki=0.000000001):
        # self.gain = kp*self.error*100/24 + ki/kp*self.gain_last
        self.gain = (kp*self.error + ki*self.error_sum)*100/24 + self.gain_last
        self.gain_last = self.gain
        print('Gain:', self.gain)
        
if __name__ == '__main__':
    try:
        a = pwm_command()
        while not rospy.is_shutdown():
            pass

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
