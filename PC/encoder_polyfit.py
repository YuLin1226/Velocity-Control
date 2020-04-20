#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
from scipy.signal import butter,filtfilt
import time
import csv
import matplotlib.pyplot as plt

class vel_filtered():
    def __init__(self):
        rospy.init_node('encoder_low_pass_filter', anonymous=True)
        self.pub = rospy.Publisher('/Filtered_vel', Float64, queue_size=10)
        self.rate = rospy.Rate(20)
        
        # '''Low Pass Filter
        self.Ts = 0.05
        self.fs = 1/self.Ts          # sample rate, Hz
        self.cutoff = 1            # desired cutoff frequency of the filter, Hz
        self.nyq = 0.5 * self.fs     # Nyquist Frequency
        self.order = 2
        # '''

        self.t0 = rospy.Time.now().to_sec()
        self.theta_list = []
        self.time = []
        self.omega_list = []
        rospy.Subscriber('/Encoder', Float64, self.get_encoder)
        rospy.spin()
        
    def get_encoder(self, msg):
        self.T = rospy.Time.now().to_sec() - self.t0
        self.theta = msg.data * 0.75

        self.theta_list.append(self.theta)
        self.time.append(self.T)
        # if len(self.theta_list) > 60:
        #     del self.theta_list[0]
        # if len(self.time) > 60:
        #     del self.time[0]

        self.poly = np.polyfit(self.time[-10:-1],self.theta_list[-10:-1], 1)
        self.omega = self.poly[0]
        self.omega_list.append(self.omega)


        # ''' Low Pass Filter
        # if len(self.omega_list) > 60:
        #     del self.omega_list[0]
        self.lp_omega_list = self.butter_lowpass_filter(self.omega_list, 
                                                        self.cutoff, 
                                                        self.fs, 
                                                        self.order, 
                                                        self.nyq)
        self.lp_omega = sum(self.lp_omega_list[-9:0])/9
        self.lp_omega = self.lp_omega_list[-1]
        # '''


        # print('theta', self.theta)
        print('omega', self.lp_omega)
        # print('lp_omega', self.lp_omega)
        self.pub.publish(self.lp_omega)
        

    def butter_lowpass_filter(self, data, cutoff, fs, order, nyq):
        self.normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        self.b, self.a = butter(order, self.normal_cutoff, btype='low', analog=False)
        self.y = filtfilt(self.b, self.a, data)
        return self.y

if __name__ == '__main__':
    try:
        a = vel_filtered()
        while not rospy.is_shutdown():
            pass

    except KeyboardInterrupt:
        pass

    finally:
        with open('LowPassOmega.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(a.lp_omega_list)
            writer.writerow(a.theta_list)
            writer.writerow(a.time)
        # plt.subplot(211)
        # plt.plot(a.time, a.theta_list,'b')
        # plt.xlabel('time[sec]')
        # plt.ylabel('theta[deg]')
        # plt.subplot(212)
        # plt.plot(a.time, a.omega_list,'r')
        # plt.xlabel('time[sec]')
        # plt.ylabel('omega[deg/sec]')
        # plt.show()
        pass
        
