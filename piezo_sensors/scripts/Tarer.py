#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from piezo_sensors.srv import Tarer
import numpy as np

class Tare_values:
    def __init__(self):
        self.store_T = rospy.Subscriber('Thumb', Float64MultiArray, self.callback_T)
        self.store_I = rospy.Subscriber('Index', Float64MultiArray, self.callback_I)  
        self.store_R = rospy.Subscriber('Ring', Float64MultiArray, self.callback_R)
        self.tarer_service = rospy.Service('tare', Tarer, self.handle_tare)
        self.pub_mean_T= rospy.Publisher('Mean_T', Float64MultiArray, queue_size=10)
        self.pub_mean_I= rospy.Publisher('Mean_I', Float64MultiArray, queue_size=10)
        self.pub_mean_R= rospy.Publisher('Mean_R', Float64MultiArray, queue_size=10)
        self.mean_T = Float64MultiArray()
        self.mean_I = Float64MultiArray()
        self.mean_R = Float64MultiArray()
        self.mean_T.data = np.zeros(8)
        self.mean_I.data = np.zeros(8)
        self.mean_R.data = np.zeros(8)
        self.T = []
        self.I = []
        self.R = []

    def callback_T(self, msg):
        self.T = msg.data
        self.new_message_T = True

    def callback_I(self, msg):
        self.I = msg.data
        self.new_message_I = True

    def callback_R(self, msg):
        self.R = msg.data
        self.new_message_R = True

    def handle_tare(self, req):
        data_T = []
        data_I = []
        data_R = []
        while len(data_T)<req.n_data:
            if self.new_message_T and self.new_message_I and self.new_message_R:
                data_T.append(self.T)
                data_I.append(self.I)
                data_R.append(self.R)
                self.new_message_T = False
                self.new_message_I = False
                self.new_message_R = False
        self.mean_T.data += np.mean(np.array(data_T), axis=0)
        self.mean_I.data += np.mean(np.array(data_I), axis=0)
        self.mean_R.data += np.mean(np.array(data_R), axis=0)
        self.pub_mean_T.publish(self.mean_T)
        self.pub_mean_I.publish(self.mean_I)
        self.pub_mean_R.publish(self.mean_R)
        return True

if __name__ == '__main__':
    rospy.init_node('Tarer')
    T = Tare_values()
    rospy.spin()