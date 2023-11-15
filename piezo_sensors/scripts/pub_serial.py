#!/usr/bin/env python

import rospy
import serial
import struct
from std_msgs.msg import Float64MultiArray
from piezo_sensors.srv import Tarer
import numpy as np

class Pub_3_Finger:
        def __init__(self):
            self.pub_T= rospy.Publisher('Thumb', Float64MultiArray, queue_size=10)
            self.pub_I= rospy.Publisher('Index', Float64MultiArray, queue_size=10)
            self.pub_R= rospy.Publisher('Ring', Float64MultiArray, queue_size=10)
            self.sub_mean_T = rospy.Subscriber('Mean_T', Float64MultiArray, self.callback_T)
            self.sub_mean_I = rospy.Subscriber('Mean_I', Float64MultiArray, self.callback_I)
            self.sub_mean_R = rospy.Subscriber('Mean_R', Float64MultiArray, self.callback_R)
            self.ser = serial.Serial('/dev/ttyUSB0', 1000000)   #!change with the port to which the sensors are connected
            self.header= []
            self.header.append('\x3C')
            self.header.append('\x3E')
            self.header.append('\x00')
            self.msg_T = Float64MultiArray()
            self.msg_I = Float64MultiArray()
            self.msg_R = Float64MultiArray()
            self.data = np.zeros((3,8))
            self.mean_T = np.zeros(8)
            self.mean_I = np.zeros(8)
            self.mean_R = np.zeros(8)

        def callback_T(self, msg):
            self.mean_T = np.array(msg.data)
        
        def callback_I(self, msg):
            self.mean_I = np.array(msg.data)

        def callback_R(self, msg):
            self.mean_R = np.array(msg.data)

        def run(self):
            self.detect_header()
            while not rospy.is_shutdown():
                    msg = self.read()
                    self.msg_T.data = msg[1,:] - self.mean_T
                    self.msg_I.data = msg[0,:] - self.mean_I
                    self.msg_R.data = msg[2,:] - self.mean_R
                    self.pub_T.publish(self.msg_T)
                    self.pub_I.publish(self.msg_I)
                    self.pub_R.publish(self.msg_R)

        def read(self):
            rawdata = self.ser.read(51)
            for j in range(0, 3):
                for k in range(0, 8):
                    self.data[j, k]= (struct.unpack('>H',rawdata[(j*8+k)*2+3]+rawdata[(j*8+k)*2+4])[0])
            return self.data
        
        def detect_header(self):
            flag=0
            i=0
            while flag==0:
                value=self.ser.read()
                if value==self.header[i]:
                    i=i+1
                else:
                    i=0
                if i==3:
                    flag=1
            trash=self.ser.read(48)

if __name__ == '__main__':
    rospy.init_node('pub_serial')
    pub= Pub_3_Finger()
    pub.run()
