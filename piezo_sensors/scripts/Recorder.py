#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Int16
from piezo_sensors.srv import Record_start, Record_stop
import numpy as np
import os

class Record_datas:
    def __init__(self):
        self.store_T = rospy.Subscriber('Thumb', Float64MultiArray, self.callback_T)
        self.store_I = rospy.Subscriber('Index', Float64MultiArray, self.callback_I)  
        self.store_R = rospy.Subscriber('Ring', Float64MultiArray, self.callback_R)
        self.store_set_T = rospy.Subscriber('Set_T', Float64MultiArray, self.callback_set_T)
        self.store_pos_T = rospy.Subscriber('Pos_T', Float64MultiArray, self.callback_pos_T)
        self.store_set_I = rospy.Subscriber('Set_I', Float64MultiArray, self.callback_set_I)
        self.store_pos_I = rospy.Subscriber('Pos_I', Float64MultiArray, self.callback_pos_I)
        self.store_set_R = rospy.Subscriber('Set_R', Float64MultiArray, self.callback_set_R)
        self.store_pos_R = rospy.Subscriber('Pos_R', Float64MultiArray, self.callback_pos_R)
        self.store_Velocity = rospy.Subscriber('Velocity', Int16, self.callback_Velocity)
        self.start_service = rospy.Service('start_recording', Record_start, self.handle_start)
        self.stop_service = rospy.Service('stop_recording', Record_stop, self.handle_stop)
        self.save_dir = ''
        self.save_what = []
        self.T = []
        self.I = []
        self.R = []        
        self.data_T=[]
        self.data_I=[]
        self.data_R=[]
        self.set_T = []
        self.pos_T = []
        self.set_I = []
        self.pos_I = []
        self.set_R = []
        self.pos_R = []
        self.velocity = []
        self.list_set_T=[]
        self.list_pos_T=[]
        self.list_set_I=[]
        self.list_pos_I=[]
        self.list_set_R=[]
        self.list_pos_R=[]
        self.list_velocity=[]
        self.recording= False
        self.start_recording= False
        self.new_message_T = False
        self.new_message_I = False
        self.new_message_R = False

    def callback_T(self, msg):
        self.T = msg.data
        self.new_message_T = True

    def callback_I(self, msg):
        self.I = msg.data
        self.new_message_I = True

    def callback_R(self, msg):
        self.R = msg.data
        self.new_message_R = True

    def callback_set_T(self, msg):
        self.set_T = msg.data
    
    def callback_pos_T(self, msg):
        self.pos_T = msg.data
    
    def callback_set_I(self, msg):
        self.set_I = msg.data

    def callback_pos_I(self, msg):
        self.pos_I = msg.data
    
    def callback_set_R(self, msg):
        self.set_R = msg.data
    
    def callback_pos_R(self, msg):
        self.pos_R = msg.data
    
    def callback_Velocity(self, msg):
        self.velocity = msg.data

    def handle_start(self, req):
        self.save_dir = req.dir
        self.save_what = req.case
        self.start_recording = True
        return True
    
    def handle_stop(self, req):
        self.start_recording = False
        return True

    def store(self):
        self.data_T.append(self.T)
        self.data_I.append(self.I)
        self.data_R.append(self.R)
        self.list_set_T.append(self.set_T)
        self.list_pos_T.append(self.pos_T)
        self.list_set_I.append(self.set_I)
        self.list_pos_I.append(self.pos_I)
        self.list_set_R.append(self.set_R)
        self.list_pos_R.append(self.pos_R)
        self.list_velocity.append(self.velocity)
        self.new_message_T = False
        self.new_message_I = False
        self.new_message_R = False

    def save(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        if self.save_what[0]:
            np.save(os.path.join(self.save_dir, 'Thumb.npy'), self.data_T)
            np.save(os.path.join(self.save_dir, 'Set_T.npy'), self.list_set_T)
            np.save(os.path.join(self.save_dir, 'Pos_T.npy'), self.list_pos_T) 
        if self.save_what[1]:
            np.save(os.path.join(self.save_dir, 'Index.npy'), self.data_I)
            np.save(os.path.join(self.save_dir, 'Set_I.npy'), self.list_set_I)
            np.save(os.path.join(self.save_dir, 'Pos_I.npy'), self.list_pos_I)
        if self.save_what[2]:
            np.save(os.path.join(self.save_dir, 'Ring.npy'), self.data_R)
            np.save(os.path.join(self.save_dir, 'Set_R.npy'), self.list_set_R)
            np.save(os.path.join(self.save_dir, 'Pos_R.npy'), self.list_pos_R)
        np.save(os.path.join(self.save_dir, 'Velocity.npy'), self.list_velocity)

    def run(self):
        while not rospy.is_shutdown():
            if self.start_recording:
                if self.new_message_T and self.new_message_I and self.new_message_R:
                    self.store()
            else:
                if len(self.data_T) != 0:
                    self.save()
                    self.data_T=[]
                    self.data_I=[]
                    self.data_R=[]
                    self.list_set_T=[]
                    self.list_pos_T=[]
                    self.list_set_I=[]
                    self.list_pos_I=[]
                    self.list_set_R=[]
                    self.list_pos_R=[]
                    self.list_velocity=[]

if __name__ == '__main__':
    rospy.init_node('Recorder')
    RD = Record_datas()
    RD.run()

    

