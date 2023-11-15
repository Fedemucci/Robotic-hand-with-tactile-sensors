#!/usr/bin/env python

import rospy
from ros_ar10_class import ar10
from piezo_sensors.srv import Grasp_routine, Grasp_init
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, Int16

class Hand(object):
    def __init__(self):
        self.rate = rospy.Rate(100)
        self.hand = ar10()
        ##                                   thumb,      pinky,      ring,       middle,      index   
        self.full_open          = np.array([5400, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000])
        self.open_position      = np.array([5400, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000])
        self.close_position     = np.array([5400, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000])
        self.joints= [0,1,8,9,4,5]
        self.vel = 1
        self.grasp_init_service= rospy.Service('grasp_initialization', Grasp_init, self.handle_grasp_initialization)
        self.grasp_service= rospy.Service('grasp_routine', Grasp_routine, self.handle_grasp_routine)
        self.pub_set_T= rospy.Publisher('Set_T', Float64MultiArray, queue_size=10)
        self.pub_pos_T= rospy.Publisher('Pos_T', Float64MultiArray, queue_size=10)
        self.pub_set_I= rospy.Publisher('Set_I', Float64MultiArray, queue_size=10)
        self.pub_pos_I= rospy.Publisher('Pos_I', Float64MultiArray, queue_size=10)
        self.pub_set_R= rospy.Publisher('Set_R', Float64MultiArray, queue_size=10)
        self.pub_pos_R= rospy.Publisher('Pos_R', Float64MultiArray, queue_size=10)
        self.pub_velocity= rospy.Publisher('Velocity', Int16, queue_size=10)
        self.set_T_msg= Float64MultiArray()
        self.pos_T_msg= Float64MultiArray()
        self.set_I_msg= Float64MultiArray()
        self.pos_I_msg= Float64MultiArray()
        self.set_R_msg= Float64MultiArray()
        self.pos_R_msg= Float64MultiArray()
        self.velocity = Int16()

    def full_initialization(self):
        for ii in range(10):
            self.hand.move(ii, self.full_open[ii])

    def hand_move(self, target):
        for ii in range(10):
            self.hand.move(ii, int(target[ii]))

    def open_hand(self):
        self.hand_move(self.open_position)

    def close_hand(self):
        self.hand_move(self.close_position)

    def update_velocity(self): #1-20
        self.hand.change_speed(self.vel)
        for ii in range(10):
            self.hand.set_speed(ii)

    def publish(self):
        set_position = []
        position = []
        self.velocity.data = self.vel

        for j in self.joints:
            set_position.append(self.hand.get_set_position(j))
            position.append(self.hand.get_position(j))
        self.set_T_msg.data= set_position[0:2]
        self.pos_T_msg.data= position[0:2]
        self.set_I_msg.data= set_position[2:4]
        self.pos_I_msg.data= position[2:4]
        self.set_R_msg.data= set_position[4:6]
        self.pos_R_msg.data= position[4:6]
        self.pub_set_T.publish(self.set_T_msg)
        self.pub_pos_T.publish(self.pos_T_msg)
        self.pub_set_I.publish(self.set_I_msg)
        self.pub_pos_I.publish(self.pos_I_msg)
        self.pub_set_R.publish(self.set_R_msg)
        self.pub_pos_R.publish(self.pos_R_msg)
        self.pub_velocity.publish(self.velocity) 

    def handle_grasp_initialization(self, req):
        self.full_initialization()
        time.sleep(2)
        for i in range(0,len(req.qi_set)):
            self.open_position[req.qi_set[i]]= req.q_set[i]
            self.close_position[req.qi_set[i]]= req.q_set[i]
        self.open_hand()
        time.sleep(2)
        return True
    
    def handle_grasp_routine(self, req):
        for vel in req.vel_steps:
            self.vel = vel
            self.update_velocity()
            for j in range(0,3):
                for i in range(0,len(req.qi_move)):
                    if req.case[i]:
                        self.close_position[req.qi_move[i]]= req.q_max[i]+req.q_depth*j
                self.close_hand()
                time.sleep(4.5+7/vel)    
                self.open_hand()
                time.sleep(4.5+7/vel)
        return True

    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Hand')
    HD = Hand()
    HD.run()