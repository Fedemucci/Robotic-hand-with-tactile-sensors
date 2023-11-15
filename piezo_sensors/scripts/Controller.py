#!/usr/bin/env python

import rospy
from piezo_sensors.srv import Record_start, Record_stop, Grasp_init, Grasp_routine, Tarer
import time
import os
import yaml


class Controller:
    def __init__(self):
        self.start_recording = rospy.ServiceProxy('start_recording', Record_start)
        self.stop_recording = rospy.ServiceProxy('stop_recording', Record_stop)
        self.grasp_init_client= rospy.ServiceProxy('grasp_initialization', Grasp_init)
        self.grasp_client= rospy.ServiceProxy('grasp_routine', Grasp_routine)
        self.tarer_client= rospy.ServiceProxy('tare', Tarer)
        #
        self.vel_steps = [5,10,20]                              #! velocity steps for the grasping routine 
        #                 Thumb     Index     Ring
        self.qi_set =  [   0,1,      8,9,      4,5   ]          #! joints to be set in open position
        self.q_set =   [5530,8000,6800,7900,7000,7800]          #! values of the joints in open position    
        self.qi_move = [    1,        9,        5    ]          #! joints to be moved to close position during grasping
        self.q_max =   [   7300,     6750,     6800  ]          #! max value of the joints in close position during grasping
        self.q_depth = 100                                      #! offset subtracted to the max value of the joints in close position ...
        self.dataset_dir = ''                                   # ... to achieve different pressure levels                                        
        #
    def dataset_folder(self, case):
        p_qi_set = []
        p_q_set = []
        p_qi_move = []
        p_q_max = []
        finger = ['T','I','R']
        p_finger = []
        for i in range(0,len(case)):
            if case[i]:
                p_finger.append(finger[i])
                p_qi_set.append(self.qi_set[2*i])
                p_qi_set.append(self.qi_set[2*i+1])
                p_q_set.append(self.q_set[2*i])
                p_q_set.append(self.q_set[2*i+1])
                p_qi_move.append(self.qi_move[i])
                p_q_max.append(self.q_max[i])
        params = {'finger': p_finger,
                  'vel_steps': self.vel_steps,
                  'qi_set': p_qi_set,
                  'q_set': p_q_set,
                  'qi_move': p_qi_move,
                  'q_max': p_q_max,
                  'q_depth': self.q_depth}
        #
        self.dataset_dir = '/home/lar03/ros/AR10_piezosensors_ws//Dataset'       #change with the path of the directory where to save the dataset
        #
        self.dataset_dir= self.dataset_dir+'/Dataset'
        for i in range (0, len (p_finger)):
            self.dataset_dir = self.dataset_dir +'_'+p_finger[i]+'_'+str(p_q_set[2*i])+'_'+str(p_q_set[2*i+1])+'_'+str(p_q_max[i])
        flag=0
        c=0
        dir_0 = self.dataset_dir
        while flag==0:
            c=c+1
            if not os.path.exists(self.dataset_dir):
                os.makedirs(self.dataset_dir)
                flag=1
            else:
                self.dataset_dir = dir_0 + '_'+str(c)
        with open(os.path.join(self.dataset_dir,'config.yaml'), 'w') as f:
            yaml.dump(params, f)

    def run(self, case):
        self.dataset_folder(case)
        rospy.wait_for_service('grasp_initialization')
        self.grasp_init_client(self.qi_set, self.q_set)
        time.sleep(2)
        rospy.wait_for_service('tare')
        self.tarer_client(3500)
        time.sleep(1)
        rospy.wait_for_service('start_recording')
        self.start_recording(case,self.qi_move, self.dataset_dir)
        time.sleep(1)
        rospy.wait_for_service('grasp_routine')
        self.grasp_client(case, self.vel_steps, self.qi_move, self.q_max, self.q_depth)
        time.sleep(1)
        rospy.wait_for_service('stop_recording')
        self.stop_recording()

if __name__ == '__main__':
    rospy.init_node('Controller')
    Ctrl = Controller()
#             T I R
    Ctrl.run([1,1,1])               # select finger (1: active, 0: inactive)
# 