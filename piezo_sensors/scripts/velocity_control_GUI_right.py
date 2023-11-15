#!/usr/bin/env python

from ros_ar10_class import ar10
import numpy as np
import rospy
import time
import random
import Tkinter as tk

from std_msgs.msg import Float64

class VelocityControl(object):
    def __init__(self):
        self.rate = rospy.Rate(0.2)
        self.hand = ar10()

        self.open_position = np.array([6000, 7900, 7500, 7900, 7900, 7900, 5000, 7900, 5000, 7900])
        self.close_position = np.array([6000, 6500, 7500, 7900, 7900, 7900, 5000, 6500, 5000, 6500])

        self.full_open = np.array([8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000])

        self.thumb_second_open_position = tk.StringVar()
        self.thumb_second_close_position = tk.StringVar()
        self.thumb_first_open_position = tk.StringVar()
        self.thumb_first_close_position = tk.StringVar()

        self.index_second_open_position = tk.StringVar()
        self.index_second_close_position = tk.StringVar()
        self.index_first_open_position = tk.StringVar()
        self.index_first_close_position = tk.StringVar()

        self.annular_second_open_position = tk.StringVar()
        self.annular_second_close_position = tk.StringVar()
        self.annular_first_open_position = tk.StringVar()
        self.annular_first_close_position = tk.StringVar()

        self.velocity = tk.StringVar()

        self.thumb_second_idx = 1
        self.index_second_idx = 9
        self.annular_second_idx = 5

        self.thumb_first_idx = 0
        self.index_first_idx = 8
        self.annular_first_idx = 4

    def initialization(self):
        for ii in range(10):
            self.hand.move(ii, self.open_position[ii])

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

    def change_close_position(self, close_position):
        self.close_position = close_position

    def change_open_position(self, open_position):
        self.open_position = open_position

    def move_finger(self, finger, value):
        self.hand.move(finger, int(value))

    def open_finger(self, finger, value):
        self.hand.move(finger, int(self.open_position[finger]))

    def open_thumb(self):
        self.move_finger(self.thumb_second_idx, self.thumb_second_open_position.get())
        self.move_finger(self.thumb_first_idx, self.thumb_first_open_position.get())

    def close_thumb(self):
        self.move_finger(self.thumb_second_idx, self.thumb_second_close_position.get())
        self.move_finger(self.thumb_first_idx, self.thumb_first_close_position.get())

    def open_index(self):
        self.move_finger(self.index_second_idx, self.index_second_open_position.get())
        self.move_finger(self.index_first_idx, self.index_first_open_position.get())

    def close_index(self):
        self.move_finger(self.index_second_idx, self.index_second_close_position.get())
        self.move_finger(self.index_first_idx, self.index_first_close_position.get())
    
    def open_annular(self):
        self.move_finger(self.annular_second_idx, self.annular_second_open_position.get())
        self.move_finger(self.annular_first_idx, self.annular_first_open_position.get())

    def close_annular(self):
        self.move_finger(self.annular_second_idx, self.annular_second_close_position.get())
        self.move_finger(self.annular_first_idx, self.annular_first_close_position.get())

    def open_triple(self):
        self.open_thumb()
        self.open_index()
        self.open_annular()

    def close_triple(self):
        self.close_thumb()
        self.close_index()
        self.close_annular()

    def print_position(self):
        print(self.close_position)

    def update_velocity(self):
        self.hand.change_speed(int(self.velocity.get()))
        for ii in range(10):
            self.hand.set_speed(ii)

    def stop(self):
        intercept = [8000 for ii in range(10)]
        slope = [-6 for ii in range(10)]
        for ii in range(10):
            current = self.hand.get_read_position(ii)
            current = slope[ii] * current + intercept[ii]
            self.hand.move(ii, current)
        self.update_velocity()
        
    def run(self):
        self.initialization()
        while not rospy.is_shutdown():
            #alternate open and close
            self.hand_move(self.open_position)
            self.rate.sleep()
            self.hand_move(self.close_position)
            self.rate.sleep()

class GUI(object):
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("AR10 Controller")
        self.root.geometry("500x1100")

        self.controller = VelocityControl()
        self.controller.full_initialization()

        ############################################################################################################
        #Velocity
        self.velocity_label = tk.Label(self.root, text="Velocity (1-20)")
        self.velocity_label.pack()
        self.velocity = tk.Entry(self.root, textvariable=self.controller.velocity)
        self.velocity.insert(0, "20")
        self.velocity.pack()

        self.button = tk.Button(self.root, text="Update Velocity", command=self.controller.update_velocity)
        self.button.pack()

        self.stop_button = tk.Button(self.root, text="Stop", command=self.controller.stop)
        self.stop_button.pack()
        ############################################################################################################

        self.space0_back = tk.Label(self.root, text="")
        self.space0_back.pack()

        self.space0 = tk.Label(self.root, text="---------------------------------------------------------------------")
        self.space0.pack()

        self.space0_front = tk.Label(self.root, text="")
        self.space0_front.pack()

        ############################################################################################################
        #THUMB
        # open first position entry label
        self.open_position_label = tk.Label(self.root, text="Thumb First Joint Open Position")
        self.open_position_label.pack()
        self.open_position = tk.Entry(self.root, textvariable=self.controller.thumb_first_open_position)
        self.open_position.insert(0, "7900")
        self.open_position.pack()

        # close first position entry label
        self.close_position_label = tk.Label(self.root, text="Thumb First Joint Close Position")
        self.close_position_label.pack()
        self.close_position = tk.Entry(self.root, textvariable=self.controller.thumb_first_close_position)
        self.close_position.insert(0, "6500")
        self.close_position.pack()

        # open second position entry label
        self.open_position_label = tk.Label(self.root, text="Thumb Second Joint Open Position")
        self.open_position_label.pack()
        self.open_position = tk.Entry(self.root, textvariable=self.controller.thumb_second_open_position)
        self.open_position.insert(0, "7900")
        self.open_position.pack()

        # close second position entry label 
        self.close_position_label = tk.Label(self.root, text="Thumb Second Joint Close Position")
        self.close_position_label.pack()
        self.close_position = tk.Entry(self.root, textvariable=self.controller.thumb_second_close_position)
        self.close_position.insert(0, "6500")
        self.close_position.pack()

        self.button = tk.Button(self.root, text="Open Thumb", command=self.controller.open_thumb)
        self.button.pack()

        self.button = tk.Button(self.root, text="Close Thumb", command=self.controller.close_thumb)
        self.button.pack()
        ############################################################################################################

        self.space1_back = tk.Label(self.root, text="")
        self.space1_back.pack()

        self.space1 = tk.Label(self.root, text="---------------------------------------------------------------------")
        self.space1.pack()

        self.space1_front = tk.Label(self.root, text="")
        self.space1_front.pack()

        ############################################################################################################
        #INDEX
        # open first position entry label
        self.open_position_label = tk.Label(self.root, text="Index First Joint Open Position")
        self.open_position_label.pack()
        self.open_position = tk.Entry(self.root, textvariable=self.controller.index_first_open_position)
        self.open_position.insert(0, "7900")
        self.open_position.pack()

        # close first position entry label
        self.close_position_label = tk.Label(self.root, text="Index First Joint Close Position")
        self.close_position_label.pack()
        self.close_position = tk.Entry(self.root, textvariable=self.controller.index_first_close_position)
        self.close_position.insert(0, "6500")
        self.close_position.pack()

        # open second position entry label
        self.open_position_label = tk.Label(self.root, text="Index Second Joint Open Position")
        self.open_position_label.pack()
        self.open_position = tk.Entry(self.root, textvariable=self.controller.index_second_open_position)
        self.open_position.insert(0, "7900")
        self.open_position.pack()

        # close second position entry label
        self.close_position_label = tk.Label(self.root, text="Index Second Joint Close Position")
        self.close_position_label.pack()
        self.close_position = tk.Entry(self.root, textvariable=self.controller.index_second_close_position)
        self.close_position.insert(0, "6500")
        self.close_position.pack()

        self.button = tk.Button(self.root, text="Open Index", command=self.controller.open_index)
        self.button.pack()

        self.button = tk.Button(self.root, text="Close Index", command=self.controller.close_index)
        self.button.pack()
        ############################################################################################################

        self.space2_back = tk.Label(self.root, text="")
        self.space2_back.pack()

        self.space2 = tk.Label(self.root, text="---------------------------------------------------------------------")
        self.space2.pack()

        self.space2_front = tk.Label(self.root, text="")
        self.space2_front.pack()

        ############################################################################################################
        #annular
        # open first position entry label
        self.open_position_label = tk.Label(self.root, text="annular First Joint Open Position")
        self.open_position_label.pack()
        self.open_position = tk.Entry(self.root, textvariable=self.controller.annular_first_open_position)
        self.open_position.insert(0, "7900")
        self.open_position.pack()

        # close first position entry label
        self.close_position_label = tk.Label(self.root, text="annular First Joint Close Position")
        self.close_position_label.pack()
        self.close_position = tk.Entry(self.root, textvariable=self.controller.annular_first_close_position)
        self.close_position.insert(0, "6500")
        self.close_position.pack()

        # open second position entry label
        self.open_position_label = tk.Label(self.root, text="annular Second Joint Open Position")
        self.open_position_label.pack()
        self.open_position = tk.Entry(self.root, textvariable=self.controller.annular_second_open_position)
        self.open_position.insert(0, "7900")
        self.open_position.pack()

        # close second position entry label
        self.close_position_label = tk.Label(self.root, text="annular Second Joint Close Position")
        self.close_position_label.pack()
        self.close_position = tk.Entry(self.root, textvariable=self.controller.annular_second_close_position)
        self.close_position.insert(0, "6500")
        self.close_position.pack()

        self.button = tk.Button(self.root, text="Open annular", command=self.controller.open_annular)
        self.button.pack()

        self.button = tk.Button(self.root, text="Close annular", command=self.controller.close_annular)
        self.button.pack()
        ############################################################################################################

        self.space3_back = tk.Label(self.root, text="")
        self.space3_back.pack()

        self.space3 = tk.Label(self.root, text="---------------------------------------------------------------------")
        self.space3.pack()

        self.space3_front1 = tk.Label(self.root, text="")
        self.space3_front1.pack()

        self.space3_front2 = tk.Label(self.root, text="")
        self.space3_front2.pack()

        self.space3_front3 = tk.Label(self.root, text="")
        self.space3_front3.pack()

        ############################################################################################################
        #OPEN HAND
        self.button = tk.Button(self.root, text="Open All Fingers", command=self.controller.open_triple)
        self.button.pack()

        self.space4_back = tk.Label(self.root, text="")
        self.space4_back.pack()

        self.button = tk.Button(self.root, text="Close All Fingers", command=self.controller.close_triple)
        self.button.pack()


        self.root.mainloop()

    def print_value(self):
        print(self.open_value.get())
        print(self.close_value.get())


if __name__ == '__main__':
    
    rospy.init_node('velocity_controller', anonymous=True)
    gui = GUI()
