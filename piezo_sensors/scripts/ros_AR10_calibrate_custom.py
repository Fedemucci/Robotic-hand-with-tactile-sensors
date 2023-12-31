#!/usr/bin/env python

from ros_ar10_class import ar10
import time

#!set the desired points of calibration for each joint
joint_targets = [[5000, 5200, 5400, 5600,],  #0
                 [7300, 7500, 7700, 7900,],  #1
                 [6000, 6500, 7000, 7500,],  #2
                 [7300, 7500, 7700, 7900,],  #3
                 [6700, 6900, 7100, 7300,],  #4
                 [6000, 6500, 7000, 7500,],  #5
                 [7300, 7500, 7700, 7900,],  #6
                 [6700, 6900, 7100, 7300,],  #7
                 [6900, 7100, 7300, 7500,],  #8
                 [6000, 6500, 7000, 7500,],] #9

def calibrate_joint(hand, joint):
    n_points = 0
    sum_x  = 0.0
    sum_y  = 0.0
    sum_xy = 0.0
    sum_xx = 0.0

    for target in joint_targets[joint]:
        hand.move(joint, target)
        hand.wait_for_hand()
        time.sleep(2.0)

        position = hand.get_read_position(joint)
        position_set = hand.get_set_position(joint)
        print (position, position_set, target)
        n_points = n_points + 1

        sum_x  = sum_x + position
        sum_y  = sum_y + target
        sum_xy = sum_xy + (position * target)
        sum_xx = sum_xx + (position * position)

    slope       = ((sum_x * sum_y) - (n_points * sum_xy)) / ((sum_x * sum_x) - (n_points * sum_xx))
    y_intercept = (sum_y - (slope * sum_x)) / n_points

    hand.move(joint, 7950)
    hand.wait_for_hand()

    if joint == 9:
        hand.move(8, 7950)
        hand.wait_for_hand()

    return y_intercept, slope

def main():
    # create hand object
    hand = ar10()

    # open calibration file
    cal_file = open("src/piezo_sensors/scripts/ros_calibration_file", "w")

    hand.open_hand()

    for joint in range(0, 10):
        y_intercept, slope = calibrate_joint(hand, joint)
        print("joint = " + str(joint) + " y intercept = " + str(y_intercept) + " slope = " + str(slope))
        cal_file.write(str(joint))
        cal_file.write("\t")
        cal_file.write(str(y_intercept))
        cal_file.write("\t")
        cal_file.write(str(slope))
        cal_file.write("\n")

    # close calibration file
    cal_file.close()

    # destroy hand object
    hand.close()

if __name__ == "__main__":
    main()


