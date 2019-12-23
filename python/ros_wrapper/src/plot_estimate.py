#!/usr/bin/env python
from __future__ import division
import rosbag
import argparse
import numpy as np
import matplotlib.pyplot as plt

def synchronize_data(bagfile):
    bag = rosbag.Bag(bagfile)
    
    pose_gts = []
    estimates = []
    for topic, msg, t in bag.read_messages(topics=["/ava/pose_gt"]):
        pose_gts.append(msg)
    for topic, msg, t in bag.read_messages(topics=["/ava/estimate"]):
        estimates.append(msg)

    # Associate the timestamps, should be nearly 1 for 1
    data_list = [] # [pose_gt, estimate]
    estimate_index = 0
    try:
        for i in range(len(pose_gts)):
            pose_time = pose_gts[i].header.stamp
            estimate_time = estimates[estimate_index].header.stamp
            while estimate_time <= pose_time:
                if estimate_time == pose_time: # found a match
                    data_list.append([pose_gts[i], estimates[estimate_index]])
                estimate_index += 1
                estimate_time = estimates[estimate_index].header.stamp
    except IndexError as e: # we've associated all of the data we can
        pass
    print("Num matches found: " + str(len(data_list)))
    
    # Extract the relevant attributes from synchronized data
    # Error bounds in x and y
    error_x = []
    error_y = []
    sigma2_x = []
    sigma2_y = []
    for i in range(len(data_list)):
        error_x.append(data_list[i][0].pose.pose.position.x - data_list[i][1].fma.data[0])
        error_y.append(data_list[i][0].pose.pose.position.y - data_list[i][1].fma.data[1])
        sigma2_x.append(2*np.sqrt(data_list[i][1].fma.data[2]))
        sigma2_y.append(2*np.sqrt(data_list[i][1].fma.data[5]))

    # Plot the x_error
    # plt.plot(error_x, color="r")
    # plt.plot(sigma2_x, color="g")
    # neg_sigma2_x = [-x for x in sigma2_x]
    # plt.plot(neg_sigma2_x, color="g")
    # plt.title("Error in x vs time", fontsize=22)
    # plt.xlabel("Index in time", fontsize=20)
    # plt.ylabel("Error in x", fontsize=20)
    # plt.xticks(fontsize=18)
    # plt.yticks(fontsize=18)
    # plt.savefig("plots/1_kalman_error_x")
    # plt.show()

    # Plot the y_error
    plt.plot(error_y, color="r")
    plt.plot(sigma2_y, color="g")
    neg_sigma2_y = [-y for y in sigma2_y]
    plt.plot(neg_sigma2_y, color="g")
    plt.title("Error in y vs time", fontsize=22)
    plt.xlabel("Index in time", fontsize=20)
    plt.ylabel("Error in y", fontsize=20)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    plt.savefig("plots/1_kalman_error_y")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag file path")
    args = parser.parse_args()
    print(args.bag)
    synchronize_data(args.bag)