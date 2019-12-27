#!/usr/bin/env python
from __future__ import division
import rosbag
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse, Circle, Rectangle

def plot_map(bagfile):
    bag = rosbag.Bag(bagfile)
    
    pose_gts = []
    estimates = []
    meas = []
    for topic, msg, t in bag.read_messages(topics=["/ava/pose_gt"]):
        pose_gts.append(msg)
    for topic, msg, t in bag.read_messages(topics=["/ava/estimate"]):
        estimates.append(msg)
    for topic, msg, t in bag.read_messages(topics=["/ava/measurements"]):
        meas.append(msg)

    # Associate the timestamps, should be nearly 1 for 1
    data_list = [] # [pose_gt, estimate, meas]
    estimate_index = 0
    meas_index = 0
    try: # sorry the following is unreadable, ik
        for i in range(len(pose_gts)):
            pose_time = pose_gts[i].header.stamp
            estimate_time = estimates[estimate_index].header.stamp
            meas_time = meas[meas_index].header.stamp
            while estimate_time <= pose_time:
                if estimate_time == pose_time: # found a match

                    # given that we've found our estimate for this time, let's find the measurement as well
                    while meas_time <= pose_time:
                        if meas_time == pose_time: # found a match
                            data_list.append([pose_gts[i], estimates[estimate_index], meas[meas_index]])
                        meas_index += 1
                        meas_time = meas[meas_index].header.stamp

                estimate_index += 1
                estimate_time = estimates[estimate_index].header.stamp
    except IndexError as e: # we've associated all of the data we can
        pass
    print("Num matches found: " + str(len(data_list)))

    truth_xs = [-4, -4, -3, -2, -1, 0, 1, 2, 2, 4, 4]
    truth_ys = [-2, 1, 3, -1, -3, 3, 1, 3, -2, 2, -1]
#     landmarks:
#   A: [-4, -2]
#   B: [-4, 1]
#   C: [-3, 3]
#   D: [-2, -1]
#   E: [-1, -3]
#   F: [0, 3]
#   G: [1, 1]
#   H: [2, 3]
#   I: [2, -2]
#   J: [4, 2]
#   K: [4, -1]

    # I need to plot 2 things: the truth position (black dot?) and the covariance elipse (red ellipse)
    # Now onto the covariance ellipse

    num_states = data_list[0][1].fma.layout.dim[0].size
    
    # Extract the relevant attributes from synchronized data
    # Error bounds in x and y
    sigmas = []
    for i in range(len(data_list)):

        sigma2_x = 2*np.sqrt(data_list[i][1].fma.data[num_states])
        sigma2_xy = 2*np.sqrt(data_list[i][1].fma.data[num_states+1])
        sigma2_yx = 2*np.sqrt(data_list[i][1].fma.data[2*num_states])
        sigma2_y = 2*np.sqrt(data_list[i][1].fma.data[2*num_states+1])
        sigmas.append(np.array([[sigma2_x, sigma2_xy], [sigma2_yx, sigma2_y]]))
        # print(sigmas[i])
        # print("---")
        # if i > 22:
            # return

    # Points used to generate the covariance matrix
    num_points = 500
    pts = np.linspace(0, 2*np.pi, num_points)
    x_pts = 2*np.cos(pts)
    y_pts = 2*np.sin(pts)
    circle_pts = np.append(x_pts, y_pts)
    circle_pts = circle_pts.reshape((2,x_pts.size))

    for i in range(len(data_list)):
        print("plotting index " + str(i))
        fig = plt.figure(0)
        fig.set_size_inches(14, 10)
        ax = fig.add_subplot(111, aspect='equal')

        # Truth
        x_truth = data_list[i][0].pose.pose.position.x
        y_truth = data_list[i][0].pose.pose.position.y
        ax.scatter(x_truth, y_truth, c="k", label="truth pos", s=100)
        ax.scatter(truth_xs, truth_ys, c="b", label="truth landmarks", s=100)


        # Uncertainty
        mu = np.array([data_list[i][1].fma.data[0:2]]).reshape((2,1))
        # sigmas = np.array([data_list[i][1].fma.data[2:]]).reshape((2,2))
        cov_pts = np.dot(circle_pts.T, np.sqrt(sigmas[i])).T + mu
        cov_pts = cov_pts.T
        ax.scatter(cov_pts[:,0], cov_pts[:,1], c="r", label="covariance", s=4)

        # Measurements
        data_list[i][2].fma.data = data_list[i][2].fma.data[2:] # trim the control input off of the meas array
        # print(int(len(data_list[i][2].fma.data)/3))
        # print(data_list[i][2].fma.data)
        for j in range(int(len(data_list[i][2].fma.data)/3)):
            x_pts = []
            y_pts = []
            x_pts.append(float(mu[0]))
            x_pts.append(float(data_list[i][2].fma.data[3*j+1] + mu[0] ))
            y_pts.append(float(mu[1]))
            y_pts.append(float(data_list[i][2].fma.data[3*j+2] + mu[1] ))
            l = Line2D(x_pts, y_pts, color="c")
            ax.add_artist(l)

        lim = 7
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        title = "plt" + str(i)
        plt.title(title, fontsize=30)
        plt.xlabel("x", fontsize=22)
        plt.ylabel("y", fontsize=22)
        plt.xticks(fontsize=18)
        plt.yticks(fontsize=18)
        ax.legend(fontsize=18)
        # plt.show()
        plt.savefig("plots/movie_slam/" + title)
        plt.clf()

    # for i in range(len(data_list)):
    #     print("plotting index " + str(i))
    #     fig = plt.figure(0)
    #     fig.set_size_inches(14, 10)
    #     ax = fig.add_subplot(111, aspect='equal')
    #     x_truth = data_list[i][0].pose.pose.position.x
    #     y_truth = data_list[i][0].pose.pose.position.y
    #     ax.scatter(x_truth, y_truth, c="k", label="truth", s=300)
    #     ax.set_xlim(-7, 7)
    #     ax.set_ylim(-7, 7)
    #     title = "plt" + str(i)
    #     plt.title(title, fontsize=30)
    #     plt.xlabel("x", fontsize=22)
    #     plt.ylabel("y", fontsize=22)
    #     plt.xticks(fontsize=18)
    #     plt.yticks(fontsize=18)
    #     # plt.show()
    #     ax.legend(fontsize=18)
    #     plt.savefig("plots/movie/" + title)
    #     plt.clf()
    
    # Extract the relevant attributes from synchronized data
    # Error bounds in x and y
    # error_x = []
    # error_y = []
    # sigma2_x = []
    # sigma2_y = []
    # for i in range(len(data_list)):
    #     error_x.append(data_list[i][0].pose.pose.position.x - data_list[i][1].fma.data[0])
    #     error_y.append(data_list[i][0].pose.pose.position.y - data_list[i][1].fma.data[1])
    #     sigma2_x.append(2*np.sqrt(data_list[i][1].fma.data[2]))
    #     sigma2_y.append(2*np.sqrt(data_list[i][1].fma.data[5]))
    


# Needs /ava/pose_gt and /ava/estimate
def plot_error_bounds(bagfile):
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

    num_states = data_list[0][1].fma.layout.dim[0].size
    
    # Extract the relevant attributes from synchronized data
    # Error bounds in x and y
    error_x = []
    error_y = []
    sigma2_x = []
    sigma2_y = []
    for i in range(len(data_list)):
        error_x.append(data_list[i][0].pose.pose.position.x - data_list[i][1].fma.data[0])
        error_y.append(data_list[i][0].pose.pose.position.y - data_list[i][1].fma.data[1])

        sigma2_x.append(2*np.sqrt(data_list[i][1].fma.data[num_states]))
        sigma2_y.append(2*np.sqrt(data_list[i][1].fma.data[2*num_states+1]))
        # print(sigma2_x)
        # print(sigma2_y)
        # print("---")

    # Plot the x_error
    # fig = plt.figure(0)
    # fig.set_size_inches(14, 10)
    # plt.plot(error_x, color="r")
    # plt.plot(sigma2_x, color="g")
    # neg_sigma2_x = [-x for x in sigma2_x]
    # plt.plot(neg_sigma2_x, color="g")
    # plt.title("Error in x vs time", fontsize=22)
    # plt.xlabel("Index in time", fontsize=20)
    # plt.ylabel("Error in x", fontsize=20)
    # plt.xticks(fontsize=18)
    # plt.yticks(fontsize=18)
    # plt.savefig("plots/1_kalman_error_x_slam")


    # Plot the y_error
    fig = plt.figure(0)
    fig.set_size_inches(14, 10)
    plt.plot(error_y, color="r")
    plt.plot(sigma2_y, color="g")
    neg_sigma2_y = [-y for y in sigma2_y]
    plt.plot(neg_sigma2_y, color="g")
    plt.title("Error in y vs time", fontsize=22)
    plt.xlabel("Index in time", fontsize=20)
    plt.ylabel("Error in y", fontsize=20)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    # plt.savefig("plots/1_kalman_error_y_slam")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag file path")
    args = parser.parse_args()
    print(args.bag)

    # plot_error_bounds(args.bag)
    plot_map(args.bag)