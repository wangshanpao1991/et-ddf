#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
from matplotlib.patches import Ellipse, Circle, Rectangle
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Pose, PoseArray
import os
import time
import argparse

class Plotter:

    def __init__(self):        
        self.landmark_debug_pub = rospy.Publisher("landmark_poses/", PoseArray, queue_size=10)
        self.meas_pub = rospy.Publisher("/ava/measurements", Float32MultiArray, queue_size=10)
        self.robot_point_gt = None
        self.pose_estimate = None
        self.landmarks = {}
        param_names = rospy.get_param_names()
        for p in param_names:
            if "landmarks" in p:
                pt = Point()
                xy_list = rospy.get_param(p)
                pt.x = xy_list[0]
                pt.y = xy_list[1]
                self.landmarks[p[-1]] = pt
        self.sensor_range = int(rospy.get_param("sensor_range"))
        self.sensor_noise = rospy.get_param("sensor_noise")
        self.plot_path = "/tmp/plots"
        self.create_plot_dir()
        self.seq = 0

        # Subscribe inputs
        rospy.Subscriber("/ava/pose_gt", Odometry, self.pose_gt_callback)
        rospy.wait_for_message("/ava/pose_gt", Odometry)
        rospy.wait_for_message("/ava/pose_gt", Odometry)
        rospy.Subscriber("/ava/estimate", Float32MultiArray, self.pose_estimate_callback, queue_size=1)

    def create_plot_dir(self):
        try:
            os.mkdir(self.plot_path)
        except OSError:
            print("Unable to create directory " + self.plot_path)

    def pose_gt_callback(self, msg):
        self.robot_point_gt = msg.pose.pose.position

        f = Float32MultiArray()
        robot_arr = np.array([[self.robot_point_gt.x, self.robot_point_gt.y]]).T
        for k in self.landmarks.keys():
            lmrk = self.landmarks[k]
            lmrk_arr = np.array([[lmrk.x, lmrk.y]]).T
            diff = lmrk_arr - robot_arr
            if( np.linalg.norm(diff) <= self.sensor_range):
                f.data.append(ord(k))
                f.data.append(diff[0] + np.random.normal(0, self.sensor_noise))
                f.data.append(diff[1] + np.random.normal(0, self.sensor_noise))
        self.meas_pub.publish(f)

    def pose_estimate_callback(self, msg):
        data = msg.data
        num_states = msg.layout.dim[0].size
        mu = np.array([[data[:num_states]]]).T
        # print(mu)
        sigma = np.array([data[num_states:]]).reshape((num_states, num_states))
        self.publish_debug_landmarks()
        if self.seq % 20 == 0:
            self.plot_data(mu, sigma)
        self.seq += 1

    def publish_debug_landmarks(self):
        pa = PoseArray()
        pa.header.frame_id = "world"
        for k in self.landmarks.keys():
            pose = Pose()
            pose.orientation.z = 0.707
            pose.orientation.w = 0.707
            pose.position = self.landmarks[k]
            pa.poses.append(pose)
        self.landmark_debug_pub.publish(pa)

    def plot_data(self, mu, sigma):
        fig = plt.figure(0)
        ax = fig.add_subplot(111, aspect='equal')

        # Plot true robot + landmark positions
        for k in self.landmarks.keys():
            pt = self.landmarks[k]
            pt_arr = np.array([[pt.x,pt.y]]).T
            c = Circle(pt_arr, 0.25, color="k")
            ax.add_artist(c)

        # Plot mu with sigma (estimated positions + uncertainty)
        [eig_vals, eig_vectors] = np.linalg.eig(sigma)
        max_index = np.argmax(eig_vals)
        large_eig_val = eig_vals[max_index]
        small_eig_val = eig_vals[max_index ^ 1] # xor to flip the bit
        theta = -np.arctan2(eig_vectors[max_index,1], eig_vectors[max_index,0])
        width = np.sqrt(5.991*large_eig_val)
        height = np.sqrt(5.991*small_eig_val)
        e = Ellipse(xy=mu, width=width, height=height, angle=theta *(180/np.pi), color="g")
        # c = Circle(mu, 0.25, color="g")
        ax.add_artist(e)

        pt_arr = np.array([[self.robot_point_gt.x, self.robot_point_gt.y]]).T
        r = Rectangle(pt_arr, 0.25, 0.25, color="k")
        ax.add_artist(r)

        ax.set_xlim(-7, 7)
        ax.set_ylim(-7, 7)
        plt.savefig(self.plot_path + "/plt" + str(self.seq) + ".png")
        plt.clf()
        # plt.show()

        # plot mean estimate (w/ covariance)
        # plot estimates of all landmarks

def create_plot_dir2(path):
    try:
        os.mkdir(path)
    except OSError:
        print("Unable to create directory " + path)

def test_plotting():
    path = "/tmp/plots"
    create_plot_dir2(path)

    mu = np.zeros((2,1))
    sigma = np.array([[2, 0], [0, 1]])
    # sigma = np.array([[1, 1], [1, 4]])
    # sigma = np.array([[4.25, 3.10], [3.10, 4.29]])
    # sigma = np.array([[4.25, -3.10], [-3.10, 4.29]])
    print('original cov matrix')
    print(sigma)

    s = 2
    [eig_vals, eig_vectors] = np.linalg.eig(sigma)
    max_index = np.argmax(eig_vals)
    large_eig_val = eig_vals[max_index]
    small_eig_val = eig_vals[max_index ^ 1] # xor to flip the bit

    print('\neigenvalues')
    print(eig_vals)
    print('\neigenvectors')
    print(eig_vectors)
 
    theta = -np.arctan2(eig_vectors[max_index,1], eig_vectors[max_index,0])
    print("\ntheta")
    print(theta * (180/np.pi))
    # eig_vals[0], eig_vals[1] = eig_vals[1], eig_vals[0]

    e = Ellipse(xy=mu, width=large_eig_val, height=small_eig_val, angle=theta *(180/np.pi))
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')
    ax.add_artist(e)

    # e.set_clip_box(ax.bbox)
    # e.set_alpha(rnd.rand())
    # e.set_facecolor(rnd.rand(3))

    ax.set_xlim(-7, 7)
    ax.set_ylim(-7, 7)
    # plt.savefig(path + "/plot.png")
    plt.show()

def plot_error_data():
    # data = [self.pose_gt, self.mu, self.sigma]

    data = pickle.load(open("data.pickle", "rb"))
    error = np.zeros((2,len(data)))
    print(error.shape)
    sigma_bounds = np.zeros((2,len(data)))
    for i in range(len(data)):
        gt_arr = np.array([[data[i][0].position.x, \
            data[i][0].position.y]]).T
        # print(gt_arr.shape)
        # print(data[i][1].shape)
        # print(error[:,i].shape)
        error[:,i] = (gt_arr - data[i][1]).reshape((2))
        sigma_bounds[0,i] = 2*np.sqrt(data[i][2][0,0]) # 2 sigma
        sigma_bounds[1,i] = 2*np.sqrt(data[i][2][1,1]) # 2 sigma
    # Plot the x error
    fig = plt.figure(0)
    fig.set_size_inches(14, 10)
    handle1 = plt.plot(error[0,:], color="r")
    handle2 = plt.plot(sigma_bounds[0,:], color="g")
    plt.plot(-sigma_bounds[0,:], color="g")
    plt.title("Error in x vs time", fontsize=30)
    plt.xlabel("Index in time", fontsize=22)
    plt.ylabel("Error in x", fontsize=22)
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    # plt.legend([handle1, handle2], ["Estimate Error", "2\sigma"])
    plt.savefig("plots/6_noisy_sigma")
    # plt.show()

if __name__ == "__main__":
    
    plot_error_data()