#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np

class Plotter:

    def __init__(self):
        rospy.Subscriber("/ava/pose_gt", Odometry, self.pose_gt_callback)
        rospy.Subscriber("/ava/estimate", Float32MultiArray, self.pose_estimate_callback)
        self.pose_gt = None
        self.pose_estimate = None

    def pose_gt_callback(self, msg):
        self.pose_gt = msg.pose.pose

    def pose_estimate_callback(self, msg):
        self.pose_estimate = None
        # Plot and save to /tmp/plots
        data = msg.data
        num_states = msg.layout.dim[0].size
        mu = np.array([data[:num_states]])
        cov = np.array([data[num_states:]]).reshape((num_states, num_states))
        rospy.loginfo("plotting...")
        print(mu)
        print(cov)

if __name__ == "__main__":
    rospy.init_node("plotter")
    p = Plotter()
    rospy.spin()