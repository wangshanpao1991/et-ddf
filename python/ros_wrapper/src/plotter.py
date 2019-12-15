#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

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
        rospy.loginfo("plotting...")

if __name__ == "__main__":
    rospy.init_node("plotter")
    p = Plotter()
    rospy.spin()