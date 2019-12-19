#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Pose, PoseArray

class Plotter:

    def __init__(self):
        rospy.Subscriber("/ava/pose_gt", Odometry, self.pose_gt_callback)
        rospy.Subscriber("/ava/estimate", Float32MultiArray, self.pose_estimate_callback)
        self.landmark_debug_pub = rospy.Publisher("landmark_poses/", PoseArray, queue_size=10)
        self.meas_pub = rospy.Publisher("/ava/measurements", Float32MultiArray, queue_size=10)
        self.pose_gt = None
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

    def pose_gt_callback(self, msg):
        self.pose_gt = msg.pose.pose
        point = self.pose_gt.position

        f = Float32MultiArray()
        robot_arr = np.array([[point.x, point.y]]).T
        for k in self.landmarks.keys():
            lmrk = self.landmarks[k]
            lmrk_arr = np.array([[lmrk.x, lmrk.y]]).T
            diff = lmrk_arr - robot_arr
            if( np.linalg.norm(diff) <= self.sensor_range):
                f.data.append(ord(k))
                f.data.append(diff[0] + np.random.normal(0, self.sensor_noise))
                f.data.append(diff[1] + np.random.normal(0, self.sensor_noise))
        print("publishing...")
        self.meas_pub.publish(f)

    def pose_estimate_callback(self, msg):
        self.pose_estimate = None
        # Plot and save to /tmp/plots
        data = msg.data
        num_states = msg.layout.dim[0].size
        mu = np.array([data[:num_states]])
        cov = np.array([data[num_states:]]).reshape((num_states, num_states))
        # rospy.loginfo("plotting...")
        # print(mu)
        # print(cov)
        self.publish_debug_landmarks()

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

def test_plotting():
    mu = np.zeros((2,1))
    # sigma = np.array([[2, 0], [0, 1]])
    sigma = np.array([[1, 1], [1, 4]])
    # sigma = np.array([[4.25, 3.10], [3.10, 4.29]])
    print('original cov matrix')
    print(sigma)

    s = 2
    [eig_vals, eig_vectors] = np.linalg.eig(sigma)

    print('\neigenvalues')
    print(eig_vals)
    print('\neigenvectors')
    print(eig_vectors)
 
    theta = np.arccos(eig_vectors[0,0])
    print("\ntheta")
    print(theta * (180/np.pi))
    # eig_vals[0], eig_vals[1] = eig_vals[1], eig_vals[0]

    e = Ellipse(xy=mu, width=eig_vals[0], height=eig_vals[1], angle=theta *(180/np.pi))
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')
    ax.add_artist(e)

    # e.set_clip_box(ax.bbox)
    # e.set_alpha(rnd.rand())
    # e.set_facecolor(rnd.rand(3))

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    plt.show()

if __name__ == "__main__":
    rospy.init_node("plotter")
    p = Plotter()
    rospy.spin()
    # test_plotting()