#!/usr/bin/env python
import rospy
import threading
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class Kalman:
    def __init__(self):
        self.lock = threading.Lock()
        self.meas_queue = []
        rospy.Subscriber("/ava/pose_noise", Odometry, self.noise_callback)
        self.pub = rospy.Publisher("/ava/estimate", Float32MultiArray, queue_size=10)

        self.mu = np.zeros((2,1))
        self.sigma = np.zeros((2,2))
        self.last_update_time = None
        self.Q = np.array([[.01, 0], [0, 0.01]])
        self.R = np.array([[.01, 0], [0, 0.01]])

    # I'll want a separate callback for running the filter --> will need to add a lock then
    def run_filter(self):
        self.lock.acquire(True)
        f = Float32MultiArray()
        mad = MultiArrayDimension()
        mad.label = "Number of states"
        mad.size = len(self.mu)
        f.layout.dim.append(mad)
        f.data = list(self.mu)
        f.data.extend(list(self.sigma.flatten()))
        self.pub.publish(f)
        # print("running filter...")
        self.mu += np.array([[0.001], [0.001]])
        self.sigma += np.array([[0.01, 0],[0, 0.01]])
        self.lock.release()

    def noise_callback(self, msg):
        self.lock.acquire(True)
        self.meas_queue.append(msg.pose.pose.position.x)
        self.meas_queue.append(msg.pose.pose.position.y)
        self.lock.release()

if __name__ == "__main__":
    rospy.init_node("kalman")
    k = Kalman()
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        k.run_filter()
        r.sleep()