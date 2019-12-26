#!/usr/bin/env python
import rospy
import threading
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import signal
import pickle
import matplotlib.pyplot as plt
from geometry_msgs.msg import TwistStamped
from etddf_ros.msg import FloatArrayStamped

class Kalman:
    def __init__(self):
        self.lock = threading.Lock()
        self.meas_queue = []
        self.control_input = np.zeros((2,1))
        self.mu = np.zeros((2,1))
        self.sigma = np.eye(2)
        self.last_update_time = rospy.Time.now()
        self.Q = np.array([[.001, 0], [0, 0.001]])
        self.R = np.eye(2) * (rospy.get_param("sensor_noise_std") ** 2)
        self.pub = rospy.Publisher("/ava/estimate", FloatArrayStamped, queue_size=10)
        rospy.Subscriber("/ava/pose_noise", Odometry, self.noise_callback)
        rospy.Subscriber("/ava/measurements", FloatArrayStamped, self.meas_callback)
        # rospy.Subscriber('/ava/new_twist', TwistStamped, self.control_callback)

    def meas_callback(self, msg):
        pass # TODO
    
    # For synchronization purposes we want to know what control input point_sim used
    # def control_callback(self, msg):
    #     self.lock.acquire(True)
    #     self.control_input = np.array([[
    #         msg.twist.linear.x, msg.twist.linear.y
    #     ]]).T
    #     self.lock.release()

    def run_filter(self, timestamp):
        # self.lock.acquire(True)
        t_now = rospy.Time.now()
        delta_t = t_now - self.last_update_time
        self.last_update_time = t_now
        A = np.eye(2)
        B = np.eye(2) * delta_t.to_sec()
        u = self.control_input
        C = np.eye(2)
        meas = np.array([[self.meas_queue.pop(0), self.meas_queue.pop(0)]]).T

        # Prediction
        mu_minus = np.dot(A, self.mu) + np.dot(B, u)
        sigma_minus = np.dot( np.dot(A, self.sigma), A.T) + self.Q

        # Correction
        tmp = np.dot( np.dot(C, sigma_minus), C.T) + self.R
        K = np.dot( np.dot(sigma_minus, C.T), np.linalg.inv(tmp))
        innovation = meas - np.dot(C, mu_minus)
        self.mu = mu_minus + np.dot(K, innovation)
        self.sigma = np.dot( (np.eye(2) - np.dot(K, C)), sigma_minus)

        self.publish_estimate(timestamp)
        # self.lock.release()

    def publish_estimate(self, timestamp): # Publish estimate
        es = FloatArrayStamped()
        mad = MultiArrayDimension()
        mad.label = "Number of states"
        mad.size = len(self.mu)
        es.fma.layout.dim.append(mad)
        es.fma.data = list(self.mu)
        es.fma.data.extend(list(self.sigma.flatten()))
        es.header.stamp = timestamp
        self.pub.publish(es)

    def noise_callback(self, msg):
        self.lock.acquire(True)
        if not self.meas_queue: # Fill queue if empty
            self.meas_queue.append(msg.pose.pose.position.x)
            self.meas_queue.append(msg.pose.pose.position.y)
        else: # only keep the latest measurement
            self.meas_queue[0] = msg.pose.pose.position.x
            self.meas_queue[1] = msg.pose.pose.position.y

        # Control Input
        self.control_input = np.array([[
            msg.twist.twist.linear.x, msg.twist.twist.linear.y
        ]]).T
        self.run_filter(msg.header.stamp)
        self.lock.release()

if __name__ == "__main__":
    rospy.init_node("kalman")
    k = Kalman()
    rospy.spin()