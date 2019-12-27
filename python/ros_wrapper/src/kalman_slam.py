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
        self.seq = 0
        self.lock = threading.Lock()
        param_names = rospy.get_param_names()
        num_landmarks = len([x for x in param_names if "landmarks" in x])
        self.mu = np.zeros((2 + 2*num_landmarks,1))
        self.control_input = np.zeros((2,1))
        self.sigma = np.eye(2 + 2*num_landmarks) * 4
        self.sigma[0,0] = 1
        self.sigma[1,1] = 1
        self.last_update_time = rospy.Time.now()
        # self.Q = np.array([[.001, 0], [0, 0.001]])
        # self.R = np.eye(2) * (rospy.get_param("sensor_noise_std") ** 2)
        self.sensor_noise_var = rospy.get_param("sensor_noise_std") ** 2
        self.Q = np.eye(self.mu.size) * 0.001
        self.pub = rospy.Publisher("/ava/estimate", FloatArrayStamped, queue_size=10)
        # rospy.Subscriber("/ava/pose_noise", Odometry, self.noise_callback)
        rospy.Subscriber("/ava/measurements", FloatArrayStamped, self.meas_callback)

    def meas_callback(self, msg): # add control input
        self.lock.acquire(True)
        # print("Received msg.")
        data = msg.fma.data
        control_input = np.array([[data[0]], [data[1]]])
        data = data[2:] # trim the control input entries
        num_meas = len(data) / 3

        # Construct C matrix and meas array
        print(data)
        meas = np.array(())
        C = np.array(())
        for i in range(num_meas):
            lmrk = int(data[3*i]) - ord('A')
            # print(lmrk)
            delta_x = data[3*i+1]
            delta_y = data[3*i+2]
            tmp = np.zeros((2,self.mu.size))
            tmp[0,0] = -1
            tmp[1,1] = -1
            tmp[0,2+2*lmrk] = 1
            tmp[1,2+2*lmrk+1] = 1

            # C = np.stack((C,tmp)).reshape((2*i+2, self.mu.size))
            C = np.append(C, tmp).reshape((2*i+2, self.mu.size))
            meas = np.append(meas, np.array([delta_x,delta_y])).reshape((2*i+2,1))
        # print("------------------------------------------------------------------------------------")
        # print("seq: " + str(self.seq))
        self.seq += 1
        # print(C)
        # print(meas)
        # if self.seq > 34:
            # return
        self.run_filter(msg.header.stamp, C, meas, control_input)
        self.lock.release()

    def run_filter(self, timestamp, C, meas, control_input):
        # self.lock.acquire(True)
        t_now = rospy.Time.now()
        delta_t = t_now - self.last_update_time
        self.last_update_time = t_now
        u = control_input
        A = np.eye(self.mu.size)

        self.R = np.eye(meas.size) * self.sensor_noise_var

        B = np.zeros((self.mu.size,2))
        B[0,0] = delta_t.to_sec()
        B[1,1] = delta_t.to_sec()


        # Prediction
        mu_minus = np.dot(A, self.mu) + np.dot(B, u)
        sigma_minus = np.dot( np.dot(A, self.sigma), A.T) + self.Q

        # Correction
        tmp1 = np.dot(C, sigma_minus)
        tmp2 = np.dot( tmp1, C.T) + self.R
        tmp3 = np.dot(sigma_minus, C.T)
        tmp4 = np.linalg.inv(tmp2)
        K = np.dot( tmp3, tmp4)
        
        innovation = meas - np.dot(C, mu_minus)
        self.mu = mu_minus + np.dot(K, innovation)
        # print("meas:")
        # print(meas)
        # print("innovation:")
        # print(innovation)
        # print("tmp1:")
        # print(tmp1)
        # print("tmp2:")
        # print(tmp2)
        # print("tmp3:")
        # print(tmp3)
        # print("tmp4:")
        # print(tmp4)
        # print("K:")
        # print(K)
        # print("C:")
        # print(C)
        # print("mu_minus:")
        # print(mu_minus)
        # print("self.mu:")
        # print(self.mu)
        # print("sigma_minus:")
        # print(sigma_minus)
        self.sigma = np.dot( (np.eye(self.mu.size) - np.dot(K, C)), sigma_minus)

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

    # def noise_callback(self, msg):
    #     self.lock.acquire(True)
    #     if not self.meas_queue: # Fill queue if empty
    #         self.meas_queue.append(msg.pose.pose.position.x)
    #         self.meas_queue.append(msg.pose.pose.position.y)
    #     else: # only keep the latest measurement
    #         self.meas_queue[0] = msg.pose.pose.position.x
    #         self.meas_queue[1] = msg.pose.pose.position.y

    #     # Control Input
    #     self.control_input = np.array([[
    #         msg.twist.twist.linear.x, msg.twist.twist.linear.y
    #     ]]).T
    #     self.run_filter(msg.header.stamp)
    #     self.lock.release()

if __name__ == "__main__":
    rospy.init_node("kalman")
    k = Kalman()
    rospy.spin()