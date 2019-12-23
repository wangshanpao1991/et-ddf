#!/usr/bin/env python
from __future__ import division
"""
Simulates the movements of points in space
"""
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, TwistStamped, PoseArray
import random
import tf
import threading
from etddf_ros.msg import FloatArrayStamped

ODOM_INDEX = 0
PUB_INDEX = 1
NOISE_INDEX = 2

class PointSim:

    def __init__(self):
        rospy.loginfo("Point Sim and Controller Initialized")
        self.load_landmarks()
        self.meas_pub = rospy.Publisher("/ava/measurements", FloatArrayStamped, queue_size=10)
        self.landmark_debug_pub = rospy.Publisher("landmark_poses/", PoseArray, queue_size=10)
        self.load_auvs()
        self.update_period = 1 / int(rospy.get_param('sim/update_freq'))
        self.timer = rospy.Timer(rospy.Duration(self.update_period), self.update_poses)
        self.sensor_noise_std = rospy.get_param("sensor_noise_std")
        self.lock = threading.Lock()

    def load_landmarks(self):
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
        self.sensor_noise_std = rospy.get_param("sensor_noise_std")
    
    def publish_measurements(self, timestamp):
        fsa = FloatArrayStamped()
        fsa.header.stamp = timestamp
        odom = self.auvs["ava"][ODOM_INDEX]
        robot_arr = np.array([[odom.pose.pose.position.x, odom.pose.pose.position.y]]).T
        for k in self.landmarks.keys():
            lmrk = self.landmarks[k]
            lmrk_arr = np.array([[lmrk.x, lmrk.y]]).T
            diff = lmrk_arr - robot_arr
            if( np.linalg.norm(diff) <= self.sensor_range):
                fsa.fma.data.append(ord(k))
                fsa.fma.data.append(diff[0] + np.random.normal(0, self.sensor_noise_std))
                fsa.fma.data.append(diff[1] + np.random.normal(0, self.sensor_noise_std))
        self.meas_pub.publish(fsa)
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

    def load_auvs(self):
        self.auvs = {} # each auv has an odom
        auv_list = rospy.get_param('/active_auvs')
        for auv in auv_list:
            print("Loading " + auv)
            start_odom = Odometry()
            start_pos = rospy.get_param(auv+'/start_pos', 'random')
            if start_pos == "random":
                start_odom.pose.pose = self.get_random_pose()
            else:
                start_odom.pose.pose = self.load_start_pose(start_pos)
            start_odom.header.seq = 0
            start_odom.header.stamp = rospy.get_rostime()
            start_odom.header.frame_id = 'world'
            start_odom.child_frame_id = auv + '/base_link'
            truth_pub = rospy.Publisher(auv + '/pose_gt', Odometry, queue_size=10)
            noise_pub = rospy.Publisher(auv + '/pose_noise', Odometry, queue_size=10)
            self.auvs[auv] = [start_odom, truth_pub, noise_pub]
            rospy.Subscriber(auv + '/new_twist', TwistStamped, self.control_callback)

    def load_start_pose(self, pose_list):
        pose = Pose()
        pose.position.x = pose_list['x']
        pose.position.y = pose_list['y']
        pose.position.z = pose_list['z']
        roll, pitch = 0,0
        yaw = pose_list['psi']
        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def get_random_pose(self):
        [min_point, max_point] = rospy.get_param('sim/random_pose_min_max')
        size = max_point - min_point
        pose = Pose()
        pose.position.x = random.random() * size + min_point
        pose.position.y = random.random() * size + min_point
        pose.position.z = random.random() * size + min_point
        if rospy.get_param('sim/random_yaw'):
            yaw = random.random() * np.pi * 2
        else:
            yaw = 0
        roll, pitch = 0,0
        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def update_poses(self, msg):
        self.lock.acquire(True)
        for auv in self.auvs:
            noise_odom = Odometry()
            odom = self.auvs[auv][ODOM_INDEX]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(( odom.pose.pose.orientation.x, \
                                                                          odom.pose.pose.orientation.y, \
                                                                          odom.pose.pose.orientation.z, \
                                                                          odom.pose.pose.orientation.w))
                                        
            roll += odom.twist.twist.angular.x * self.update_period
            pitch += odom.twist.twist.angular.y * self.update_period
            yaw += odom.twist.twist.angular.z * self.update_period

            noise_roll = roll + np.random.normal(0, np.pi / 10)
            noise_pitch = pitch + np.random.normal(0, np.pi / 10)
            noise_yaw = yaw + np.random.normal(0, np.pi / 10)

            roll = self.correct_angles(roll)
            pitch = self.correct_angles(pitch)
            yaw = self.correct_angles(yaw)

            noise_roll = self.correct_angles(noise_roll)
            noise_pitch = self.correct_angles(noise_pitch)
            noise_yaw = self.correct_angles(noise_yaw)

            quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            odom.pose.pose.orientation.x = quat_list[0]
            odom.pose.pose.orientation.y = quat_list[1]
            odom.pose.pose.orientation.z = quat_list[2]
            odom.pose.pose.orientation.w = quat_list[3]

            noise_quat_list = tf.transformations.quaternion_from_euler(noise_roll, noise_pitch, noise_yaw)
            noise_odom.pose.pose.orientation.x = noise_quat_list[0]
            noise_odom.pose.pose.orientation.y = noise_quat_list[1]
            noise_odom.pose.pose.orientation.z = noise_quat_list[2]
            noise_odom.pose.pose.orientation.w = noise_quat_list[3]
            
            odom.pose.pose.position.x += odom.twist.twist.linear.x * self.update_period
            odom.pose.pose.position.y += odom.twist.twist.linear.y * self.update_period
            odom.pose.pose.position.z += odom.twist.twist.linear.z * self.update_period

            noise_odom.pose.pose.position.x += odom.pose.pose.position.x + np.random.normal(0, self.sensor_noise_std)
            noise_odom.pose.pose.position.y = odom.pose.pose.position.y + np.random.normal(0, self.sensor_noise_std)
            noise_odom.pose.pose.position.z = odom.pose.pose.position.z + np.random.normal(0, self.sensor_noise_std)
            noise_odom.twist.twist.linear = odom.twist.twist.linear
            noise_odom.twist.twist.angular = odom.twist.twist.angular

            t_now = rospy.get_rostime()
            odom.header.stamp = t_now
            noise_odom.header.stamp = t_now
            self.auvs[auv][ODOM_INDEX] = odom
            self.auvs[auv][PUB_INDEX].publish(odom)
            self.auvs[auv][NOISE_INDEX].publish(noise_odom)
            self.publish_measurements(t_now)
        self.lock.release()

    def correct_angles(self, angle):
        """ Map all angles between -pi to pi """
        while angle < -np.pi or angle > np.pi:
            if angle < -np.pi:
                angle += 2 * np.pi
            else:
                angle -= 2 * np.pi
        return angle
        
    def control_callback(self, msg):
        self.lock.acquire(True)
        topic = msg._connection_header['topic']
        auv = None
        for auv_name in self.auvs:
            if auv_name in topic:
                auv = auv_name
                break

        """ According to nav_msgs.msg/Odometry standards, the pose is given in header/frame_id and the twist is given in child_frame_id
        """
        self.auvs[auv][ODOM_INDEX].child_frame_id = msg.header.frame_id
        self.auvs[auv][ODOM_INDEX].header = msg.header        
        self.auvs[auv][ODOM_INDEX].header.frame_id = 'world'
        self.auvs[auv][ODOM_INDEX].twist.twist.linear = msg.twist.linear
        self.auvs[auv][ODOM_INDEX].twist.twist.angular = msg.twist.angular
        # rospy.loginfo(self.auvs[auv][ODOM_INDEX])

        # Republish the new transform msg.header.frame_id -> world
        self.lock.release()

def main():
    rospy.init_node('point_sim_contoller')
    ps = PointSim()
    rospy.spin()

if __name__ == "__main__":
    main()
