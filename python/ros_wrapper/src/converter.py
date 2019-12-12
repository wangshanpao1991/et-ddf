#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Converter:
    def __init__(self):
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("/ava/new_twist", Twist, queue_size=10)
        self.internal_x = 0
        self.internal_y = 0

    def callback(self, msg):
        self.internal_x += msg.linear.x * (0.5) * 0.1
        self.internal_y += msg.angular.z * 0.5 * 0.1

        t = Twist()
        t.linear.x = self.internal_x
        t.linear.y = self.internal_y
        self.pub.publish(t)

if __name__ == "__main__":
    rospy.init_node("converter")
    c = Converter()
    rospy.spin()