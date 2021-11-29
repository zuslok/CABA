#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math

from geometry_msgs.msg import Pose, Twist 
from nav_msgs.msg import Odometry

class Caba:
    def __init__(self, cmd_topic = "/cmd_vel", odom_topic = "/odom"):
        rospy.init_node("Caba_tester", anonymous=True)
        
        self.vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb)

        self.desired_pose = Pose()
        self.curr_pose = None
        self.desired_pose.position.x = 1.0
        self.vel_msg = Twist()

        self.rate = rospy.Rate(10)

        self.max_lin_vel = 0.3

    def odom_cb(self, data):
        self.curr_pose = data.pose.pose

    def main_func(self):
            while not rospy.is_shutdown():
                if self.curr_pose != None:
                    x_dist_left = self.desired_pose.position.x - self.curr_pose.position.x

                    lin_vel = 0.3 * x_dist_left

                    if lin_vel > self.max_lin_vel:
                        lin_vel = self.max_lin_vel
                    if lin_vel < -self.max_lin_vel:
                        lin_vel = -self.max_lin_vel
                    
                    print("linear vel: {}, distance left: {}".format(lin_vel, x_dist_left))

                    if x_dist_left < 0.05:
                        self.vel_msg.linear.x = 0
                    else:
                        self.vel_msg.linear.x = lin_vel
                    
                    self.vel_pub.publish(self.vel_msg)
                
                self.rate.sleep()
                

if __name__ == '__main__':
    caba = Caba()
    caba.main_func()