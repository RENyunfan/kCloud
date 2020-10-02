#!/bin/python
# Author: Wang Siqiang@NRSL
# Date: 2019-10-08
# File Name: kf_at.py

import math
from random import uniform
from std_msgs.msg import Header
from std_msgs.msg import Float64
import rospy
import os
import math
try:
    # Python3
    import configparser as cp
except ImportError:
    # Python2
    import ConfigParser as cp
import getpass
import numpy as np
import random
import time
import datetime
import csv

from apriltag_ros.msg import AprilTagDetectionArray

class Info(object):
    """
    info process
    """
    def __init__(self):
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        self.a_pub = rospy.Publisher('/sth',Float64,queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.t = -1
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
        

    def tag_callback(self, data):
        """
        info tag_callback
        """
        try:
            # if data.detections[0].id[0] == 537:
            l = len(data.detections)
            for _ in range(l):
                a = Float64()
                a.data = self.vy
                self.a_pub.publish(a)
            # print(data.detections[0].pose.pose.pose.position)
                ll = data.detections[0].pose.pose.pose.position
                if self.t != -1:
                    dt = (data.detections[0].pose.header.stamp.to_sec()-self.t.to_sec())
                    if abs(dt) >= 0.0000001:
                        # print('wtf?',data.detections[0].pose.header.stamp.to_nsec(),self.t.to_nsec())
                        self.vx = (self.x - ll.x) / dt
                        self.vy = (self.y - ll.y) / dt
                        self.vz = (self.z - ll.z) / dt
                        self.x = ll.x
                        self.y = ll.y
                        self.z = ll.z
                        # print(self.vy)
                        a = Float64()
                        a.data = self.vy
                        # self.a_pub.publish(a)
                        self.t = data.detections[0].pose.header.stamp
                else:
                    self.t = data.detections[0].pose.header.stamp
        except IndexError as e:
            print(e)
        




if __name__ == '__main__':
    try:
        rospy.init_node('kf_at')
        info = Info()
        # rate = rospy.Rate(50)
        # today = time.strftime("%Y-%m-%d")
        # second = time.strftime("%H:%M:%S")
        # current_path = os.path.abspath(__file__)
        # # get parent folder
        # father_path = os.path.abspath(os.path.dirname(current_path) + os.path.sep + ".")
        # usr = getpass.getuser()
        # logPath = os.path.join("/home", usr, "catkin_ws", "log", today)
        
        # while not rospy.is_shutdown():
        #     rate.sleep()

    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
