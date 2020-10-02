#! /bin/python
# Author: Wang Siqiang@NRSL
# Date: 2019-09-09
# File Name: simple.py

# disable pylint warning for unused wildcard import
# pylint: disable=unused-wildcard-import

"""
STOP The arms

Args:
    python zero.py # all joints back to zero

    python zero.py # all joints stops where they are
"""
#! /bin/python
# Author: Wang Siqiang@NRSL
# Date: 2019-10-15
# File Name: utils.py

"""
Simulation utils
"""
import __future__
import os
import sys
import math
import time
import datetime
import csv
import rospy
import getpass
import tf
import sys
import scipy.linalg
import tf2_ros
import tf2_msgs.msg

import numpy as np
import sympy as sp
try:
    # Python3
    import configparser as cp
except ImportError:
    # Python2
    import ConfigParser as cp

# import from ROS
from random import uniform
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_msgs.srv import GetJointProperties
from apriltag_ros.msg import AprilTagDetectionArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Vector3



class simplest(object):
    """
    Class to stop arm
    """
    
    hlx = 100
    hly = 100
    hlz = 100
    hax = 100
    hay = 100
    haz = 100

    plx = 100
    ply = 100
    plz = 100
    pax = 100
    pay = 100
    paz = 100

    ilx = 100
    ily = 100
    ilz = 100
    iax = 100
    iay = 100
    iaz = 100

    detected = False

    def __init__(self):
        rospy.Subscriber('/pbvs_linear_x', Float64, self.plx_callback, queue_size=1)
        rospy.Subscriber('/pbvs_linear_y', Float64, self.ply_callback, queue_size=1)
        rospy.Subscriber('/pbvs_linear_z', Float64, self.plz_callback, queue_size=1)
        rospy.Subscriber('/pbvs_angular_x', Float64, self.pax_callback, queue_size=1)
        rospy.Subscriber('/pbvs_angular_y', Float64, self.pay_callback, queue_size=1)
        rospy.Subscriber('/pbvs_angular_z', Float64, self.paz_callback, queue_size=1)

        rospy.Subscriber('/ibvs_linear_x', Float64, self.ilx_callback, queue_size=1)
        rospy.Subscriber('/ibvs_linear_y', Float64, self.ily_callback, queue_size=1)
        rospy.Subscriber('/ibvs_linear_z', Float64, self.ilz_callback, queue_size=1)
        rospy.Subscriber('/ibvs_angular_x', Float64, self.iax_callback, queue_size=1)
        rospy.Subscriber('/ibvs_angular_y', Float64, self.iay_callback, queue_size=1)
        rospy.Subscriber('/ibvs_angular_z', Float64, self.iaz_callback, queue_size=1)
        
        rospy.Subscriber('/hybird_linear_x', Float64, self.hlx_callback, queue_size=1)
        rospy.Subscriber('/hybird_linear_y', Float64, self.hly_callback, queue_size=1)
        rospy.Subscriber('/hybird_linear_z', Float64, self.hlz_callback, queue_size=1)
        rospy.Subscriber('/hybird_angular_x', Float64, self.hax_callback, queue_size=1)
        rospy.Subscriber('/hybird_angular_y', Float64, self.hay_callback, queue_size=1)
        rospy.Subscriber('/hybird_angular_z', Float64, self.haz_callback, queue_size=1)
        rospy.Subscriber('/tag_detections',AprilTagDetectionArray, self.detection_callback,queue_size=1)
    
# callbacks
    def detection_callback(self, data):
        if len(data.detections) == 0:
            self.detected = False
        else:
            self.detected = True

    # pbvs part
    def plx_callback(self, data):
        """
        plx_callback
        """
        self.plx = data.data
    
    def ply_callback(self, data):
        """
        ply_callback
        """
        self.ply = data.data
    
    def plz_callback(self, data):
        """
        plz_callback
        """
        self.plz = data.data
    
    def pax_callback(self, data):
        """
        plz_callback
        """
        self.pax = data.data
    
    def pay_callback(self, data):
        """
        plz_callback
        """
        self.pay = data.data
    
    def paz_callback(self, data):
        """
        plz_callback
        """
        self.paz = data.data

    # ibvs part

    def ilx_callback(self, data):
        """
        ilx_callback
        """
        self.ilx = data.data
    
    def ily_callback(self, data):
        """
        ily_callback
        """
        self.ily = data.data
    
    def ilz_callback(self, data):
        """
        ilz_callback
        """
        self.ilz = data.data
    
    def iax_callback(self, data):
        """
        ilz_callback
        """
        self.iax = data.data
    
    def iay_callback(self, data):
        """
        ilz_callback
        """
        self.iay = data.data
    
    def iaz_callback(self, data):
        """
        ilz_callback
        """
        self.iaz = data.data

    # hbvs part

    def hlx_callback(self, data):
        """
        hlx_callback
        """
        self.hlx = data.data
    
    def hly_callback(self, data):
        """
        hly_callback
        """
        self.hly = data.data
    
    def hlz_callback(self, data):
        """
        hlz_callback
        """
        self.hlz = data.data
    
    def hax_callback(self, data):
        """
        hlz_callback
        """
        self.hax = data.data
    
    def hay_callback(self, data):
        """
        hlz_callback
        """
        self.hay = data.data
    
    def haz_callback(self, data):
        """
        hlz_callback
        """
        self.haz = data.data


# functions
        

    def zero(self):
        """
        control the grippers and arm, record data
        """
        rospy.init_node('stops')
        jo1_pub = rospy.Publisher('/hummingbird/jo1/command', Float64, queue_size=10)
        rospy.Subscriber('/hybrid_angular_y',Float64,self.hay_callback)
        jo2_pub = rospy.Publisher('/hummingbird/jo2/command', Float64, queue_size=10)
        jo3_pub = rospy.Publisher('/hummingbird/jo3/command', Float64, queue_size=10)
        jo4_pub = rospy.Publisher('/hummingbird/jo4/command', Float64, queue_size=10)
        jo5_pub = rospy.Publisher('/hummingbird/jo5/command', Float64, queue_size=10)
        jo6_pub = rospy.Publisher('/hummingbird/jo6/command', Float64, queue_size=10)

        gripper_left_pub = rospy.Publisher('/hummingbird/gripper_left/command', Float64, queue_size=10)
        gripper_right_pub = rospy.Publisher('/hummingbird/gripper_right/command', Float64, queue_size=10)
        rate = rospy.Rate(10)
        
        # Create the messaged
        v_jo1 = Float64()
        v_jo2 = Float64()
        v_jo3 = Float64()
        v_jo4 = Float64()
        v_jo5 = Float64()
        v_jo6 = Float64()
        v_j0 = Float64()
        v_jt = Float64()
        v_gripper_left = Float64()
        v_gripper_right = Float64()
        v_j0.data = 0
        v_jt.data = -0.05

        # Joint names for arm
        rospy.wait_for_service('/gazebo/get_joint_properties')
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        jo1_pub.publish(v_j0)
        jo2_pub.publish(v_j0)
        jo3_pub.publish(v_j0)
        jo4_pub.publish(v_j0)
        jo5_pub.publish(v_j0)
        jo6_pub.publish(v_j0)
        time.sleep(0.2)
        while not rospy.is_shutdown():
            try:
                if len(sys.argv) < 2:
                    getJ = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
                    # v = math.sin(t)
                    v_j = [v_jo1, v_jo2, v_jo3, v_jo4, v_jo5, v_jo6]
                    summer = 0
                    for i in range(6):
                        temp = getJ(joint_names[i]).position[0]
                        while temp >= 2 * math.pi:
                            temp = temp - 2* math.pi
                        while temp <= 0:
                            temp = temp + 2 * math.pi
                        if temp > math.pi:
                            v_j[i].data = (2 * math.pi - temp)
                        else:
                            v_j[i].data = -1 * temp
                        summer += abs(v_j[i].data)
                    v_gripper_left.data = -1 * (getJ('finger_joint_left').position[0] + 0.5)
                    v_gripper_right.data = -1 * (getJ('finger_joint_right').position[0] - 0.5)
                    # publish message
                    # print(getJ('joint1').position[0],getJ('joint2').position[0],getJ('joint3').position[0],getJ('joint4').position[0],getJ('joint5').position[0],getJ('joint6').position[0])
                    jo1_pub.publish(v_jo1)
                    jo2_pub.publish(v_jo2)
                    jo3_pub.publish(v_jo3)
                    jo4_pub.publish(v_jo4)
                    jo5_pub.publish(v_jo5)
                    jo6_pub.publish(v_jo6)
                    if summer <= 0.01:
                        print("Stablized")
                        jo1_pub.publish(v_j0)
                        jo2_pub.publish(v_j0)
                        jo3_pub.publish(v_j0)
                        jo4_pub.publish(v_j0)
                        jo5_pub.publish(v_j0)
                        jo6_pub.publish(v_j0)
                        gripper_left_pub.publish(v_j0)
                        gripper_right_pub.publish(v_j0)
                        break
                    gripper_left_pub.publish(v_gripper_left)
                    gripper_right_pub.publish(v_gripper_right)
                    rate.sleep()
                elif sys.argv[1] == 'stop':
                    for _ in range(5):
                        jo1_pub.publish(v_j0)
                        jo2_pub.publish(v_j0)
                        jo3_pub.publish(v_j0)
                        jo4_pub.publish(v_j0)
                        jo5_pub.publish(v_j0)
                        jo6_pub.publish(v_j0)
                        gripper_left_pub.publish(v_j0)
                        gripper_right_pub.publish(v_j0)
                        time.sleep(0.1)
                    print('Stopped')
                    break
                elif sys.argv[1] == '1':
                    jo1_pub.publish(v_jt)
                    time.sleep(0.1)
                elif sys.argv[1] == '2':
                    jo2_pub.publish(v_jt)
                    time.sleep(0.1)
                elif sys.argv[1] == '3':
                    jo3_pub.publish(v_jt)
                    time.sleep(0.1)
                elif sys.argv[1] == '4':
                    jo4_pub.publish(v_jt)
                    time.sleep(0.1)
                elif sys.argv[1] == '5':
                    jo5_pub.publish(v_jt)
                    time.sleep(0.1)
                elif sys.argv[1] == '6':
                    jo6_pub.publish(v_jt)
                    time.sleep(0.1)
                elif sys.argv[1] == 'all':
                    jo1_pub.publish(v_jt)
                    jo2_pub.publish(v_jt)
                    jo3_pub.publish(v_jt)
                    jo4_pub.publish(v_jt)
                    jo5_pub.publish(v_jt)
                    jo6_pub.publish(v_jt)
                    time.sleep(0.1)
            except rospy.ServiceException as e:
                print(e)
                pass    



if __name__ == '__main__':
    try:
        S = simplest()
        S.zero()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")
