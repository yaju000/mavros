#!/usr/bin/env python

import math
import numpy as np
import rospy
from pandas import DataFrame
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import ParamSet
from std_msgs.msg import Float64
from math import *
from datetime import datetime
import time
import heapq

from AOA_v2 import AOA
from LeastQ_v1 import least_square
from hsv_fw import hsv

AOA = AOA()
LQ = least_square()
hsv = hsv()

enu_pos = []
azimuth = []
ob_points = []
est_list = []
P_img_list = []
est_orin = []

class aoa_info(object):
    def __init__(self):
        self.imu_msg = Imu()
        self.gps_msg = Odometry()

        self.gps_pose = [0,0,0]
        self.ned_pose = [0,0,0]
        self.imu_x = 0
        self.lamda = 0
        self.roll, self.pitch, self.yaw = 0,0,0
        self.vision_roll, self.vision_pitch, self.vision_yaw = 0, 0, 0
        self.hd_deg = 0
        self.quat = [0,0,0,0]
        self.u, self.v = 0, 0
        self.u_u, self.v_v = 0, 0
        self.P_img_x, self.P_img_y, self.P_img_z = 0, 0, 0
        self.angle_a_w = 0
        self.angle_e_w = 0
        self.angle_a = [0, 0]
        self.angle_e = [0, 0]

        self.est_position = [0, 0, 0]
        self.ob_point = [0, 0, 0]
        self.est_n, self.est_e, self.est_d = 0, 0, 0
        self.vector_n, self.vector_e, self.vector_d = 0, 0, 0

        rospy.Subscriber("/plane_cam_0/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/plane_cam_0/mavros/global_position/local", Odometry, self.gps_callback) 
        rospy.Subscriber("/plane_cam_0/mavros/global_position/compass_hdg", Float64, self.hdg_callback)   

    def gps_callback(self, msg):
        self.gps_msg = msg
        self.gps_pose[0] = msg.pose.pose.position.x
        self.gps_pose[1] = msg.pose.pose.position.y
        self.gps_pose[2] = msg.pose.pose.position.z

        self.ned_pose[0], self.ned_pose[1], self.ned_pose[2] = self.ENU_to_NED(self.gps_pose[0], self.gps_pose[1], self.gps_pose[2])
        
        enu_pos.append([self.gps_pose[0], self.gps_pose[1], self.gps_pose[2]])

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.quat[0] = msg.orientation.w
        self.quat[1] = msg.orientation.x
        self.quat[2] = msg.orientation.y
        self.quat[3] = msg.orientation.z

        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.roll_n, self.pitch_e, self.yaw_d = self.ENU_to_NED(self.roll, self.pitch, self.yaw)

    def hdg_callback(self, msg): #enu # pi~(-pi)
        self.hdg_msg = msg
        heading_angle = msg.data
        self.hd_deg = heading_angle
        if heading_angle > 180:
            self.heading = np.deg2rad(heading_angle-360)
        else:
            self.heading = np.deg2rad(heading_angle)

        # print('heading angle(deg) = ')
        # print(heading_angle)
        # print('heading angle(rad) = ')
        # print(self.heading)

    def ENU_to_NED(self, x, y, z):
  
        R = [[0, 1, 0],[1, 0, 0],[0, 0, -1]]
        q = [x, y, z]
        ned = np.matmul(R,q)
        a = ned[0]
        b = ned[1]
        c = ned[2]
      
        return a, b, c

    def NED_to_ENU(self, x, y, z):
  
        R = [[0, 1, 0],[1, 0, 0],[0, 0, -1]]
        q = [x, y, z]
        ned = np.matmul(R,q)
        a = ned[0]
        b = ned[1]
        c = ned[2]
      
        return a, b, c

    def cal_aoa_info(self):

        self.u, self.v = hsv.value_callback()

        if [self.u, self.v]!=[self.u_u, self.v_v]:
            self.u_u = self.u
            self.v_v = self.v

            #position_vector
            size_u = 320
            size_v = 240
            u_0 = size_u/2
            v_0 = size_v/2
            # focal length
            f = 277.191356
            # before
            self.P_img_x = v_0 - self.v_v
            self.P_img_y = self.u_u - u_0 
            self.P_img_z = f
            # after
            # self.P_img_x = u_0 - self.u_u
            # self.P_img_y = v_0 - self.v_v 
            # self.P_img_z = f
            P_img = [self.P_img_x, self.P_img_y, self.P_img_z]
            # print("u, v = ")
            # print(self.u, self.v)
            # print('------------')
            # print("P_img = ")
            # print(P_img)

            ### Least square ###
            print('Number of iteration=',len(azimuth))
            #before
            self.angle_a_w, self.angle_e_w, self.angle_a, self.angle_e, self.est_n, self.est_e, self.est_d, self.vector_n, self.vector_e, self.vector_d = AOA.AOA_v0(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2], self.roll, self.pitch, self.yaw, self.P_img_x, self.P_img_y, self.P_img_z)
            #after
            #self.angle_a_w, self.angle_e_w, self.angle_a, self.angle_e, self.est_n, self.est_e, self.est_d, self.vector_n, self.vector_e, self.vector_d = AOA.AOA_v2(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2], self.roll_n, self.pitch_e, self.yaw_d, self.P_img_x, self.P_img_y, self.P_img_z)
            print('azimuth =', self.angle_a_w)
            print('est position =',self.est_n, self.est_e, self.est_d)
            ob_point = [self.ned_pose[0], self.ned_pose[1], self.ned_pose[2]]
            azimuth.append(self.angle_a_w)
            ob_points.append(ob_point)
            est_orin.append([self.est_n, self.est_e, self.est_d])

            if  len(azimuth)>=2:
                Est_n,Est_e,Est_d = LQ.LeastQ_m(ob_points, azimuth)
                Est_position = [Est_n,Est_e,Est_d]
                print('Est_n,Est_e,Est_d = ')
                print(Est_position)
                est_list.append(Est_position)
        else:
            pass

    def cal_P_img(self, u, v):
        #position_vector
        size_u = 320
        size_v = 240
        u_0 = size_u/2
        v_0 = size_v/2
        # focal length
        f = 277.191356
        P_img_x = v_0 - v
        P_img_y = u - u_0
        P_img_z = f

        #print("P_img = ")
        #print(P_img_x, P_img_y, P_img_z)
        return P_img_x, P_img_y, P_img_z

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def iteration(self, event):
        
        F = 30
        #print('heading angel=',self.hd_deg)

        if self.hd_deg % F < 2:

            self.cal_aoa_info()
        
        #print('hey')
        #self.setpoint.header.stamp = rospy.Time.now()
        #print(datetime.utcfromtimestamp(rospy.Time.now().to_sec()))
        #print("ENU coordinate :")
        #print(self.gps_pose) 
        #print("NED coordinate :")
        #print(self.ned_pose) 
        #print("uav_pose :")
        #print(self.roll*57.3, self.pitch*57.3, self.yaw*57.3)
        #print("camera_pose :")
        #print(self.vision_roll*57.3, self.vision_pitch*57.3, self.vision_yaw*57.3)

        #print("a_w, e_w, a, e = ")
        #print(self.angle_a_w, self.angle_e_w)
        #print(self.angle_a, self.angle_e)


if __name__ == '__main__':
    rospy.init_node('aoa_info_only', anonymous=True)
    dt = 1.0/10
    #dt = 1/5.0
    pathplan_run = aoa_info()
    rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
    rospy.spin()

    df = DataFrame({'enu_pos': enu_pos})
    df.to_excel('fw_tt_path.xlsx', sheet_name='sheet1', index=False)
    dp = DataFrame({'est_position':est_list})
    dp.to_excel('fw_tt_est.xlsx', sheet_name='sheet1', index=False)
    dr = DataFrame({'est_orin':est_orin})
    dr.to_excel('est_orin.xlsx', sheet_name='sheet1', index=False)

