#!/usr/bin/env python

import math
import numpy as np
import rospy
from pandas import DataFrame
# from mavros_msgs.msg import State
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import String

import copy
import cv2
from sensor_msgs.msg import Image 
from math import *
from datetime import datetime
import time
from AOA_v1 import AOA
from dop import cost_function
from LeastQ_v1 import least_square
import heapq
from offb_pos import offb_pos
from offb_land import offb_land
from hsv import hsv

AOA = AOA()
cal_dop = cost_function()
LeastQ = least_square()
current_state = State()
offb_pos = offb_pos()
offb_land = offb_land()
hsv = hsv()

enu_pos = []
pos_command = []
P_img = []
azimuth = []
ob_list = []
roi_num = []
est_position = []
lamda = []

class aoa_info(object):
    def __init__(self):
        self.imu_msg = Imu()
        self.gps_msg = Odometry()
        self.current_state = State()

        self.gps_pose = [0,0,0]
        self.ned_pose = [0,0,0]
        self.ob_pose = [0,0,0]
        self.imu_x = 0
        self.roll, self.pitch, self.yaw = 0,0,0
        self.quat = [0,0,0,0]
        self.u, self.v = 0, 0
        self.uu, self.vv = 50, 50
        self.a, self.b = 0, 0
        self.P_img_x, self.P_img_y, self.P_img_z = 0, 0, 0
        self.angle_a_w = 0
        self.angle_e_w = 0
        self.angle_a = [0, 0]
        self.angle_e = [0, 0]
        self.est_position = []
        self.est_x, self.est_y, self.est_z = 0, 0, 0
        self.lamda = 0

        self.next_pos = [0, 0, 0]
        self.heading = 0
        self.ob_point = [0, 0, 0]

        self.waypoint_1 = [0, 0]
        self.waypoint_2 = [0, 0]

        self.heading_1, self.heading_2 = 0, 0

        self.est_n, self.est_e, self.est_d = 0, 0, 0
        self.vector_n, self.vector_e, self.vector_d = 0, 0, 0
        self.last_req = rospy.Time.now()

        rospy.wait_for_service("/iris_fpv_cam_0/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/iris_fpv_cam_0/mavros/set_mode", SetMode)

        rospy.Subscriber("/iris_fpv_cam_0/mavros/state", State, self.state_cb)
        rospy.Subscriber("/iris_fpv_cam_0/mavros/imu/data", Imu, self.imu_callback)
        #drone position
        rospy.Subscriber("/iris_fpv_cam_0/mavros/global_position/local", Odometry, self.gps_callback)  

        self.pos_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pose = PoseStamped()
        self.setpoint_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_raw/local", PositionTarget, queue_size=1) 
        self.setpoint = PositionTarget()
     
        #self.setpoint = PositionTarget()
        #self.setpoint_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        self.position_min = self.ned_pose

    def state_cb(self, msg):
        self.current_state = msg

    def gps_callback(self, msg):
        self.gps_msg = msg
        self.gps_pose[0] = msg.pose.pose.position.x
        self.gps_pose[1] = msg.pose.pose.position.y
        self.gps_pose[2] = msg.pose.pose.position.z

        self.ned_pose[0], self.ned_pose[1], self.ned_pose[2] = self.ENU_to_NED(self.gps_pose[0], self.gps_pose[1], self.gps_pose[2])
        #print(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2])

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.quat[0] = msg.orientation.w
        self.quat[1] = msg.orientation.x
        self.quat[2] = msg.orientation.y
        self.quat[3] = msg.orientation.z

        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def cal_aoa_info(self):

        self.u, self.v = hsv.value_callback()

        if ([self.roll, self.pitch]<=[0.01, 0.01]) and ([np.abs(self.uu-self.gps_pose[0]), np.abs(self.vv-self.gps_pose[1])]>=[5, 5]):

            self.uu = self.gps_pose[0]
            self.vv = self.gps_pose[1]

            #print('self.u, self.v = ')
            #print(self.u, self.v)
            # position_vector
            size_u = 320
            size_v = 240
            u_0 = size_u/2
            v_0 = size_v/2
            # focal length
            f = 277.191356
            self.P_img_x = (v_0 - self.v)
            self.P_img_y = (self.u - u_0)
            self.P_img_z = f

            # observstion point
            self.ob_point[0] = self.ned_pose[0]
            self.ob_point[1] = self.ned_pose[1]
            self.ob_point[2] = self.ned_pose[2]

            # print()
            # print()
            # print('uav position ned')
            # print(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2])

            ###################### AOA ######################
            self.angle_a_w, self.angle_e_w, self.angle_a, self.angle_e, self.est_n, self.est_e, self.est_d, self.vector_n, self.vector_e, self.vector_d, self.lamda = AOA.AOA_v1(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2], self.roll, self.pitch, self.yaw, self.P_img_x, self.P_img_y, self.P_img_z)
            #print("---------------AOA----------------")
            # print('self.angle_a_w, self.angle_e_w = ')
            # print(self.angle_a_w, self.angle_e_w)
            # print('self.angle_a, self.angle_e = ')
            # print(self.angle_a, self.angle_e)

            #print('Target_position_world (ned) = ')
            #print(self.est_n, self.est_e, self.est_d)
        
            ############## collect data for observation point #####################
            est_position.append([self.est_n, self.est_e, self.est_d])
            azimuth.append(self.angle_a_w)
            azimuth_num = len(azimuth)
            #print('azimuth_num')
            #print(azimuth_num)
            ob_list.append([self.ob_point[0],self.ob_point[1],self.ob_point[2]])
            # print('azimuth = ')
            # print(azimuth)
            # print('ob_list = ')
            # print(ob_list)
            # print('est_position = ')
            # print(est_position)

    def mission(self):

        if len(est_position)==1:

            ###################### path planning ######################
            value = []

            next_position_list =  cal_dop.next_position(azimuth[0], est_position[0])
            # print("next_position_list : ")
            # print(next_position_list)

            for i in range (20):
                a = [next_position_list[i][0], next_position_list[i][1], next_position_list[i][2]]
                #print("next_position = ")
                #print(a)
                #n = [self.gps_pose[1], self.gps_pose[0], -self.gps_pose[2]]
                GDOP = cal_dop.calculate_dop(ob_list[0], a, est_position[0])
                value.append(GDOP)

            # print("GDOP_list = ")
            # print(value)

            ### Find observaiton point ###
            copy_list = copy.deepcopy(value)
            min_index_list = []
            min_position = []

            for _ in range(2):
                number = min(copy_list)
                index = copy_list.index(number)
                copy_list[index] = 10
                min_position.append(number)
                min_index_list.append(index)
            
            copy_list = []

            # print('min_index_list =')
            # print(min_index_list)
            # print('min_position =')
            # print(min_position)
            ##################################

            self.waypoint_1 = next_position_list[min_index_list[0]]
            self.waypoint_2 = next_position_list[min_index_list[1]]
            self.heading_1 = np.arctan2(-20-self.waypoint_1[1], 50-self.waypoint_1[0])
            self.heading_2 = np.arctan2(-20-self.waypoint_2[1], 50-self.waypoint_2[0])

            print("waypoint1, waypoint2 = ")
            print(self.waypoint_1, self.waypoint_2)
            print("heading1, heading2 = ")
            print(self.heading_1, self.heading_2)
            
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

    def iteration(self, event):

        if len(azimuth)<=1:
            self.cal_aoa_info()
            print('azimuth = ')
            print(azimuth)
            print('ob_list = ')
            print(ob_list)
            print('est_position = ')
            print(est_position)

        self.mission()

        w1 = self.waypoint_1
        w2 = self.waypoint_2

        h1 = self.heading_1
        h2 = self.heading_2

        # print('w1,w2 = ')
        # print(w1,w2)
        # print('h1,h2 = ')
        # print(h1,h2)

        #offb_pos.next_waypoint(self.waypoint_1, self.waypoint_2, self.waypoint_3, self.heading_1, self.heading_2, self.heading_3)
        if [w1, w2] != 0:
            if (self.current_state.mode != "OFFBOARD" and (rospy.Time.now()-self.last_req) > rospy.Duration(0.5)):
                offb_set_mode = SetModeRequest()
                offb_set_mode.custom_mode = 'OFFBOARD'
                if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                self.last_req = rospy.Time.now()

            else:
                #print('mission start')
                if np.sqrt(np.square(self.gps_pose[2]-10)+np.square(self.gps_pose[0]- 0)+np.square(self.gps_pose[1]- 0))<= 0.5:
                    print("Go to waypoint 1")
                    
                    self.pose.pose.position.x = w1[1]
                    self.pose.pose.position.y = w1[0]
                    self.pose.pose.position.z = 10
                    self.setpoint.yaw = h1
                    self.setpoint_pub.publish(self.setpoint) 
                    self.pos_pub.publish(self.pose)
                    
                elif np.sqrt(np.square(self.gps_pose[2]-10)+np.square(self.gps_pose[0]-w1[1])+np.square(self.gps_pose[1]-w1[0]))<= 0.5:

                    print("Go to waypoint 2")
                    self.cal_aoa_info()
                    self.pose.pose.position.x = w2[1]
                    self.pose.pose.position.y = w2[0]
                    self.pose.pose.position.z = 10
                    self.setpoint.yaw = h2
                    self.setpoint_pub.publish(self.setpoint)
                    self.pos_pub.publish(self.pose)

                elif np.sqrt(np.square(self.gps_pose[2]-10)+np.square(self.gps_pose[0]-w2[1])+np.square(self.gps_pose[1]-w2[0]))<= 0.5:
                    self.cal_aoa_info()

                    # ob_list.pop(0)
                    # azimuth.pop(0)

                    # print('azimuth = ')
                    # print(azimuth)
                    # print('ob_list = ')
                    # print(ob_list)

                    # ob_list.pop(0)
                    # azimuth.pop(0)

                    print('azimuth = ')
                    print(azimuth)
                    print('ob_list = ')
                    print(ob_list)

                    Est_Target_x, Est_Target_y, Est_Target_z = LeastQ.LeastQ(ob_list,azimuth) #ned
                    land_pos = [Est_Target_x, Est_Target_y, 10]

                    print('Target_position_world (ned) = ')
                    print(est_position[0])
                    print("Est_Target(NED) = ")
                    print(Est_Target_x, Est_Target_y, Est_Target_z)
                    
                    print('landing')
                    offb_land.land_waypoint(land_pos)

            self.setpoint_pub.publish(self.setpoint)
            self.pos_pub.publish(self.pose)

        enu_pos.append([self.gps_pose[0], self.gps_pose[1], self.gps_pose[2]])
        pos_command.append([self.next_pos[0], self.next_pos[1], 10])


if __name__ == '__main__':
    rospy.init_node('aoa_info_drone', anonymous=True)
    dt = 1.0/10
    #dt = 5
    pathplan_run = aoa_info()
    pathplan_run.mission()
    rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
    rospy.spin()
    
    df = DataFrame({'enu_pos': enu_pos})
    df.to_excel('drone_position.xlsx', sheet_name='sheet1', index=False)
    dd = DataFrame({'azimuth':azimuth,'ob_list': ob_list,'est_position': est_position})
    dd.to_excel('drone_result.xlsx', sheet_name='sheet1', index=False)



