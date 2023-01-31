#!/usr/bin/env python

import rospy
# from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from math import *
from datetime import datetime

class test(object):
    def __init__(self):
        self.imu_msg = Imu()
        self.gps_msg = Odometry()
        self.gps_pose = [0,0,0]
        self.imu_x = 0
        self.roll, self.pitch, self.yaw = 0,0,0
        self.quat = [0,0,0,0]
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/mavros/global_position/local", Odometry, self.gps_callback)
        self.setpoint = PositionTarget()
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

    def gps_callback(self, msg):
        self.gps_msg = msg
        self.gps_pose[0] = msg.pose.pose.position.x
        self.gps_pose[1] = msg.pose.pose.position.y
        self.gps_pose[2] = msg.pose.pose.position.z

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.quat[0] = msg.orientation.w
        self.quat[1] = msg.orientation.x
        self.quat[2] = msg.orientation.y
        self.quat[3] = msg.orientation.z
        # self.roll = atan2(2.0 * (self.quat[3] * self.quat[2] + self.quat[0] * self.quat[1]), 1.0 - 2.0 * (self.quat[1] * self.quat[1] + self.quat[2] * self.quat[2]))*57.3
        # self.pitch = asin(2.0 * (self.quat[2] * self.quat[0] - self.quat[3] * self.quat[1]))*57.3
        # self.yaw = atan2(2.0 * (self.quat[3] * self.quat[0] + self.quat[1] * self.quat[2]), 1.0 - 2.0 * (self.quat[2] * self.quat[2] + self.quat[3] * self.quat[3]))*57.3
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # self.roll = atan2(2.0*(w*z + x*y), 1.0-2.0*(y*y + z*z))*57.3
        # self.pitch = asin(2.0 * (z*x - w*y))*57.3
        # self.yaw = atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (z*z + w*w))*57.3
        # pass

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
        # print('hey')
        self.setpoint.header.stamp = rospy.Time.now()
        print(datetime.utcfromtimestamp(rospy.Time.now().to_sec()))
	self.setpoint.position.x = 1
	self.setpoint.position.y = 2
	self.setpoint.position.z = 3
        self.setpoint_pub.publish(self.setpoint)
        # print(self.imu_msg.orientation.x)
        # print(self.imu_x)
        print(self.roll*57.3, self.pitch*57.3, self.yaw*57.3)
        print(self.gps_pose)
        print('\n')
        # pass

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    dt = 1.0/5
    pathplan_run = test()
    rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
    rospy.spin()
