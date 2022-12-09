#! /usr/bin/env python
# coding=utf-8

import robotcontrol
import math
import serial
import struct
from math import pi

import numpy as np
import transforms3d as tfs


import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker

# from inspire_gripper import *

# # 夹爪连接
# ser=serial.Serial('/dev/ttyUSB2',115200,timeout=None)
# ser.timeout = 0.01
# ser.isOpen()

# # 机械臂夹爪控制
# movemax(1000)
# time.sleep(3)
# movemin(1000,1000)
# time.sleep(3)

class robot_ar():
    def __init__(self):

        # 初始化ROS节点
        rospy.init_node('auboe5_node', log_level=rospy.INFO)
        #   订阅AR码话题
        rospy.Subscriber('/aruco_tracker/pose', PoseStamped, self.AR_callback)

        #   订阅机械臂控制码话题
        rospy.Subscriber('/aubo_con', Int32, self.AUBO_callback)
        self.position, self.orientation = None, None
        #   初始化
        self.init = robotcontrol.Auboi5Robot().initialize()
        print("Auboi5Robot().initialize() is {0}".format(self.init))
        #   导入接口库并且实例化机械臂控制类
        self.robot = robotcontrol.Auboi5Robot()
        self.handle = self.robot.create_context()
        self.ar_flag = 0
        self.ip = '192.168.3.20'
        self.port = 8899
        #   碰撞等级范围(0~10) 缺省：6
        self.collision = 6 
        #   运动学参数（位置，负载，惯量）
        self.tool_dynamics = {"position":(0,0,0),"payload":3.0,"intertia":(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)} 

        #   初始化机械臂控制全局属性
        self.robot.init_profile()

        #   机械臂连接
        self.result = self.robot.connect(self.ip, self.port)
        #   设置机械臂末端最大线加速度
        self.robot.set_end_max_line_acc(0.1)
        #   设置机械臂末端最大线速度
        self.robot.set_end_max_line_velc(0.1)
        #   设置机械臂末端最大角加速度
        self.robot.set_end_max_angle_acc(0.1)
        #   设置机械臂末端最大角速度
        self.robot.set_end_max_angle_velc(0.1)

        #   启动机械臂
        self.ret = self.robot.robot_startup(self.collision, self.tool_dynamics)
        print("robot_startup ret is {0}".format(self.ret))

        if (self.result == 0):
            #   给出笛卡尔坐标值和欧拉角，机械臂轴动到目标位置和姿态,pos:位置坐标（x，y，z），单位(m),rpy：欧拉角（rx，ry，rz）,单位（度）
            self.robot_status_pose = self.robot.move_to_target_in_cartesian([-0.5718853324898443, -0.10799053406374882, 0.42713791278609834],[180, 0, 0])
            # 获取当前机械臂姿态
            self.robot_pose = self.robot.get_current_waypoint()

        while (self.result == 0):
            if self.ar_flag == 1:
                # #  base to wrist 齐次变换矩阵
                self.T_BaseToWrist = self.StaPose_To_HomTranMatrix(self.robot_pose)

                # print(self.position,self.orientation)
                # #  cam to AR 齐次变换矩阵
                self.CamToAR = self.Pose_To_HomTranMatrix(self.position, self.orientation)
                # base to ar齐次变换矩阵，控制机械臂移动到AR码位置
                self.robot_pose = self.HomTranMatrix_To_Move(self.T_BaseToWrist, self.CamToAR)
                self.ar_flag = 0
                self.robot_status_pose = self.robot.move_to_target_in_cartesian([-0.5718853324898443, -0.10799053406374882, 0.42713791278609834],[180, 0, 0])
                self.robot_pose = self.robot.get_current_waypoint()
                
    def AR_callback(self, data):
        """"
        * FUNCTION:    AR_callback
        * DESCRIPTION: AR码位姿获取回调函数
        * INPUTS:      pose
        * OUTPUTS:     
        * RETURNS:     成功返回: None
        *              失败返回: 其他
        * NOTES:
        """
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        ow = data.pose.orientation.w
        ox = data.pose.orientation.x
        oy = data.pose.orientation.y
        oz = data.pose.orientation.z
        self.position = (x, y, z)
        self.orientation = (ow, ox, oy, oz)

    def AUBO_callback(self, data):
        """"
        * FUNCTION:    AUBO_callback
        * DESCRIPTION: 机械臂控制信号获取回调函数
        * INPUTS:      data
        * OUTPUTS:     
        * RETURNS:     成功返回: None
        *              失败返回: 其他
        * NOTES:
        """
        self.ar_flag = data.data


    def Pose_To_HomTranMatrix(self, Pos_vector, Quaternion):
        """"
        * FUNCTION:    Pose_To_HomTranMatrix
        * DESCRIPTION: 位姿转换成齐次变换矩阵
        * INPUTS:      位置向量Pos_vector（x, y, z）单位为(m);四元数Quaternion（w, x, y, z）
        * OUTPUTS:     齐次变换矩阵Hom_tran_matrix
        * RETURNS:     成功返回: Hom_tran_matrix
        *              失败返回: 其他
        * NOTES:
        """

        # 四元数转换成旋转矩阵(w,x,y,z)
        Rot_matrix = tfs.quaternions.quat2mat(Quaternion)
        Hom_tran_matrix = tfs.affines.compose(Pos_vector, Rot_matrix, [1,1,1])
        return Hom_tran_matrix

    def StaPose_To_HomTranMatrix(self, robot_pose):
        """"
        * FUNCTION:    StaPose_To_HomTranMatrix
        * DESCRIPTION: 当前相机相对于机械臂状态转换成齐次变换矩阵
        * INPUTS:      robot_pose，通过get_current_waypoint()函数获取的机械臂末端状态robot_pose
        * OUTPUTS:     齐次变换矩阵Hom_tran_matrix
        * RETURNS:     成功返回: Hom_tran_matrix
        *              失败返回: 其他
        * NOTES:
        """

        T_BaseToWrist = self.Pose_To_HomTranMatrix(robot_pose['pos'], robot_pose['ori'])
        # print(self.T_BaseToWrist)

        #  相机手眼标定的欧拉角转四元数,欧拉角(rpy),四元数(w, x, y, z)
        camera_ori = self.robot.rpy_to_quaternion([-0.6701801445821992, -1.5493313821216614, -0.8248594513756673])
        # 相机相对于机械臂末端的齐次变换矩阵
        T_WristToCam = self.Pose_To_HomTranMatrix([0.01871432659445273, 0.10446660197697127, 0.006906680525053534], camera_ori)
        # print(self.T_WristToCam)

        # base to cam齐次变换矩阵
        T_BaseToCam = np.dot(T_BaseToWrist, T_WristToCam)
        return T_BaseToCam

    def HomTranMatrix_To_Move(self, HomTranMatrix_start, HomTranMatrix_end):
        """"
        * FUNCTION:    HomTranMatrix_To_Move
        * DESCRIPTION: 控制机械臂运动到指定的目标pose
        * INPUTS:      HomTranMatrix_start(起始齐次变换矩阵);HomTranMatrix_end(目标齐次变换矩阵)。
        * OUTPUTS:      
        * RETURNS:     成功返回: robot_pose
        *              失败返回: 其他
        * NOTES:
        """
        # 矩阵相乘
        T_StartToEnd = np.dot(HomTranMatrix_start, HomTranMatrix_end)
        # 分解为固定轴欧拉角和平移向量
        rpy, pose = tfs.euler.mat2euler(T_StartToEnd[0:3,0:3]), T_StartToEnd[:3,3:4]
        # 考虑到爪相对于机械臂末端有一个偏移量，整体往上移动0.2m
        pose[2] = pose[2] + 0.20
        print(type(pose))
        #   tuple转numpy.ndarray
        rpy = np.array(rpy, dtype=float)

        print(type(rpy))
        print(rpy)
        #   弧度转角度
        rpy[0] = (rpy[0] * 180) / pi + 180
        rpy[1] = (rpy[1] * 180) / pi
        # 末端夹爪变换
        rpy[2] = (rpy[2] * 180) / pi - 43.89
        print(rpy)
        #   给出笛卡尔坐标值和欧拉角，机械臂轴动到目标位置和姿态,pos:位置坐标（x，y，z），单位(m),rpy：欧拉角（rx，ry，rz）,单位（度）
        # self.robot.move_to_target_in_cartesian(pose, [180, 0, 0])
        self.robot.move_to_target_in_cartesian(pose, rpy)

        robot_pose = self.robot.get_current_waypoint()
        return robot_pose

if __name__ =='__main__':
    robot_ar()

