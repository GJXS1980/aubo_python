#! /usr/bin/env python
# coding=utf-8
import robotcontrol
import math
import serial
import struct
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

def robot_login():
    #   初始化
    ret = robotcontrol.Auboi5Robot().initialize()
    print("Auboi5Robot().initialize() is {0}".format(ret))
    #   导入接口库并且实例化机械臂控制类
    robot = robotcontrol.Auboi5Robot()
    handle = robot.create_context()
    ip = '192.168.3.20'
    port =8899
    #   机械臂连接
    result =robot.connect(ip,port)

    if (result==0):
        #   碰撞等级范围(0~10) 缺省：6
        collision=6 
        #   运动学参数（位置，负载，惯量）
        tool_dynamics = {"position":(0,0,0),"payload":3.0,"intertia":(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)} 
        #   启动机械臂
        ret = robot.robot_startup(collision,tool_dynamics)
        print("robot_startup ret is {0}".format(ret))
        #   关节运动
        #   初始化机械臂控制全局属性
        robot.init_profile()
        #   设置六个关节的最大加速度
        robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
        #   设置六个关节的最大速度
        robot.set_joint_maxvelc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))

        joint =(math.radians(-4),math.radians(6),math.radians(-85),
                math.radians(-3),math.radians(-88),math.radians(-82.682980))
        #   机械臂轴动
        # ret =robot.move_joint(joint)
        # # print("robot move_joint ret is {0}".format(ret))
        # #   获取机械臂当前位置信息
        # robot_pose = robot.get_current_waypoint()
        
        # print(robot_pose)
        # #   四元数转欧拉角,四元数(w, x, y, z)
        # robot_ori = robot.quaternion_to_rpy(robot_pose['ori'])
        # print(robot_ori)

        #   给出笛卡尔坐标值和欧拉角，机械臂轴动到目标位置和姿态,pos:位置坐标（x，y，z），单位(m),rpy：欧拉角（rx，ry，rz）,单位（度）
        robot.move_to_target_in_cartesian([-0.5718853324898443, -0.10799053406374882, 0.42713791278609834],[180, 0, 0])

        robot_pose = robot.get_current_waypoint()
        print(robot_pose)

        # robot.move_to_target_in_cartesian(robot_pose['pos'],[180, 0, 0])
        # robot_pose = robot.get_current_waypoint()
        # print(robot_pose)

    else:
        print("login failed!")



        
if __name__ =='__main__':
    robot_login()
    #串口设置
    # ser=serial.Serial('/dev/ttyUSB2',115200,timeout=None)
    # ser.timeout = 0.01
    # ser.isOpen()
    # movemax(1000)
    # time.sleep(3)
    # movemin(1000,1000)
    # time.sleep(3)
