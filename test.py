#! /usr/bin/env python
# coding=utf-8
import robotcontrol
import math
import serial
import struct
from inspire_gripper import *
ser=serial.Serial('/dev/ttyUSB2',115200,timeout=None)
ser.timeout = 0.01
ser.isOpen()
movemax(1000)
time.sleep(3)
movemin(1000,1000)
time.sleep(3)
def robot_login():
    ret = robotcontrol.Auboi5Robot().initialize()
    print("Auboi5Robot().initialize() is {0}".format(ret))
    robot = robotcontrol.Auboi5Robot()
    handle = robot.create_context()
    ip = '192.168.5.20'
    port =8899
    result =robot.connect(ip,port)
    if (result==0):
        collision=6
        tool_dynamics = {"position":(0,0,0),"payload":0.0,"intertia":(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}
        ret = robot.robot_startup(collision,tool_dynamics)
        print("robot_startup ret is {0}".format(ret))
        #关节运动
        #初始化全局运动
        robot.init_profile()
        robot.set_joint_maxacc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))
        robot.set_joint_maxvelc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))
        joint =(math.radians(-4),math.radians(6),math.radians(-85),
                math.radians(-3),math.radians(-88),math.radians(-82.682980))
        ret =robot.move_joint(joint)
        print("robot move_joint ret is {0}".format(ret))

    else:
        print("login failed!")
if __name__ =='__main__':
    robot_login()
    #串口设置
    ser=serial.Serial('/dev/ttyUSB2',115200,timeout=None)
    ser.timeout = 0.01
    ser.isOpen()
    movemax(1000)
    time.sleep(3)
    movemin(1000,1000)
    time.sleep(3)
