#! /usr/bin/env python
# coding=utf-8
from robotcontrol import *
import robotcontrol
import math
import serial
import logging
import struct
import os
from inspire_gripper import *
import time
import libpyauboi5
import logging
from logging.handlers import RotatingFileHandler
from multiprocessing import Process, Queue
import os
from math import pi
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


def test1(test_count=1):
    
    try:

        # 链接服务器
        ip = '192.168.5.20'
        #ip = '192.168.199.200'
        ret = robotcontrol.Auboi5Robot().initialize()
        robot = robotcontrol.Auboi5Robot()
        handle = robot.create_context()
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            #robot.robot_shutdown()
            #
            # # 上电
            robot.robot_startup()
            #
            # # 设置碰撞等级
            robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            #robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            #tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            #logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            #robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)


            # 获取控制柜用户DO
            #io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            #logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            #logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # 循环测试
            while test_count > 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                logger.info("joint_status={0}".format(joint_status))

                # 初始化全局配置文件
                robot.init_profile()

                # 设置关节最大加速度
                robot.set_joint_maxacc((1, 1, 1, 1, 1, 1))

                # 设置关节最大加速度
                robot.set_joint_maxvelc((1, 1, 1, 1, 1, 1))

                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                logger.info("move joint to {0}".format(joint_radian))

                robot.move_joint(joint_radian)

                # 获取关节最大加速度
                logger.info(robot.get_joint_maxacc())

                # 正解测试
                fk_ret = robot.forward_kin((-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008))
                logger.info(fk_ret)

                # 逆解
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                ik_result = robot.inverse_kin(joint_radian, fk_ret['pos'], fk_ret['ori'])
                logger.info(ik_result)

                # 轴动1
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # 轴动2
                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # 轴动3
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.5)

                # 获取机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_velc(0.2)

                # 清除所有已经设置的全局路点
                robot.remove_all_waypoint()

                # 添加全局路点1,用于轨迹运动
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                robot.add_waypoint(joint_radian)

                # 添加全局路点2,用于轨迹运动
                joint_radian = (-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680)
                robot.add_waypoint(joint_radian)

                # 添加全局路点3,用于轨迹运动
                joint_radian = (-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191)
                robot.add_waypoint(joint_radian)

                # 设置圆运动圈数
                robot.set_circular_loop_times(3)

                # 圆弧运动
                logger.info("move_track ARC_CIR")
                robot.move_track(RobotMoveTrackType.ARC_CIR)

                # 清除所有已经设置的全局路点
                robot.remove_all_waypoint()

                # 机械臂轴动 回到0位
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))


    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

if __name__ =='__main__':
    test1(1)
    #串口设置
    ser=serial.Serial('/dev/ttyUSB2',115200,timeout=None)
    ser.timeout = 0.01
    ser.isOpen()
    movemax(1000)
    time.sleep(3)
    movemin(1000,1000)
    time.sleep(3)
