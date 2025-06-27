#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
from rclpy.qos import QoSProfile
from piper_sdk import C_PiperInterface_V2
from sensor_msgs.msg import  JointState

class PiperArmNode(Node):
    def __init__(self):
        super().__init__('piper_arm_node')
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        self.factor = 57324.840764  # 1000 * 180 / 3.14
        self.position = [0, 0, 0, 0, 0, 0, 0]
        self.enable_fun()

        self.create_subscription(
            JointState,
            "/joint_states_single",
            self.callback_joint_control,
            qos_profile=QoSProfile(depth=10)
        )

    def enable_fun(self):
        '''
        使能機能：機械アームの電源投入確認と初期化
        '''
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False

        self.piper.EnableArm(7)

        while not enable_flag:
            elapsed_time = time.time() - start_time
            info = self.piper.GetArmLowSpdInfoMsgs()
            enable_flag = (
                info.motor_1.foc_status.driver_enable_status and
                info.motor_2.foc_status.driver_enable_status and
                info.motor_3.foc_status.driver_enable_status and
                info.motor_4.foc_status.driver_enable_status and
                info.motor_5.foc_status.driver_enable_status and
                info.motor_6.foc_status.driver_enable_status
            )
            self.get_logger().info(f'使能状態: {enable_flag}')
            self.piper.GripperCtrl(0, 1000, 0x01, 0)

            if elapsed_time > timeout:
                self.get_logger().error('使能超時、ノードを終了します')
                rclpy.shutdown()
                return
            time.sleep(1)

    def callback_joint_control(self, msg):
        joint = msg.position

        self.get_logger().info("received joint")
        self.position = [joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], joint[6]]

        joint_values = [round(j * self.factor) for j in self.position[:6]]
        # joint_6 = round(self.position[6] * 1000 * 1000)
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(*joint_values)
        # self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

        status = self.piper.GetArmStatus()
        self.get_logger().debug(f"Status: {status}")
        self.get_logger().debug(f"Position: {self.position}")

def main(args=None):
    rclpy.init(args=args)
    node = PiperArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
