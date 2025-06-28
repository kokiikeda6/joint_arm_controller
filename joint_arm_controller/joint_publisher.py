#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import math
from rclpy.qos import QoSProfile
from piper_sdk import C_PiperInterface_V2
from sensor_msgs.msg import  JointState
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class PiperArmNode(Node):
    def __init__(self):
        super().__init__('piper_arm_node')

        range_joint1 = FloatingPointRange(
            from_value=-150.0,
            to_value=150.0,
            step=0.0001
        )

        # range_joint1 = FloatingPointRange(
        #     from_value=-2.6179,
        #     to_value=2.6179,
        #     step=0.0001
        # )

        descriptor_joint1 = ParameterDescriptor(
            description='joint1 value between -2.6179 and 2.6179',
            floating_point_range=[range_joint1]
        )

        range_joint4 = FloatingPointRange(
            from_value=-1.745,
            to_value=1.745,
            step=0.0001
        )

        descriptor_joint4 = ParameterDescriptor(
            description='joint4 value between -1.745 and 1.745',
            floating_point_range=[range_joint4]
        )

        self.declare_parameter('joint1', 0.0, descriptor_joint1)
        self.declare_parameter('joint4', 0.0, descriptor_joint4)
        self.declare_parameter('joint6', 0.0)

        self.add_on_set_parameters_callback(self.change_params)

        self.joint1 = self.get_parameter('joint1').get_parameter_value().double_value
        self.joint4 = self.get_parameter('joint4').get_parameter_value().double_value
        self.joint6 = self.get_parameter('joint6').get_parameter_value().double_value

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
            self.get_logger().info(f'接続状態: {enable_flag}')
            self.piper.GripperCtrl(0, 1000, 0x01, 0)

            if elapsed_time > timeout:
                self.get_logger().error('接続失敗、ノードを終了します')
                rclpy.shutdown()
                return
            time.sleep(1)

    def callback_joint_control(self, msg):
        self.joint = msg.position

        self.get_logger().info("received joint")

        # リーダフォロワ
        # self.position = [self.joint[0], self.joint[1], self.joint[2], self.joint[3], self.joint[4], self.joint[5], self.joint[6]]
        
        # ジョイント制限リーダフォロワ
        self.position = [self.joint1*math.pi/180, self.joint[1], self.joint[2], self.joint4, self.joint[4], self.joint6, self.joint[6]]


        joint_values = [round(j * self.factor) for j in self.position[:6]]
        # joint_6 = round(self.position[6] * 1000 * 1000)
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(*joint_values)
        # self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

        status = self.piper.GetArmStatus()
        self.get_logger().debug(f"Status: {status}")
        self.get_logger().debug(f"Position: {self.position}")

    def change_params(self, data):
        for parameter in data:
            if parameter.name == "joint1":
                if parameter.type_ == Parameter.Type.DOUBLE:
                    self.joint1 = parameter.value
        self.get_logger().warn("parameter changed... {}".format(self.joint1))
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = PiperArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
