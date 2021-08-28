#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory
import math

import numpy as np
import os
import json

class InsImuPlugin(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('ins_imu_plugin')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
    
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.loadParam()

        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin',
            self.listenerCallback,
            qos_profile=qos_policy)

        self.msg = Imu()

        self.pub_ins = self.create_publisher(Imu, '/ins_imu_topic', 10)
        self.rate = 10

        self.create_timer(1/self.rate, self.publishInsImu)
        
    def listenerCallback(self, imu_msg):
        self.msg = imu_msg

    def probabilisticModel(self, msg):

        orient_err = [0, 0, 0]
        angular_vel_err = [0, 0, 0]
        linear_acc_err = [0, 0, 0]

        for i in range(len(orient_err)):
            orient_err[i] = np.random.normal(0.0, self.accuracy_orientation/3)
            angular_vel_err[i] = np.random.normal(0.0, self.accuracy_angular_velocity/3)
            linear_acc_err[i] = np.random.normal(0.0, self.accuracy_linear_acceleration/3)

        rpy = self.euleFromQuaternion(msg.orientation)

        rpy = np.array(rpy) + np.array(orient_err)

        q = self.quaternionFromEuler(*rpy.tolist())

        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]
        msg.orientation.w = q[0]

        msg.angular_velocity.x += angular_vel_err[0]
        msg.angular_velocity.y += angular_vel_err[1]
        msg.angular_velocity.z += angular_vel_err[2]

        msg.linear_acceleration.x = msg.linear_acceleration.x + linear_acc_err[0] 
        msg.linear_acceleration.y = msg.linear_acceleration.y + linear_acc_err[1]
        msg.linear_acceleration.z = msg.linear_acceleration.z + linear_acc_err[2]

        return msg

    def publishInsImu(self):
        self.msg = self.probabilisticModel(self.msg)
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_ins.publish(self.msg)
    
    def loadParam(self):
        config_file = os.path.join(get_package_share_directory('copter_model'),
                        'config', 'ins_config.json')

        with open(config_file, 'r') as json_file:
            data = json.load(json_file)
            self.rate = data['rate']
            self.accuracy_orientation = data['accuracy_orientation']
            self.accuracy_angular_velocity = data['accuracy_angular_velocity']
            self.accuracy_linear_acceleration = data['accuracy_linear_acceleration']

    def euleFromQuaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]
    
    def quaternionFromEuler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
        
def main():
    node = InsImuPlugin()
    rclpy.spin(node)

if __name__ == '__main__':
    main()