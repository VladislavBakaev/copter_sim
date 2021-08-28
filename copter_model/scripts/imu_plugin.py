#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import time

class ImuPlugin(Node):

    def __init__(self):
        super().__init__('imu_plugin')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._output_timer = self.create_timer(1/10, self.on_timer)

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)

        self.imuPublisher = self.create_publisher(Imu, '/imu_plugin', 10)
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.listenerCallback,
            qos_profile=qos_policy)


        self.last_transform = TransformStamped()
        self.last_time = time.time()

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0

        self.x_acc = 0.0
        self.y_acc = 0.0
        self.z_acc = 0.0

    def on_timer(self):
        from_frame_rel = 'copter_actual'
        to_frame_rel = 'world'

        try:
            now = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now,
                timeout=Duration(seconds=1.0))
        except LookupException:
            self.get_logger().info('transform not ready')
            return
        current_time = time.time()

        time_step = current_time - self.last_time

        delta_x = trans.transform.translation.x - self.last_transform.transform.translation.x
        delta_y = trans.transform.translation.y - self.last_transform.transform.translation.y
        delta_z = trans.transform.translation.z - self.last_transform.transform.translation.z

        x_vel = delta_x/time_step
        x_acc = (x_vel - self.x_vel)/time_step

        y_vel = delta_y/time_step
        y_acc = (y_vel - self.y_vel)/time_step

        z_vel = delta_z/time_step
        z_acc = (z_vel - self.z_vel)/time_step

        self.x_vel = x_vel
        self.x_acc = x_acc

        self.y_vel = y_vel
        self.y_acc = y_acc

        self.z_vel = z_vel
        self.z_acc = z_acc

        self.last_transform = trans
        self.last_time = current_time

    def listenerCallback(self, msg):
        msg.linear_acceleration.x = self.x_acc
        msg.linear_acceleration.y = self.y_acc
        msg.linear_acceleration.z = self.z_acc

        self.imuPublisher.publish(msg)

def main():
    rclpy.init()
    node = ImuPlugin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()