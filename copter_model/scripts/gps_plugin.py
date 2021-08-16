#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import NavSatFix

from math import sin, cos, sqrt

class GpsPlugin(Node):

    def __init__(self):
        super().__init__('gps_plugin')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._output_timer = self.create_timer(1/1000, self.on_timer)
        self.pub_gsp = self.create_publisher(NavSatFix, '/ideal_gps', 10)

        self.a = 6378206.4
        self.b = 6356583.8
        self.e_2 = 1 - self.b**2/self.a**2

        self.start_f = 55.773037
        self.start_lam = 37.698766
        self.start_height = 0
        self.diff_f = 0
        self.diff_lam = 0
        self.diff_height = 0
        self.msg = NavSatFix()

    def on_timer(self):
        from_frame_rel = 'world'
        to_frame_rel = 'copter_actual'

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

        current_radius = self.get_current_radius()

        self.diff_f = trans.transform.translation.x/current_radius
        self.diff_lam = trans.transform.translation.y/current_radius
        self.diff_height = -trans.transform.translation.z

        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.latitude = self.diff_f + self.start_f
        self.msg.longitude = self.diff_lam + self.start_lam
        self.msg.altitude = self.start_height + self.diff_height
        self.pub_gsp.publish(self.msg)
    
    def get_current_radius(self):
        N = self.a/(sqrt(1-self.e_2*sin(self.start_f+self.diff_f)**2))

        x = (N + self.start_height + self.diff_height)*cos(self.start_f+self.diff_f)*cos(self.start_lam+self.diff_lam)
        y = (N + self.start_height + self.diff_height)*cos(self.start_f+self.diff_f)*sin(self.start_lam+self.diff_lam)

        z = (self.b**2/self.a**2*N + self.start_height + self.diff_height)*sin(self.start_f+self.diff_f)

        return sqrt(x**2 + y**2 + z**2)


def main():
    rclpy.init()
    node = GpsPlugin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()