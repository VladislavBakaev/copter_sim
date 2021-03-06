#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from math import sin, cos, sqrt, pi
from sensor_msgs.msg import NavSatFix
import numpy as np
import os
import json

class AionPlugin(Node):
    def __init__(self):
        super().__init__('aion_plugin')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)

        self.loadParam()

        self.a = 6378206.4
        self.b = 6356583.8
        self.e_2 = 1 - self.b**2/self.a**2

        self.subscription = self.create_subscription(
            NavSatFix,
            '/ideal_gps',
            self.listenerCallback,
            qos_profile=qos_policy)

        self.pub_aion = self.create_publisher(NavSatFix, '/aion_nav_topic', 10)
        self.msg = NavSatFix()

        #probabilistic param
        self.w1 = 0.5
        self.w2 = 0.9
        self.w3 = 1
        self.accuracy_blowout_k = 2
        self.acuuracy_max_blowout_k = 3


        self.create_timer(1/self.rate, self.publishAion)
        
    def listenerCallback(self, gps_msg):
        self.msg = gps_msg

    def probabilisticModel(self, msg):

        err = [0, 0]

        accuracy_grad = np.array(self.accuracy)/self.getCurrentRadius(msg.latitude, msg.longitude, msg.altitude)*180/pi

        for i, _ in enumerate(err):
            w = np.random.rand()
            if w < self.w1:
                err[i] = np.random.normal(0, accuracy_grad[i]/3)
            elif w < self.w2:
                err[i] = 2*self.accuracy_blowout_k*accuracy_grad[i]*np.random.rand()-self.accuracy_blowout_k*accuracy_grad[i]
            else:
                err[i] = 2*self.acuuracy_max_blowout_k*accuracy_grad[i]*np.random.rand()-self.acuuracy_max_blowout_k*accuracy_grad[i]
        
        msg.latitude += err[0]
        msg.longitude += err[1]

        return msg

    def publishAion(self):
        self.msg = self.probabilisticModel(self.msg)

        self.msg.altitude = 0.0

        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_aion.publish(self.msg)
    
    def loadParam(self):
        config_file = os.path.join(get_package_share_directory('copter_model'),
                        'config', 'aion_config.json')

        with open(config_file, 'r') as json_file:
            data = json.load(json_file)
            self.accuracy = [data['accuracy_lat'], data['accuracy_lon']]
            self.rate = data['rate']
    
    def getCurrentRadius(self, lat, lon, alt):
        N = self.a/(sqrt(1-self.e_2*sin(lat)**2))

        x = (N + alt)*cos(lat)*cos(lon)
        y = (N + alt)*cos(lat)*sin(lon)

        z = (self.b**2/self.a**2*N + alt)*sin(lat)

        return sqrt(x**2 + y**2 + z**2)


def main():
    rclpy.init()
    node = AionPlugin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()