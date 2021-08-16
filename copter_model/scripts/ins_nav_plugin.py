#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix
import numpy as np
import os
import json

class InsNavPlugin(Node):
    def __init__(self):
        super().__init__('ins_nav_plugin')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.loadParam()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)

        self.subscription = self.create_subscription(
            NavSatFix,
            '/ideal_gps',
            self.listenerCallback,
            qos_profile=qos_policy)

        self.pub_ins = self.create_publisher(NavSatFix, '/ins_nav_topic', 10)
        
        self.msg = NavSatFix()

        # #probabilistic param
        self.w1 = 0.5
        self.w2 = 0.9
        self.w3 = 1
        self.accuracy_blowout_k = [2, 2, 0]
        self.acuuracy_max_blowout_k = [3, 3, 0]

        self.create_timer(1/self.rate, self.publishInsNav)
        
    def listenerCallback(self, gps_msg):
        self.msg = gps_msg

    def probabilisticModel(self, msg):

        err = [0, 0, 0]

        for i, _ in enumerate(err):
            w = np.random.rand()
            if w < self.w1:
                err[i] = np.random.normal(0, self.accuracy[i]/3)
            elif w < self.w2:
                err[i] = 2*self.accuracy_blowout_k[i]*self.accuracy[i]*np.random.rand()-self.accuracy_blowout_k[i]*self.accuracy[i]
            else:
                err[i] = 2*self.acuuracy_max_blowout_k[i]*self.accuracy[i]*np.random.rand()-self.acuuracy_max_blowout_k[i]*self.accuracy[i]
        
        msg.latitude += err[0]
        msg.longitude += err[1]
        msg.altitude += err[2]

        return msg

    def publishInsNav(self):
        self.msg = self.probabilisticModel(self.msg)

        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_ins.publish(self.msg)
    
    def loadParam(self):
        config_file = os.path.join(get_package_share_directory('copter_model'),
                        'config', 'ins_config.json')

        with open(config_file, 'r') as json_file:
            data = json.load(json_file)
            self.accuracy = [data['accuracy_lat'], data['accuracy_lon'], data['accuracy_alt']]
            self.rate = data['rate']


def main():
    rclpy.init()
    node = InsNavPlugin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()
