#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Range
import os
import json
import numpy as np

class LaserDistPlugin(Node):
    def __init__(self):
        super().__init__('laser_dist_plugin')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)
        
        self.loadParam()
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ideal_gps',
            self.listenerCallback,
            qos_profile=qos_policy)

        self.pub_ins = self.create_publisher(Range, '/laser_dist_topic', 10)

        self.range_ = 0.0

        #probabilistic param
        self.w1 = 0.85
        self.w2 = 0.97
        self.w3 = 1

        self.create_timer(1/self.rate, self.publishLaser)
        
    def listenerCallback(self, gps_msg):
        range_ = gps_msg.altitude
        if range_ < self.min_distance:
            self.range_ = self.max_distance
        else:
            self.range_ = range_

    def probabilisticModel(self, range_):
        range_new = 0.0
        w = np.random.rand()
        if w < self.w1:
            range_new = np.random.normal(range_, self.accuracy/3)
        elif w < self.w2:
            range_new = (range_-self.min_distance)*np.random.rand() + self.min_distance
        else:
            renge_new = 5*np.random.rand() + self.max_distance-5

        return range_new

    def publishLaser(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.range_ == self.max_distance:
            msg.range = self.range_
        else:
            msg.range = self.probabilisticModel(self.range_)
        self.pub_ins.publish(msg)
    
    def loadParam(self):
        config_file = os.path.join(get_package_share_directory('copter_model'),
                        'config', 'laser_config.json')

        with open(config_file, 'r') as json_file:
            data = json.load(json_file)
            self.accuracy = data['accuracy']
            self.rate = data['rate']
            self.min_distance = data['min_distance']
            self.max_distance = data['max_distance']


def main():
    rclpy.init()
    node = LaserDistPlugin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()