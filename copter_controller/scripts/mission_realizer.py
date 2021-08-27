#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Int16
from sensor_msgs.msg import NavSatFix
import os
import re
import time

class MissionRealizer(Node):

    def __init__(self):
        super().__init__('mission_realizer')
        self.mission_point = {}
        self.loadMission()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)

        self.subscription = self.create_subscription(
            Int16,
            '/copter_status',
            self.listenerCopterStateCallback,
            qos_profile=qos_policy)
        
        self.pointPublisher = self.create_publisher(NavSatFix, '/target_nav_topic', 10)
        self.create_timer(0.5, self.realize_mission)
        self.copter_status = 0
        self.current_point_ind = 0

    def loadMission(self):
        config_file = os.path.join(get_package_share_directory('copter_controller'),
                'mission', 'mission.txt')

        with open(config_file, 'r') as txt_file:
            data = txt_file.read()
            data = re.findall(r'\[(\d+)\]\s(.*)\s(.*)\s(.*)\s(.*)\s(.*)', data)
            for point in data:
                self.mission_point[point[0]] = {}
                for i in range(1, len(point)):
                    param = point[i].split('=')
                    self.mission_point[point[0]].update({param[0]:float(param[1])})
            
            self.iter_seq = sorted(self.mission_point.keys())

    def realize_mission(self):
        print(self.current_point_ind)
        if self.copter_status == 0 and self.current_point_ind != -1:
            if self.current_point_ind == len(self.iter_seq):
                self.current_point_ind = -1
            else:
                msg  = NavSatFix()
                point = self.mission_point[self.iter_seq[self.current_point_ind]]
                msg.latitude = point['Lat']
                msg.longitude = point['Lon']
                msg.altitude = point['Alt']
                msg.position_covariance[0] = point['MaxHSpeed']
                msg.position_covariance[1] = point['MaxVSpeed']
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pointPublisher.publish(msg)
                self.current_point_ind += 1

    def listenerCopterStateCallback(self, msg):
        self.copter_status = msg.data

def main():
    rclpy.init()
    node = MissionRealizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()