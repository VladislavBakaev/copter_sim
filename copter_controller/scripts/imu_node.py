#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Imu

from complexTool import ImuData
from kalmanFilter import KalmanFilter


class ImuTest(Node):
    def __init__(self):
        super().__init__('imu_test')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                    depth=1)

        self.subscription = self.create_subscription(
            Imu,
            '/ins_imu_topic',
            self.listenerCallback,
            qos_profile=qos_policy)
        
        self.create_timer(0.5, self.printData)

        self.imu_sensor = ImuData()
        
    def listenerCallback(self, msg):
        self.imu_sensor.update_date([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    
    def printData(self):
        print("{0} {1} {2}".format(self.imu_sensor.northIns,\
                                    self.imu_sensor.eastIns,\
                                    self.imu_sensor.heightIns))

        # print("{0} {1} {2}".format(self.imu_sensor.linearNorthVel,\
        #                             self.imu_sensor.linearEastVel,\
        #                             self.imu_sensor.linearHeightVel))

def main():
    rclpy.init()
    node = ImuTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()