#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

from kalmanFilter import KalmanFilter

class ComplexData(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('complex_data_plot')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
    
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.subscription = self.create_subscription(
            NavSatFix,
            '/ins_nav_topic',
            self.listenerInsNavCallback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            Imu,
            '/ins_imu_topic',
            self.listenerInsImuCallback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            NavSatFix,
            '/aion_nav_topic',
            self.listenerAionCallback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            Range,
            '/laser_dist_topic',
            self.listenerLaserCallback,
            qos_profile=qos_policy)

        self.filtredInsNavPub = self.create_publisher(NavSatFix, '/filter/ins_nav_topic', 10)
        self.filtredInsImuPub = self.create_publisher(Imu, '/filter/ins_imu_topic', 10)
        self.filtredAionPub = self.create_publisher(NavSatFix, '/filter/aion_nav_topic', 10)
        self.filtredLaserPub = self.create_publisher(Range, '/filter/laser_dist_topic', 10)
        self.complexNavPub = self.create_publisher(NavSatFix, '/complex_nav_topic', 10)

        self.insNavFilter = KalmanFilter(3)
        self.insImuFilter = KalmanFilter(10)
        self.aionNavFilter = KalmanFilter(2)
        self.laserDistFilter = KalmanFilter(1)

    def listenerInsNavCallback(self, msg):
        filtered_value = self.insNavFilter.getValue([msg.latitude, msg.longitude, msg.altitude])
        msg.latitude = filtered_value[0]
        msg.longitude = filtered_value[1]
        msg.altitude = filtered_value[2]
        self.filtredInsNavPub.publish(msg)
        self.complexNavPub.publish(msg)
    
    def listenerInsImuCallback(self, msg):
        filtered_value = self.insImuFilter.getValue([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                                                     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                                     msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        msg.orientation.x = filtered_value[0]
        msg.orientation.y = filtered_value[1]
        msg.orientation.z = filtered_value[2]
        msg.orientation.w = filtered_value[3]

        msg.angular_velocity.x = filtered_value[4]
        msg.angular_velocity.y = filtered_value[5]
        msg.angular_velocity.z = filtered_value[6]

        msg.linear_acceleration.x = filtered_value[7]
        msg.linear_acceleration.y = filtered_value[8]
        msg.linear_acceleration.z = filtered_value[9]
        self.filtredInsImuPub.publish(msg)

    def listenerAionCallback(self, msg):
        filtered_value = self.aionNavFilter.getValue([msg.latitude, msg.longitude])
        msg.latitude = filtered_value[0]
        msg.longitude = filtered_value[1]
        self.filtredAionPub.publish(msg)

    def listenerLaserCallback(self, msg):
        filtered_value = self.laserDistFilter.getValue([msg.range])
        msg.range = filtered_value[0]
        self.filtredLaserPub.publish(msg)

    def filterParamInit(self):
        self.insNavFilter.weight_k = 0.7

def main():
    node = ComplexData()
    rclpy.spin(node)

if __name__ == '__main__':
    main()