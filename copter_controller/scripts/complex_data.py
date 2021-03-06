#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from math import sqrt, sin, cos, pi
from ament_index_python.packages import get_package_share_directory
import os
import json

from kalmanFilter import KalmanFilterComplex
from complexTool import ImuData

class ComplexData(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('complex_data')

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
        
        self.subscription = self.create_subscription(
            Bool,
            '/antispoof_topic',
            self.listenerAntispoofCallback,
            qos_profile=qos_policy)
        
        self.loadParam()

        self.complexNavPub = self.create_publisher(NavSatFix, '/complex_nav_topic', 10)
        self.imuNavPub = self.create_publisher(NavSatFix, '/imu_nav_data_topic', 10)

        self.imuData = ImuData()
        self.complexFilterSns = KalmanFilterComplex()
        self.complexFilterAion = KalmanFilterComplex()

        self.a = 6378206.4
        self.b = 6356583.8
        self.e_2 = 1 - self.b**2/self.a**2

        self.antispoofStatus = False
        self.current_radius = self.get_current_radius(self.start_latitude, self.start_longitude, 0.0)

        self.northSns = 0
        self.eastSns = 0
        self.heightSns = 0

        self.northAion = 0
        self.eastAion = 0
        self.heightLaser = 0

        self.complexRate = 30
        self.complexFilterSns.dt = 1/self.complexRate
        self.complexFilterAion.dt = 1/self.complexRate
        self.create_timer(1/self.complexRate, self.publishComplexData)

    def listenerInsNavCallback(self, msg):
        current_radius = self.get_current_radius(msg.latitude,\
                                                 msg.longitude,\
                                                 msg.altitude)
        self.northSns = (msg.latitude - self.start_latitude)*current_radius*pi/180
        self.eastSns = (msg.longitude - self.start_longitude)*current_radius*pi/180
        self.heightSns = msg.altitude
        self.current_radius = current_radius

    def listenerInsImuCallback(self, msg):
        self.imuData.update_date([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def listenerAionCallback(self, msg):
        current_radius = self.get_current_radius(msg.latitude,\
                                                 msg.longitude,\
                                                 msg.altitude)
        self.northAion = (msg.latitude - self.start_latitude)*current_radius*pi/180
        self.eastAion = (msg.longitude - self.start_longitude)*current_radius*pi/180

    def listenerLaserCallback(self, msg):
        self.heightLaser = msg.range

    def get_current_radius(self, lat, lon, alt):
        N = self.a/(sqrt(1-self.e_2*sin(lat)**2))

        x = (N + alt)*cos(lat)*cos(lon)
        y = (N + alt)*cos(lat)*sin(lon)

        z = (self.b**2/self.a**2*N + alt)*sin(lat)

        return sqrt(x**2 + y**2 + z**2)
    
    def listenerAntispoofCallback(self, msg):
        self.antispoofStatus = msg.data
    
    def publishComplexData(self):
        # ins_sns_err = [self.northSns,\
        #                self.heightSns,\
        #                self.eastSns]

        # ins_aion_err = [self.northAion,\
        #                self.heightLaser,\
        #                self.eastAion]

        ins_sns_err = [float(self.imuData.northIns) - self.northSns,\
                       float(self.imuData.heightIns) - self.heightSns,\
                       float(self.imuData.eastIns) - self.eastSns]

        ins_aion_err = [float(self.imuData.northIns) - self.northAion,\
                       float(self.imuData.heightIns) - self.heightLaser,\
                       float(self.imuData.eastIns) - self.eastAion]

        imu_latitude = self.start_latitude + float(self.imuData.northIns)/self.current_radius*180/pi
        imu_longitude = self.start_longitude + float(self.imuData.eastIns)/self.current_radius*180/pi
        imu_height = float(self.imuData.heightIns)

        ins_msg = NavSatFix()
        ins_msg.latitude = imu_latitude
        ins_msg.longitude = imu_longitude
        ins_msg.altitude = imu_height
        ins_msg.header.stamp = self.get_clock().now().to_msg()
        self.imuNavPub.publish(ins_msg)

        self.complexFilterSns.kalmanUpdate(ins_sns_err, [0,0,0])
        self.complexFilterAion.kalmanUpdate(ins_aion_err, [0,0,0])
        if not self.antispoofStatus:
            # latitude = self.start_latitude + self.complexFilterSns.xErr[0]/self.current_radius*180/pi
            # longitude = self.start_longitude + self.complexFilterSns.xErr[2]/self.current_radius*180/pi
            # altitude = self.complexFilterSns.xErr[1]
            latitude = self.start_latitude + (self.northSns + self.complexFilterSns.xErr[0])/2/self.current_radius*180/pi
            longitude = self.start_longitude + (self.eastSns + self.complexFilterSns.xErr[2]/2)/self.current_radius*180/pi
            altitude = self.heightSns + self.complexFilterSns.xErr[1]/2
        else:
            latitude = self.start_latitude + self.complexFilterSns.xErr[0]/self.current_radius*180/pi
            longitude = self.start_longitude + self.complexFilterSns.xErr[2]/self.current_radius*180/pi
            altitude = self.complexFilterSns.xErr[1]
            # latitude = self.start_latitude + (self.northAion + self.complexFilterAion.xErr[0])/self.current_radius*180/pi
            # longitude = self.start_longitude + (self.eastAion + self.complexFilterAion.xErr[2])/self.current_radius*180/pi
            # altitude = self.heightLaser + self.complexFilterAion.xErr[1]
        
        msg = NavSatFix()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        msg.header.stamp = self.get_clock().now().to_msg()
        self.complexNavPub.publish(msg)

    def loadParam(self):
        config_file = os.path.join(get_package_share_directory('copter_model'),
                        'config', 'start_config.json')

        with open(config_file, 'r') as json_file:
            data = json.load(json_file)
            self.start_latitude = data['start_latitude']
            self.start_longitude = data['start_longitude']
            self.start_height = data['start_altitude']

def main():
    node = ComplexData()
    rclpy.spin(node)

if __name__ == '__main__':
    main()