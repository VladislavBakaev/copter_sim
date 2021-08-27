#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int16
import time
from math import sqrt, sin, cos

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
    
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.subscription = self.create_subscription(
            NavSatFix,
            '/complex_nav_topic',
            self.listenerCurrentNavCallback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            NavSatFix,
            '/target_nav_topic',
            self.listenerTargetNavCallback,
            qos_profile=qos_policy)

        self.cmdVelPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmdCopterStatus = self.create_publisher(Int16, '/copter_status', 10)
        self.create_timer(0.1, self.copter_status_pub)
        self.copter_status = Int16()
        self.copter_status.data = 0

        debug = False

        if debug:
            target_msg = NavSatFix()
            target_msg.latitude = 55.77303800
            target_msg.longitude = 37.69876700
            target_msg.altitude = 1.0
            self.target_nav_state = target_msg                
            self.maxHSpeed = 1.0
            self.maxVSpeed = 1.0

        
        else:
            self.target_nav_state = None            

        self.current_nav_state = None
        self.a = 6378206.4
        self.b = 6356583.8
        self.e_2 = 1 - self.b**2/self.a**2
        self.prop_koef_planar = 1
        self.prop_koef_height = 1


    def listenerCurrentNavCallback(self, msg):
        if not self.current_nav_state is None and not self.target_nav_state is None:
            current_radius = self.get_current_radius(msg.latitude, msg.longitude, msg.altitude)

            time_delta = msg.header.stamp.nanosec-self.current_nav_state.header.stamp.nanosec
            lat_speed = (msg.latitude*current_radius - self.current_nav_state.latitude*current_radius)/time_delta
            lon_speed = (msg.longitude*current_radius - self.current_nav_state.longitude*current_radius)/time_delta

            lat_speed_proection = lat_speed/sqrt(lat_speed**2 + lon_speed**2)
            lon_speed_proection = lon_speed/sqrt(lat_speed**2 + lon_speed**2)

            lat_path = (self.target_nav_state.latitude - msg.latitude) * current_radius
            lon_path = (self.target_nav_state.longitude - msg.longitude) * current_radius
            path_module = sqrt(lat_path**2 + lon_path**2)
            if path_module > 0.2 and sqrt(lat_speed**2+lon_speed**2) > 10**(-10):
                lat_proection = lat_path/path_module
                lon_proection = lon_path/path_module

                lat_proection_error = lat_proection - lat_speed_proection
                lon_proection_error = lon_proection - lat_speed_proection

                x_vel = lat_proection + lat_proection_error*self.prop_koef_planar
                y_vel = lon_proection + lon_proection_error*self.prop_koef_planar

                if path_module > 3:
                    x_vel = x_vel/sqrt(x_vel**2+y_vel**2)*self.maxHSpeed
                    y_vel = y_vel/sqrt(x_vel**2+y_vel**2)*self.maxHSpeed
                else:
                    x_vel = x_vel/sqrt(x_vel**2+y_vel**2)*self.maxHSpeed/3
                    y_vel = y_vel/sqrt(x_vel**2+y_vel**2)*self.maxHSpeed/3

            elif sqrt(lat_speed**2+lon_speed**2) < 10**(-10) and path_module > 0.5:
                lat_proection = lat_path/path_module
                lon_proection = lon_path/path_module
                x_vel = lat_proection
                y_vel = lon_proection
            else:
                x_vel = 0.0
                y_vel = 0.0

            alt_error = self.target_nav_state.altitude - msg.altitude
            if abs(alt_error) > 1:
                z_vel = self.maxHSpeed*alt_error/abs(alt_error)
            
            elif 0.2<abs(alt_error)<1:
                z_vel = self.maxHSpeed*alt_error
            else:
                z_vel = 0.0


            # self.get_logger().info("latitude speed {0}, longitude speed {1}".format(lat_speed_proection, lon_speed_proection))
            # self.get_logger().info("latitude speed {0}, longitude speed {1}".format(x_vel, y_vel))
            # self.get_logger().info("path {0}, alt err {1}".format(path_module, alt_error))
            # self.get_logger().info("lat error {0}, lon error {1}".format(x_vel, y_vel))

            msg_vel = Twist()
            msg_vel.linear.x = x_vel
            msg_vel.linear.y = y_vel
            msg_vel.linear.z = z_vel
            if x_vel == z_vel == y_vel == 0.0:
                self.copter_status.data = 0
                self.target_nav_state = None
            else:
                self.copter_status.data = 1
            self.cmdVelPublisher.publish(msg_vel)

        self.current_nav_state = msg

    def listenerTargetNavCallback(self, msg):
        print(msg)
        self.target_nav_state = msg
        self.maxHSpeed = self.target_nav_state.position_covariance[0]
        self.maxVSpeed = self.target_nav_state.position_covariance[1]

    def get_current_radius(self, lat, lon, alt):
        N = self.a/(sqrt(1-self.e_2*sin(lat)**2))

        x = (N + alt)*cos(lat)*cos(lon)
        y = (N + alt)*cos(lat)*sin(lon)

        z = (self.b**2/self.a**2*N + alt)*sin(lat)

        return sqrt(x**2 + y**2 + z**2)
    
    def copter_status_pub(self):
        self.cmdCopterStatus.publish(self.copter_status)
            
def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()