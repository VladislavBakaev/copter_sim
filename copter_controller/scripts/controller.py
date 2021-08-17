#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.on_timer()

    def on_timer(self):
        msg = Twist()

        msg.linear.z = 2.0
        self.pub_vel.publish(msg)
        time.sleep(7.0)

        msg.linear.z = 0.0
        msg.linear.x = 2.0
        self.pub_vel.publish(msg)
        time.sleep(6.0)

        msg.angular.z = 0.0
        msg.linear.y = 2.0
        self.pub_vel.publish(msg)
        time.sleep(6.0)

        msg.linear.y = 1.0
        msg.linear.x = -2.0
        self.pub_vel.publish(msg)
        time.sleep(6.0)

        msg.linear.y = 0.0
        msg.linear.x = 0.0
        msg.linear.z = -2.0
        self.pub_vel.publish(msg)
        time.sleep(7.0)
        
        msg.linear.z = 0.0
        self.pub_vel.publish(msg)


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()