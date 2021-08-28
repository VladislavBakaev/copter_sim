#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Node(package='copter_controller', 
        #     executable='complex_data.py', 
        #     name="complex_data",
        #     output='screen'),
        
        # Node(package='copter_controller', 
        #     executable='controller.py', 
        #     name="controller",
        #     output='screen'),
    ])
