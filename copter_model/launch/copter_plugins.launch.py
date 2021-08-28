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

        Node(package='copter_model', 
            executable='spawn_copter.py', 
            name="spawn_copter",
            output='screen'),
        
        Node(package='copter_model', 
            executable='copter_plugin', 
            name="copter_plugin",
            output='screen'),

        Node(package='copter_model', 
            executable='gps_plugin.py', 
            name="gps_plugin",
            output='screen'),

        Node(package='copter_model', 
            executable='aion_plugin.py', 
            name="aion_plugin",
            output='screen'),

        Node(package='copter_model', 
            executable='ins_nav_plugin.py', 
            name="ins_nav_plugin",
            output='screen'),

        Node(package='copter_model', 
            executable='ins_imu_plugin.py', 
            name="ins_imu_plugin",
            output='screen'),

        Node(package='copter_model', 
            executable='laser_dist_plugin.py', 
            name="laser_dist_plugin",
            output='screen'),
        Node(package='copter_model', 
            executable='imu_plugin.py', 
            name="imu_plugin",
            output='screen'),
    ])
