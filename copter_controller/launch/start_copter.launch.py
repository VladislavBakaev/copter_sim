#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    world_pkg_dir = LaunchConfiguration(
        'copter_model_pkg_dir',
        default=os.path.join(get_package_share_directory('copter_model'), 'launch'))

        
    controller_pkg_dir = LaunchConfiguration(
        'copter_controller_pkg_dir',
        default=os.path.join(get_package_share_directory('copter_controller'), 'launch'))

    # print (robot_desc) # Printing urdf information.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [world_pkg_dir, '/copter_plugins.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [world_pkg_dir, '/copter_world.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [controller_pkg_dir, '/controller.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])