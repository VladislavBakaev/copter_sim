#!/usr/bin/python3
# -*- coding: utf-8 -*-

from ament_index_python.packages import get_package_share_directory
import os
from gazebo_msgs.srv import SpawnEntity
import rclpy

def main():
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')
    
    sdf_file_path = os.path.join(get_package_share_directory('copter_model'),'models','copter_robot','model.sdf')
    content = ''
    with open(sdf_file_path, 'r') as content_file:
        content = content_file.read()

    request = SpawnEntity.Request()
    request.name = "copter"
    request.robot_namespace = ''
    request.xml = content
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.0

    node.get_logger().info('Spawning Robot using service: `/spawn_entity`')

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()