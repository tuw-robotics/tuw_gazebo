#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
import math
from gazebo_msgs.srv import SpawnEntity
import tf_transformations

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    node.declare_parameter('X', 0.)
    node.declare_parameter('Y', 0.)
    node.declare_parameter('Theta', 0.)
    
    pose = [0.0, 0.0, 0.0]
    pose[0] = node.get_parameter('X').get_parameter_value().double_value
    pose[1] = node.get_parameter('Y').get_parameter_value().double_value
    pose[2] = node.get_parameter('Theta').get_parameter_value().double_value

    req = SpawnEntity.Request()
    req.name = "pioneer3dx"
    req.xml = content
    req.robot_namespace = ""
    req.reference_frame = "world"
    req.initial_pose.position.x = pose[0]
    req.initial_pose.position.y = pose[1]
    req.initial_pose.position.z = 0.
    q = tf_transformations.quaternion_from_euler(0, 0, pose[2])
    req.initial_pose.orientation.x =  q[0]
    req.initial_pose.orientation.y =  q[1]
    req.initial_pose.orientation.z =  q[2]
    req.initial_pose.orientation.w =  q[3]
    

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
