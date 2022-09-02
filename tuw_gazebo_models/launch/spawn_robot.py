#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from pydoc import ModuleScanner
from re import S
import sys
import rclpy
import math
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import GetModelList
import tf_transformations


def exists_model_already(node):

    model_name = node.get_parameter('model_name').get_parameter_value().string_value
    cli = node.create_client(GetModelList, '/get_model_list')

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = GetModelList.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        models = future.result().model_names
        for model in models:
            if model == model_name:
                node.get_logger().info('found: ' + model)
                return True
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    return False

def spawn_model(node):
    
    cli = node.create_client(SpawnEntity, '/spawn_entity')
    content = sys.argv[1]
    
    pose = [0.0, 0.0, 0.0]
    pose[0] = node.get_parameter('X').get_parameter_value().double_value
    pose[1] = node.get_parameter('Y').get_parameter_value().double_value
    pose[2] = node.get_parameter('Theta').get_parameter_value().double_value
    model_name = node.get_parameter('model_name').get_parameter_value().string_value
    namespace = node.get_parameter('namespace').get_parameter_value().string_value 

    req = SpawnEntity.Request()
    req.name = model_name
    req.xml = content
    req.robot_namespace = namespace
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


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    node.declare_parameter('X', 0.)
    node.declare_parameter('Y', 0.)
    node.declare_parameter('Theta', 0.)
    node.declare_parameter('model_name', "pioneer3dx")
    node.declare_parameter('namespace', "r0")
    
    if exists_model_already(node) :
        model_name = node.get_parameter('model_name').get_parameter_value().string_value 
        node.get_logger().info('model exists allready ' + model_name)
    else:
        spawn_model(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
