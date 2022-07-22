#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution 
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node

# Add string commands if not empty

def generate_launch_description():

    tuw_gazebo_models = get_package_share_directory('tuw_gazebo_models')

    def rviz_launch_configuration(context):
        file = os.path.join(tuw_gazebo_models, 'config', context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)
    
    rviz_config_arg = DeclareLaunchArgument('config', 
                default_value=TextSubstitution(text='empty'), 
                description='Use empty, cave or roblab to load a TUW enviroment')

    return LaunchDescription([
        rviz_config_arg,
        rviz_launch_configuration_arg,
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [LaunchConfiguration('config')]]
        )
    ])