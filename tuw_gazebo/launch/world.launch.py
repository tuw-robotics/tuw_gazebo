#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution 
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration

# Add string commands if not empty

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tuw_gazebo = get_package_share_directory('tuw_gazebo')
    pkg_tuw_gazebo_models = get_package_share_directory('tuw_gazebo_models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] \
            + ':' + pkg_tuw_gazebo_models + '/models'
    else:
        model_path =  pkg_tuw_gazebo_models + '/models'

    def world_file_launch_configuration(context):
        file = os.path.join(pkg_tuw_gazebo, 'worlds', context.launch_configurations['room'] + '.world')
        return [SetLaunchConfiguration('world', file)]

    world_file_launch_configuration_arg = OpaqueFunction(function=world_file_launch_configuration)
    
    room_arg = DeclareLaunchArgument('room', 
                default_value=TextSubstitution(text='empty'), 
                description='Use empty, cave or roblab to load a TUW enviroment')

    return LaunchDescription([
        room_arg,
        world_file_launch_configuration_arg,
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'verbose': 'true'}.items(),
        ) 
    ])