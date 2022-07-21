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

# Add string commands if not empty

def generate_launch_description():

    room_arg    = DeclareLaunchArgument('room', default_value=TextSubstitution(text='roblab'))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tuw_gazebo = get_package_share_directory('tuw_gazebo')
    pkg_tuw_gazebo_models = get_package_share_directory('tuw_gazebo_models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] \
            + ':' + pkg_tuw_gazebo_models + '/models'
    else:
        model_path =  pkg_tuw_gazebo_models + '/models'

    return LaunchDescription([
        room_arg,
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            # This is working, but I like to substitute cave in cave.world with my argument
            #launch_arguments={'world': os.path.join(pkg_tuw_gazebo, 'worlds', 'cave.world')}.items(),  
            # This is NOT working, I don't know how to access the room argument?
            launch_arguments={'world': pkg_tuw_gazebo + '/worlds/' + LaunchConfiguration('room') + '.world'}.items(),
        ) 
    ])