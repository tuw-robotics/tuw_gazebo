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

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tuw_gazebo = get_package_share_directory('tuw_gazebo')
    pkg_tuw_gazebo_models = get_package_share_directory('tuw_gazebo_models')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        model_path =  os.environ['GZ_SIM_RESOURCE_PATH'] \
            + ':' + pkg_tuw_gazebo_models + '/models'
    else:
        model_path =  pkg_tuw_gazebo_models + '/models'

    def world_file_launch_configuration(context):
        file = os.path.join(pkg_tuw_gazebo, 'worlds', context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world', file)]

    world_file_launch_configuration_arg = OpaqueFunction(function=world_file_launch_configuration)
    
    world_arg = DeclareLaunchArgument('world', 
                default_value=TextSubstitution(text='empty'), 
                description='Use empty, cave or roblab to load a TUW enviroment')
    world = LaunchConfiguration('world')

    return LaunchDescription([
        world_arg,
        world_file_launch_configuration_arg,
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=model_path),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-v4 ', world, ' --render-engine ogre'], 'on_exit_shutdown': 'true'}.items())
    ])