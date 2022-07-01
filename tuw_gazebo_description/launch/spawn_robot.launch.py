import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file = os.path.join(get_package_share_directory('tuw_gazebo_description'), 'models/pioneer3dx', 'main.xacro')    
    assert os.path.exists(xacro_file), "The main.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    #print(robot_desc)
    
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='tuw_gazebo_description', executable='spawn_robot.py', arguments=[robot_desc], output='screen'),
    ])