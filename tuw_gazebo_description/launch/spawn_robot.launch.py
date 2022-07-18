import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file = os.path.join(get_package_share_directory('tuw_gazebo_description'), 'models/pioneer3dx', 'main.xacro')    
    assert os.path.exists(xacro_file), "The main.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    model_file = os.path.join(get_package_share_directory('tuw_gazebo_description'), 'models/pioneer3dx', 'main.xacro')    
    assert os.path.exists(xacro_file), "The main.xacro doesnt exist in "+str(xacro_file)

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("tuw_gazebo_description"),
            "config",
            "pioneer2dx_controllers.yaml",
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(package='tuw_gazebo_description', name="publisher_robot", executable='spawn_robot.py', arguments=[robot_desc], output='screen'),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc}],
            namespace='r0',
            output="screen"),
    ])