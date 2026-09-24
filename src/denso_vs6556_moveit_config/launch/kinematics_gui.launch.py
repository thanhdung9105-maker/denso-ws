#!/usr/bin/env python3
# kinematics_gui.launch.py - Denso VS-6556 Standalone Launch
# Usage:
#   ros2 launch denso_vs6556_moveit_config kinematics_gui.launch.py
#   ros2 launch denso_vs6556_moveit_config kinematics_gui.launch.py launch_rviz:=true
#   ros2 launch denso_vs6556_moveit_config kinematics_gui.launch.py launch_gui:=false
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    denso_pkg  = get_package_share_directory('denso_vs6556')
    moveit_pkg = get_package_share_directory('denso_vs6556_moveit_config')
    urdf_path  = os.path.join(denso_pkg, 'urdf', 'denso_vs6556.urdf')
    rviz_cfg   = os.path.join(moveit_pkg, 'config', 'moveit.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = {'robot_description': f.read()}

    use_sim = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('launch_gui',   default_value='true',  description='Khoi dong GUI Dashboard'),
        DeclareLaunchArgument('launch_rviz',  default_value='false', description='Khoi dong RViz2 xem robot 3D'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Dung simulation time'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_transform_publisher', output='log',
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
             parameters=[{'use_sim_time': use_sim}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[robot_description, {'use_sim_time': use_sim}]),
        Node(package='denso_vs6556_moveit_config', executable='denso_mock_controller.py',
             name='denso_mock_controller', output='screen',
             parameters=[{'use_sim_time': use_sim}]),
        Node(package='denso_vs6556_moveit_config', executable='denso_kinematics_gui.py',
             name='denso_kinematics_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('launch_gui'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('launch_rviz')),
             parameters=[robot_description, {'use_sim_time': use_sim}]),
    ])