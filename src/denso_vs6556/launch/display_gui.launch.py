import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('denso_vs6556')
    urdf_file = os.path.join(pkg_share, 'urdf', 'denso_vs6556.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('WAYLAND_DISPLAY', ''),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'FBO'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '4.5'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
        SetEnvironmentVariable('QT_SCALE_FACTOR', '1'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            arguments=[urdf_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            on_exit=Shutdown(),
        ),
    ])
