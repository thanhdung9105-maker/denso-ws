import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_denso_gazebo = get_package_share_directory('denso_gazebo')
    pkg_denso_vs6556 = get_package_share_directory('denso_vs6556')
    pkg_moveit = get_package_share_directory('denso_vs6556_moveit_config')

    urdf_path = os.path.join(pkg_denso_vs6556, 'urdf', 'denso_vs6556.urdf')
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("denso_vs6556", package_name="denso_vs6556_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/denso_vs6556.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (server only)'
    )

    # Launch Gazebo Simulation with controllers
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_denso_gazebo, 'launch', 'denso_gazebo.launch.py')
        ),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items()
    )

    # MoveGroup Node (with use_sim_time)
    move_group_params = [
        moveit_config.to_dict(),
        {'use_sim_time': True}
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # RViz2 Node (with use_sim_time)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([
        headless_arg,
        gazebo_launch,
        move_group_node,
        rviz_node,
    ])
