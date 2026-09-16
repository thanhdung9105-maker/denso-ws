import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    denso_pkg = get_package_share_directory('denso_vs6556')
    urdf_path = os.path.join(denso_pkg, 'urdf', 'denso_vs6556.urdf')
    moveit_pkg = get_package_share_directory('denso_vs6556_moveit_config')
    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')

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

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Static TF world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    # Mock Controller Node
    mock_controller_node = Node(
        package="denso_vs6556_moveit_config",
        executable="denso_mock_controller.py",
        name="denso_mock_controller",
        output="screen",
    )

    # RViz2 Node with MotionPlanning
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
        ],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        mock_controller_node,
        move_group_node,
        rviz_node,
    ])
