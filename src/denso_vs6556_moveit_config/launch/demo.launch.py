import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    launch_gui_arg = DeclareLaunchArgument(
        "launch_gui", default_value="false",
        description="Khoi dong Dynamics GUI cung MoveIt (true/false)"
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true",
        description="Khoi dong RViz2 (true/false)"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Su dung simulation clock (/clock) khi chay voi Gazebo (true/false)"
    )
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="false",
        description="Khi chay cung Gazebo: tat mock controller va tranh xung dot TF (true/false)"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")

    denso_pkg = get_package_share_directory("denso_vs6556")
    urdf_path = os.path.join(denso_pkg, "urdf", "denso_vs6556.urdf")
    moveit_pkg = get_package_share_directory("denso_vs6556_moveit_config")
    rviz_config = os.path.join(moveit_pkg, "config", "moveit.rviz")

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

    # MoveGroup Node (supports both mock controller and Gazebo simulation)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
    )

    # Robot State Publisher (only launched if not using Gazebo, as Gazebo already runs one)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_gazebo),
    )

    ,
    )

    # Mock Controller Node (only in standalone mock mode)
    mock_controller_node = Node(
        package="denso_vs6556_moveit_config",
        executable="denso_mock_controller.py",
        name="denso_mock_controller",
        output="screen",
        condition=UnlessCondition(use_gazebo),
    )

    kinematics_gui_node = Node(
        package="denso_vs6556_moveit_config",
        executable="denso_kinematics_gui.py",
        name="denso_kinematics_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_gui")),
    )

    rviz_node_cond = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription([
        launch_gui_arg,
        launch_rviz_arg,
        use_sim_time_arg,
        use_gazebo_arg,
        
        robot_state_publisher,
        mock_controller_node,
        move_group_node,
        rviz_node_cond,
        kinematics_gui_node,
    ])
