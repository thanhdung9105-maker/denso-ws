import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_denso_gazebo = get_package_share_directory("denso_gazebo")
    pkg_denso_vs6556 = get_package_share_directory("denso_vs6556")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Paths
    xacro_file = os.path.join(pkg_denso_gazebo, "urdf", "denso_vs6556_gazebo.urdf.xacro")
    controllers_file = os.path.join(pkg_denso_gazebo, "config", "denso_controllers.yaml")
    world_file = os.path.join(pkg_denso_gazebo, "worlds", "denso_world.sdf")
    bridge_config = os.path.join(pkg_denso_gazebo, "config", "gz_bridge.yaml")

    # Process Xacro
    doc = xacro.process_file(xacro_file, mappings={"controllers_file": controllers_file})
    robot_description = {"robot_description": doc.toxml()}

    # Ensure Gazebo Sim can find meshes via model:// or package://
    resource_paths = [
        os.path.dirname(pkg_denso_vs6556),
        pkg_denso_vs6556,
        os.path.dirname(pkg_denso_gazebo),
    ]
    resource_path_str = ":".join(resource_paths)

    gz_resource_env = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path_str
    )

    gz_plugin_env = AppendEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value="/opt/ros/jazzy/lib"
    )

    # Launch Arguments
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode (server only, no GUI)"
    )
    headless = LaunchConfiguration("headless")

    control_gui_arg = DeclareLaunchArgument(
        "control_gui",
        default_value="false",
        description="Launch interactive joint control panel GUI alongside Gazebo"
    )
    control_gui = LaunchConfiguration("control_gui")

    launch_rsp_arg = DeclareLaunchArgument(
        "launch_rsp",
        default_value="true",
        description="Launch robot_state_publisher node"
    )
    launch_rsp = LaunchConfiguration("launch_rsp")

    # Gazebo Sim (GUI mode)
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        condition=UnlessCondition(headless)
    )

    # Gazebo Sim (Headless / Server mode)
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-s -r {world_file}"}.items(),
        condition=IfCondition(headless)
    )

    # Robot State Publisher (publishes TF based on URDF and /joint_states)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
        condition=IfCondition(launch_rsp)
    )

    # Spawn Denso robot into Gazebo Sim
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "denso_vs6556",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Bridge for /clock (parameter_bridge must NOT use sim_time since it bridges the clock)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
        }]
    )

    # Controller Spawner: joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "30"
        ],
        output="screen",
    )

    # Controller Spawner: arm_controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "30"
        ],
        output="screen",
    )

    # Event handlers to sequence spawners after robot is created in Gazebo
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Interactive Control Panel GUI (Optional)
    control_panel_node = Node(
        package="denso_gazebo",
        executable="denso_gazebo_control_panel.py",
        name="denso_gazebo_control_panel",
        output="screen",
        condition=IfCondition(control_gui)
    )

    return LaunchDescription([
        gz_resource_env,
        gz_plugin_env,
        headless_arg,
        control_gui_arg,
        launch_rsp_arg,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
        load_joint_state_broadcaster,
        load_arm_controller,
        control_panel_node,
    ])
