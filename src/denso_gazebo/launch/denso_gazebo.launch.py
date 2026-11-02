import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
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
    gui_config_file = os.path.join(pkg_denso_gazebo, "config", "gui.config")

    # Đồng bộ cấu hình góc nhìn camera Gazebo vào ~/.gz/sim/8/gui.config (không dùng cờ --gui-config vì CLI gz sim sẽ nuốt mất đường dẫn world_file)
    try:
        gz_user_dir = os.path.expanduser("~/.gz/sim/8")
        os.makedirs(gz_user_dir, exist_ok=True)
        shutil.copyfile(gui_config_file, os.path.join(gz_user_dir, "gui.config"))
    except Exception:
        pass

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

    gz_partition_env = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value="denso_ws"
    )

    gz_ip_env = SetEnvironmentVariable(
        name="GZ_IP",
        value="127.0.0.1"
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

    # Gazebo Sim (GUI mode) - Chay truc tiep ExecuteProcess de tranh loi nuot tham so world cua gz_sim.launch.py
    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        condition=UnlessCondition(headless)
    )

    # Gazebo Sim (Headless / Server mode)
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_file],
        output="screen",
        condition=IfCondition(headless)
    )

    # Robot State Publisher (publishes TF based on URDF and /joint_states)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        ros_arguments=["--log-level", "kdl_parser:=ERROR"],
        parameters=[robot_description, {"use_sim_time": True}],
        condition=IfCondition(launch_rsp)
    )

    # Static transform world -> base_link to anchor TF tree with base_link registered first
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0", "--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rsp)
    )

    # Spawn Denso robot into Gazebo Sim truc tiep qua chuoi XML (-string) de khong phai cho topic /robot_description
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "denso_world",
            "-name", "denso_vs6556",
            "-string", doc.toxml(),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    # Bridge for /clock (parameter_bridge must NOT use sim_time since it bridges the clock)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            "use_sim_time": False,
            "qos_overrides./clock.publisher.durability": "volatile",
            "qos_overrides./clock.publisher.reliability": "reliable",
        }]
    )

    # Khởi chạy spawn robot nhanh (0.5s) - node create tự động thăm dò service khi world sẵn sàng
    delayed_spawn_robot = TimerAction(
        period=0.5,
        actions=[spawn_robot],
    )

    # 1. Spawner cho joint_state_broadcaster: phai nap truoc de phat ngay /joint_states cho RViz va MoveIt
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 2. Spawner cho arm_controller: nap sau khi joint_state_broadcaster da chay on dinh
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Kích hoạt tuần tự tốc độ cao: spawn_robot xong -> nạp ngay joint_state_broadcaster -> nạp arm_controller
    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=0.1,
                    actions=[arm_controller_spawner],
                )
            ],
        )
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=0.1,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
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
        gz_partition_env,
        gz_ip_env,
        headless_arg,
        control_gui_arg,
        launch_rsp_arg,
        gazebo_gui,
        gazebo_headless,
        bridge,
        robot_state_publisher,
        static_transform_publisher,
        delayed_spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        control_panel_node,
    ])
