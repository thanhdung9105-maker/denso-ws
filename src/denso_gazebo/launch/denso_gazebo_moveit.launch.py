import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["GZ_RELAY"] = "127.0.0.1"
    os.environ["GZ_PARTITION"] = "denso_ws"

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

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 visualizer'
    )

    # Launch Gazebo Simulation with controllers and single robot_state_publisher
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_denso_gazebo, 'launch', 'denso_gazebo.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'launch_rsp': 'true'
        }.items()
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
        ros_arguments=[
            "--log-level", "kdl_parser:=ERROR",
            "--log-level", "move_group.moveit.moveit.ros.occupancy_map_monitor:=FATAL",
        ],
        parameters=move_group_params,
    )

    # Center of Mass and Link Masses Visualizer Node for RViz2
    com_visualizer_node = Node(
        package='denso_vs6556_moveit_config',
        executable='denso_com_visualizer.py',
        name='denso_com_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Cấu hình môi trường đồ họa độc lập cho RViz2:
    # Sử dụng LLVMpipe (OpenGL 4.5) để RViz2 không bao giờ tranh chấp Direct3D 12 với Gazebo Sim,
    # loại bỏ hoàn toàn lỗi 'D3D12: Removing Device' và hiện tượng xám màn hình trong WSLg.
    rviz_env = {
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "GALLIUM_DRIVER": "llvmpipe",
        "LP_NUM_THREADS": "4",
        "MESA_GL_VERSION_OVERRIDE": "4.5",
        "OGRE_RTT_MODE": "FBO",
        "QT_QPA_PLATFORM": "xcb",
        "WAYLAND_DISPLAY": "",
    }

    # RViz2 Node (with use_sim_time)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_visualizer",
        output="screen",
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        arguments=["-d", rviz_config],
        additional_env=rviz_env,
        ros_arguments=[
            "--log-level", "rcl.logging_rosout:=ERROR",
            "--log-level", "rviz_visualizer.moveit.ros.motion_planning_frame:=FATAL",
            "--log-level", "kdl_parser:=ERROR",
            "--log-level", "class_loader:=ERROR",
            "--log-level", "class_loader.impl:=ERROR",
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ],
    )

    # Đồng bộ chính xác: Chờ Gazebo controllers và TF tree [world -> link_6] sẵn sàng 100%
    wait_node = Node(
        package="denso_gazebo",
        executable="denso_wait_for_controllers.py",
        name="denso_wait_for_controllers",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    # RViz khởi động sau khi MoveGroup đã sẵn sàng và Gazebo GUI đã khởi tạo xong cửa sổ (tránh giật lag và xung đột đồ họa)
    start_rviz_after_move_group = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=move_group_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[rviz_node]
                )
            ],
        )
    )

    # Kích hoạt MoveGroup và COM Visualizer ngay khi controllers & TF sẵn sàng
    start_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_node,
            on_exit=[
                move_group_node,
                com_visualizer_node,
            ],
        )
    )

    gz_partition_env = SetEnvironmentVariable(name="GZ_PARTITION", value="denso_ws")
    gz_ip_env = SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1")
    gz_relay_env = SetEnvironmentVariable(name="GZ_RELAY", value="127.0.0.1")

    return LaunchDescription([
        gz_partition_env,
        gz_ip_env,
        gz_relay_env,
        headless_arg,
        launch_rviz_arg,
        gazebo_launch,
        wait_node,
        start_moveit,
        start_rviz_after_move_group,
    ])
