cmake_minimum_required(VERSION 3.8)
project(denso_vs6556_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  FILES .setup_assistant
  DESTINATION share/denso_vs6556_moveit_config
)

install(
  DIRECTORY config launch
  DESTINATION share/denso_vs6556_moveit_config
)

install(
  PROGRAMS
    scripts/denso_mock_controller.py
    scripts/denso_animation_demo.py
    scripts/denso_joint_by_joint_demo.py
    scripts/denso_kinematics_gui.py
    scripts/denso_dynamics_engine.py
  DESTINATION lib/denso_vs6556_moveit_config
)

ament_package()
