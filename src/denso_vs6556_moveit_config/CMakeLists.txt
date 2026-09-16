cmake_minimum_required(VERSION 3.8)
project(denso_vs6556_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/denso_mock_controller.py
    scripts/denso_animation_demo.py
    scripts/denso_joint_by_joint_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
