cmake_minimum_required(VERSION 3.8)
project(denso_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/test_gazebo_motion.py
  scripts/denso_gazebo_control_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
