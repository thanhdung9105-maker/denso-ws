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
  drawing_samples
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/test_gazebo_motion.py
  scripts/denso_gazebo_control_panel.py
  scripts/denso_gcode_vision_engine.py
  scripts/denso_gcode_energy_optimizer.py
  scripts/denso_gcode_energy_optimizer_tab.py
  scripts/denso_mpc_trajectory_controller.py
  scripts/denso_robot_energy_parameters.py
  scripts/denso_rl_policy_optimizer.py
  scripts/demo_ofoegbu2026_empc.py
  scripts/denso_gcode_joint_table.py
  scripts/denso_gcode_catalog.py
  scripts/denso_trajectory_benchmark_tab.py
  scripts/denso_window_arranger.py
  scripts/denso_log_cleaner.py
  scripts/denso_wait_for_controllers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
