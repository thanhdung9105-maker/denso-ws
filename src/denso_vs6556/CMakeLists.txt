cmake_minimum_required(VERSION 3.8)
project(denso_vs6556)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch rviz
  DESTINATION share/
)

ament_package()
