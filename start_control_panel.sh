#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/dung/denso_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dung/cyclonedds.xml
export DISPLAY=:0
exec python3 /home/dung/denso_ws/src/denso_gazebo/scripts/denso_gazebo_control_panel.py

