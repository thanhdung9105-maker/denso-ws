#!/bin/bash
source /opt/ros/jazzy/setup.bash
export DISPLAY=:0
export PYTHONPATH=/home/dung/denso_ws/install/denso_gazebo/lib/python3.12/site-packages:$PYTHONPATH
python3 /home/dung/denso_ws/src/denso_gazebo/scripts/denso_gazebo_control_panel.py

