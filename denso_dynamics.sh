#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic pub --once /denso/cmd std_msgs/msg/String 'data: toggle_dynamics' >/dev/null 2>&1
echo "[INFO] Da dao trang thai An/Hien cac thong so va mui ten dong luc hoc trong RViz2!"
