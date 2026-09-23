#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/dung/denso_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ros2 topic pub --once /denso/cmd std_msgs/msg/String 'data: demo' >/dev/null 2>&1
echo "[INFO] Đã gửi lệnh bắt đầu chu trình chuyển động Link 1 -> Link 5 (2.0s / link)!"
