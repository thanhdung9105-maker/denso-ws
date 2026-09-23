#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Kết nối tới Gazebo Simulation trên Domain 0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

export DISPLAY=${DISPLAY:-:0}
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

echo "[INFO] Đang mở Bảng Nhập & Điều Khiển Trực Tiếp Gazebo cho Denso VS-6556 [ROS_DOMAIN_ID=${ROS_DOMAIN_ID}]..."
python3 /home/dung/denso_ws/src/denso_gazebo/scripts/denso_gazebo_control_panel.py "$@"
