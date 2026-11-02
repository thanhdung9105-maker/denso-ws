#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

# Dọn dẹp phiên điều khiển cũ để tránh trùng lặp tiến trình
pkill -9 -f denso_gazebo_control_panel 2>/dev/null || true

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dung/cyclonedds.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# Kết nối tới Gazebo Simulation trên Domain 0
export ROS_DOMAIN_ID=0
export GZ_IP=127.0.0.1
export GZ_PARTITION=denso_ws

export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=""
# GPU Acceleration via DirectX 12 on NVIDIA GTX 1650 (Toi uu do muot, triet tieu CPU lag)
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_GL_VERSION_OVERRIDE
unset LP_NUM_THREADS
export OGRE_RTT_MODE=FBO
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

echo "[INFO] Khởi động Giao diện Điều khiển Tương tác Robot Denso VS-6556 trong Gazebo Sim [Domain ID: 0]..."
exec python3 /home/dung/denso_ws/src/denso_gazebo/scripts/denso_gazebo_control_panel.py ""
