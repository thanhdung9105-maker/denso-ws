#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

# Dọn dẹp phiên điều khiển cũ để tránh trùng lặp tiến trình
pkill -9 -f denso_gazebo_control_panel 2>/dev/null || true

# Kích hoạt multicast trên vòng lặp lo (bỏ qua nếu đã bật để tối ưu thời gian khởi động)
if ! ip link show lo 2>/dev/null | grep -q "MULTICAST"; then
    sudo ip link set lo multicast on 2>/dev/null || true
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dung/cyclonedds.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# Kết nối tới Gazebo Simulation trên Domain 0
export ROS_DOMAIN_ID=0
export GZ_IP=127.0.0.1
export GZ_PARTITION=denso_ws

export DISPLAY=:0
export WAYLAND_DISPLAY=""
# Khởi động đồ họa nhanh (tiết kiệm ~1 giây so với d3d12)
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

echo "[INFO] Khởi động Giao diện Điều khiển Tương tác Robot Denso VS-6556 trong Gazebo Sim [Domain ID: 0]..."
exec python3 /home/dung/denso_ws/src/denso_gazebo/scripts/denso_gazebo_control_panel.py "$@"
