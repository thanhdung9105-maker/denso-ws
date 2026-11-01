#!/bin/bash
# Dọn dẹp tiến trình Gazebo từ phiên chạy trước
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gz_sim 2>/dev/null || true
killall -9 gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
killall -9 parameter_bridge robot_state_publisher static_transform_publisher spawner create 2>/dev/null || true
pkill -9 -f denso_gazebo_control_panel 2>/dev/null || true
pgrep -f "ros2 launch denso_gazebo" | xargs -r kill -9 2>/dev/null || true
pkill -9 -f denso_mock_controller 2>/dev/null || true
rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/cyclonedds* 2>/dev/null || true
sleep 0.5

source /opt/ros/jazzy/setup.bash
sudo ip link set lo multicast on 2>/dev/null || true
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dung/cyclonedds.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=0
export GZ_IP=127.0.0.1
export GZ_PARTITION=denso_ws

export GZ_SIM_RESOURCE_PATH=/home/dung/denso_ws/src:/home/dung/denso_ws/install/denso_vs6556/share:/home/dung/denso_ws/install/denso_gazebo/share:
export IGN_GAZEBO_RESOURCE_PATH=
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:

# GPU Acceleration via DirectX 12 on NVIDIA GTX 1650
export DISPLAY=:0
export WAYLAND_DISPLAY=""
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_GL_VERSION_OVERRIDE
export OGRE_RTT_MODE=FBO
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

# Đảm bảo cấu hình góc nhìn camera Gazebo luôn đúng vị trí bàn vẽ phía sau
if [ -f /home/dung/denso_ws/src/denso_gazebo/config/gui.config ]; then
    mkdir -p /home/dung/.gz/sim/8/
    cp /home/dung/denso_ws/src/denso_gazebo/config/gui.config /home/dung/.gz/sim/8/gui.config 2>/dev/null || true
fi

echo "[INFO] 🚀 Khởi chạy mô phỏng vật lý Gazebo Harmonic cho Denso VS-6556 [ROS_DOMAIN_ID=0]..."
CLEANER="/home/dung/denso_ws/install/denso_gazebo/lib/denso_gazebo/denso_log_cleaner.py"
if [ ! -f "$CLEANER" ]; then
    CLEANER="/home/dung/denso_ws/src/denso_gazebo/scripts/denso_log_cleaner.py"
fi

if [ -f "$CLEANER" ]; then
    ros2 launch denso_gazebo denso_gazebo.launch.py "$@" 2>&1 | python3 "$CLEANER"
else
    ros2 launch denso_gazebo denso_gazebo.launch.py "$@"
fi
