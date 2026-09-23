#!/bin/bash
# Dọn dẹp tiến trình Gazebo từ phiên chạy trước
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gz_sim 2>/dev/null || true
killall -9 gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
killall -9 parameter_bridge robot_state_publisher static_transform_publisher spawner 2>/dev/null || true
pkill -9 -f denso_gazebo_control_panel 2>/dev/null || true
pgrep -f "ros2 launch denso_gazebo" | xargs -r kill -9 2>/dev/null || true
# Nếu mock controller đang chạy từ trước, dừng nó để tránh xung đột với Gazebo
pkill -9 -f denso_mock_controller 2>/dev/null || true
rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/cyclonedds* 2>/dev/null || true
sleep 0.5

source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Thống nhất Domain 0 cho toàn bộ hệ sinh thái Denso (đồng bộ với RViz2 và GUI)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Đảm bảo Gazebo Harmonic tìm thấy plugin gz_ros2_control
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:${LD_LIBRARY_PATH}

export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=""
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS=2
export MESA_GL_VERSION_OVERRIDE=4.5
export OGRE_RTT_MODE=FBO
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

echo "[INFO] 🚀 Khởi chạy mô phỏng vật lý Gazebo Harmonic cho Denso VS-6556 [ROS_DOMAIN_ID=${ROS_DOMAIN_ID}]..."
ros2 launch denso_gazebo denso_gazebo.launch.py "$@"
