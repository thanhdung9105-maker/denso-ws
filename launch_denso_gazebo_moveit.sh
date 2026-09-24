#!/bin/bash
# Dọn dẹp triệt để tiến trình cũ
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gz_sim 2>/dev/null || true
killall -9 gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
killall -9 parameter_bridge robot_state_publisher static_transform_publisher spawner 2>/dev/null || true
rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/cyclonedds* 2>/dev/null || true
sleep 1

source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

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

echo "[INFO] Khởi tạo hệ thống mô phỏng tích hợp Denso VS-6556: Gazebo Sim & MoveIt 2 [Domain ID: 0]..."
echo "[INFO] Cấu hình CycloneDDS, đồng bộ clock hệ thống và nạp bộ điều khiển quỹ đạo..."
ros2 launch denso_gazebo denso_gazebo_moveit.launch.py "$@"
