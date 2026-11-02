#!/bin/bash
# Dọn dẹp nhanh các tiến trình cũ (giữ nguyên shader cache để Gazebo mở tức thì trong 2s)
killall -9 -q parameter_bridge robot_state_publisher static_transform_publisher spawner create rviz2 gz gz-sim ruby move_group 2>/dev/null || true
pkill -9 -f "denso_gazebo_control_panel|denso_com_visualizer|denso_log_cleaner|ros_gz_bridge|ros_gz_sim" 2>/dev/null || true
rm -rf /dev/shm/* /tmp/launch_params* /tmp/create* /tmp/ros2* 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
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
export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=""
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_GL_VERSION_OVERRIDE
unset LP_NUM_THREADS
export OGRE_RTT_MODE=FBO
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

if [ -f /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden ]; then
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/src/denso_vs6556_moveit_config/config/moveit.rviz
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/install/denso_vs6556_moveit_config/share/denso_vs6556_moveit_config/config/moveit.rviz 2>/dev/null || true
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/install/denso_vs6556/share/denso_vs6556/rviz/display.rviz 2>/dev/null || true
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/.rviz2/default.rviz 2>/dev/null || true
fi

# Đảm bảo cấu hình góc nhìn camera Gazebo luôn đúng vị trí bàn vẽ phía sau
if [ -f /home/dung/denso_ws/src/denso_gazebo/config/gui.config ]; then
    mkdir -p /home/dung/.gz/sim/8/
    cp /home/dung/denso_ws/src/denso_gazebo/config/gui.config /home/dung/.gz/sim/8/gui.config 2>/dev/null || true
fi

echo "[INFO] 🚀 Khởi tạo hệ thống mô phỏng tích hợp Denso VS-6556: Gazebo Sim & MoveIt 2 [Domain ID: 0]..."
echo "[INFO] ⚙️  Cấu hình CycloneDDS, đồng bộ clock hệ thống và nạp bộ điều khiển quỹ đạo..."

CLEANER="/home/dung/denso_ws/install/denso_gazebo/lib/denso_gazebo/denso_log_cleaner.py"
if [ ! -f "$CLEANER" ]; then
    CLEANER="/home/dung/denso_ws/src/denso_gazebo/scripts/denso_log_cleaner.py"
fi

if [ -f "$CLEANER" ]; then
    ros2 launch denso_gazebo denso_gazebo_moveit.launch.py "$@" 2>&1 | python3 "$CLEANER"
else
    ros2 launch denso_gazebo denso_gazebo_moveit.launch.py "$@"
fi
