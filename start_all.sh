#!/bin/bash
# ==============================================================================
# SCRIPT KHỞI ĐỘNG ĐỒNG BỘ 3 CỬA SỔ ROBOT DENSO VS-6556 TRÊN GAZEBO & RVIZ2
# 1. Gazebo Sim Harmonic (Thế giới 3D, Robot & Bàn vẽ)
# 2. RViz2 (Không gian MoveIt 2 & Quỹ đạo động học)
# 3. Bảng Điều Khiển Tương Tác 7-Tab (G-Code, Động Lực Học, So Sánh Thuật Toán)
# ==============================================================================

set -e

# BƯỚC 1: Dọn dẹp triệt để 100% tiến trình cũ và shared memory bị treo
killall -9 -q parameter_bridge robot_state_publisher static_transform_publisher spawner create rviz2 gz gz-sim ruby ruby3.2 move_group 2>/dev/null || true
pkill -9 -f "denso_gazebo_control_panel" 2>/dev/null || true
pkill -9 -f "denso_com_visualizer" 2>/dev/null || true
pkill -9 -f "denso_log_cleaner" 2>/dev/null || true
pkill -9 -f "denso_mock_controller" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "ros_gz_sim" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
rm -rf /dev/shm/* /tmp/launch_params* /tmp/create* /tmp/ros2* /tmp/gz* /tmp/ign* 2>/dev/null || true
sleep 0.3

# BƯỚC 2: Cấu hình ROS 2 Jazzy & CycloneDDS tối ưu cho WSL2
source /opt/ros/jazzy/setup.bash
sudo -n ip link set lo multicast on 2>/dev/null || true
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dung/cyclonedds.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=0
export GZ_IP=127.0.0.1
export GZ_RELAY=127.0.0.1
export GZ_PARTITION=denso_ws

export GZ_SIM_RESOURCE_PATH=/home/dung/denso_ws/src:/home/dung/denso_ws/install/denso_vs6556/share:/home/dung/denso_ws/install/denso_gazebo/share:
export IGN_GAZEBO_RESOURCE_PATH=""
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:

# BƯỚC 3: Cấu hình hiển thị đồ họa GPU NVIDIA D3D12 tăng tốc phần cứng cho WSLg
unset LIBGL_ALWAYS_SOFTWARE MESA_GL_VERSION_OVERRIDE
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset LP_NUM_THREADS
export DISPLAY=${DISPLAY:-:0}
export OGRE_RTT_MODE=FBO
export QT_QPA_PLATFORM=xcb
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=1

# BƯỚC 4: Đồng bộ cấu hình Camera Gazebo & RViz2 Golden View
if [ -f /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden ]; then
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/src/denso_vs6556_moveit_config/config/moveit.rviz
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/install/denso_vs6556_moveit_config/share/denso_vs6556_moveit_config/config/moveit.rviz 2>/dev/null || true
fi

if [ -f /home/dung/denso_ws/src/denso_gazebo/config/gui.config ]; then
    mkdir -p /home/dung/.gz/sim/8/
    cp /home/dung/denso_ws/src/denso_gazebo/config/gui.config /home/dung/.gz/sim/8/gui.config 2>/dev/null || true
fi

echo "=============================================================================="
echo "🚀 ĐANG KHỞI ĐỘNG ĐỒNG BỘ 3 CỬA SỔ ROBOT DENSO VS-6556 [DOMAIN ID: 0]..."
echo "  1. Gazebo Sim (Thế giới mô phỏng 3D vật lý)"
echo "  2. RViz2 (Không gian lập kế hoạch MoveIt 2)"
echo "  3. Bảng Điều Khiển Tương Tác 7-Tab"
echo "=============================================================================="

# BƯỚC 5: Khởi chạy Gazebo Sim + MoveIt 2 + RViz2 dưới nền
bash /home/dung/launch_denso_gazebo_moveit.sh &
SIM_PID=$!

# BƯỚC 6: Chờ hệ thống mô phỏng sẵn sàng rồi mở ngay Bảng điều khiển
echo "⏳ Đang kiểm tra tín hiệu khởi động các cửa sổ..."
for i in {1..40}; do
    if ros2 topic list 2>/dev/null | grep -q "/joint_states"; then
        echo "✅ Gazebo Sim & MoveIt 2 đã sẵn sàng! Mở Bảng Điều Khiển ngay lập tức..."
        break
    fi
    sleep 0.5
done

# BƯỚC 7: Khởi động Bảng điều khiển GUI
exec python3 /home/dung/denso_ws/src/denso_gazebo/scripts/denso_gazebo_control_panel.py "$@"

