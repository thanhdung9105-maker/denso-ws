#!/bin/bash
# Dọn dẹp tiến trình RViz/MoveIt từ phiên chạy trước (KHÔNG tắt Gazebo nếu đang chạy)
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f denso_mock_controller 2>/dev/null || true
pkill -9 -f denso_joint_by_joint_demo 2>/dev/null || true
pkill -9 -f joint_state_publisher 2>/dev/null || true
if ! pgrep -f "gz sim|gz_sim" >/dev/null 2>&1; then
    pkill -9 -f move_group 2>/dev/null || true
    killall -9 robot_state_publisher static_transform_publisher parameter_bridge 2>/dev/null || true
    rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/cyclonedds* 2>/dev/null || true
fi
sleep 0.5

source /opt/ros/jazzy/setup.bash
if [ -f /home/dung/denso_ws/install/setup.bash ]; then
    source /home/dung/denso_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Thống nhất Domain 0 cho toàn bộ hệ sinh thái Denso (RViz2, MoveIt 2, Gazebo, GUI)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

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

if [ ! -f /home/dung/denso_ws/src/denso_vs6556/urdf/denso_vs6556.urdf ]; then
    cp /home/dung/denso_ws/src/denso_vs6556/urdf/denso_vs6556.urdf.golden /home/dung/denso_ws/src/denso_vs6556/urdf/denso_vs6556.urdf
fi

if [ -f /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden ]; then
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/install/denso_vs6556/share/denso_vs6556/rviz/display.rviz 2>/dev/null || true
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/rviz.rviz 2>/dev/null || true
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/.rviz2/default.rviz 2>/dev/null || true
fi

# Tự động phát hiện nếu Gazebo Harmonic đang chạy để đồng bộ hóa thời gian thực
if pgrep -f "gz sim|gz_sim" >/dev/null 2>&1; then
    echo "[INFO] 🚀 Phát hiện Gazebo Sim đang chạy! Khởi chạy RViz2 + MoveIt 2 ĐỒNG BỘ VẬT LÝ VỚI GAZEBO (use_sim_time:=true, Domain=${ROS_DOMAIN_ID})..."
    ros2 launch denso_vs6556 display.launch.py use_sim_time:=true use_mock:=false launch_rsp:=false "$@"
else
    echo "[INFO] 🤖 Khởi chạy RViz2 + MoveIt 2 Chế Độ Độc Lập / Mock Standalone (Domain=${ROS_DOMAIN_ID})..."
    ros2 launch denso_vs6556 display.launch.py use_sim_time:=false use_mock:=true launch_rsp:=true "$@"
fi
