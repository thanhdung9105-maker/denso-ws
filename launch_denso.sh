#!/bin/bash
# Complete process and DDS cleanup
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f joint_state_publisher 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f denso_mock_controller 2>/dev/null || true
pkill -9 -f denso_joint_by_joint_demo 2>/dev/null || true
rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/cyclonedds* 2>/dev/null
sleep 1.0

source /opt/ros/jazzy/setup.bash
source /home/dung/denso_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:0
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
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/denso_ws/install/denso_vs6556/share/denso_vs6556/rviz/display.rviz
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/rviz.rviz 2>/dev/null || true
    cp /home/dung/denso_ws/src/denso_vs6556/rviz/display.rviz.golden /home/dung/.rviz2/default.rviz 2>/dev/null || true
fi

echo "[INFO] Khoi chay canh tay robot Denso VS-6556 (RViz2 + MoveIt 2 Motion Planning)..."
ros2 launch denso_vs6556 display.launch.py
