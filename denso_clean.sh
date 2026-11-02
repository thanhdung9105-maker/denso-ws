#!/bin/bash
echo "[CLEAN] Đang dọn dẹp triệt để 100% toàn bộ tiến trình Gazebo, MoveIt và ROS 2..."
killall -9 -q parameter_bridge robot_state_publisher static_transform_publisher spawner create rviz2 gz gz-sim ruby ruby3.2 move_group 2>/dev/null || true
pkill -9 -f "denso_gazebo_control_panel" 2>/dev/null || true
pkill -9 -f "denso_com_visualizer" 2>/dev/null || true
pkill -9 -f "denso_log_cleaner" 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "ros_gz" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
pkill -9 -f "move_group" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
rm -rf /dev/shm/* 2>/dev/null || true
rm -rf /tmp/gz* /tmp/ign* /tmp/create* /tmp/launch_params* /tmp/ros2* 2>/dev/null || true
sleep 1
echo "[CLEAN OK] Đã dọn dẹp sạch sẽ 100%! Bộ nhớ và socket đã giải phóng hoàn toàn."
exit 0
