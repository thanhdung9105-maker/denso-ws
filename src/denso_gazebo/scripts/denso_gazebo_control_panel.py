#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 GAZEBO INTERACTIVE CONTROL PANEL
Bảng nhập và điều khiển trực tiếp các khớp robot trong môi trường Gazebo Sim.
Hỗ trợ:
- Tab 1: Nhập vị trí Link 5 (Cartesian X, Y, Z), tính nghịch học (IK), hiển thị góc và điều khiển Gazebo
- Tab 2: Thanh trượt Slider tương tác mượt mà từng khớp (Live Drag, Presets)
- Cập nhật thời gian thực (/joint_states: góc, sai lệch, mô-men lực)
"""

import sys
import os
import math
import time
import csv
import datetime
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, TransformStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
try:
    import tf2_ros
except ImportError:
    tf2_ros = None
import json

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "denso_vs6556_moveit_config", "scripts"))
try:
    from denso_dynamics_engine import (
        DensoDynamicsEngine, rot_matrix_to_rpy, rot_matrix_to_quaternion,
        JOINT_LIMITS, JOINT_NAMES
    )
except ImportError:
    sys.path.append("/home/dung/denso_ws/src/denso_vs6556_moveit_config/scripts")
    from denso_dynamics_engine import (
        DensoDynamicsEngine, rot_matrix_to_rpy, rot_matrix_to_quaternion,
        JOINT_LIMITS, JOINT_NAMES
    )

# Vision & G-code engine is loaded on-demand or preloaded in background for sub-second startup
ImageGCodeDrawingTab = None

from PyQt5.QtWidgets import (
    QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView, QComboBox, QGridLayout,
    QApplication, QMainWindow, QProgressBar, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QFrame, QRadioButton, QButtonGroup, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
JOINT_NAME_TO_IDX = {name: i for i, name in enumerate(JOINT_NAMES)}

JOINT_INFO = [
    {
        'id': 'joint_1',
        'name': 'Khớp 1 (Base Yaw)',
        'axis': 'Trục Z',
        'rad_min': -2.967,
        'rad_max': 2.967,
        'deg_min': -170.0,
        'deg_max': 170.0,
        'effort_limit': 80.0
    },
    {
        'id': 'joint_2',
        'name': 'Khớp 2 (Shoulder Pitch)',
        'axis': 'Trục Y',
        'rad_min': -2.094,
        'rad_max': 2.356,
        'deg_min': -120.0,
        'deg_max': 135.0,
        'effort_limit': 100.0
    },
    {
        'id': 'joint_3',
        'name': 'Khớp 3 (Elbow Pitch)',
        'axis': 'Trục Y',
        'rad_min': -2.077,
        'rad_max': 2.897,
        'deg_min': -119.0,
        'deg_max': 166.0,
        'effort_limit': 60.0
    },
    {
        'id': 'joint_4',
        'name': 'Khớp 4 (Forearm Roll)',
        'axis': 'Trục Z',
        'rad_min': -3.316,
        'rad_max': 3.316,
        'deg_min': -190.0,
        'deg_max': 190.0,
        'effort_limit': 25.0
    },
    {
        'id': 'joint_5',
        'name': 'Khớp 5 (Wrist Pitch)',
        'axis': 'Trục Y',
        'rad_min': -2.094,
        'rad_max': 2.094,
        'deg_min': -120.0,
        'deg_max': 120.0,
        'effort_limit': 20.0
    },
    {
        'id': 'joint_6',
        'name': 'Khớp 6 (Mặt bích / Roll)',
        'axis': 'Trục Z',
        'rad_min': -6.283,
        'rad_max': 6.283,
        'deg_min': -360.0,
        'deg_max': 360.0,
        'effort_limit': 15.0
    }
]

JOINT_NAMES_VI = [
    "J1 Đế (Base Yaw)",
    "J2 Vai (Shoulder Pitch)",
    "J3 Khuỷu (Elbow Pitch)",
    "J4 Cẳng tay (Roll)",
    "J5 Cổ tay (Wrist Pitch)",
    "J6 Mặt bích (Flange Roll)"
]

PRESETS = {
    '🏠 Về Home (0°)': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    '🎯 Sẵn sàng (Ready)': [0.0, 30.0, 45.0, 0.0, 30.0, 0.0],
    '📦 Vị trí Gắp (Pick)': [45.0, 45.0, 60.0, 0.0, -30.0, 0.0],
    '📥 Vị trí Đặt (Place)': [-45.0, 30.0, 75.0, 0.0, -15.0, 0.0],
    '🔭 Vươn xa (Reach)': [0.0, 15.0, 30.0, 0.0, 0.0, 0.0],
}


class GazeboRosBridge:
    def __init__(self):
        self.node = None
        self.traj_pub = None
        self.ee_pose_pub = None
        self.link_poses_pub = None
        self.fk_info_pub = None
        self.ik_target_pub = None
        self.ik_sol_pub = None
        self.marker_pub = None
        self.tf_broadcaster = None

        self.latest_positions = [0.0] * len(JOINT_NAMES)
        self.latest_velocities = [0.0] * len(JOINT_NAMES)
        self.latest_efforts = [0.0] * len(JOINT_NAMES)
        self.last_msg_time = 0.0
        self.lock = threading.Lock()
        self.pending_cartesian_target = None

    def start(self):
        rclpy.init(args=None)
        self.node = Node('denso_gazebo_control_panel')

        # Standard trajectory publisher & joint states subscriber
        self.traj_pub = self.node.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # -------------------------------------------------------------
        # ROS 2 FORWARD & INVERSE KINEMATICS EXPORT TOPICS
        # -------------------------------------------------------------
        # 1. Forward Kinematics End-Effector / Flange PoseStamped
        self.ee_pose_pub = self.node.create_publisher(
            PoseStamped,
            '/denso/forward_kinematics/ee_pose',
            10
        )
        # 2. Forward Kinematics All Links PoseArray
        self.link_poses_pub = self.node.create_publisher(
            PoseArray,
            '/denso/forward_kinematics/link_poses',
            10
        )
        # 3. Comprehensive Kinematics Telemetry & DH Matrices JSON
        self.fk_info_pub = self.node.create_publisher(
            String,
            '/denso/forward_kinematics/info',
            10
        )
        # 4. Inverse Kinematics Target Cartesian PoseStamped
        self.ik_target_pub = self.node.create_publisher(
            PoseStamped,
            '/denso/inverse_kinematics/target_pose',
            10
        )
        # 5. Inverse Kinematics Solved JointState
        self.ik_sol_pub = self.node.create_publisher(
            JointState,
            '/denso/inverse_kinematics/solution',
            10
        )
        # 6. RViz 3D Visual Markers (Skeleton, TCP axes, Target sphere)
        self.marker_pub = self.node.create_publisher(
            MarkerArray,
            '/denso/kinematics_markers',
            10
        )

        # 7. TF Broadcaster for base_link -> denso_fk_tcp & denso_ik_target
        if tf2_ros:
            try:
                self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
            except Exception as e:
                self.node.get_logger().warn(f"TF broadcaster init error: {e}")

        # 8. External Cartesian Target Subscribers
        self.node.create_subscription(
            PoseStamped,
            '/denso/cartesian_target',
            self._cartesian_target_cb,
            10
        )
        self.node.create_subscription(
            PointStamped,
            '/denso/cartesian_point_target',
            self._cartesian_point_cb,
            10
        )

        def _spin_worker():
            try:
                rclpy.spin(self.node)
            except Exception:
                pass
        threading.Thread(target=_spin_worker, daemon=True).start()

    def _joint_state_cb(self, msg: JointState):
        with self.lock:
            for i, name in enumerate(msg.name):
                idx = JOINT_NAME_TO_IDX.get(name)
                if idx is not None:
                    if i < len(msg.position):
                        self.latest_positions[idx] = float(msg.position[i])
                    if i < len(msg.velocity):
                        self.latest_velocities[idx] = float(msg.velocity[i])
                    if i < len(msg.effort):
                        self.latest_efforts[idx] = float(msg.effort[i])
            self.last_msg_time = time.time()

    def stop(self):
        if self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass

    def _cartesian_target_cb(self, msg: PoseStamped):
        with self.lock:
            # incoming position in meters -> convert to mm
            self.pending_cartesian_target = (
                float(msg.pose.position.x) * 1000.0,
                float(msg.pose.position.y) * 1000.0,
                float(msg.pose.position.z) * 1000.0
            )

    def _cartesian_point_cb(self, msg: PointStamped):
        with self.lock:
            self.pending_cartesian_target = (
                float(msg.point.x) * 1000.0,
                float(msg.point.y) * 1000.0,
                float(msg.point.z) * 1000.0
            )

    def pop_pending_cartesian_target(self):
        with self.lock:
            tar = self.pending_cartesian_target
            self.pending_cartesian_target = None
            return tar

    def get_latest_state(self):
        with self.lock:
            return (
                list(self.latest_positions),
                list(self.latest_velocities),
                list(self.latest_efforts),
                self.last_msg_time
            )

    def send_trajectory(self, target_rads, duration_sec=2.0, zero_velocity=True):
        if not self.traj_pub:
            return
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(r) for r in target_rads]
        if zero_velocity:
            point.velocities = [0.0] * len(target_rads)

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        traj.points = [point]
        self.traj_pub.publish(traj)

    def publish_forward_kinematics(self, fk_data):
        if not self.node or not self.ee_pose_pub:
            return

        now = self.node.get_clock().now().to_msg()
        pos_m = fk_data["position_m"]
        quat = fk_data["quaternion"]

        # 1. Publish EE PoseStamped
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "base_link"
        ps.pose.position.x = float(pos_m[0])
        ps.pose.position.y = float(pos_m[1])
        ps.pose.position.z = float(pos_m[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        self.ee_pose_pub.publish(ps)

        # 2. Publish Link Poses (PoseArray)
        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = "base_link"
        for T in fk_data["transforms"]:
            p = Pose()
            p.position.x = float(T[0, 3])
            p.position.y = float(T[1, 3])
            p.position.z = float(T[2, 3])
            q_i = rot_matrix_to_quaternion(T[:3, :3])
            p.orientation.x = float(q_i[0])
            p.orientation.y = float(q_i[1])
            p.orientation.z = float(q_i[2])
            p.orientation.w = float(q_i[3])
            pa.poses.append(p)
        self.link_poses_pub.publish(pa)

        # 3. Publish JSON Summary
        info_dict = {
            "timestamp": now.sec + now.nanosec * 1e-9,
            "frame_id": "base_link",
            "position_mm": [round(float(v), 2) for v in fk_data["position_mm"]],
            "rpy_deg": [round(float(v), 2) for v in fk_data["rpy_deg"]],
            "quaternion": [round(float(v), 4) for v in quat],
            "T_target": [[round(float(val), 4) for val in row] for row in fk_data["T_target"]]
        }
        msg_str = String()
        msg_str.data = json.dumps(info_dict)
        self.fk_info_pub.publish(msg_str)

        # 4. Broadcast TF base_link -> denso_fk_tcp
        if self.tf_broadcaster:
            tfs = TransformStamped()
            tfs.header.stamp = now
            tfs.header.frame_id = "base_link"
            tfs.child_frame_id = "denso_fk_tcp"
            tfs.transform.translation.x = float(pos_m[0])
            tfs.transform.translation.y = float(pos_m[1])
            tfs.transform.translation.z = float(pos_m[2])
            tfs.transform.rotation.x = float(quat[0])
            tfs.transform.rotation.y = float(quat[1])
            tfs.transform.rotation.z = float(quat[2])
            tfs.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tfs)

    def publish_inverse_kinematics(self, target_pos_m, q_sol, success, iters, err_m, use_flange=False):
        if not self.node or not self.ik_target_pub:
            return

        now = self.node.get_clock().now().to_msg()

        # 1. Target Pose
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "base_link"
        ps.pose.position.x = float(target_pos_m[0])
        ps.pose.position.y = float(target_pos_m[1])
        ps.pose.position.z = float(target_pos_m[2])
        ps.pose.orientation.w = 1.0
        self.ik_target_pub.publish(ps)

        # 2. Solved JointState
        if q_sol is not None:
            js = JointState()
            js.header.stamp = now
            js.name = JOINT_NAMES
            js.position = [float(v) for v in q_sol]
            js.velocity = [float(iters)] * len(JOINT_NAMES)  # pass iteration count in velocity field
            js.effort = [float(err_m * 1000.0)] * len(JOINT_NAMES)  # pass error in mm in effort field
            self.ik_sol_pub.publish(js)

        # 3. Broadcast TF base_link -> denso_ik_target
        if self.tf_broadcaster:
            tfs = TransformStamped()
            tfs.header.stamp = now
            tfs.header.frame_id = "base_link"
            tfs.child_frame_id = "denso_ik_target"
            tfs.transform.translation.x = float(target_pos_m[0])
            tfs.transform.translation.y = float(target_pos_m[1])
            tfs.transform.translation.z = float(target_pos_m[2])
            tfs.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tfs)

    def publish_kinematics_markers(self, fk_data, target_pos_m, ik_success, use_flange=False):
        if not self.node or not self.marker_pub:
            return

        now = self.node.get_clock().now().to_msg()
        ma = MarkerArray()

        # 1. Arm skeleton Line Strip
        m_lines = Marker()
        m_lines.header.stamp = now
        m_lines.header.frame_id = "base_link"
        m_lines.ns = "denso_kinematics_skeleton"
        m_lines.id = 0
        m_lines.type = Marker.LINE_STRIP
        m_lines.action = Marker.ADD
        m_lines.scale.x = 0.012  # line width
        m_lines.color.r = 0.22
        m_lines.color.g = 0.74
        m_lines.color.b = 0.97
        m_lines.color.a = 0.85

        for T in fk_data["transforms"]:
            p = Point()
            p.x = float(T[0, 3])
            p.y = float(T[1, 3])
            p.z = float(T[2, 3])
            m_lines.points.append(p)
        ma.markers.append(m_lines)

        # 2. Target Sphere
        m_tar = Marker()
        m_tar.header.stamp = now
        m_tar.header.frame_id = "base_link"
        m_tar.ns = "denso_ik_target_sphere"
        m_tar.id = 1
        m_tar.type = Marker.SPHERE
        m_tar.action = Marker.ADD
        m_tar.pose.position.x = float(target_pos_m[0])
        m_tar.pose.position.y = float(target_pos_m[1])
        m_tar.pose.position.z = float(target_pos_m[2])
        m_tar.scale.x = 0.04
        m_tar.scale.y = 0.04
        m_tar.scale.z = 0.04
        if ik_success:
            m_tar.color.r = 0.06
            m_tar.color.g = 0.72
            m_tar.color.b = 0.50
            m_tar.color.a = 0.85
        else:
            m_tar.color.r = 0.93
            m_tar.color.g = 0.27
            m_tar.color.b = 0.27
            m_tar.color.a = 0.85
        ma.markers.append(m_tar)

        # 3. TCP Axes (Arrows: X=Red, Y=Green, Z=Blue)
        pos_m = fk_data["position_m"]
        R = fk_data["T_target"][:3, :3]
        axis_len = 0.06
        cols = [
            (1.0, 0.0, 0.0),  # X
            (0.0, 1.0, 0.0),  # Y
            (0.0, 0.5, 1.0)   # Z
        ]
        for ax_i in range(3):
            m_ax = Marker()
            m_ax.header.stamp = now
            m_ax.header.frame_id = "base_link"
            m_ax.ns = "denso_tcp_axes"
            m_ax.id = 10 + ax_i
            m_ax.type = Marker.ARROW
            m_ax.action = Marker.ADD
            m_ax.scale.x = 0.006  # shaft diameter
            m_ax.scale.y = 0.012  # head diameter
            m_ax.scale.z = 0.015  # head length
            m_ax.color.r = cols[ax_i][0]
            m_ax.color.g = cols[ax_i][1]
            m_ax.color.b = cols[ax_i][2]
            m_ax.color.a = 0.95

            p_start = Point(x=float(pos_m[0]), y=float(pos_m[1]), z=float(pos_m[2]))
            p_end = Point(
                x=float(pos_m[0] + axis_len * R[0, ax_i]),
                y=float(pos_m[1] + axis_len * R[1, ax_i]),
                z=float(pos_m[2] + axis_len * R[2, ax_i])
            )
            m_ax.points = [p_start, p_end]
            ma.markers.append(m_ax)

        self.marker_pub.publish(ma)


class DensoGazeboPanel(QMainWindow):
    def __init__(self, bridge: GazeboRosBridge):
        super().__init__()
        self.bridge = bridge
        self.is_deg_mode = True
        self.live_drag = False
        self.target_rads = [0.0] * len(JOINT_NAMES)
        self.actual_rads = [0.0] * len(JOINT_NAMES)
        self.actual_efforts = [0.0] * len(JOINT_NAMES)
        self.filtered_efforts = [0.0] * len(JOINT_NAMES)
        self._prev_eff_colors = [None] * len(JOINT_NAMES)
        self._prev_act_colors = [None] * len(JOINT_NAMES)
        self.updating_widgets = False
        self.engine = DensoDynamicsEngine()
        self.ik_target_pos = [15.0, 0.0, 900.0]
        self.last_ik_sol = None
        self.ik_method = "hybrid"
        self.ik_elbow_up = True
        self.matrix_view_idx = 5
        self.latest_fk_data = None
        self.telemetry_payload = 0.0
        self.prev_velocities = [0.0] * len(JOINT_NAMES)
        self.actual_accels = [0.0] * len(JOINT_NAMES)
        self.filtered_vels = [0.0] * len(JOINT_NAMES)
        self.filtered_taus = [0.0] * len(JOINT_NAMES)
        self.filtered_loads = [0.0] * len(JOINT_NAMES)
        self._tab4_prev_load_levels = [-1] * len(JOINT_NAMES)
        self._tab4_prev_badge_levels = [-1] * len(JOINT_NAMES)
        self.prev_time = time.time()

        self.init_ui()
        self.setup_connections()

        # Telemetry Timer chạy ở 20 FPS (50ms)
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self._update_telemetry)
        self.telemetry_timer.start(50)

        # Tính toán trước IK vị trí ban đầu
        self._calc_ik()

    def init_ui(self):
        self.setWindowTitle('DENSO VS-6556 - BẢNG ĐIỀU KHIỂN ROBOT TRONG GAZEBO')
        self.resize(1260, 840)
        self.setMinimumSize(960, 680)

        self.setStyleSheet("""
            QMainWindow { background-color: #0b1120; }
            QWidget { color: #e2e8f0; font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; }
            QTabWidget::pane {
                border: 1px solid #1e293b;
                border-radius: 8px;
                background-color: #0f172a;
                top: -1px;
            }
            QTabBar::tab {
                background: #1e293b;
                color: #94a3b8;
                padding: 10px 22px;
                border: 1px solid #334155;
                border-bottom: none;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                font-weight: bold;
                font-size: 13px;
                margin-right: 4px;
            }
            QTabBar::tab:selected {
                background: #0284c7;
                color: #ffffff;
                border-bottom: 2px solid #38bdf8;
            }
            QGroupBox {
                background-color: #111827;
                border: 1px solid #1f2937;
                border-radius: 8px;
                margin-top: 14px;
                font-weight: bold;
                font-size: 13px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 8px;
                color: #38bdf8;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #334155;
                border-radius: 4px;
            }
            QSlider::sub-page:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #f8fafc;
                border: 2px solid #0284c7;
                width: 20px;
                margin-top: -6px;
                margin-bottom: -6px;
                border-radius: 10px;
            }
            QSlider::handle:horizontal:hover { background: #38bdf8; }
            QDoubleSpinBox {
                background-color: #0f172a;
                color: #38bdf8;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 4px 8px;
                font-size: 14px;
                font-weight: bold;
            }
            QDoubleSpinBox:focus { border: 2px solid #38bdf8; }
            QPushButton {
                background-color: #0284c7;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover { background-color: #0369a1; }
            QPushButton:pressed { background-color: #0c4a6e; }
            QRadioButton, QCheckBox { font-size: 13px; font-weight: 500; }
        """)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(14, 14, 14, 14)

        # 1. Header Banner
        header = QFrame(self)
        header.setStyleSheet('background-color: #0f172a; border: 1px solid #1e293b; border-radius: 8px; padding: 6px 12px;')
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(4, 4, 4, 4)
        
        title_lbl = QLabel('🤖 BẢNG ĐIỀU KHIỂN ROBOT TRỰC TIẾP TRONG GAZEBO', self)
        title_lbl.setStyleSheet('font-size: 16px; font-weight: bold; color: #38bdf8;')
        h_layout.addWidget(title_lbl)
        h_layout.addStretch()

        self.status_badge = QLabel('🔴 Đang dò tìm Gazebo...', self)
        self.status_badge.setStyleSheet('background-color: #7f1d1d; color: #fecaca; padding: 5px 14px; border-radius: 12px; font-weight: bold; font-size: 12px;')
        h_layout.addWidget(self.status_badge)
        main_layout.addWidget(header)

        # Bộ hẹn giờ chống đơ (Debounce Timer): khi gõ số nhanh chỉ tính toán 1 lần sau khi dừng gõ 200ms
        self.ik_calc_timer = QTimer(self)
        self.ik_calc_timer.setSingleShot(True)
        self.ik_calc_timer.setInterval(200)
        self.ik_calc_timer.timeout.connect(self._calc_ik)

        # 2. Tabs Container
        self.tabs = QTabWidget(self)
        
        tab_link5 = self.create_link5_control_tab()
        self.tabs.addTab(tab_link5, "🎯 1. ĐIỀU KHIỂN TỌA ĐỘ LINK 5 & GÓC KHỚP")

        tab_kinematics = self.create_kinematics_tab()
        self.tabs.addTab(tab_kinematics, "📐 2. BẢNG PHÂN TÍCH ĐỘNG HỌC (KINEMATICS)")

        tab_joints = self.create_joints_tab()
        self.tabs.addTab(tab_joints, "🎛️ 3. ĐIỀU KHIỂN TỪNG KHỚP (SLIDERS)")

        tab_telemetry = self.create_telemetry_tab()
        self.tabs.addTab(tab_telemetry, "📊 4. BẢNG THÔNG SỐ ĐỘNG LỰC HỌC (TELEMETRY)")

        # Tab 5: G-code & Image Vision Trajectory Drawing (Nạp trực tiếp không chờ)
        from denso_gcode_vision_engine import ImageGCodeDrawingTab
        self.tab_gcode = ImageGCodeDrawingTab(self.bridge, self.engine, self)
        self.tabs.addTab(self.tab_gcode, "🎨 5. KHẮC / VẼ HÌNH ẢNH & G-CODE")
        self.tabs.currentChanged.connect(self._on_tab_changed_lazy)

        # Tab 6: BẢNG TRA GÓC 6-LINK (A, B, C, U, V, W) RIÊNG BIỆT - HIỂN THỊ ĐẦY ĐỦ TOÀN BỘ SỐ LIỆU CÁC LINK
        from denso_gcode_joint_table import GCodeJointAnglesTableWidget
        self.tab_joint_table = GCodeJointAnglesTableWidget(
            bridge=self.bridge,
            engine=self.engine,
            is_embedded=False,
            parent=self
        )
        self.tabs.addTab(self.tab_joint_table, "🤖 6. BẢNG TRA GÓC 6-LINK (A, B, C, U, V, W)")

        if self.tab_gcode is not None:
            self.tab_gcode.joint_table_widget = self.tab_joint_table
            self.tab_joint_table.sig_point_selected.connect(self.tab_gcode._on_table_point_selected)
            if getattr(self.tab_gcode, "current_strokes", None):
                self.tab_joint_table.set_strokes(
                    self.tab_gcode.current_strokes,
                    item_name=self.tab_gcode.lbl_img_path.text()
                )

        main_layout.addWidget(self.tabs, 1)

    def _on_tab_changed_lazy(self, index):
        if index in [0, 1]:
            self._refresh_ik_table()

    def _ensure_tab_gcode_loaded(self):
        pass

    def _make_kpi_card(self, title, initial_val, color="#38bdf8"):
        box = QFrame()
        box.setStyleSheet('background-color: #0b1120; border: 1px solid #1e293b; border-radius: 8px; padding: 8px 12px;')
        v = QVBoxLayout(box)
        v.setContentsMargins(4, 4, 4, 4)
        v.setSpacing(4)
        lbl_t = QLabel(title)
        lbl_t.setStyleSheet('color: #94a3b8; font-size: 11px; font-weight: bold; text-transform: uppercase;')
        lbl_v = QLabel(initial_val)
        lbl_v.setStyleSheet(f'color: {color}; font-size: 15px; font-weight: bold; font-family: monospace;')
        v.addWidget(lbl_t)
        v.addWidget(lbl_v)
        box.val_lbl = lbl_v
        return box

    def _on_ik_input_changed(self):
        if hasattr(self, "ik_calc_timer"):
            self.ik_calc_timer.start(200)

    # -------------------------------------------------------------------------
    # TAB 1: ĐIỀU KHIỂN TỌA ĐỘ LINK 5 & GÓC QUAY CÁC KHỚP (GIAO DIỆN TẬP TRUNG, MƯỢT MÀ)
    # -------------------------------------------------------------------------
    def create_link5_control_tab(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        # CỘT TRÁI: NHẬP TỌA ĐỘ LINK 5 CẦN DI CHUYỂN TỚI & ĐIỀU KHIỂN (40%)
        left_box = QGroupBox("🎯 NHẬP TỌA ĐỘ LINK 5 CẦN DI CHUYỂN TỚI")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(14, 16, 14, 14)
        left_layout.setSpacing(10)

        # 1. Khâu mục tiêu
        mode_row = QHBoxLayout()
        lbl_mode = QLabel("Khâu mục tiêu:")
        lbl_mode.setStyleSheet("font-weight: bold; font-size: 12px; color: #e2e8f0;")
        self.ik_target_mode = QComboBox()
        self.ik_target_mode.addItems([
            "Khâu 5 - Cổ tay (Point P, Z_home=900mm)",
            "Mặt Bích / Tool TCP (Flange, Z_home=980mm)"
        ])
        self.ik_target_mode.setStyleSheet("background-color: #0b1120; color: #38bdf8; border: 1px solid #334155; padding: 5px; font-weight: bold;")
        self.ik_target_mode.currentIndexChanged.connect(self._on_target_mode_changed)
        mode_row.addWidget(lbl_mode)
        mode_row.addWidget(self.ik_target_mode)
        left_layout.addLayout(mode_row)

        # 2. Grid Tọa độ X, Y, Z, Duration
        grid_in = QGridLayout()
        grid_in.setSpacing(8)

        lbl_x = QLabel("Tọa độ X (mm):")
        lbl_x.setStyleSheet("font-weight: bold; color: #e2e8f0;")
        self.spin_ik_x = QDoubleSpinBox()
        self.spin_ik_x.setRange(-850.0, 850.0)
        self.spin_ik_x.setValue(15.0)
        self.spin_ik_x.setSingleStep(10.0)
        self.spin_ik_x.setKeyboardTracking(False)
        self.spin_ik_x.valueChanged.connect(self._on_ik_input_changed)
        self.spin_ik_x.editingFinished.connect(self._calc_ik)

        lbl_y = QLabel("Tọa độ Y (mm):")
        lbl_y.setStyleSheet("font-weight: bold; color: #e2e8f0;")
        self.spin_ik_y = QDoubleSpinBox()
        self.spin_ik_y.setRange(-850.0, 850.0)
        self.spin_ik_y.setValue(0.0)
        self.spin_ik_y.setSingleStep(10.0)
        self.spin_ik_y.setKeyboardTracking(False)
        self.spin_ik_y.valueChanged.connect(self._on_ik_input_changed)
        self.spin_ik_y.editingFinished.connect(self._calc_ik)

        lbl_z = QLabel("Tọa độ Z (mm):")
        lbl_z.setStyleSheet("font-weight: bold; color: #e2e8f0;")
        self.spin_ik_z = QDoubleSpinBox()
        self.spin_ik_z.setRange(-250.0, 1150.0)
        self.spin_ik_z.setValue(900.0)
        self.spin_ik_z.setSingleStep(10.0)
        self.spin_ik_z.setKeyboardTracking(False)
        self.spin_ik_z.valueChanged.connect(self._on_ik_input_changed)
        self.spin_ik_z.editingFinished.connect(self._calc_ik)

        lbl_dur = QLabel("Thời gian chuyển động:")
        lbl_dur.setStyleSheet("font-weight: bold; color: #e2e8f0;")
        self.spin_ik_dur = QDoubleSpinBox()
        self.spin_ik_dur.setRange(0.5, 10.0)
        self.spin_ik_dur.setValue(2.0)
        self.spin_ik_dur.setSingleStep(0.5)
        self.spin_ik_dur.setSuffix(" s")
        self.spin_ik_dur.setKeyboardTracking(False)

        grid_in.addWidget(lbl_x, 0, 0)
        grid_in.addWidget(self.spin_ik_x, 0, 1)
        grid_in.addWidget(lbl_y, 1, 0)
        grid_in.addWidget(self.spin_ik_y, 1, 1)
        grid_in.addWidget(lbl_z, 2, 0)
        grid_in.addWidget(self.spin_ik_z, 2, 1)
        grid_in.addWidget(lbl_dur, 3, 0)
        grid_in.addWidget(self.spin_ik_dur, 3, 1)
        left_layout.addLayout(grid_in)

        # 3. Gợi ý vị trí mẫu Presets
        lbl_pre = QLabel("GỢI Ý VỊ TRÍ MẪU TRONG GAZEBO:")
        lbl_pre.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold; margin-top: 4px;")
        left_layout.addWidget(lbl_pre)

        grid_p = QGridLayout()
        grid_p.setSpacing(6)
        presets = [
            ("🏠 Home", self._preset_home),
            ("➡️ Phía Trước", self._preset_front),
            ("➡️ Bên Phải", self._preset_right),
            ("⬅️ Bên Trái", self._preset_left),
            ("🎨 Bàn Vẽ (Phía Sau)", self._preset_table),
            ("🔭 Vươn Cao", self._preset_reach)
        ]
        for pi, (pname, pfn) in enumerate(presets):
            b = QPushButton(pname)
            b.setStyleSheet("background-color: #1e293b; padding: 6px;")
            b.clicked.connect(pfn)
            grid_p.addWidget(b, pi // 3, pi % 3)
        left_layout.addLayout(grid_p)

        # 4. Hàng Nút Thao Tác
        b_row = QHBoxLayout()
        b_row.setSpacing(8)
        btn_read_cur = QPushButton("📥 Lấy Tọa Độ Hiện Tại")
        btn_read_cur.setStyleSheet("background-color: #334155; padding: 8px; font-weight: bold;")
        btn_read_cur.clicked.connect(self._read_current_cartesian)
        b_row.addWidget(btn_read_cur)

        btn_calc = QPushButton("🧮 TÍNH TOÁN GÓC (PREVIEW)")
        btn_calc.setStyleSheet("background-color: #0d9488; padding: 8px; font-weight: bold;")
        btn_calc.clicked.connect(self._calc_ik)
        b_row.addWidget(btn_calc)
        left_layout.addLayout(b_row)

        # 5. Nút Lớn: CHUYỂN ĐỘNG ROBOT
        self.btn_exec_ik = QPushButton("🚀 CHUYỂN ĐỘNG ROBOT TRONG GAZEBO")
        self.btn_exec_ik.setFixedHeight(46)
        self.btn_exec_ik.setStyleSheet("""
            QPushButton {
                background-color: #16a34a; font-size: 14px; font-weight: bold;
                border-radius: 8px; color: #ffffff;
            }
            QPushButton:hover { background-color: #15803d; }
            QPushButton:pressed { background-color: #166534; }
        """)
        self.btn_exec_ik.clicked.connect(self._exec_ik_motion)
        left_layout.addWidget(self.btn_exec_ik)

        # 6. Status Card
        self.ik_status_card = QFrame()
        self.ik_status_card.setStyleSheet("background-color: #0b1120; border-radius: 6px; padding: 8px; border-left: 4px solid #0284c7;")
        v_st = QVBoxLayout(self.ik_status_card)
        v_st.setContentsMargins(6, 6, 6, 6)
        v_st.setSpacing(4)
        self.lbl_ik_status = QLabel("● Sẵn sàng điều khiển vị trí Link 5.")
        self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #38bdf8;")
        self.lbl_ik_info = QLabel("Tầm vươn an toàn Link 5: ~653 mm (Point P) / ~733 mm (Flange)")
        self.lbl_ik_info.setStyleSheet("font-size: 11px; color: #94a3b8;")
        v_st.addWidget(self.lbl_ik_status)
        v_st.addWidget(self.lbl_ik_info)
        left_layout.addWidget(self.ik_status_card)

        left_layout.addStretch()
        layout.addWidget(left_box, 4)

        # CỘT PHẢI: HIỂN THỊ GÓC QUAY CỦA CÁC LINK (60%)
        right_box = QGroupBox("🔄 GÓC QUAY CỦA CÁC LINK (ROTATION ANGLES)")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(14, 16, 14, 14)
        right_layout.setSpacing(10)

        # 1. Thẻ Tóm Tắt Nhanh Link 5 (3 KPI Cards)
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(8)
        self.card_link5_cur = self._make_kpi_card("TỌA ĐỘ HIỆN TẠI LINK 5", "[+0, +0, +0] mm", "#4ade80")
        self.card_link5_tar = self._make_kpi_card("TỌA ĐỘ MỤC TIÊU CẦN TỚI", "[+0, +0, +0] mm", "#38bdf8")
        self.card_link5_dist = self._make_kpi_card("KHOẢNG CÁCH ΔD", "0.0 mm", "#facc15")
        cards_layout.addWidget(self.card_link5_cur)
        cards_layout.addWidget(self.card_link5_tar)
        cards_layout.addWidget(self.card_link5_dist)
        right_layout.addLayout(cards_layout)

        # 2. Bảng Góc Quay Của 5 Khớp / Link
        self.table_link_angles = QTableWidget(len(JOINT_NAMES), 7)
        self.table_link_angles.setHorizontalHeaderLabels([
            "Khớp & Khâu", "Trục Quay", "Góc Hiện Tại (Gazebo)", "Góc Mục Tiêu (Target)", "Góc Cần Quay (Δq)", "Giới Hạn An Toàn", "Trạng Thái Khớp"
        ])

        h_h = self.table_link_angles.horizontalHeader()
        h_h.setSectionResizeMode(0, QHeaderView.Interactive)
        h_h.setSectionResizeMode(1, QHeaderView.Interactive)
        h_h.setSectionResizeMode(2, QHeaderView.Stretch)
        h_h.setSectionResizeMode(3, QHeaderView.Stretch)
        h_h.setSectionResizeMode(4, QHeaderView.Interactive)
        h_h.setSectionResizeMode(5, QHeaderView.Interactive)
        h_h.setSectionResizeMode(6, QHeaderView.Stretch)

        self.table_link_angles.setColumnWidth(0, 110)
        self.table_link_angles.setColumnWidth(1, 140)
        self.table_link_angles.setColumnWidth(4, 110)
        self.table_link_angles.setColumnWidth(5, 120)

        self.table_link_angles.verticalHeader().setVisible(False)
        self.table_link_angles.verticalHeader().setDefaultSectionSize(40)
        self.table_link_angles.setEditTriggers(QTableWidget.NoEditTriggers)

        self.table_link_angles.setStyleSheet("""
            QTableWidget {
                background-color: #0b1120;
                color: #e2e8f0;
                gridline-color: #1e293b;
                font-family: 'Segoe UI', Consolas, monospace;
                font-size: 13px;
                border: 1px solid #1e293b;
                border-radius: 8px;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #38bdf8;
                font-weight: bold;
                border: 1px solid #334155;
                padding: 6px;
                height: 32px;
                font-size: 12px;
            }
        """)
        right_layout.addWidget(self.table_link_angles, 4)

        # 3. Chân trang & liên kết điều hướng
        foot_layout = QHBoxLayout()
        lbl_hint = QLabel("💡 Bảng hiển thị góc quay thực tế từ Gazebo (/joint_states) và góc giải ra để tới Link 5.")
        lbl_hint.setStyleSheet("color: #94a3b8; font-size: 11px; font-style: italic;")
        foot_layout.addWidget(lbl_hint, 1)

        btn_go_kin = QPushButton("📐 Xem Bảng Động Học Chi Tiết →")
        btn_go_kin.setStyleSheet("background-color: #1e293b; color: #38bdf8; border: 1px solid #0284c7; padding: 6px 12px; border-radius: 6px; font-weight: bold;")
        btn_go_kin.clicked.connect(lambda: self.tabs.setCurrentIndex(1))
        foot_layout.addWidget(btn_go_kin)
        right_layout.addLayout(foot_layout)

        right_layout.addStretch()
        layout.addWidget(right_box, 6)
        return panel

    # -------------------------------------------------------------------------
    # TAB 2: BẢNG PHÂN TÍCH ĐỘNG HỌC THUẬN - NGHỊCH CHI TIẾT (KINEMATICS DEDICATED)
    # -------------------------------------------------------------------------
    def create_kinematics_tab(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        # CỘT TRÁI: CẤU HÌNH THUẬT TOÁN ĐỘNG HỌC & XUẤT DỮ LIỆU (38%)
        left_box = QGroupBox("⚙️ CẤU HÌNH & XUẤT DỮ LIỆU ĐỘNG HỌC")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(14, 16, 14, 14)
        left_layout.setSpacing(10)

        algo_box = QGroupBox("📐 THUẬT TOÁN ĐỘNG HỌC NGHỊCH (IK)")
        algo_box.setStyleSheet("background-color: #0b1120; border: 1px solid #1e293b; border-radius: 6px; padding: 8px;")
        config_grid = QGridLayout(algo_box)
        config_grid.setSpacing(6)

        lbl_method = QLabel("Thuật toán IK:")
        lbl_method.setStyleSheet("font-weight: bold; font-size: 11px; color: #94a3b8;")
        self.combo_ik_method = QComboBox()
        self.combo_ik_method.addItem("🚀 Hybrid (Giải Tích + DLS Tối Ưu)", "hybrid")
        self.combo_ik_method.addItem("⚡ Giải Tích Hình Học (Analytical - 0ms)", "analytical")
        self.combo_ik_method.addItem("🎯 DLS Số Trị (Numerical DLS)", "dls")
        self.combo_ik_method.setStyleSheet("background-color: #0f172a; color: #38bdf8; border: 1px solid #334155; padding: 4px; font-weight: bold;")
        self.combo_ik_method.currentIndexChanged.connect(self._calc_ik)

        lbl_elbow = QLabel("Cấu hình khuỷu:")
        lbl_elbow.setStyleSheet("font-weight: bold; font-size: 11px; color: #94a3b8;")
        self.combo_ik_elbow = QComboBox()
        self.combo_ik_elbow.addItem("Cùi chỏ ngửa lên (Elbow Up)", True)
        self.combo_ik_elbow.addItem("Cùi chỏ úp xuống (Elbow Down)", False)
        self.combo_ik_elbow.setStyleSheet("background-color: #0f172a; color: #38bdf8; border: 1px solid #334155; padding: 4px; font-weight: bold;")
        self.combo_ik_elbow.currentIndexChanged.connect(self._calc_ik)

        config_grid.addWidget(lbl_method, 0, 0)
        config_grid.addWidget(self.combo_ik_method, 0, 1)
        config_grid.addWidget(lbl_elbow, 1, 0)
        config_grid.addWidget(self.combo_ik_elbow, 1, 1)
        left_layout.addWidget(algo_box)

        export_box = QGroupBox("📡 XUẤT DỮ LIỆU ĐỘNG HỌC & ROS 2 TOPICS")
        export_box.setStyleSheet("background-color: #0b1120; border: 1px solid #1e293b; border-radius: 6px; padding: 8px;")
        export_layout = QVBoxLayout(export_box)
        export_layout.setContentsMargins(8, 8, 8, 8)
        export_layout.setSpacing(6)

        self.lbl_ros_topics = QLabel("🟢 ROS 2 Topics: /denso/forward_kinematics & /denso/inverse_kinematics (20Hz)")
        self.lbl_ros_topics.setStyleSheet("color: #38bdf8; font-size: 10px; font-family: monospace;")
        export_layout.addWidget(self.lbl_ros_topics)

        exp_btns = QHBoxLayout()
        btn_exp_csv = QPushButton("📸 Xuất Snapshot (CSV)")
        btn_exp_csv.setStyleSheet("background-color: #0284c7; padding: 6px; font-size: 11px; font-weight: bold;")
        btn_exp_csv.clicked.connect(self._export_kinematics_csv)
        btn_exp_json = QPushButton("📄 Xuất Chi Tiết (JSON)")
        btn_exp_json.setStyleSheet("background-color: #475569; padding: 6px; font-size: 11px; font-weight: bold;")
        btn_exp_json.clicked.connect(self._export_kinematics_json)
        exp_btns.addWidget(btn_exp_csv)
        exp_btns.addWidget(btn_exp_json)
        export_layout.addLayout(exp_btns)
        left_layout.addWidget(export_box)

        dh_box = QGroupBox("📘 THÔNG SỐ DH DENSO VS-6556")
        dh_box.setStyleSheet("background-color: #0b1120; border: 1px solid #1e293b; border-radius: 6px; padding: 8px;")
        dh_layout = QVBoxLayout(dh_box)
        dh_table = QTableWidget(len(JOINT_NAMES), 5)
        dh_table.setHorizontalHeaderLabels(["Khâu (i)", "a_i (mm)", "α_i (deg)", "d_i (mm)", "θ_i / Trục"])
        dh_table.verticalHeader().setVisible(False)
        dh_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        dh_table.setEditTriggers(QTableWidget.NoEditTriggers)
        dh_table.setFixedHeight(145)
        dh_table.setStyleSheet("""
            QTableWidget {
                background-color: #020617;
                color: #94a3b8;
                font-family: Consolas, monospace;
                font-size: 10px;
                border: 1px solid #1e293b;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #38bdf8;
                font-size: 10px;
                padding: 3px;
                border: 1px solid #334155;
            }
        """)
        dh_data = [
            ("1 (Base->L1)", "-75", "-90°", "335", "q1 (Z)"),
            ("2 (Vai->L2)", "270", "0°", "0", "-q2 - 90° (-Y)"),
            ("3 (Khuỷu->L3)", "90", "+90°", "0", "-q3 + 90° (-Y)"),
            ("4 (Cẳng tay)", "0", "-90°", "295", "q4 (Z)"),
            ("5 (Cổ tay)", "0", "+90°", "0", "-q5 (-Y)"),
            ("6 (Mặt bích)", "0", "0°", "90", "q6 (Z)")
        ]
        for row_i, (k, a, alp, d, th) in enumerate(dh_data):
            for col_i, txt in enumerate([k, a, alp, d, th]):
                it = QTableWidgetItem(txt)
                it.setTextAlignment(Qt.AlignCenter)
                dh_table.setItem(row_i, col_i, it)
        dh_layout.addWidget(dh_table)
        left_layout.addWidget(dh_box)

        left_layout.addStretch()
        layout.addWidget(left_box, 4)

        # CỘT PHẢI: BẢNG ĐỘNG HỌC THUẬN (FK), SO SÁNH IK & MA TRẬN 4x4 (62%)
        right_box = QGroupBox("📊 ĐỘNG HỌC THUẬN (FK GAZEBO) & SO SÁNH NGHIỆM IK")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(14, 16, 14, 14)
        right_layout.setSpacing(10)

        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(8)
        self.card_cur = self._make_kpi_card("VỊ TRÍ THỰC TẾ (FK)", "[+0, +0, +0] mm", "#4ade80")
        self.card_rpy = self._make_kpi_card("GÓC EULER RPY (FK)", "R:0° P:0° Y:0°", "#a78bfa")
        self.card_tar = self._make_kpi_card("MỤC TIÊU (IK)", "[+0, +0, +0] mm", "#38bdf8")
        self.card_dist = self._make_kpi_card("SAI LỆCH ΔD", "0.0 mm", "#facc15")
        cards_layout.addWidget(self.card_cur)
        cards_layout.addWidget(self.card_rpy)
        cards_layout.addWidget(self.card_tar)
        cards_layout.addWidget(self.card_dist)
        right_layout.addLayout(cards_layout)

        self.ik_table = QTableWidget(len(JOINT_NAMES), 6)
        self.ik_table.setHorizontalHeaderLabels([
            "Khớp", "Trục & Tên", "Hiện Tại Gazebo (FK)", "Mục Tiêu (IK)", "Sai Lệch Δq", "Tọa Độ Link [X, Y, Z] (mm)"
        ])

        h_h = self.ik_table.horizontalHeader()
        h_h.setSectionResizeMode(0, QHeaderView.Interactive)
        h_h.setSectionResizeMode(1, QHeaderView.Interactive)
        h_h.setSectionResizeMode(2, QHeaderView.Stretch)
        h_h.setSectionResizeMode(3, QHeaderView.Stretch)
        h_h.setSectionResizeMode(4, QHeaderView.Interactive)
        h_h.setSectionResizeMode(5, QHeaderView.Stretch)

        self.ik_table.setColumnWidth(0, 75)
        self.ik_table.setColumnWidth(1, 140)
        self.ik_table.setColumnWidth(4, 90)

        self.ik_table.verticalHeader().setVisible(False)
        self.ik_table.verticalHeader().setDefaultSectionSize(32)
        self.ik_table.setEditTriggers(QTableWidget.NoEditTriggers)

        self.ik_table.setStyleSheet("""
            QTableWidget {
                background-color: #0b1120;
                color: #e2e8f0;
                gridline-color: #1e293b;
                font-family: 'Segoe UI', Consolas, monospace;
                font-size: 12px;
                border: 1px solid #1e293b;
                border-radius: 6px;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #38bdf8;
                font-weight: bold;
                border: 1px solid #334155;
                padding: 6px;
                height: 30px;
                font-size: 11px;
            }
        """)
        right_layout.addWidget(self.ik_table, 3)

        matrix_box = QGroupBox("📐 MA TRẬN BIẾN ĐỔI THUẦN NHẤT 4x4 (HOMOGENEOUS TRANSFORM T0_i)")
        matrix_box.setStyleSheet("background-color: #0b1120; border: 1px solid #1e293b; border-radius: 6px; padding: 6px;")
        matrix_layout = QVBoxLayout(matrix_box)
        matrix_layout.setContentsMargins(8, 8, 8, 8)
        matrix_layout.setSpacing(6)

        m_top_row = QHBoxLayout()
        lbl_sel_m = QLabel("Chọn mắt xích (Coordinate Frame):")
        lbl_sel_m.setStyleSheet("font-weight: bold; font-size: 11px; color: #94a3b8;")
        self.combo_matrix_frame = QComboBox()
        self.combo_matrix_frame.addItems([
            "Khâu 1 (T0_1: Base -> Link 1)",
            "Khâu 2 (T0_2: Base -> Link 2 Vai)",
            "Khâu 3 (T0_3: Base -> Link 3 Khuỷu)",
            "Khâu 4 (T0_4: Base -> Link 4 Cẳng tay)",
            "Khâu 5 (T0_5: Base -> Link 5 Point P)",
            "Mặt Bích TCP (T0_Flange: Base -> Flange +80mm)"
        ])
        self.combo_matrix_frame.setCurrentIndex(5)
        self.combo_matrix_frame.setStyleSheet("background-color: #0f172a; color: #38bdf8; border: 1px solid #334155; padding: 3px; font-weight: bold;")
        self.combo_matrix_frame.currentIndexChanged.connect(self._refresh_matrix_display)
        m_top_row.addWidget(lbl_sel_m)
        m_top_row.addWidget(self.combo_matrix_frame)
        m_top_row.addStretch()
        matrix_layout.addLayout(m_top_row)

        self.table_matrix = QTableWidget(4, 4)
        self.table_matrix.setHorizontalHeaderLabels(["nx / X", "ox / Y", "ax / Z", "px (Tọa độ X)"])
        self.table_matrix.setVerticalHeaderLabels(["Row 1 (X)", "Row 2 (Y)", "Row 3 (Z)", "Row 4"])
        self.table_matrix.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_matrix.verticalHeader().setDefaultSectionSize(26)
        self.table_matrix.setFixedHeight(138)
        self.table_matrix.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table_matrix.setStyleSheet("""
            QTableWidget {
                background-color: #020617;
                color: #38bdf8;
                gridline-color: #1e293b;
                font-family: 'Consolas', monospace;
                font-size: 11px;
                border: 1px solid #1e293b;
                border-radius: 4px;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #94a3b8;
                font-weight: bold;
                font-size: 10px;
                padding: 4px;
                border: 1px solid #334155;
            }
        """)
        matrix_layout.addWidget(self.table_matrix)
        right_layout.addWidget(matrix_box, 2)

        lbl_foot = QLabel("💡 Trạng thái: Dữ liệu Động Học Thuận (FK) & Nghịch (IK) được xuất liên tục lên ROS 2 và đồng bộ thời gian thực 20Hz.")
        lbl_foot.setStyleSheet("color: #94a3b8; font-size: 11px; font-style: italic;")
        right_layout.addWidget(lbl_foot)

        layout.addWidget(right_box, 6)
        return panel

    def _on_target_mode_changed(self, idx):
        use_flange = (idx == 1)
        if self.spin_ik_z.value() in (900.0, 980.0):
            self.spin_ik_z.setValue(980.0 if use_flange else 900.0)
        self._calc_ik()

    def _preset_home(self):
        use_flange = (self.ik_target_mode.currentIndex() == 1)
        self._set_spin_values_silently(15.0, 0.0, 980.0 if use_flange else 900.0)
        self._calc_ik()

    def _preset_front(self):
        self._set_spin_values_silently(350.0, 0.0, 600.0)
        self._calc_ik()

    def _preset_right(self):
        self._set_spin_values_silently(300.0, -250.0, 550.0)
        self._calc_ik()

    def _preset_left(self):
        self._set_spin_values_silently(300.0, 250.0, 550.0)
        self._calc_ik()

    def _preset_table(self):
        self._set_spin_values_silently(-380.0, 0.0, 180.0)
        self._calc_ik()

    def _preset_reach(self):
        self._set_spin_values_silently(250.0, 0.0, 850.0)
        self._calc_ik()

    def _set_spin_values_silently(self, x, y, z):
        self.spin_ik_x.blockSignals(True)
        self.spin_ik_y.blockSignals(True)
        self.spin_ik_z.blockSignals(True)
        self.spin_ik_x.setValue(float(x))
        self.spin_ik_y.setValue(float(y))
        self.spin_ik_z.setValue(float(z))
        self.spin_ik_x.blockSignals(False)
        self.spin_ik_y.blockSignals(False)
        self.spin_ik_z.blockSignals(False)

    def _read_current_cartesian(self):
        use_flange = (self.ik_target_mode.currentIndex() == 1)
        fk = self.engine.compute_forward_kinematics(self.actual_rads, use_flange=use_flange)
        cur_cart = fk["position_mm"]
        self._set_spin_values_silently(
            round(float(cur_cart[0]), 1),
            round(float(cur_cart[1]), 1),
            round(float(cur_cart[2]), 1)
        )
        self._calc_ik()

    def _set_table_item(self, table, row, col, text, color=None):
        it = table.item(row, col)
        if it is None:
            it = QTableWidgetItem(text)
            it.setTextAlignment(Qt.AlignCenter if col > 0 else Qt.AlignLeft | Qt.AlignVCenter)
            table.setItem(row, col, it)
        else:
            if it.text() != text:
                it.setText(text)
        if color:
            if it.foreground().color() != color:
                it.setForeground(color)

    def _calc_ik(self):
        tx = self.spin_ik_x.value()
        ty = self.spin_ik_y.value()
        tz = self.spin_ik_z.value()
        self.ik_target_pos = [tx, ty, tz]
        target_m = [tx / 1000.0, ty / 1000.0, tz / 1000.0]
        use_flange = (self.ik_target_mode.currentIndex() == 1) if hasattr(self, "ik_target_mode") else False

        mode = self.combo_ik_method.currentData() if hasattr(self, 'combo_ik_method') else "hybrid"
        elbow_up = self.combo_ik_elbow.currentData() if hasattr(self, 'combo_ik_elbow') else True

        q_sol, success, iters, err_m = self.engine.inverse_kinematics(
            target_m, q_seed=self.actual_rads, use_flange=use_flange,
            elbow_up=elbow_up, mode=mode
        )
        self.last_ik_sol = q_sol if success else None
        err_mm = err_m * 1000.0

        is_col, col_reason = self.engine.check_collision(q_sol) if q_sol is not None else (False, "")

        method_name = "Giải tích 0ms" if iters == 0 else f"DLS {iters} bước"
        if is_col:
            self.ik_status_card.setStyleSheet("background-color: #7f1d1d; border-radius: 6px; padding: 8px; border-left: 4px solid #ef4444;")
            self.lbl_ik_status.setText(f"⚠️ CẢNH BÁO VA CHẠM: {col_reason}")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #fecaca;")
            self.lbl_ik_info.setText("Vui lòng chọn vị trí an toàn nằm trong không gian làm việc của robot!")
            if hasattr(self, 'card_tar'):
                self.card_tar.val_lbl.setText(f"[{tx:+.0f}, {ty:+.0f}, {tz:+.0f}] mm")
            if hasattr(self, 'card_link5_tar'):
                self.card_link5_tar.val_lbl.setText(f"[{tx:+.0f}, {ty:+.0f}, {tz:+.0f}] mm")
        elif success:
            self.ik_status_card.setStyleSheet("background-color: #064e3b; border-radius: 6px; padding: 8px; border-left: 4px solid #10b981;")
            self.lbl_ik_status.setText(f"✅ Đã giải thành công góc khớp ({method_name} | Sai số: {err_mm:.3f} mm | An toàn)")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #a7f3d0;")
            deg_sol_str = ", ".join([f"{math.degrees(a):+.1f}°" for a in q_sol])
            self.lbl_ik_info.setText(f"Góc mục tiêu: [{deg_sol_str}]")
            if hasattr(self, 'card_tar'):
                self.card_tar.val_lbl.setText(f"[{tx:+.0f}, {ty:+.0f}, {tz:+.0f}] mm")
            if hasattr(self, 'card_link5_tar'):
                self.card_link5_tar.val_lbl.setText(f"[{tx:+.0f}, {ty:+.0f}, {tz:+.0f}] mm")
        else:
            self.ik_status_card.setStyleSheet("background-color: #7f1d1d; border-radius: 6px; padding: 8px; border-left: 4px solid #ef4444;")
            self.lbl_ik_status.setText(f"⚠️ Vị trí ngoài tầm với an toàn! (Sai số: {err_mm:.1f} mm)")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #fecaca;")
            self.lbl_ik_info.setText("Vui lòng nhập vị trí nằm trong tầm vươn của cánh tay robot.")
            if hasattr(self, 'card_tar'):
                self.card_tar.val_lbl.setText("---")
            if hasattr(self, 'card_link5_tar'):
                self.card_link5_tar.val_lbl.setText("---")

        # Xuất dữ liệu nghiệm IK lên ROS 2 Topics
        self.bridge.publish_inverse_kinematics(target_m, q_sol, success, iters, err_m, use_flange=use_flange)

        # Cập nhật hiển thị bảng ngay lập tức
        self._refresh_ik_table()
        return q_sol, success

    def _exec_ik_motion(self):
        q_sol, success = self._calc_ik()
        if not success or q_sol is None:
            self.ik_status_card.setStyleSheet("background-color: #7f1d1d; border-radius: 6px; padding: 8px; border-left: 4px solid #ef4444;")
            self.lbl_ik_status.setText("❌ Không thể chuyển động: Vị trí mục tiêu ngoài tầm với!")
            return

        is_col, col_reason = self.engine.check_collision(q_sol)
        if is_col:
            self.ik_status_card.setStyleSheet("background-color: #7f1d1d; border-radius: 6px; padding: 8px; border-left: 4px solid #ef4444;")
            self.lbl_ik_status.setText(f"❌ DỪNG AN TOÀN: {col_reason}")
            return

        dur = self.spin_ik_dur.value()
        self.bridge.send_trajectory(q_sol, duration_sec=dur)

        # Cập nhật target_rads và đồng bộ giao diện Tab 3 (Joint Sliders)
        self.target_rads = list(q_sol)
        self.updating_widgets = True
        for i in range(len(JOINT_NAMES)):
            self._sync_slider_from_target(i)
            if self.is_deg_mode:
                self.spin_boxes[i].setValue(math.degrees(q_sol[i]))
            else:
                self.spin_boxes[i].setValue(q_sol[i])
        self.updating_widgets = False

        self.ik_status_card.setStyleSheet("background-color: #1e3a8a; border-radius: 6px; padding: 8px; border-left: 4px solid #3b82f6;")
        self.lbl_ik_status.setText(f"🚀 Đang gửi lệnh chuyển động robot trong Gazebo ({dur:.1f}s)...")
        self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #93c5fd;")

    def _refresh_ik_table(self):
        use_flange = (self.ik_target_mode.currentIndex() == 1) if hasattr(self, "ik_target_mode") else False
        fk = self.engine.compute_forward_kinematics(self.actual_rads, use_flange=use_flange)
        self.latest_fk_data = fk
        cur_cart = fk["position_mm"]
        rpy_deg = fk["rpy_deg"]

        # Cập nhật KPI Tab 1 (Link 5 Control)
        if hasattr(self, "card_link5_cur"):
            self.card_link5_cur.val_lbl.setText(f"[{cur_cart[0]:+4.0f}, {cur_cart[1]:+4.0f}, {cur_cart[2]:+4.0f}] mm")
        if hasattr(self, "card_link5_tar"):
            if self.last_ik_sol is not None:
                self.card_link5_tar.val_lbl.setText(f"[{self.ik_target_pos[0]:+.0f}, {self.ik_target_pos[1]:+.0f}, {self.ik_target_pos[2]:+.0f}] mm")
            else:
                self.card_link5_tar.val_lbl.setText("---")
        if hasattr(self, "card_link5_dist"):
            if self.last_ik_sol is not None:
                dist = float(np.linalg.norm(np.array(self.ik_target_pos) - cur_cart))
                self.card_link5_dist.val_lbl.setText(f"ΔD: {dist:5.1f} mm")
                self.card_link5_dist.val_lbl.setStyleSheet("color: #4ade80; font-size: 15px; font-weight: bold; font-family: monospace;" if dist < 3.0 else "color: #facc15; font-size: 15px; font-weight: bold; font-family: monospace;")
            else:
                self.card_link5_dist.val_lbl.setText("ΔD: --- mm")

        # Cập nhật Bảng Góc Quay Khớp ở Tab 1
        if hasattr(self, "table_link_angles"):
            for i in range(len(JOINT_NAMES)):
                cur_deg = math.degrees(self.actual_rads[i])
                cur_rad = self.actual_rads[i]

                self._set_table_item(self.table_link_angles, i, 0, f"Khớp {i+1} (Link {i+1})")
                self._set_table_item(self.table_link_angles, i, 1, JOINT_NAMES_VI[i])
                self._set_table_item(self.table_link_angles, i, 2, f"{cur_deg:+6.1f}° ({cur_rad:+5.2f} rad)", QColor("#4ade80"))

                if self.last_ik_sol is not None:
                    tar_rad = self.last_ik_sol[i]
                    tar_deg = math.degrees(tar_rad)
                    diff = abs(tar_deg - cur_deg)
                    self._set_table_item(self.table_link_angles, i, 3, f"{tar_deg:+6.1f}° ({tar_rad:+5.2f} rad)", QColor("#38bdf8"))
                    diff_col = QColor("#4ade80") if diff < 0.8 else QColor("#facc15")
                    self._set_table_item(self.table_link_angles, i, 4, f"{diff:5.2f}°", diff_col)

                    if diff < 0.8:
                        st_text = "✅ Đã tới đích"
                        st_col = QColor("#4ade80")
                    else:
                        dir_str = "quay +" if tar_deg > cur_deg else "quay -"
                        st_text = f"⚡ Cần {dir_str}{diff:.1f}°"
                        st_col = QColor("#38bdf8")
                    self._set_table_item(self.table_link_angles, i, 6, st_text, st_col)
                else:
                    self._set_table_item(self.table_link_angles, i, 3, "---", QColor("#64748b"))
                    self._set_table_item(self.table_link_angles, i, 4, "---", QColor("#64748b"))
                    self._set_table_item(self.table_link_angles, i, 6, "● Chờ tính góc", QColor("#64748b"))

                lim_min = JOINT_INFO[i]['deg_min']
                lim_max = JOINT_INFO[i]['deg_max']
                self._set_table_item(self.table_link_angles, i, 5, f"[{lim_min:+.0f}°, {lim_max:+.0f}°]", QColor("#94a3b8"))

        # Cập nhật KPI Tab 2 (Kinematics)
        if hasattr(self, "card_cur"):
            self.card_cur.val_lbl.setText(f"[{cur_cart[0]:+4.0f}, {cur_cart[1]:+4.0f}, {cur_cart[2]:+4.0f}] mm")
        if hasattr(self, "card_rpy"):
            self.card_rpy.val_lbl.setText(f"R:{rpy_deg[0]:+4.0f}° P:{rpy_deg[1]:+4.0f}° Y:{rpy_deg[2]:+4.0f}°")
        if hasattr(self, "card_tar"):
            if self.last_ik_sol is not None:
                self.card_tar.val_lbl.setText(f"[{self.ik_target_pos[0]:+.0f}, {self.ik_target_pos[1]:+.0f}, {self.ik_target_pos[2]:+.0f}] mm")
            else:
                self.card_tar.val_lbl.setText("---")
        if hasattr(self, "card_dist"):
            if self.last_ik_sol is not None:
                dist = float(np.linalg.norm(np.array(self.ik_target_pos) - cur_cart))
                self.card_dist.val_lbl.setText(f"ΔD: {dist:5.1f} mm")
                self.card_dist.val_lbl.setStyleSheet("color: #4ade80; font-size: 15px; font-weight: bold; font-family: monospace;" if dist < 3.0 else "color: #facc15; font-size: 15px; font-weight: bold; font-family: monospace;")
            else:
                self.card_dist.val_lbl.setText("ΔD: --- mm")

        # Cập nhật Bảng Động Học Thuận Tab 2
        if hasattr(self, "ik_table"):
            link_pts = fk["link_positions_mm"]
            for i in range(len(JOINT_NAMES)):
                cur_deg = math.degrees(self.actual_rads[i])
                cur_rad = self.actual_rads[i]

                self._set_table_item(self.ik_table, i, 0, f"Joint {i+1}")
                self._set_table_item(self.ik_table, i, 1, JOINT_NAMES_VI[i])
                self._set_table_item(self.ik_table, i, 2, f"{cur_deg:+6.1f}° ({cur_rad:+5.2f})", QColor("#4ade80"))

                if self.last_ik_sol is not None:
                    tar_rad = self.last_ik_sol[i]
                    tar_deg = math.degrees(tar_rad)
                    diff = abs(tar_deg - cur_deg)
                    self._set_table_item(self.ik_table, i, 3, f"{tar_deg:+6.1f}° ({tar_rad:+5.2f})", QColor("#38bdf8"))
                    diff_col = QColor("#4ade80") if diff < 0.8 else QColor("#facc15")
                    self._set_table_item(self.ik_table, i, 4, f"{diff:5.2f}°", diff_col)
                else:
                    self._set_table_item(self.ik_table, i, 3, "---")
                    self._set_table_item(self.ik_table, i, 4, "---")

                lp = link_pts[i + 1] if i + 1 < len(link_pts) else np.zeros(3)
                self._set_table_item(self.ik_table, i, 5, f"[{lp[0]:+5.0f}, {lp[1]:+5.0f}, {lp[2]:+5.0f}]", QColor("#94a3b8"))

        # Cập nhật ma trận 4x4 (chỉ khi đang mở Tab 2 để tối ưu CPU)
        if hasattr(self, "tabs") and self.tabs.currentIndex() == 1:
            self._refresh_matrix_display()

        # Xuất dữ liệu Động Học Thuận & Visual Markers lên ROS 2 Topics
        self.bridge.publish_forward_kinematics(fk)
        target_m = [self.ik_target_pos[0]/1000.0, self.ik_target_pos[1]/1000.0, self.ik_target_pos[2]/1000.0]
        self.bridge.publish_kinematics_markers(fk, target_m, (self.last_ik_sol is not None), use_flange=use_flange)

    def _refresh_matrix_display(self):
        if not hasattr(self, 'table_matrix') or not hasattr(self, 'combo_matrix_frame'):
            return
        if not self.latest_fk_data:
            return

        idx = self.combo_matrix_frame.currentIndex()
        t_list = self.latest_fk_data["transforms"]
        target_trans_idx = min(idx + 1, len(t_list) - 1)
        T = t_list[target_trans_idx]

        for r in range(4):
            for c in range(4):
                val = T[r, c]
                if r < 3 and c == 3:
                    txt = f"{val * 1000.0:+7.1f} mm"
                    col = QColor("#4ade80")
                elif r == 3:
                    txt = f"{val:.0f}"
                    col = QColor("#64748b")
                else:
                    txt = f"{val:+7.4f}"
                    col = QColor("#38bdf8")

                it = self.table_matrix.item(r, c)
                if it is None:
                    it = QTableWidgetItem(txt)
                    it.setTextAlignment(Qt.AlignCenter)
                    self.table_matrix.setItem(r, c, it)
                else:
                    it.setText(txt)
                it.setForeground(col)

    def _export_kinematics_csv(self):
        save_dir = os.path.expanduser("~/denso_ws/kinematics_snapshots")
        os.makedirs(save_dir, exist_ok=True)
        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"denso_kinematics_{ts_str}.csv"
        path = os.path.join(save_dir, filename)

        try:
            use_flange = (self.ik_target_mode.currentIndex() == 1) if hasattr(self, "ik_target_mode") else False
            fk = self.engine.compute_forward_kinematics(self.actual_rads, use_flange=use_flange)
            target = self.ik_target_pos
            sol = self.last_ik_sol if self.last_ik_sol is not None else [0.0]*5

            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["=== DENSO VS-6556 KINEMATICS SNAPSHOT REPORT ==="])
                writer.writerow(["Timestamp", ts_str])
                writer.writerow(["Target_Mode", "Flange" if use_flange else "Point_P"])
                writer.writerow(["Target_X_mm", target[0], "Target_Y_mm", target[1], "Target_Z_mm", target[2]])
                writer.writerow(["Actual_X_mm", f"{fk['position_mm'][0]:.2f}", "Actual_Y_mm", f"{fk['position_mm'][1]:.2f}", "Actual_Z_mm", f"{fk['position_mm'][2]:.2f}"])
                writer.writerow(["Euler_Roll_deg", f"{fk['rpy_deg'][0]:.2f}", "Euler_Pitch_deg", f"{fk['rpy_deg'][1]:.2f}", "Euler_Yaw_deg", f"{fk['rpy_deg'][2]:.2f}"])
                writer.writerow(["Quat_X", f"{fk['quaternion'][0]:.4f}", "Quat_Y", f"{fk['quaternion'][1]:.4f}", "Quat_Z", f"{fk['quaternion'][2]:.4f}", "Quat_W", f"{fk['quaternion'][3]:.4f}"])
                writer.writerow([])
                writer.writerow(["Joint_ID", "Joint_Name", "Actual_Deg", "Actual_Rad", "Target_Deg", "Target_Rad", "Diff_Deg", "Limit_Min_Deg", "Limit_Max_Deg", "Link_X_mm", "Link_Y_mm", "Link_Z_mm"])

                for i in range(len(JOINT_NAMES)):
                    act_deg = math.degrees(self.actual_rads[i])
                    tar_deg = math.degrees(sol[i])
                    lp = fk["link_positions_mm"][i + 1]
                    writer.writerow([
                        JOINT_NAMES[i],
                        JOINT_NAMES_VI[i],
                        f"{act_deg:.2f}",
                        f"{self.actual_rads[i]:.4f}",
                        f"{tar_deg:.2f}",
                        f"{sol[i]:.4f}",
                        f"{abs(tar_deg - act_deg):.2f}",
                        f"{JOINT_INFO[i]['deg_min']:.1f}",
                        f"{JOINT_INFO[i]['deg_max']:.1f}",
                        f"{lp[0]:.1f}", f"{lp[1]:.1f}", f"{lp[2]:.1f}"
                    ])

            self.lbl_ik_status.setText(f"📸 Đã xuất snapshot động học CSV → {path}")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #a7f3d0;")
        except Exception as e:
            self.lbl_ik_status.setText(f"❌ Lỗi khi xuất CSV: {e}")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #fecaca;")

    def _export_kinematics_json(self):
        save_dir = os.path.expanduser("~/denso_ws/kinematics_snapshots")
        os.makedirs(save_dir, exist_ok=True)
        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"denso_kinematics_{ts_str}.json"
        path = os.path.join(save_dir, filename)

        try:
            use_flange = (self.ik_target_mode.currentIndex() == 1) if hasattr(self, "ik_target_mode") else False
            fk = self.engine.compute_forward_kinematics(self.actual_rads, use_flange=use_flange)
            sol = self.last_ik_sol if self.last_ik_sol is not None else [0.0]*5

            report = {
                "timestamp": ts_str,
                "target_mode": "flange" if use_flange else "point_p",
                "cartesian_target_mm": self.ik_target_pos,
                "cartesian_actual_mm": [round(float(v), 2) for v in fk["position_mm"]],
                "rpy_deg": [round(float(v), 2) for v in fk["rpy_deg"]],
                "quaternion": [round(float(v), 4) for v in fk["quaternion"]],
                "joints": [
                    {
                        "id": JOINT_NAMES[i],
                        "name": JOINT_NAMES_VI[i],
                        "actual_deg": round(math.degrees(self.actual_rads[i]), 2),
                        "actual_rad": round(self.actual_rads[i], 4),
                        "target_deg": round(math.degrees(sol[i]), 2),
                        "target_rad": round(sol[i], 4),
                        "link_pos_mm": [round(float(v), 1) for v in fk["link_positions_mm"][i + 1]]
                    } for i in range(len(JOINT_NAMES))
                ],
                "transforms_T0": [
                    [[round(float(val), 4) for val in row] for row in T]
                    for T in fk["transforms"]
                ]
            }

            with open(path, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2, ensure_ascii=False)

            self.lbl_ik_status.setText(f"📄 Đã xuất báo cáo động học JSON → {path}")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #a7f3d0;")
        except Exception as e:
            self.lbl_ik_status.setText(f"❌ Lỗi khi xuất JSON: {e}")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #fecaca;")

    # ---------------------------------------------------------
    # TAB 3: DEDICATED DYNAMICS TELEMETRY TABLE (NỀN TRẮNG RÕ NÉT)
    # ---------------------------------------------------------
    def create_telemetry_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # 1. Top KPI Summary Cards (White Cards)
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(10)

        def make_white_card(title, initial_val, subtext, val_color="#0284c7"):
            box = QFrame()
            box.setStyleSheet("""
                QFrame {
                    background-color: #ffffff;
                    border: 1px solid #cbd5e1;
                    border-radius: 8px;
                    padding: 8px 12px;
                }
            """)
            v = QVBoxLayout(box)
            v.setContentsMargins(4, 4, 4, 4)
            v.setSpacing(3)
            lbl_t = QLabel(title)
            lbl_t.setStyleSheet("color: #64748b; font-size: 11px; font-weight: bold; text-transform: uppercase;")
            lbl_v = QLabel(initial_val)
            lbl_v.setStyleSheet(f"color: {val_color}; font-size: 19px; font-weight: bold; font-family: 'Segoe UI', monospace;")
            lbl_s = QLabel(subtext)
            lbl_s.setStyleSheet("color: #94a3b8; font-size: 11px;")
            v.addWidget(lbl_t)
            v.addWidget(lbl_v)
            v.addWidget(lbl_s)
            return box, lbl_v, lbl_s

        self.kpi_tau_box, self.kpi_tau_val, self.kpi_tau_sub = make_white_card(
            "MÔ-MEN LỚN NHẤT (MAX TORQUE)", "0.0 N·m", "Định mức tối đa 100 N·m", "#0284c7"
        )
        self.kpi_load_box, self.kpi_load_val, self.kpi_load_sub = make_white_card(
            "TẢI MOTOR CAO NHẤT (% LOAD)", "0.0%", "Ngưỡng an toàn < 80%", "#10b981"
        )
        self.kpi_power_box, self.kpi_power_val, self.kpi_power_sub = make_white_card(
            "TỔNG CÔNG SUẤT (POWER)", "0.0 W", "P = Σ |τ_i · q̇_i| cơ học", "#8b5cf6"
        )
        self.kpi_safety_box, self.kpi_safety_val, self.kpi_safety_sub = make_white_card(
            "AN TOÀN & VA CHẠM (STATUS)", "● AN TOÀN", "Không phát hiện va chạm tự thân", "#059669"
        )

        cards_layout.addWidget(self.kpi_tau_box)
        cards_layout.addWidget(self.kpi_load_box)
        cards_layout.addWidget(self.kpi_power_box)
        cards_layout.addWidget(self.kpi_safety_box)
        layout.addLayout(cards_layout)

        # 2. Main White Telemetry Table
        tbl_container = QGroupBox("BẢNG THEO DÕI ĐỘNG LỰC HỌC 5 KHỚP ROBOT DENSO THỜI GIAN THỰC")
        tbl_container.setFont(QFont("Segoe UI", 11, QFont.Bold))
        tbl_container.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 14px;
                padding: 14px 10px 10px 10px;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 8px;
                color: #0284c7;
                font-weight: bold;
                background-color: #ffffff;
            }
        """)
        tbl_layout = QVBoxLayout(tbl_container)
        tbl_layout.setContentsMargins(6, 10, 6, 6)

        self.telemetry_table = QTableWidget(len(JOINT_NAMES), 10)
        self.telemetry_table.setHorizontalHeaderLabels([
            "Khớp", "Trục & Vai Trò", "Vị Trí Góc q", "Vận Tốc q̇", "Gia Tốc q̈",
            "Mô-Men τ (N·m)", "Giới Hạn τ_max", "Tải Động Cơ (%)", "Công Suất P (W)", "Trạng Thái"
        ])
        h_header = self.telemetry_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
            h_header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(6, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(7, QHeaderView.Fixed)
            h_header.setSectionResizeMode(8, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(9, QHeaderView.Fixed)
        self.telemetry_table.setColumnWidth(7, 160)
        self.telemetry_table.setColumnWidth(9, 130)

        v_header = self.telemetry_table.verticalHeader()
        if v_header is not None:
            v_header.setVisible(False)
            v_header.setDefaultSectionSize(46)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.telemetry_table.setFixedHeight(275)
        self.telemetry_table.setAlternatingRowColors(True)

        for row in range(len(JOINT_NAMES)):
            self.telemetry_table.setRowHeight(row, 46)

        self.telemetry_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                color: #0f172a;
                gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                font-family: 'Segoe UI', 'DejaVu Sans', sans-serif;
                font-size: 13px;
                selection-background-color: #f1f5f9;
            }
            QHeaderView::section {
                background-color: #f1f5f9;
                color: #0f172a;
                font-weight: bold;
                font-size: 12px;
                padding: 8px 6px;
                border: 1px solid #e2e8f0;
                border-top: none;
                border-left: none;
            }
            QTableWidget::item {
                padding: 4px 6px;
                color: #0f172a;
            }
            QTableWidget::item:alternate {
                background-color: #f8fafc;
            }
        """)

        self.tab3_progress_bars = []
        self.tab3_status_badges = []

        for row in range(len(JOINT_NAMES)):
            it_name = QTableWidgetItem(f"Joint {row+1}")
            it_name.setFont(QFont("Segoe UI", 11, QFont.Bold))
            it_name.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 0, it_name)

            it_axis = QTableWidgetItem(JOINT_NAMES_VI[row])
            it_axis.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.telemetry_table.setItem(row, 1, it_axis)

            it_q = QTableWidgetItem("0.0° (0.000 rad)")
            it_q.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 2, it_q)

            it_qd = QTableWidgetItem("0.0°/s (0.000 rad/s)")
            it_qd.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 3, it_qd)

            it_qdd = QTableWidgetItem("0.0°/s² (0.00 rad/s²)")
            it_qdd.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 4, it_qdd)

            it_tau = QTableWidgetItem("0.00 N·m")
            it_tau.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            it_tau.setFont(QFont("Segoe UI", 11, QFont.Bold))
            self.telemetry_table.setItem(row, 5, it_tau)

            it_lim = QTableWidgetItem(f"{JOINT_INFO[row]['effort_limit']:.0f} N·m")
            it_lim.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 6, it_lim)

            # Motor Load Gauge
            pb_container = QWidget()
            pb_container.setStyleSheet("background-color: transparent;")
            pb_box = QHBoxLayout(pb_container)
            pb_box.setContentsMargins(6, 0, 6, 0)
            pb_box.setSpacing(8)
            pb_box.setAlignment(Qt.AlignmentFlag.AlignVCenter)

            pb = QProgressBar()
            pb.setRange(0, 100)
            pb.setValue(0)
            pb.setFixedHeight(16)
            pb.setTextVisible(False)
            pb.setStyleSheet("""
                QProgressBar {
                    background-color: #f1f5f9;
                    border: 1px solid #cbd5e1;
                    border-radius: 4px;
                }
                QProgressBar::chunk {
                    background-color: #10b981;
                    border-radius: 3px;
                }
            """)

            lbl_pct = QLabel("0.0%")
            lbl_pct.setFixedWidth(50)
            lbl_pct.setFont(QFont("Segoe UI", 10, QFont.Bold))
            lbl_pct.setStyleSheet("color: #0f172a;")
            lbl_pct.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

            pb_box.addWidget(pb)
            pb_box.addWidget(lbl_pct)
            self.telemetry_table.setCellWidget(row, 7, pb_container)
            self.tab3_progress_bars.append((pb, lbl_pct))

            it_power = QTableWidgetItem("0.0 W")
            it_power.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 8, it_power)

            # Safety Badge
            badge = QLabel("● AN TOÀN")
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setFont(QFont("Segoe UI", 9, QFont.Bold))
            badge.setFixedHeight(26)
            badge.setMinimumWidth(110)
            badge.setStyleSheet("""
                background-color: #ecfdf5;
                color: #065f46;
                border: 1.5px solid #10b981;
                border-radius: 12px;
                font-weight: bold;
                font-size: 11px;
                padding: 0px 6px;
            """)
            badge_container = QWidget()
            badge_container.setStyleSheet("background-color: transparent;")
            badge_box = QHBoxLayout(badge_container)
            badge_box.setContentsMargins(4, 0, 4, 0)
            badge_box.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge_box.addWidget(badge)
            self.telemetry_table.setCellWidget(row, 9, badge_container)
            self.tab3_status_badges.append(badge)

        tbl_layout.addWidget(self.telemetry_table)
        layout.addWidget(tbl_container)

        # 3. Control & Quick Actions Toolbar (White Box)
        ctrl_box = QGroupBox("⚙️ ĐIỀU CHỈNH THÔNG SỐ ĐỘNG LỰC & TIỆN ÍCH")
        ctrl_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        ctrl_box.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 14px;
                padding: 12px 14px 10px 14px;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 8px;
                color: #0284c7;
                font-weight: bold;
                background-color: #ffffff;
            }
        """)
        ctrl_layout = QHBoxLayout(ctrl_box)
        ctrl_layout.setSpacing(12)

        lbl_p = QLabel("Tải Trọng Đầu Gắp (kg):")
        lbl_p.setStyleSheet("color: #0f172a; font-weight: bold; font-size: 13px;")
        ctrl_layout.addWidget(lbl_p)

        self.sp_tab3_payload = QDoubleSpinBox()
        self.sp_tab3_payload.setRange(0.0, 7.0)
        self.sp_tab3_payload.setValue(0.0)
        self.sp_tab3_payload.setSingleStep(0.5)
        self.sp_tab3_payload.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #f8fafc;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 5px 8px;
                font-weight: bold;
                font-size: 13px;
            }
        """)
        self.sp_tab3_payload.valueChanged.connect(self._on_payload_changed)
        ctrl_layout.addWidget(self.sp_tab3_payload)

        ctrl_layout.addSpacing(10)

        btn_csv = QPushButton("📸 Xuất Bảng CSV (Snapshot)")
        btn_csv.setStyleSheet("background-color: #0284c7; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_csv.clicked.connect(self._export_telemetry_csv)
        ctrl_layout.addWidget(btn_csv)

        btn_home = QPushButton("⌂ Về Gốc (Home 0°)")
        btn_home.setStyleSheet("background-color: #059669; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_home.clicked.connect(lambda: self.apply_preset([0.0]*5))
        ctrl_layout.addWidget(btn_home)

        btn_stop = QPushButton("⏹ Dừng Robot")
        btn_stop.setStyleSheet("background-color: #dc2626; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_stop.clicked.connect(self.stop_and_hold)
        ctrl_layout.addWidget(btn_stop)

        ctrl_layout.addStretch()
        layout.addWidget(ctrl_box)

        # Snapshot Feedback Bar
        self.tab3_log_label = QLabel("🟢 Hệ thống động lực học Denso VS-6556 sẵn sàng. Dữ liệu được tính toán RNEA thời gian thực ở 20Hz.")
        self.tab3_log_label.setStyleSheet("color: #0284c7; font-size: 12px; font-weight: 500; padding: 4px;")
        layout.addWidget(self.tab3_log_label)

        layout.addStretch()
        return tab

    def _on_payload_changed(self, val):
        self.telemetry_payload = val
        self.tab3_log_label.setText(f"⚖️ Đã cập nhật tải trọng đầu gắp: {val:.2f} kg (Mô-men RNEA tự động tính bù quán tính)")

    def _export_telemetry_csv(self):
        save_dir = os.path.expanduser("~/denso_ws/telemetry_snapshots")
        os.makedirs(save_dir, exist_ok=True)
        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"denso_telemetry_{ts_str}.csv"
        path = os.path.join(save_dir, filename)

        try:
            pos, vel, eff, _ = self.bridge.get_latest_state()
            dyn_res = self.engine.inverse_dynamics(
                pos, vel, self.actual_accels, payload_mass=self.telemetry_payload
            )
            taus = dyn_res['tau']
            pct_loads = dyn_res['percent_load']

            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Joint_ID", "Joint_Name", "Position_Deg", "Position_Rad",
                    "Velocity_Deg_s", "Velocity_Rad_s", "Accel_Deg_s2", "Accel_Rad_s2",
                    "Torque_RNEA_Nm", "Effort_Gazebo_Nm", "Limit_Nm", "Load_Percent",
                    "Power_Watts", "Payload_kg"
                ])
                for i in range(len(JOINT_NAMES)):
                    writer.writerow([
                        JOINT_NAMES[i],
                        JOINT_NAMES_VI[i],
                        f"{math.degrees(pos[i]):.3f}",
                        f"{pos[i]:.4f}",
                        f"{math.degrees(vel[i]):.3f}",
                        f"{vel[i]:.4f}",
                        f"{math.degrees(self.actual_accels[i]):.3f}",
                        f"{self.actual_accels[i]:.4f}",
                        f"{taus[i]:.3f}",
                        f"{eff[i]:.3f}",
                        f"{JOINT_INFO[i]['effort_limit']:.1f}",
                        f"{pct_loads[i]:.1f}",
                        f"{abs(taus[i] * vel[i]):.2f}",
                        f"{self.telemetry_payload:.2f}"
                    ])

            self.tab3_log_label.setText(f"📸 Đã xuất snapshot dữ liệu thành công → {path}")
            self.tab3_log_label.setStyleSheet("color: #059669; font-size: 12px; font-weight: bold; padding: 4px;")
        except Exception as e:
            self.tab3_log_label.setText(f"❌ Lỗi khi xuất file CSV: {e}")
            self.tab3_log_label.setStyleSheet("color: #dc2626; font-size: 12px; font-weight: bold; padding: 4px;")

    def _refresh_telemetry_table(self):
        if not hasattr(self, 'telemetry_table'):
            return

        pos, vel, eff, _ = self.bridge.get_latest_state()

        # Differentiate velocity to obtain numerical acceleration
        now = time.time()
        dt = max(1e-3, now - self.prev_time)
        self.prev_time = now

        # Khử rung vị trí quanh mốc 0 (Deadband < 0.05 độ để triệt tiêu hiện tượng "-0.0°")
        clean_pos = list(pos)
        for i in range(len(JOINT_NAMES)):
            if abs(math.degrees(clean_pos[i])) < 0.05:
                clean_pos[i] = 0.0

        # Lọc thông thấp EMA cho vận tốc và Deadband khi robot đứng yên
        alpha_vel = 0.20
        clean_vel = [0.0] * len(JOINT_NAMES)
        for i in range(len(JOINT_NAMES)):
            self.filtered_vels[i] = (1.0 - alpha_vel) * self.filtered_vels[i] + alpha_vel * vel[i]
            # Vận tốc dưới 0.002 rad/s (~0.11°/s) xem như robot đứng yên hoàn toàn
            if abs(self.filtered_vels[i]) < 0.002:
                self.filtered_vels[i] = 0.0
            clean_vel[i] = self.filtered_vels[i]

        # Tính gia tốc số học từ vận tốc đã lọc mịn, kèm phân rã khi dừng chuyển động
        for i in range(len(JOINT_NAMES)):
            if clean_vel[i] == 0.0 and self.prev_velocities[i] == 0.0:
                raw_acc = 0.0
                self.actual_accels[i] = 0.6 * self.actual_accels[i]
            else:
                raw_acc = (clean_vel[i] - self.prev_velocities[i]) / dt
                self.actual_accels[i] = 0.85 * self.actual_accels[i] + 0.15 * raw_acc

            # Deadband gia tốc dưới 0.01 rad/s² (~0.5°/s²) cố định về 0.0
            if abs(self.actual_accels[i]) < 0.01:
                self.actual_accels[i] = 0.0
        self.prev_velocities = list(clean_vel)

        # Tính toán động lực học ngược RNEA bằng bộ trạng thái đã khử nhiễu
        dyn_res = self.engine.inverse_dynamics(
            clean_pos, clean_vel, self.actual_accels, payload_mass=self.telemetry_payload
        )
        raw_taus = dyn_res['tau']

        # Lọc thông thấp EMA cho Mô-men lực và Tải động cơ, khử giật số
        alpha_tau = 0.15
        taus = [0.0] * len(JOINT_NAMES)
        pct_loads = [0.0] * len(JOINT_NAMES)
        for i in range(len(JOINT_NAMES)):
            self.filtered_taus[i] = (1.0 - alpha_tau) * self.filtered_taus[i] + alpha_tau * raw_taus[i]
            # Deadband lực dưới 0.02 N·m cố định về 0.0 (khử -0.00 N·m cho trục 1 Base Yaw)
            if abs(self.filtered_taus[i]) < 0.02:
                self.filtered_taus[i] = 0.0
            taus[i] = self.filtered_taus[i]

            limit = JOINT_INFO[i]['effort_limit']
            raw_pct = (abs(taus[i]) / limit) * 100.0
            self.filtered_loads[i] = 0.85 * self.filtered_loads[i] + 0.15 * raw_pct
            if self.filtered_loads[i] < 0.05:
                self.filtered_loads[i] = 0.0
            pct_loads[i] = min(100.0, self.filtered_loads[i])

        # Self-collision check
        is_col, col_reason = self.engine.check_collision(clean_pos)

        # Cập nhật Top KPI Cards (chống nhấp nháy giao diện)
        max_tau = float(max(abs(taus[j]) for j in range(len(JOINT_NAMES))))
        if max_tau < 0.05:
            max_tau = 0.0
        max_load = float(max(pct_loads[j] for j in range(len(JOINT_NAMES))))
        if max_load < 0.05:
            max_load = 0.0
        total_power = sum(abs(taus[j] * clean_vel[j]) for j in range(len(JOINT_NAMES)))
        if total_power < 0.05:
            total_power = 0.0

        new_tau_text = f"{max_tau:.1f} N·m"
        if self.kpi_tau_val.text() != new_tau_text:
            self.kpi_tau_val.setText(new_tau_text)

        new_load_text = f"{max_load:.1f}%"
        if self.kpi_load_val.text() != new_load_text:
            self.kpi_load_val.setText(new_load_text)

        new_power_text = f"{total_power:.1f} W"
        if self.kpi_power_val.text() != new_power_text:
            self.kpi_power_val.setText(new_power_text)

        if is_col:
            if self.kpi_safety_val.text() != "⚠️ VA CHẠM TỰ THÂN":
                self.kpi_safety_val.setText("⚠️ VA CHẠM TỰ THÂN")
                self.kpi_safety_val.setStyleSheet("color: #dc2626; font-size: 16px; font-weight: bold;")
                self.kpi_safety_sub.setText(col_reason)
        elif max_load > 85.0:
            if self.kpi_safety_val.text() != "🔥 QUÁ TẢI MOTOR":
                self.kpi_safety_val.setText("🔥 QUÁ TẢI MOTOR")
                self.kpi_safety_val.setStyleSheet("color: #dc2626; font-size: 18px; font-weight: bold;")
                self.kpi_safety_sub.setText("Tải động cơ vượt ngưỡng 85%")
        elif max_load > 60.0:
            if self.kpi_safety_val.text() != "⚡ TẢI CAO":
                self.kpi_safety_val.setText("⚡ TẢI CAO")
                self.kpi_safety_val.setStyleSheet("color: #d97706; font-size: 18px; font-weight: bold;")
                self.kpi_safety_sub.setText("Đang hoạt động mức tải trung bình")
        else:
            if self.kpi_safety_val.text() != "● AN TOÀN":
                self.kpi_safety_val.setText("● AN TOÀN")
                self.kpi_safety_val.setStyleSheet("color: #059669; font-size: 19px; font-weight: bold;")
                self.kpi_safety_sub.setText("Tất cả khớp hoạt động trong định mức")

        # Cập nhật 5 hàng dữ liệu bảng telemetry (chống giật, triệt tiêu -0.0)
        for i in range(len(JOINT_NAMES)):
            # Pos
            deg_val = math.degrees(clean_pos[i])
            if abs(deg_val) < 0.05:
                deg_val = 0.0
            rad_val = 0.0 if abs(clean_pos[i]) < 0.0005 else clean_pos[i]
            pos_str = f"{deg_val:+.1f}° ({rad_val:+.3f})"

            # Vel
            deg_s = math.degrees(clean_vel[i])
            if abs(deg_s) < 0.05:
                deg_s = 0.0
            rad_s = 0.0 if abs(clean_vel[i]) < 0.005 else clean_vel[i]
            vel_str = f"{deg_s:+.1f}°/s ({rad_s:+.2f})"

            # Acc
            deg_s2 = math.degrees(self.actual_accels[i])
            if abs(deg_s2) < 0.05:
                deg_s2 = 0.0
            acc_str = f"{deg_s2:+.1f}°/s²"

            # Torque
            t_val = 0.0 if abs(taus[i]) < 0.005 else taus[i]
            tau_str = f"{t_val:+.2f} N·m"

            # Power
            p_joint = abs(taus[i] * clean_vel[i])
            if p_joint < 0.05:
                p_joint = 0.0
            p_str = f"{p_joint:.1f} W"

            # Cập nhật ô chỉ khi chuỗi văn bản thay đổi để tránh Qt re-render liên tục
            it_pos = self.telemetry_table.item(i, 2)
            if it_pos and it_pos.text() != pos_str:
                it_pos.setText(pos_str)

            it_vel = self.telemetry_table.item(i, 3)
            if it_vel and it_vel.text() != vel_str:
                it_vel.setText(vel_str)

            it_acc = self.telemetry_table.item(i, 4)
            if it_acc and it_acc.text() != acc_str:
                it_acc.setText(acc_str)

            it_tau = self.telemetry_table.item(i, 5)
            if it_tau and it_tau.text() != tau_str:
                it_tau.setText(tau_str)

            it_pwr = self.telemetry_table.item(i, 8)
            if it_pwr and it_pwr.text() != p_str:
                it_pwr.setText(p_str)

            # Motor load gauge
            pb, lbl_pct = self.tab3_progress_bars[i]
            cur_pct = min(100, max(0, int(round(pct_loads[i]))))
            if pb.value() != cur_pct:
                pb.setValue(cur_pct)
            pct_str = f"{pct_loads[i]:.1f}%"
            if lbl_pct.text() != pct_str:
                lbl_pct.setText(pct_str)

            load_level = 2 if pct_loads[i] > 85 else (1 if pct_loads[i] > 60 else 0)
            if not hasattr(self, '_tab4_prev_load_levels'):
                self._tab4_prev_load_levels = [-1] * len(JOINT_NAMES)
            if self._tab4_prev_load_levels[i] != load_level:
                if load_level == 2:
                    pb.setStyleSheet("QProgressBar { background-color: #fee2e2; border: 1px solid #fca5a5; border-radius: 4px; } QProgressBar::chunk { background-color: #ef4444; border-radius: 3px; }")
                    lbl_pct.setStyleSheet("color: #ef4444; font-weight: bold;")
                elif load_level == 1:
                    pb.setStyleSheet("QProgressBar { background-color: #fef3c7; border: 1px solid #fde68a; border-radius: 4px; } QProgressBar::chunk { background-color: #f59e0b; border-radius: 3px; }")
                    lbl_pct.setStyleSheet("color: #d97706; font-weight: bold;")
                else:
                    pb.setStyleSheet("QProgressBar { background-color: #ecfdf5; border: 1px solid #a7f3d0; border-radius: 4px; } QProgressBar::chunk { background-color: #10b981; border-radius: 3px; }")
                    lbl_pct.setStyleSheet("color: #059669; font-weight: bold;")
                self._tab4_prev_load_levels[i] = load_level

            # Badge
            badge = self.tab3_status_badges[i]
            badge_level = 3 if is_col else (2 if pct_loads[i] > 85 else (1 if pct_loads[i] > 60 else 0))
            if not hasattr(self, '_tab4_prev_badge_levels'):
                self._tab4_prev_badge_levels = [-1] * len(JOINT_NAMES)
            if self._tab4_prev_badge_levels[i] != badge_level:
                if badge_level == 3:
                    badge.setText("⚠️ VA CHẠM")
                    badge.setStyleSheet("background-color: #fee2e2; color: #991b1b; border: 1.5px solid #ef4444; border-radius: 12px; font-weight: bold; font-size: 11px;")
                elif badge_level == 2:
                    badge.setText("🔥 QUÁ TẢI")
                    badge.setStyleSheet("background-color: #fee2e2; color: #991b1b; border: 1.5px solid #ef4444; border-radius: 12px; font-weight: bold; font-size: 11px;")
                elif badge_level == 1:
                    badge.setText("⚠️ CẢNH BÁO")
                    badge.setStyleSheet("background-color: #fef3c7; color: #92400e; border: 1.5px solid #f59e0b; border-radius: 12px; font-weight: bold; font-size: 11px;")
                else:
                    badge.setText("● AN TOÀN")
                    badge.setStyleSheet("background-color: #ecfdf5; color: #065f46; border: 1.5px solid #10b981; border-radius: 12px; font-weight: bold; font-size: 11px;")
                self._tab4_prev_badge_levels[i] = badge_level

    def create_joints_tab(self):
        panel = QWidget()
        tab_layout = QVBoxLayout(panel)
        tab_layout.setContentsMargins(12, 12, 12, 12)
        tab_layout.setSpacing(10)

        # Options Bar
        options_box = QFrame(self)
        options_box.setStyleSheet('background-color: #111827; border: 1px solid #1f2937; border-radius: 8px;')
        opt_layout = QHBoxLayout(options_box)
        opt_layout.setContentsMargins(12, 8, 12, 8)

        unit_lbl = QLabel('Đơn vị:', self)
        unit_lbl.setStyleSheet('font-weight: bold;')
        opt_layout.addWidget(unit_lbl)

        self.rb_deg = QRadioButton('Độ (°)', self)
        self.rb_rad = QRadioButton('Radian (rad)', self)
        self.rb_deg.setChecked(True)
        self.unit_group = QButtonGroup(self)
        self.unit_group.addButton(self.rb_deg)
        self.unit_group.addButton(self.rb_rad)
        opt_layout.addWidget(self.rb_deg)
        opt_layout.addWidget(self.rb_rad)

        opt_layout.addSpacing(20)

        self.cb_live = QCheckBox('⚡ Chế độ Kéo Trực Tiếp (Live Drag)', self)
        self.cb_live.setToolTip('Khi bật, kéo slider đến đâu robot trong Gazebo chuyển động theo tức thì!')
        self.cb_live.setStyleSheet('color: #facc15; font-weight: bold;')
        opt_layout.addWidget(self.cb_live)

        opt_layout.addSpacing(20)

        dur_lbl = QLabel('Thời gian chuyển động:', self)
        opt_layout.addWidget(dur_lbl)
        self.sp_duration = QDoubleSpinBox(self)
        self.sp_duration.setRange(0.2, 10.0)
        self.sp_duration.setValue(2.0)
        self.sp_duration.setSingleStep(0.2)
        self.sp_duration.setSuffix(' s')
        self.sp_duration.setFixedWidth(85)
        opt_layout.addWidget(self.sp_duration)

        tab_layout.addWidget(options_box)

        # Presets Bar
        preset_box = QFrame(self)
        preset_box.setStyleSheet('background-color: #111827; border: 1px solid #1f2937; border-radius: 8px;')
        p_layout = QHBoxLayout(preset_box)
        p_layout.setContentsMargins(12, 6, 12, 6)

        p_lbl = QLabel('Tư thế mẫu:', self)
        p_lbl.setStyleSheet('font-weight: bold; color: #38bdf8;')
        p_layout.addWidget(p_lbl)

        for name, angles in PRESETS.items():
            btn = QPushButton(name, self)
            btn.setStyleSheet('background-color: #1e293b; padding: 6px 12px; font-size: 12px;')
            btn.clicked.connect(lambda _, a=angles: self.apply_preset(a))
            p_layout.addWidget(btn)

        tab_layout.addWidget(preset_box)

        # Joint Cards
        joints_box = QGroupBox('ĐIỀU CHỈNH TỪNG KHỚP (JOINT CONTROLS)', self)
        j_layout = QVBoxLayout(joints_box)
        j_layout.setSpacing(6)
        j_layout.setContentsMargins(12, 14, 12, 10)

        self.sliders = []
        self.spin_boxes = []
        self.actual_labels = []
        self.effort_labels = []

        for i, info in enumerate(JOINT_INFO):
            card = self.create_joint_row(i, info)
            j_layout.addWidget(card)

        tab_layout.addWidget(joints_box)

        # Bottom Buttons
        btn_box = QFrame(self)
        b_layout = QHBoxLayout(btn_box)
        b_layout.setContentsMargins(0, 4, 0, 0)

        self.btn_send = QPushButton('🚀 GỬI LỆNH ĐẾN GAZEBO', self)
        self.btn_send.setFixedHeight(44)
        self.btn_send.setStyleSheet('background-color: #16a34a; font-size: 14px; font-weight: bold;')

        self.btn_read = QPushButton('📥 ĐỌC GÓC HIỆN TẠI', self)
        self.btn_read.setFixedHeight(44)
        self.btn_read.setStyleSheet('background-color: #334155; font-size: 13px;')

        self.btn_stop = QPushButton('🛑 DỪNG KHẨN CẤP / GIỮ VỊ TRÍ', self)
        self.btn_stop.setFixedHeight(44)
        self.btn_stop.setStyleSheet('background-color: #dc2626; font-size: 13px;')

        b_layout.addWidget(self.btn_send, 3)
        b_layout.addWidget(self.btn_read, 2)
        b_layout.addWidget(self.btn_stop, 2)
        tab_layout.addWidget(btn_box)

        return panel

    def create_joint_row(self, idx, info):
        card = QFrame(self)
        card.setStyleSheet('background-color: #0b1120; border: 1px solid #1e293b; border-radius: 6px;')
        row = QHBoxLayout(card)
        row.setContentsMargins(12, 6, 12, 6)
        row.setSpacing(12)

        lbl_name = QLabel(f"{info['name']}\n[{info['axis']}]", self)
        lbl_name.setFixedWidth(170)
        lbl_name.setStyleSheet('font-weight: bold; color: #38bdf8; font-size: 12px;')
        row.addWidget(lbl_name)

        slider = QSlider(Qt.Horizontal, self)
        slider.setRange(-1000, 1000)
        slider.setValue(0)
        self.sliders.append(slider)
        row.addWidget(slider, 1)

        spin = QDoubleSpinBox(self)
        spin.setFixedWidth(110)
        spin.setDecimals(2)
        spin.setRange(info['deg_min'], info['deg_max'])
        spin.setValue(0.0)
        spin.setSuffix(' °')
        spin.setSingleStep(1.0)
        self.spin_boxes.append(spin)
        row.addWidget(spin)

        act_lbl = QLabel('Gazebo: 0.00°', self)
        act_lbl.setFixedWidth(130)
        act_lbl.setStyleSheet('color: #4ade80; font-weight: bold; font-size: 12px;')
        self.actual_labels.append(act_lbl)
        row.addWidget(act_lbl)

        eff_lbl = QLabel('Lực: 0.0 Nm', self)
        eff_lbl.setFixedWidth(120)
        eff_lbl.setStyleSheet('color: #94a3b8; font-size: 11px;')
        self.effort_labels.append(eff_lbl)
        row.addWidget(eff_lbl)

        return card

    def setup_connections(self):
        self.rb_deg.toggled.connect(self.update_unit_display)
        self.cb_live.toggled.connect(self._toggle_live_drag)
        self.btn_send.clicked.connect(self.send_to_gazebo)
        self.btn_read.clicked.connect(self.read_actual_as_target)
        self.btn_stop.clicked.connect(self.stop_and_hold)

        for i in range(len(JOINT_NAMES)):
            self.spin_boxes[i].valueChanged.connect(lambda val, idx=i: self._on_spin_changed(idx, val))
            self.sliders[i].valueChanged.connect(lambda val, idx=i: self._on_slider_changed(idx, val))

        # Tự động tính toán lại và cập nhật xem trước IK khi thay đổi khâu mục tiêu
        self.ik_target_mode.currentIndexChanged.connect(lambda _: self._calc_ik())

    def _toggle_live_drag(self, enabled):
        self.live_drag = enabled
        if enabled:
            self.btn_send.setEnabled(False)
            self.btn_send.setText('⚡ ĐANG Ở CHẾ ĐỘ LIVE DRAG')
        else:
            self.btn_send.setEnabled(True)
            self.btn_send.setText('🚀 GỬI LỆNH ĐẾN GAZEBO')

    def update_unit_display(self):
        self.is_deg_mode = self.rb_deg.isChecked()
        self.updating_widgets = True

        for i, info in enumerate(JOINT_INFO):
            spin = self.spin_boxes[i]
            if self.is_deg_mode:
                spin.setSuffix(' °')
                spin.setRange(info['deg_min'], info['deg_max'])
                spin.setSingleStep(1.0)
                spin.setValue(math.degrees(self.target_rads[i]))
            else:
                spin.setSuffix(' rad')
                spin.setRange(info['rad_min'], info['rad_max'])
                spin.setSingleStep(0.05)
                spin.setValue(self.target_rads[i])

            self._sync_slider_from_target(i)

        self.updating_widgets = False

    def _sync_slider_from_target(self, idx):
        info = JOINT_INFO[idx]
        min_val = info['rad_min']
        max_val = info['rad_max']
        ratio = (self.target_rads[idx] - min_val) / (max_val - min_val)
        slider_val = int(-1000 + ratio * 2000)
        self.sliders[idx].setValue(slider_val)

    def _on_spin_changed(self, idx, val):
        if self.updating_widgets:
            return
        self.updating_widgets = True
        rad_val = math.radians(val) if self.is_deg_mode else val
        self.target_rads[idx] = rad_val
        self._sync_slider_from_target(idx)
        self.updating_widgets = False

        if self.live_drag:
            self.bridge.send_trajectory(self.target_rads, duration_sec=0.1)

    def _on_slider_changed(self, idx, val):
        if self.updating_widgets:
            return
        self.updating_widgets = True
        ratio = (val + 1000) / 2000.0
        info = JOINT_INFO[idx]
        rad_val = info['rad_min'] + ratio * (info['rad_max'] - info['rad_min'])
        self.target_rads[idx] = rad_val

        spin = self.spin_boxes[idx]
        if self.is_deg_mode:
            spin.setValue(math.degrees(rad_val))
        else:
            spin.setValue(rad_val)
        self.updating_widgets = False

        if self.live_drag:
            self.bridge.send_trajectory(self.target_rads, duration_sec=0.1)

    def apply_preset(self, deg_angles):
        self.updating_widgets = True
        for i in range(len(JOINT_NAMES)):
            rad = math.radians(deg_angles[i])
            self.target_rads[i] = rad
            spin = self.spin_boxes[i]
            if self.is_deg_mode:
                spin.setValue(deg_angles[i])
            else:
                spin.setValue(rad)
            self._sync_slider_from_target(i)
        self.updating_widgets = False

        if not self.live_drag:
            dur = self.sp_duration.value()
            self.bridge.send_trajectory(self.target_rads, duration_sec=dur)

    def send_to_gazebo(self):
        dur = self.sp_duration.value()
        self.bridge.send_trajectory(self.target_rads, duration_sec=dur)

    def read_actual_as_target(self):
        self.updating_widgets = True
        for i in range(len(JOINT_NAMES)):
            rad = self.actual_rads[i]
            self.target_rads[i] = rad
            spin = self.spin_boxes[i]
            if self.is_deg_mode:
                spin.setValue(math.degrees(rad))
            else:
                spin.setValue(rad)
            self._sync_slider_from_target(i)
        self.updating_widgets = False

    def stop_and_hold(self):
        self.read_actual_as_target()
        self.bridge.send_trajectory(self.actual_rads, duration_sec=0.2)

    def _update_telemetry(self):
        # Kiểm tra nếu có lệnh mục tiêu Cartesian gửi qua ROS 2 topic /denso/cartesian_target
        pending_tar = self.bridge.pop_pending_cartesian_target()
        if pending_tar is not None:
            self._set_spin_values_silently(pending_tar[0], pending_tar[1], pending_tar[2])
            self._calc_ik()
        pos, vel, eff, last_time = self.bridge.get_latest_state()
        is_conn = (time.time() - last_time < 1.5) and (last_time > 0)

        if is_conn:
            self.status_badge.setText('🟢 ĐÃ KẾT NỐI GAZEBO')
            self.status_badge.setStyleSheet('background-color: #064e3b; color: #a7f3d0; padding: 5px 14px; border-radius: 12px; font-weight: bold; font-size: 12px;')
        else:
            self.status_badge.setText('🔴 CHỜ KẾT NỐI GAZEBO...')
            self.status_badge.setStyleSheet('background-color: #7f1d1d; color: #fecaca; padding: 5px 14px; border-radius: 12px; font-weight: bold; font-size: 12px;')

        self.actual_rads = pos
        self.actual_efforts = eff

        # Cập nhật telemetry Tab 3 (Joints: Sliders & Smooth Effort)
        alpha_eff = 0.12  # Hệ số lọc thông thấp EMA: làm mượt rung lắc lực trong Gazebo
        for i in range(len(JOINT_NAMES)):
            if self.is_deg_mode:
                cur_deg = math.degrees(pos[i])
                if abs(cur_deg) < 0.05:
                    cur_deg = 0.0
                pos_str = f'Gazebo: {cur_deg:+.2f}°'
            else:
                cur_rad = pos[i]
                if abs(cur_rad) < 0.0005:
                    cur_rad = 0.0
                pos_str = f'Gazebo: {cur_rad:+.3f} rad'

            err = abs(self.target_rads[i] - pos[i])
            color = '#4ade80' if err < 0.02 else '#38bdf8'
            if self.actual_labels[i].text() != pos_str:
                self.actual_labels[i].setText(pos_str)
            if self._prev_act_colors[i] != color:
                self.actual_labels[i].setStyleSheet(f'color: {color}; font-weight: bold; font-size: 12px;')
                self._prev_act_colors[i] = color

            # Lọc thông thấp Exponential Moving Average (EMA) triệt tiêu dao động tần số cao từ ODE solver
            raw_e = eff[i]
            self.filtered_efforts[i] = (1.0 - alpha_eff) * self.filtered_efforts[i] + alpha_eff * raw_e
            smoothed_eff = self.filtered_efforts[i]

            # Deadband khử rung quanh 0: nếu lực < 0.08 Nm, cố định về 0.0 để triệt tiêu hiện tượng "-0.0 Nm"
            if abs(smoothed_eff) < 0.08:
                smoothed_eff = 0.0

            limit = JOINT_INFO[i]['effort_limit']
            eff_pct = min(100.0, (abs(smoothed_eff) / limit) * 100.0)
            eff_color = '#94a3b8' if eff_pct < 60 else '#facc15' if eff_pct < 85 else '#ef4444'

            # Hiển thị số đo sạch sẽ, không bị nhảy giật liên tục
            if smoothed_eff == 0.0:
                eff_text = 'Lực: 0.0 Nm (0%)'
            else:
                eff_text = f'Lực: {smoothed_eff:+.1f} Nm ({eff_pct:.0f}%)'

            if self.effort_labels[i].text() != eff_text:
                self.effort_labels[i].setText(eff_text)
            # Chỉ cập nhật stylesheet khi đổi màu để tránh Qt re-parse CSS 20 lần/giây gây lag
            if self._prev_eff_colors[i] != eff_color:
                self.effort_labels[i].setStyleSheet(f'color: {eff_color}; font-size: 11px;')
                self._prev_eff_colors[i] = eff_color

        self.actual_efforts = list(self.filtered_efforts)

        # Cập nhật telemetry Tab 1 & Tab 2 (Cartesian IK & Table) chỉ khi đang xem Tab 1 hoặc Tab 2
        if hasattr(self, 'tabs') and self.tabs.currentIndex() in [0, 1]:
            self._refresh_ik_table()

        # Chỉ tính toán động lực học ngược RNEA khi đang xem Tab 4 (Telemetry) để tránh ngốn CPU gây lag
        if hasattr(self, 'tabs') and self.tabs.currentIndex() == 3:
            self._refresh_telemetry_table()
        # Cập nhật nét vẽ tức thời trong Gazebo Sim theo chuyển động thực tế
        if hasattr(self, 'tab_gcode') and self.tab_gcode is not None:
            trail_pub = getattr(self.tab_gcode, 'trail_pub', None)
            worker = getattr(self.tab_gcode, 'worker', None)
            is_worker_running = worker is not None and worker.isRunning()
            
            # Khi không chạy luồng G-Code tự động, theo dõi ngòi bút khi người dùng điều khiển thủ công
            if trail_pub is not None and not is_worker_running and is_conn:
                try:
                    tfs, _ = self.engine.get_dh_transforms(pos)
                    # Flange TCP / Stylus Nib
                    tcp = tfs[5][:3, 3]
                    x_m, y_m, z_m = float(tcp[0]), float(tcp[1]), float(tcp[2])
                    
                    # Kiểm tra ngòi bút chạm bàn vẽ (X: -0.56~-0.20m phía sau, Y: -0.18~0.18m, Z <= 0.182m)
                    on_board = (-0.56 <= x_m <= -0.20) and (-0.18 <= y_m <= 0.18)
                    is_touching = on_board and (z_m <= 0.182)

                    if is_touching:
                        trail_pub.add_point(x_m * 1000.0, y_m * 1000.0, z_m * 1000.0, is_draw=True)
                    else:
                        if getattr(self, '_was_manual_touching', False):
                            trail_pub.end_stroke()
                        if hasattr(trail_pub, 'gz_trail') and trail_pub.gz_trail:
                            trail_pub.gz_trail.update_pen_tip(x_m, y_m, z_m, is_draw=False)
                    self._was_manual_touching = is_touching
                except Exception:
                    pass



def main():
    bridge = GazeboRosBridge()
    bridge.start()

    app = QApplication(sys.argv)
    window = DensoGazeboPanel(bridge)
    if "--tab" in sys.argv:
        try:
            idx = sys.argv.index("--tab") + 1
            if idx < len(sys.argv):
                val = sys.argv[idx].lower()
                if val in ["4", "gcode", "draw", "drawing", "vision"]:
                    window.tabs.setCurrentIndex(4)
                elif val in ["0", "link5"]:
                    window.tabs.setCurrentIndex(0)
                elif val in ["1", "kinematics"]:
                    window.tabs.setCurrentIndex(1)
                elif val in ["2", "joints", "sliders"]:
                    window.tabs.setCurrentIndex(2)
                elif val in ["3", "telemetry"]:
                    window.tabs.setCurrentIndex(3)
        except Exception:
            pass
    window.show()
    ret = app.exec_()
    try:
        bridge.stop()
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(ret)

if __name__ == '__main__':
    main()
