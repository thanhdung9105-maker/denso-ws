#!/usr/bin/env python3
"""
DENSO VS-6556 KINEMATICS & DYNAMICS DASHBOARD (ROS 2)
Bảng điều khiển & Giám sát toàn diện:
- Động học thuận (FK) & Động học nghịch (IK)
- Động lực học nghịch (Inverse Dynamics - ID): Tính toán mô-men xoắn torque tau, phân tích thành phần M*qdd, C*qd, g(q), ma sát, tải trọng
- Động lực học thuận (Forward Dynamics - FD): Tính toán gia tốc góc qdd từ torque tau, mô phỏng phản ứng vật lý thời gian thực
- Trực quan hóa 3D trực tiếp trong RViz2 tại các khớp (Torque Arrow Markers & 3D Text Labels)
"""

import sys
import os
import math
import time
import threading
import numpy as np

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QProgressBar, QFrame, QSplitter, QTabWidget, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette

# Import Dynamics Engine
sys.path.append(os.path.dirname(__file__))
from denso_dynamics_engine import (
    DensoDynamicsEngine, TORQUE_LIMITS, JOINT_LIMITS, AXES
)

# -------------------------------------------------------------
# KINEMATICS ENGINE FOR DENSO VS-6556
# -------------------------------------------------------------
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

def rot_x(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def rot_y(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def rot_z(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def trans(x, y, z):
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def forward_kinematics(q):
    """Calculates End-Effector 4x4 Transformation Matrix from 5 joint angles (rad)."""
    q1, q2, q3, q4, q5 = q
    T1 = trans(0, 0, 0.185) @ rot_z(q1)
    T2 = T1 @ trans(-0.075, -0.0545, 0.150) @ rot_y(q2)
    T3 = T2 @ trans(0, 0.0055, 0.270) @ rot_y(q3)
    T4 = T3 @ trans(-0.088, 0.049, 0.090) @ rot_x(q4)
    T5 = T4 @ trans(-0.207, 0, 0) @ rot_y(q5)
    return T5

def get_cartesian_pose(q):
    """Returns X, Y, Z (m), Roll, Pitch, Yaw (deg) from joint angles."""
    T = forward_kinematics(q)
    pos = T[:3, 3]
    R = T[:3, :3]
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    return {
        'x': pos[0],
        'y': pos[1],
        'z': pos[2],
        'roll': np.rad2deg(roll),
        'pitch': np.rad2deg(pitch),
        'yaw': np.rad2deg(yaw)
    }

def solve_inverse_kinematics(target_pos, q_init=None, max_iter=70, tol=1e-3):
    """Damped Least Squares IK solver for target position [x, y, z]."""
    if q_init is None:
        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        q = np.array(q_init, dtype=float)

    damping = 0.015
    step_size = 0.45
    eps = 1e-6
    q_min = np.array([lim[0] for lim in JOINT_LIMITS])
    q_max = np.array([lim[1] for lim in JOINT_LIMITS])

    for _ in range(max_iter):
        T = forward_kinematics(q)
        curr_pos = T[:3, 3]
        err = target_pos - curr_pos
        err_norm = np.linalg.norm(err)
        if err_norm < tol:
            return q, True, err_norm

        J = np.zeros((3, 5))
        for j in range(5):
            q_p = q.copy()
            q_p[j] += eps
            p_p = forward_kinematics(q_p)[:3, 3]
            J[:, j] = (p_p - curr_pos) / eps

        JJt = J @ J.T + (damping**2) * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, err)
        q = q + step_size * dq
        q = np.clip(q, q_min, q_max)

    final_pos = forward_kinematics(q)[:3, 3]
    final_err = np.linalg.norm(target_pos - final_pos)
    return q, (final_err < tol * 2), final_err


# -------------------------------------------------------------
# ROS 2 THREAD & BRIDGE
# -------------------------------------------------------------
class RosBridge(QObject):
    joint_states_received = pyqtSignal(list, list, list)

    def __init__(self):
        super().__init__()
        self.node = None
        self.cmd_pub = None
        self.marker_pub = None
        self.latest_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_efforts = [0.0, 0.0, 0.0, 0.0, 0.0]

    def start_ros(self):
        rclpy.init(args=None)
        self.node = Node('denso_kinematics_gui')
        self.node.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.cmd_pub = self.node.create_publisher(String, '/denso/cmd', 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, '/denso/joint_dynamics_markers', 10)

        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _js_cb(self, msg: JointState):
        name_map = {name: i for i, name in enumerate(JOINT_NAMES)}
        updated = False
        for i, name in enumerate(msg.name):
            if name in name_map and i < len(msg.position):
                idx = name_map[name]
                self.latest_joints[idx] = float(msg.position[i])
                if len(msg.velocity) > i:
                    self.latest_velocities[idx] = float(msg.velocity[i])
                if len(msg.effort) > i:
                    self.latest_efforts[idx] = float(msg.effort[i])
                updated = True
        if updated:
            self.joint_states_received.emit(
                list(self.latest_joints),
                list(self.latest_velocities),
                list(self.latest_efforts)
            )

    def send_cmd(self, cmd_str: str):
        if self.cmd_pub:
            msg = String()
            msg.data = cmd_str
            self.cmd_pub.publish(msg)

    def publish_markers(self, marker_array):
        if self.marker_pub and marker_array is not None:
            self.marker_pub.publish(marker_array)


# -------------------------------------------------------------
# MAIN DASHBOARD GUI
# -------------------------------------------------------------
class DensoKinematicsGUI(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.dynamics_engine = DensoDynamicsEngine()

        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_efforts = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.solved_ik_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Forward dynamics real-time simulation state
        self.sim_active = False
        self.sim_q = np.zeros(5)
        self.sim_qd = np.zeros(5)
        self.sim_tau = np.zeros(5)
        self.dynamics_visible = True

        self.setWindowTitle("DENSO VS-6556 - Bảng Động Học & Động Lực Học RViz2 (ROS 2)")
        self.resize(1200, 840)
        self.init_ui()
        self.apply_dark_theme()

        # Connect ROS signal
        self.bridge.joint_states_received.connect(self.on_joint_states)

        # Refresh timer (10Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer_tick)
        self.timer.start(100)

        # Simulation loop timer (20Hz)
        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(self.on_sim_tick)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 12, 15, 12)
        main_layout.setSpacing(10)

        # --- HEADER ---
        header_layout = QHBoxLayout()
        title_box = QVBoxLayout()
        title_label = QLabel("DENSO VS-6556 KINEMATICS & DYNAMICS DASHBOARD")
        title_label.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title_label.setStyleSheet("color: #00e5ff; letter-spacing: 1px;")
        sub_label = QLabel("Tính toán Động Học (FK/IK) & Động Lực Học Thuận/Nghịch tại 5 khớp với Visual Marker 3D trên RViz2")
        sub_label.setFont(QFont("Segoe UI", 10))
        sub_label.setStyleSheet("color: #a0a0b0;")
        title_box.addWidget(title_label)
        title_box.addWidget(sub_label)
        header_layout.addLayout(title_box)

        header_layout.addStretch()

        self.status_badge = QLabel("🟢 ROS 2 CONNECTED")
        self.status_badge.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.status_badge.setStyleSheet(
            "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
            "border-radius: 6px; padding: 6px 14px;"
        )
        header_layout.addWidget(self.status_badge)

        self.btn_toggle_dynamics = QPushButton("👁️ ĐỘNG LỰC HỌC RVIZ2: ĐANG HIỆN")
        self.btn_toggle_dynamics.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_toggle_dynamics.setStyleSheet(
            "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
            "border-radius: 6px; padding: 6px 14px;"
        )
        self.btn_toggle_dynamics.setToolTip("Bấm để ẨN hoặc HIỆN các vector mô-men xoắn và nhãn chữ 3D tại các khớp trong RViz2")
        self.btn_toggle_dynamics.clicked.connect(self.on_toggle_dynamics)
        header_layout.addWidget(self.btn_toggle_dynamics)

        main_layout.addLayout(header_layout)

        # --- TABS CONTAINER ---
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("Segoe UI", 10, QFont.Bold))

        # Tab 1: Động học (FK & IK)
        tab_kinematics = QWidget()
        kin_layout = QVBoxLayout(tab_kinematics)
        kin_layout.setContentsMargins(5, 10, 5, 5)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.create_fk_panel())
        splitter.addWidget(self.create_ik_panel())
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        kin_layout.addWidget(splitter)
        self.tabs.addTab(tab_kinematics, "📍 1. ĐỘNG HỌC (FK & IK)")

        # Tab 2: Động lực học nghịch (Inverse Dynamics)
        tab_id = QWidget()
        id_layout = QVBoxLayout(tab_id)
        id_layout.setContentsMargins(5, 10, 5, 5)
        id_layout.addWidget(self.create_id_panel())
        self.tabs.addTab(tab_id, "⚡ 2. ĐỘNG LỰC HỌC NGHỊCH (INVERSE DYNAMICS)")

        # Tab 3: Động lực học thuận (Forward Dynamics)
        tab_fd = QWidget()
        fd_layout = QVBoxLayout(tab_fd)
        fd_layout.setContentsMargins(5, 10, 5, 5)
        fd_layout.addWidget(self.create_fd_panel())
        self.tabs.addTab(tab_fd, "🚀 3. ĐỘNG LỰC HỌC THUẬN (FORWARD DYNAMICS)")

        main_layout.addWidget(self.tabs, 1)

        # --- FOOTER / LOG BAR ---
        self.log_label = QLabel("Trạng thái: Đang kết nối tới topic /joint_states và /denso/joint_dynamics_markers...")
        self.log_label.setStyleSheet("color: #00e676; font-size: 11px; padding: 4px;")
        main_layout.addWidget(self.log_label)

    # ---------------------------------------------------------
    # TAB 1: FORWARD KINEMATICS (FK)
    # ---------------------------------------------------------
    def create_fk_panel(self):
        panel = QGroupBox("📍 ĐỘNG HỌC THUẬN (FORWARD KINEMATICS)")
        panel.setFont(QFont("Segoe UI", 11, QFont.Bold))
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        lbl_joints = QLabel("Bảng Góc Khớp Hiện Tại (Joint States):")
        lbl_joints.setStyleSheet("color: #ffb86c; font-weight: bold;")
        layout.addWidget(lbl_joints)

        self.fk_table = QTableWidget(5, 4)
        self.fk_table.setHorizontalHeaderLabels(["Khớp", "Tên Trục", "Góc (Rad)", "Góc (Độ °)"])
        self.fk_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.fk_table.verticalHeader().setVisible(False)
        self.fk_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.fk_table.setFixedHeight(175)

        joint_descs = ["J1: Base (Z)", "J2: Shoulder (Y)", "J3: Elbow (Y)", "J4: Forearm (X)", "J5: Wrist (Y)"]
        for row in range(5):
            self.fk_table.setItem(row, 0, QTableWidgetItem(f"Joint {row+1}"))
            self.fk_table.setItem(row, 1, QTableWidgetItem(joint_descs[row]))
            self.fk_table.setItem(row, 2, QTableWidgetItem("0.0000"))
            self.fk_table.setItem(row, 3, QTableWidgetItem("0.00°"))
            for col in range(4):
                self.fk_table.item(row, col).setTextAlignment(Qt.AlignCenter)
        layout.addWidget(self.fk_table)

        lbl_cart = QLabel("Tọa Độ & Hướng Đầu Gắp (End-Effector / Flange):")
        lbl_cart.setStyleSheet("color: #50fa7b; font-weight: bold;")
        layout.addWidget(lbl_cart)

        cart_box = QGroupBox()
        cart_box.setStyleSheet("background-color: #21222c; border: 1px solid #44475a; border-radius: 8px;")
        grid = QGridLayout(cart_box)
        grid.setContentsMargins(12, 10, 12, 10)
        grid.setHorizontalSpacing(15)
        grid.setVerticalSpacing(8)

        self.val_x = QLabel("-370.0 mm (-0.370 m)")
        self.val_y = QLabel("0.0 mm (0.000 m)")
        self.val_z = QLabel("695.0 mm (0.695 m)")
        self.val_r = QLabel("0.0°")
        self.val_p = QLabel("0.0°")
        self.val_yaw = QLabel("0.0°")
        self.val_dist = QLabel("787.5 mm")

        row = 0
        grid.addWidget(QLabel("Tọa độ X:"), row, 0)
        grid.addWidget(self.val_x, row, 1)
        grid.addWidget(QLabel("Góc Roll (X):"), row, 2)
        grid.addWidget(self.val_r, row, 3)

        row += 1
        grid.addWidget(QLabel("Tọa độ Y:"), row, 0)
        grid.addWidget(self.val_y, row, 1)
        grid.addWidget(QLabel("Góc Pitch (Y):"), row, 2)
        grid.addWidget(self.val_p, row, 3)

        row += 1
        grid.addWidget(QLabel("Tọa độ Z:"), row, 0)
        grid.addWidget(self.val_z, row, 1)
        grid.addWidget(QLabel("Góc Yaw (Z):"), row, 2)
        grid.addWidget(self.val_yaw, row, 3)

        row += 1
        grid.addWidget(QLabel("Bán kính gốc R:"), row, 0)
        grid.addWidget(self.val_dist, row, 1)

        for w in [self.val_x, self.val_y, self.val_z, self.val_r, self.val_p, self.val_yaw, self.val_dist]:
            w.setStyleSheet("color: #00e5ff; font-weight: bold; font-family: monospace; font-size: 12px;")

        layout.addWidget(cart_box)

        lbl_jog = QLabel("Kéo thử góc khớp (FK Jogging Preview):")
        lbl_jog.setStyleSheet("color: #bd93f9; font-weight: bold;")
        layout.addWidget(lbl_jog)

        self.jog_sliders = []
        for i in range(5):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"J{i+1}:"))
            sl = QSlider(Qt.Horizontal)
            min_deg = int(np.rad2deg(JOINT_LIMITS[i][0]))
            max_deg = int(np.rad2deg(JOINT_LIMITS[i][1]))
            sl.setRange(min_deg, max_deg)
            sl.setValue(0)
            sl.valueChanged.connect(self.on_jog_slider_changed)
            h.addWidget(sl)
            val_lbl = QLabel("0°")
            val_lbl.setFixedWidth(40)
            val_lbl.setStyleSheet("color: #f1fa8c; font-family: monospace;")
            h.addWidget(val_lbl)
            self.jog_sliders.append((sl, val_lbl))
            layout.addLayout(h)

        layout.addStretch()
        return panel

    # ---------------------------------------------------------
    # TAB 1: INVERSE KINEMATICS (IK)
    # ---------------------------------------------------------
    def create_ik_panel(self):
        panel = QGroupBox("🎯 ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS)")
        panel.setFont(QFont("Segoe UI", 11, QFont.Bold))
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        lbl_target = QLabel("Nhập Tọa Độ Mục Tiêu Đầu Gắp (Target Pose):")
        lbl_target.setStyleSheet("color: #ff79c6; font-weight: bold;")
        layout.addWidget(lbl_target)

        inp_box = QGroupBox()
        inp_box.setStyleSheet("background-color: #21222c; border: 1px solid #44475a; border-radius: 8px;")
        grid = QGridLayout(inp_box)
        grid.setContentsMargins(12, 10, 12, 10)
        grid.setHorizontalSpacing(15)

        grid.addWidget(QLabel("Target X (mm):"), 0, 0)
        self.sp_x = QDoubleSpinBox()
        self.sp_x.setRange(-700.0, 700.0)
        self.sp_x.setValue(-370.0)
        self.sp_x.setSingleStep(10.0)
        grid.addWidget(self.sp_x, 0, 1)

        grid.addWidget(QLabel("Target Y (mm):"), 1, 0)
        self.sp_y = QDoubleSpinBox()
        self.sp_y.setRange(-700.0, 700.0)
        self.sp_y.setValue(0.0)
        self.sp_y.setSingleStep(10.0)
        grid.addWidget(self.sp_y, 1, 1)

        grid.addWidget(QLabel("Target Z (mm):"), 2, 0)
        self.sp_z = QDoubleSpinBox()
        self.sp_z.setRange(0.0, 950.0)
        self.sp_z.setValue(695.0)
        self.sp_z.setSingleStep(10.0)
        grid.addWidget(self.sp_z, 2, 1)

        btn_copy = QPushButton("📌 Lấy tọa độ hiện tại")
        btn_copy.setStyleSheet("background-color: #44475a; color: white; padding: 5px;")
        btn_copy.clicked.connect(self.copy_current_to_target)
        grid.addWidget(btn_copy, 0, 2, 3, 1)

        layout.addWidget(inp_box)

        self.btn_solve = QPushButton("⚙️ GIẢI ĐỘNG HỌC NGHỊCH (SOLVE IK)")
        self.btn_solve.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_solve.setStyleSheet(
            "background-color: #00b4d8; color: #03045e; padding: 10px; border-radius: 6px; font-weight: bold;"
        )
        self.btn_solve.clicked.connect(self.on_solve_ik)
        layout.addWidget(self.btn_solve)

        lbl_res = QLabel("Nghiệm Góc Khớp Động Học Nghịch (IK Solution):")
        lbl_res.setStyleSheet("color: #ffb86c; font-weight: bold;")
        layout.addWidget(lbl_res)

        self.ik_table = QTableWidget(5, 3)
        self.ik_table.setHorizontalHeaderLabels(["Khớp", "Nghiệm (Rad)", "Nghiệm (Độ °)"])
        self.ik_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ik_table.verticalHeader().setVisible(False)
        self.ik_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.ik_table.setFixedHeight(175)

        for row in range(5):
            self.ik_table.setItem(row, 0, QTableWidgetItem(f"Joint {row+1}"))
            self.ik_table.setItem(row, 1, QTableWidgetItem("-"))
            self.ik_table.setItem(row, 2, QTableWidgetItem("-"))
            for col in range(3):
                self.ik_table.item(row, col).setTextAlignment(Qt.AlignCenter)
        layout.addWidget(self.ik_table)

        self.ik_status_lbl = QLabel("Sai số hội tụ: Chưa giải")
        self.ik_status_lbl.setStyleSheet("color: #8be9fd; font-style: italic;")
        layout.addWidget(self.ik_status_lbl)

        self.btn_send = QPushButton("🚀 GỬI TỚI ROBOT TRONG RVIZ2")
        self.btn_send.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_send.setStyleSheet(
            "background-color: #50fa7b; color: #1e1e24; padding: 10px; border-radius: 6px; font-weight: bold;"
        )
        self.btn_send.setEnabled(False)
        self.btn_send.clicked.connect(self.on_send_to_robot)
        layout.addWidget(self.btn_send)

        layout.addStretch()
        return panel

    # ---------------------------------------------------------
    # TAB 2: INVERSE DYNAMICS (ID)
    # ---------------------------------------------------------
    def create_id_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setSpacing(15)

        # Left Column: Inputs & Commands
        left_box = QGroupBox("📥 THÔNG SỐ ĐẦU VÀO ĐỘNG LỰC HỌC NGHỊCH")
        left_layout = QVBoxLayout(left_box)

        # Joint Inputs Table (q, qd, qdd)
        lbl_q = QLabel("Vị trí (q), Vận tốc (q̇) và Gia tốc (q̈) tại 5 khớp:")
        lbl_q.setStyleSheet("color: #ffb86c; font-weight: bold;")
        left_layout.addWidget(lbl_q)

        self.id_input_table = QTableWidget(5, 4)
        self.id_input_table.setHorizontalHeaderLabels(["Khớp", "q (độ)", "q̇ (rad/s)", "q̈ (rad/s²)"])
        self.id_input_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.id_input_table.verticalHeader().setVisible(False)
        self.id_input_table.setFixedHeight(175)

        for i in range(5):
            self.id_input_table.setItem(i, 0, QTableWidgetItem(f"Joint {i+1}"))
            self.id_input_table.item(i, 0).setFlags(Qt.ItemIsEnabled)
            self.id_input_table.setItem(i, 1, QTableWidgetItem("0.0"))
            self.id_input_table.setItem(i, 2, QTableWidgetItem("0.0"))
            self.id_input_table.setItem(i, 3, QTableWidgetItem("0.0"))
            for c in range(4):
                self.id_input_table.item(i, c).setTextAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.id_input_table)

        # Payload & External Force Box
        load_box = QGroupBox("Tải Trọng Đầu Gắp (Payload) & Ngoại Lực:")
        load_grid = QGridLayout(load_box)
        load_grid.addWidget(QLabel("Khối lượng tải (kg):"), 0, 0)
        self.sp_payload = QDoubleSpinBox()
        self.sp_payload.setRange(0.0, 7.0)
        self.sp_payload.setValue(0.0)
        self.sp_payload.setSingleStep(0.5)
        load_grid.addWidget(self.sp_payload, 0, 1)

        load_grid.addWidget(QLabel("Lực Fz (N):"), 0, 2)
        self.sp_fz = QDoubleSpinBox()
        self.sp_fz.setRange(-100.0, 100.0)
        self.sp_fz.setValue(0.0)
        load_grid.addWidget(self.sp_fz, 0, 3)
        left_layout.addWidget(load_box)

        # Quick Actions
        btn_sync = QPushButton("📌 Lấy góc & vận tốc hiện tại từ Robot")
        btn_sync.setStyleSheet("background-color: #44475a; color: white; padding: 6px;")
        btn_sync.clicked.connect(self.on_id_sync_robot)
        left_layout.addWidget(btn_sync)

        btn_grav_comp = QPushButton("⚖️ Cân Bằng Trọng Lực Tĩnh (q̇=0, q̈=0)")
        btn_grav_comp.setStyleSheet("background-color: #6272a4; color: white; padding: 6px;")
        btn_grav_comp.clicked.connect(self.on_id_gravity_comp)
        left_layout.addWidget(btn_grav_comp)

        self.btn_calc_id = QPushButton("⚡ TÍNH TOÁN MÔ-MEN XOẮN (INVERSE DYNAMICS)")
        self.btn_calc_id.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_calc_id.setStyleSheet("background-color: #ff79c6; color: #1e1e24; padding: 10px; border-radius: 6px;")
        self.btn_calc_id.clicked.connect(self.on_compute_id)
        left_layout.addWidget(self.btn_calc_id)

        self.chk_auto_marker = QCheckBox("🔴 Phát Visual Marker 3D liên tục lên RViz2")
        self.chk_auto_marker.setChecked(True)
        self.chk_auto_marker.setStyleSheet("color: #50fa7b; font-weight: bold; margin-top: 5px;")
        left_layout.addWidget(self.chk_auto_marker)

        left_layout.addStretch()
        layout.addWidget(left_box, 1)

        # Right Column: Torque Outputs & Breakdown
        right_box = QGroupBox("📊 KẾT QUẢ PHÂN RÃ MÔ-MEN XOẮN (TORQUE BREAKDOWN)")
        right_layout = QVBoxLayout(right_box)

        self.id_res_table = QTableWidget(5, 7)
        self.id_res_table.setHorizontalHeaderLabels([
            "Khớp", "Quán tính (M·q̈)", "Coriolis (C·q̇)", "Trọng lực (g)", "Tải trọng", "Tổng τ (N·m)", "% Tải"
        ])
        self.id_res_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.id_res_table.verticalHeader().setVisible(False)
        self.id_res_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.id_res_table.setFixedHeight(180)

        for i in range(5):
            for c in range(7):
                item = QTableWidgetItem("-" if c > 0 else f"Joint {i+1}")
                item.setTextAlignment(Qt.AlignCenter)
                self.id_res_table.setItem(i, c, item)
        right_layout.addWidget(self.id_res_table)

        # Progress bars for torque load
        lbl_bars = QLabel("Mức Độ Tải Trọng Động Cơ So Với Giới Hạn Tối Đa (Motor Load %):")
        lbl_bars.setStyleSheet("color: #8be9fd; font-weight: bold; margin-top: 5px;")
        right_layout.addWidget(lbl_bars)

        self.load_bars = []
        bar_box = QGroupBox()
        bar_grid = QGridLayout(bar_box)
        for i in range(5):
            bar_grid.addWidget(QLabel(f"J{i+1} (Max {TORQUE_LIMITS[i]:.0f}Nm):"), i, 0)
            pb = QProgressBar()
            pb.setRange(0, 100)
            pb.setValue(0)
            pb.setStyleSheet("""
                QProgressBar {
                    border: 1px solid #44475a;
                    border-radius: 4px;
                    text-align: center;
                    background-color: #282a36;
                    color: white;
                    font-weight: bold;
                }
                QProgressBar::chunk {
                    background-color: #50fa7b;
                    border-radius: 3px;
                }
            """)
            bar_grid.addWidget(pb, i, 1)
            self.load_bars.append(pb)
        right_layout.addWidget(bar_box)

        self.id_status_msg = QLabel("Trạng thái tải: An toàn (Tất cả các khớp hoạt động bình thường)")
        self.id_status_msg.setStyleSheet("color: #50fa7b; font-weight: bold; font-size: 12px;")
        right_layout.addWidget(self.id_status_msg)

        right_layout.addStretch()
        layout.addWidget(right_box, 1)

        return panel

    # ---------------------------------------------------------
    # TAB 3: FORWARD DYNAMICS (FD)
    # ---------------------------------------------------------
    def create_fd_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setSpacing(15)

        # Left Column: Input Joint Torques
        left_box = QGroupBox("📥 NHẬP MÔ-MEN XOẮN ĐỘNG LỰC HỌC THUẬN (TORQUE INPUT)")
        left_layout = QVBoxLayout(left_box)

        lbl_desc = QLabel("Điều chỉnh mô-men xoắn τ (N·m) áp vào từng trục động cơ:")
        lbl_desc.setStyleSheet("color: #bd93f9; font-weight: bold;")
        left_layout.addWidget(lbl_desc)

        self.fd_sliders = []
        self.fd_spinboxes = []
        for i in range(5):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"τ{i+1}:"))
            sl = QSlider(Qt.Horizontal)
            t_max = int(TORQUE_LIMITS[i])
            sl.setRange(-t_max, t_max)
            sl.setValue(0)

            sp = QDoubleSpinBox()
            sp.setRange(-float(t_max), float(t_max))
            sp.setValue(0.0)
            sp.setSingleStep(1.0)
            sp.setFixedWidth(80)

            # Link slider and spinbox
            sl.valueChanged.connect(lambda v, s=sp: s.setValue(float(v)))
            sp.valueChanged.connect(lambda v, s=sl: s.setValue(int(v)))

            h.addWidget(sl)
            h.addWidget(sp)
            self.fd_sliders.append(sl)
            self.fd_spinboxes.append(sp)
            left_layout.addLayout(h)

        h_btns = QHBoxLayout()
        btn_zero = QPushButton("🔄 Đặt về 0 N·m")
        btn_zero.setStyleSheet("background-color: #44475a; color: white; padding: 6px;")
        btn_zero.clicked.connect(self.on_fd_zero_torques)
        h_btns.addWidget(btn_zero)

        btn_hold_g = QPushButton("⚖️ Đặt Mô-men Cân Bằng Trọng Lực")
        btn_hold_g.setStyleSheet("background-color: #6272a4; color: white; padding: 6px;")
        btn_hold_g.clicked.connect(self.on_fd_set_gravity_torques)
        h_btns.addWidget(btn_hold_g)
        left_layout.addLayout(h_btns)

        self.btn_calc_fd = QPushButton("⚙️ TÍNH TOÁN GIA TỐC GÓC (FORWARD DYNAMICS)")
        self.btn_calc_fd.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_calc_fd.setStyleSheet("background-color: #00e5ff; color: #1e1e24; padding: 10px; border-radius: 6px;")
        self.btn_calc_fd.clicked.connect(self.on_compute_fd)
        left_layout.addWidget(self.btn_calc_fd)

        # Real-time Physics Simulation Control Box
        sim_box = QGroupBox("🎮 Mô Phỏng Vật Lý Tương Tác Thời Gian Thực Trên RViz2:")
        sim_box.setStyleSheet("background-color: #21222c; border: 1px solid #ff79c6; border-radius: 8px;")
        sim_vbox = QVBoxLayout(sim_box)

        lbl_sim_desc = QLabel(
            "Khi bật mô phỏng, cánh tay robot trong RViz2 sẽ chuyển động tự do theo đúng lực mô men xoắn áp vào từ thanh trượt!"
        )
        lbl_sim_desc.setStyleSheet("color: #f1fa8c; font-size: 11px;")
        lbl_sim_desc.setWordWrap(True)
        sim_vbox.addWidget(lbl_sim_desc)

        h_sim_btns = QHBoxLayout()
        self.btn_start_sim = QPushButton("▶️ BẮT ĐẦU MÔ PHỎNG VẬT LÝ RVIZ2")
        self.btn_start_sim.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_start_sim.setStyleSheet("background-color: #50fa7b; color: #1e1e24; padding: 8px; border-radius: 4px;")
        self.btn_start_sim.clicked.connect(self.on_start_simulation)
        h_sim_btns.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton("⏹ DỪNG MÔ PHỎNG")
        self.btn_stop_sim.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_stop_sim.setStyleSheet("background-color: #ff5555; color: white; padding: 8px; border-radius: 4px;")
        self.btn_stop_sim.setEnabled(False)
        self.btn_stop_sim.clicked.connect(self.on_stop_simulation)
        h_sim_btns.addWidget(self.btn_stop_sim)
        sim_vbox.addLayout(h_sim_btns)

        left_layout.addWidget(sim_box)
        left_layout.addStretch()
        layout.addWidget(left_box, 1)

        # Right Column: Output Acceleration & Mass Matrix
        right_box = QGroupBox("📊 GIA TỐC KHỚP VÀ MA TRẬN QUÁN TÍNH M(q)")
        right_layout = QVBoxLayout(right_box)

        lbl_acc = QLabel("Gia Tốc Góc Khớp Thu Được (q̈ = M⁻¹(τ - C·q̇ - g)):")
        lbl_acc.setStyleSheet("color: #50fa7b; font-weight: bold;")
        right_layout.addWidget(lbl_acc)

        self.fd_res_table = QTableWidget(5, 4)
        self.fd_res_table.setHorizontalHeaderLabels(["Khớp", "Gia tốc q̈ (rad/s²)", "Gia tốc q̈ (độ/s²)", "Hướng gia tốc"])
        self.fd_res_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.fd_res_table.verticalHeader().setVisible(False)
        self.fd_res_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.fd_res_table.setFixedHeight(175)

        for i in range(5):
            self.fd_res_table.setItem(i, 0, QTableWidgetItem(f"Joint {i+1}"))
            self.fd_res_table.setItem(i, 1, QTableWidgetItem("0.000"))
            self.fd_res_table.setItem(i, 2, QTableWidgetItem("0.00°/s²"))
            self.fd_res_table.setItem(i, 3, QTableWidgetItem("Đứng yên"))
            for c in range(4):
                self.fd_res_table.item(i, c).setTextAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.fd_res_table)

        lbl_mat = QLabel("Ma Trận Khối Lượng Quán Tính Đối Xứng M(q) (5x5 kg·m²):")
        lbl_mat.setStyleSheet("color: #8be9fd; font-weight: bold; margin-top: 8px;")
        right_layout.addWidget(lbl_mat)

        self.mass_table = QTableWidget(5, 5)
        self.mass_table.setHorizontalHeaderLabels(["J1", "J2", "J3", "J4", "J5"])
        self.mass_table.setVerticalHeaderLabels(["J1", "J2", "J3", "J4", "J5"])
        self.mass_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.mass_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.mass_table.setFixedHeight(160)

        for r in range(5):
            for c in range(5):
                item = QTableWidgetItem("0.000")
                item.setTextAlignment(Qt.AlignCenter)
                self.mass_table.setItem(r, c, item)
        right_layout.addWidget(self.mass_table)

        right_layout.addStretch()
        layout.addWidget(right_box, 1)

        return panel

    # ---------------------------------------------------------
    # DYNAMICS CALCULATION LOGIC
    # ---------------------------------------------------------
    def on_toggle_dynamics(self):
        """Toggles visibility of dynamics torque arrows and 3D labels in RViz2."""
        self.dynamics_visible = not self.dynamics_visible
        if self.dynamics_visible:
            self.bridge.send_cmd("dynamics_on")
            self.btn_toggle_dynamics.setText("👁️ ĐỘNG LỰC HỌC RVIZ2: ĐANG HIỆN")
            self.btn_toggle_dynamics.setStyleSheet(
                "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
                "border-radius: 6px; padding: 6px 14px;"
            )
            self.log_label.setText("👁️ Đã HIỆN các vector mô-men xoắn và nhãn 3D động lực học trong RViz2.")
        else:
            self.bridge.send_cmd("dynamics_off")
            # Clear markers immediately
            from visualization_msgs.msg import Marker, MarkerArray
            del_array = MarkerArray()
            m = Marker()
            m.action = Marker.DELETEALL
            del_array.markers.append(m)
            self.bridge.publish_markers(del_array)

            self.btn_toggle_dynamics.setText("🙈 ĐỘNG LỰC HỌC RVIZ2: ĐÃ ẨN")
            self.btn_toggle_dynamics.setStyleSheet(
                "background-color: #3d1a24; color: #ff5555; border: 1px solid #ff5555; "
                "border-radius: 6px; padding: 6px 14px;"
            )
            self.log_label.setText("🙈 Đã ẨN toàn bộ vector mô-men xoắn và nhãn 3D động lực học trong RViz2.")

    def on_id_sync_robot(self):
        """Copies real robot positions and velocities into ID input table."""
        for i in range(5):
            self.id_input_table.item(i, 1).setText(f"{np.rad2deg(self.current_joints[i]):.2f}")
            self.id_input_table.item(i, 2).setText(f"{self.current_velocities[i]:.3f}")
            self.id_input_table.item(i, 3).setText("0.00")
        self.log_label.setText("📌 Đã đồng bộ vị trí và vận tốc hiện tại từ Robot vào Động Lực Học Nghịch.")

    def on_id_gravity_comp(self):
        """Sets qd=0, qdd=0 to calculate pure gravity holding torques."""
        for i in range(5):
            self.id_input_table.item(i, 2).setText("0.00")
            self.id_input_table.item(i, 3).setText("0.00")
        self.on_compute_id()
        self.log_label.setText("⚖️ Đã tính toán mô-men bù trọng lực tĩnh (Gravity Compensation).")

    def on_compute_id(self):
        """Calculates Inverse Dynamics and updates results table and RViz2 markers."""
        try:
            q = np.array([np.deg2rad(float(self.id_input_table.item(i, 1).text())) for i in range(5)])
            qd = np.array([float(self.id_input_table.item(i, 2).text()) for i in range(5)])
            qdd = np.array([float(self.id_input_table.item(i, 3).text()) for i in range(5)])
        except ValueError:
            self.log_label.setText("❌ Lỗi định dạng số trong bảng nhập thông số khớp!")
            return

        payload = self.sp_payload.value()
        fz = self.sp_fz.value()
        f_ext = np.array([0.0, 0.0, fz]) if abs(fz) > 1e-3 else None

        res = self.dynamics_engine.inverse_dynamics(q, qd, qdd, payload_mass=payload, f_ext=f_ext)

        for i in range(5):
            self.id_res_table.item(i, 1).setText(f"{res['M_qdd'][i]:+.2f}")
            self.id_res_table.item(i, 2).setText(f"{res['C_qd'][i]:+.2f}")
            self.id_res_table.item(i, 3).setText(f"{res['g'][i]:+.2f}")
            self.id_res_table.item(i, 4).setText(f"{res['payload'][i]:+.2f}")
            self.id_res_table.item(i, 5).setText(f"{res['tau'][i]:+.2f}")
            pct = res['percent_load'][i]
            self.id_res_table.item(i, 6).setText(f"{pct:.1f}%")

            # Update progress bar
            pb = self.load_bars[i]
            val = int(min(100, pct))
            pb.setValue(val)
            if pct < 50:
                pb_color = "#50fa7b"  # Green
            elif pct < 80:
                pb_color = "#ffb86c"  # Yellow
            else:
                pb_color = "#ff5555"  # Red
            pb.setStyleSheet(f"""
                QProgressBar {{
                    border: 1px solid #44475a; border-radius: 4px; text-align: center;
                    background-color: #282a36; color: white; font-weight: bold;
                }}
                QProgressBar::chunk {{ background-color: {pb_color}; border-radius: 3px; }}
            """)

        if res['is_overload']:
            self.id_status_msg.setText("⚠️ CẢNH BÁO QUÁ TẢI: Có khớp vượt quá 100% mô-men xoắn định mức!")
            self.id_status_msg.setStyleSheet("color: #ff5555; font-weight: bold; font-size: 12px;")
        else:
            self.id_status_msg.setText("✅ Trạng thái an toàn: Tất cả các khớp hoạt động trong ngưỡng cho phép.")
            self.id_status_msg.setStyleSheet("color: #50fa7b; font-weight: bold; font-size: 12px;")

        # Publish visual markers to RViz2 if checked
        if self.chk_auto_marker.isChecked():
            markers = self.dynamics_engine.build_marker_array(q, res['tau'], qdd, frame_id="world")
            self.bridge.publish_markers(markers)

        self.log_label.setText(f"⚡ Đã tính xong Động Lực Học Nghịch. Tổng Torque = {np.round(res['tau'], 2)} N·m")

    def on_fd_zero_torques(self):
        for sp in self.fd_spinboxes:
            sp.setValue(0.0)

    def on_fd_set_gravity_torques(self):
        """Computes holding torques and populates FD torque spinboxes."""
        g_tau = self.dynamics_engine.gravity_compensation(self.current_joints)
        for i in range(5):
            val = float(g_tau[i])
            limit = float(TORQUE_LIMITS[i])
            val_clipped = max(-limit, min(limit, val))
            self.fd_spinboxes[i].setValue(val_clipped)
        self.log_label.setText("⚖️ Đã thiết lập các thanh trượt mô-men xoắn bằng đúng giá trị bù trọng lực.")

    def on_compute_fd(self):
        """Calculates Forward Dynamics: computes qdd from current q, qd and input tau."""
        tau = np.array([sp.value() for sp in self.fd_spinboxes])
        q = np.array(self.current_joints)
        qd = np.array(self.current_velocities)

        M = self.dynamics_engine.compute_mass_matrix(q)
        qdd = self.dynamics_engine.forward_dynamics(q, qd, tau)

        # Update Acceleration Table
        for i in range(5):
            rad_s2 = qdd[i]
            deg_s2 = np.rad2deg(rad_s2)
            self.fd_res_table.item(i, 1).setText(f"{rad_s2:+.3f}")
            self.fd_res_table.item(i, 2).setText(f"{deg_s2:+.2f}°/s²")
            if abs(rad_s2) < 0.05:
                direction = "Cân bằng"
            elif rad_s2 > 0:
                direction = "Gia tốc dương (+)"
            else:
                direction = "Gia tốc âm (-)"
            self.fd_res_table.item(i, 3).setText(direction)

        # Update Mass Matrix Table
        for r in range(5):
            for c in range(5):
                self.mass_table.item(r, c).setText(f"{M[r, c]:.4f}")

        # Update RViz2 Markers
        markers = self.dynamics_engine.build_marker_array(q, tau, qdd, frame_id="world")
        self.bridge.publish_markers(markers)

        self.log_label.setText(f"⚙️ Đã giải xong Động Lực Học Thuận: q̈ = {np.round(qdd, 3)} rad/s²")

    # ---------------------------------------------------------
    # REAL-TIME PHYSICS SIMULATION IN RVIZ2
    # ---------------------------------------------------------
    def on_start_simulation(self):
        self.sim_active = True
        self.sim_q = np.array(self.current_joints, dtype=float)
        self.sim_qd = np.zeros(5)
        self.btn_start_sim.setEnabled(False)
        self.btn_stop_sim.setEnabled(True)
        self.sim_timer.start(50)  # 20Hz
        self.log_label.setText("▶️ Bắt đầu mô phỏng tương tác thời gian thực: Hãy kéo thanh trượt mô-men xoắn!")

    def on_stop_simulation(self):
        self.sim_active = False
        self.sim_timer.stop()
        self.btn_start_sim.setEnabled(True)
        self.btn_stop_sim.setEnabled(False)
        self.log_label.setText("⏹ Đã dừng mô phỏng tương tác thời gian thực.")

    def on_sim_tick(self):
        if not self.sim_active:
            return

        dt = 0.05
        tau = np.array([sp.value() for sp in self.fd_spinboxes])

        self.sim_q, self.sim_qd, qdd = self.dynamics_engine.integrate_step(
            self.sim_q, self.sim_qd, tau, dt=dt
        )

        # Send target positions to controller so robot moves in RViz2
        cmd = f"goto {self.sim_q[0]:.4f} {self.sim_q[1]:.4f} {self.sim_q[2]:.4f} {self.sim_q[3]:.4f} {self.sim_q[4]:.4f}"
        self.bridge.send_cmd(cmd)

        # Update acceleration table
        for i in range(5):
            self.fd_res_table.item(i, 1).setText(f"{qdd[i]:+.3f}")
            self.fd_res_table.item(i, 2).setText(f"{np.rad2deg(qdd[i]):+.2f}°/s²")

        # Publish visual markers to RViz2
        markers = self.dynamics_engine.build_marker_array(self.sim_q, tau, qdd, frame_id="world")
        self.bridge.publish_markers(markers)

    # ---------------------------------------------------------
    # TIMER & CALLBACKS
    # ---------------------------------------------------------
    def on_joint_states(self, positions, velocities, efforts):
        self.current_joints = list(positions)
        self.current_velocities = list(velocities)
        self.current_efforts = list(efforts)

    def on_timer_tick(self):
        self.update_fk_display()

        # If on ID tab and auto-sync is enabled, update live
        if self.tabs.currentIndex() == 1 and not self.sim_active:
            pass

    def update_fk_display(self):
        q = self.current_joints
        for i in range(5):
            self.fk_table.item(i, 2).setText(f"{q[i]:.4f}")
            self.fk_table.item(i, 3).setText(f"{np.rad2deg(q[i]):.1f}°")

        pose = get_cartesian_pose(q)
        x_mm = pose['x'] * 1000.0
        y_mm = pose['y'] * 1000.0
        z_mm = pose['z'] * 1000.0
        dist_mm = np.sqrt(x_mm**2 + y_mm**2 + z_mm**2)

        self.val_x.setText(f"{x_mm:.1f} mm ({pose['x']:.3f} m)")
        self.val_y.setText(f"{y_mm:.1f} mm ({pose['y']:.3f} m)")
        self.val_z.setText(f"{z_mm:.1f} mm ({pose['z']:.3f} m)")
        self.val_r.setText(f"{pose['roll']:.1f}°")
        self.val_p.setText(f"{pose['pitch']:.1f}°")
        self.val_yaw.setText(f"{pose['yaw']:.1f}°")
        self.val_dist.setText(f"{dist_mm:.1f} mm")

    def on_jog_slider_changed(self):
        q_jog = []
        for i, (sl, lbl) in enumerate(self.jog_sliders):
            deg = sl.value()
            lbl.setText(f"{deg}°")
            q_jog.append(np.deg2rad(deg))

        pose = get_cartesian_pose(q_jog)
        x_mm = pose['x'] * 1000.0
        y_mm = pose['y'] * 1000.0
        z_mm = pose['z'] * 1000.0
        dist_mm = np.sqrt(x_mm**2 + y_mm**2 + z_mm**2)

        self.val_x.setText(f"{x_mm:.1f} mm [JOG]")
        self.val_y.setText(f"{y_mm:.1f} mm [JOG]")
        self.val_z.setText(f"{z_mm:.1f} mm [JOG]")
        self.val_r.setText(f"{pose['roll']:.1f}°")
        self.val_p.setText(f"{pose['pitch']:.1f}°")
        self.val_yaw.setText(f"{pose['yaw']:.1f}°")
        self.val_dist.setText(f"{dist_mm:.1f} mm")

    def copy_current_to_target(self):
        pose = get_cartesian_pose(self.current_joints)
        self.sp_x.setValue(pose['x'] * 1000.0)
        self.sp_y.setValue(pose['y'] * 1000.0)
        self.sp_z.setValue(pose['z'] * 1000.0)
        self.log_label.setText("📌 Đã lấy tọa độ hiện tại làm mục tiêu giải IK.")

    def on_solve_ik(self):
        target_pos = np.array([
            self.sp_x.value() / 1000.0,
            self.sp_y.value() / 1000.0,
            self.sp_z.value() / 1000.0
        ])

        t0 = time.perf_counter()
        q_sol, success, err = solve_inverse_kinematics(target_pos, q_init=self.current_joints)
        calc_time = (time.perf_counter() - t0) * 1000.0

        if success:
            self.solved_ik_joints = list(q_sol)
            for i in range(5):
                self.ik_table.item(i, 1).setText(f"{q_sol[i]:.4f}")
                self.ik_table.item(i, 2).setText(f"{np.rad2deg(q_sol[i]):.1f}°")

            err_mm = err * 1000.0
            self.ik_status_lbl.setText(f"✅ Hội tụ thành công trong {calc_time:.2f}ms! Sai số vị trí: {err_mm:.2f} mm")
            self.ik_status_lbl.setStyleSheet("color: #50fa7b; font-weight: bold;")
            self.btn_send.setEnabled(True)
            self.log_label.setText(f"✅ Giải IK thành công cho tọa độ {np.round(target_pos*1000, 1)} mm.")
        else:
            self.ik_status_lbl.setText(f"❌ Không hội tụ tới sai số mong muốn ({err*1000.0:.1f} mm). Ngoài tầm với!")
            self.ik_status_lbl.setStyleSheet("color: #ff5555; font-weight: bold;")
            self.btn_send.setEnabled(False)
            self.log_label.setText("❌ Không tìm thấy nghiệm IK phù hợp.")

    def on_send_to_robot(self):
        q = self.solved_ik_joints
        cmd = f"goto {q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f} {q[4]:.4f}"
        self.bridge.send_cmd(cmd)
        self.log_label.setText(f"🚀 Đã gửi lệnh chuyển động tới Robot: {cmd}")

    def apply_dark_theme(self):
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor("#1e1e24"))
        palette.setColor(QPalette.WindowText, QColor("#f8f8f2"))
        palette.setColor(QPalette.Base, QColor("#282a36"))
        palette.setColor(QPalette.AlternateBase, QColor("#1e1e24"))
        palette.setColor(QPalette.ToolTipBase, QColor("#f8f8f2"))
        palette.setColor(QPalette.ToolTipText, QColor("#f8f8f2"))
        palette.setColor(QPalette.Text, QColor("#f8f8f2"))
        palette.setColor(QPalette.Button, QColor("#44475a"))
        palette.setColor(QPalette.ButtonText, QColor("#f8f8f2"))
        palette.setColor(QPalette.BrightText, QColor("#ff5555"))
        palette.setColor(QPalette.Highlight, QColor("#bd93f9"))
        palette.setColor(QPalette.HighlightedText, QColor("#282a36"))
        self.setPalette(palette)

        self.setStyleSheet("""
            QTabBar::tab {
                background: #282a36;
                color: #a0a0b0;
                padding: 8px 18px;
                border: 1px solid #44475a;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 3px;
            }
            QTabBar::tab:selected {
                background: #44475a;
                color: #00e5ff;
                font-weight: bold;
                border-bottom: 2px solid #00e5ff;
            }
            QTabWidget::pane {
                border: 1px solid #44475a;
                border-radius: 6px;
                top: -1px;
                background-color: #1e1e24;
            }
            QGroupBox {
                border: 1px solid #44475a;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
                color: #00e5ff;
            }
            QTableWidget {
                background-color: #21222c;
                gridline-color: #44475a;
                border: 1px solid #44475a;
                border-radius: 4px;
                color: #f8f8f2;
                font-family: monospace;
            }
            QHeaderView::section {
                background-color: #282a36;
                color: #f8f8f2;
                padding: 4px;
                border: 1px solid #44475a;
                font-weight: bold;
            }
            QDoubleSpinBox {
                background-color: #282a36;
                color: #50fa7b;
                border: 1px solid #6272a4;
                border-radius: 4px;
                padding: 4px;
                font-weight: bold;
                font-family: monospace;
                font-size: 13px;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #44475a;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #bd93f9;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #ff79c6;
                width: 14px;
                margin-top: -4px;
                margin-bottom: -4px;
                border-radius: 7px;
            }
        """)


# -------------------------------------------------------------
# ENTRY POINT
# -------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    bridge = RosBridge()
    bridge.start_ros()

    gui = DensoKinematicsGUI(bridge)
    gui.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
