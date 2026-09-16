#!/usr/bin/env python3
"""
DENSO VS-6556 KINEMATICS DASHBOARD (ROS 2)
Bảng hiển thị Động học thuận (FK) và Động học nghịch (IK) thời gian thực.
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

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QProgressBar, QFrame, QSplitter
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon

# -------------------------------------------------------------
# KINEMATICS ENGINE FOR DENSO VS-6556
# -------------------------------------------------------------
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
JOINT_LIMITS = [
    (-2.96, 2.96),  # Joint 1: Base Z [-170°, +170°]
    (-1.74, 1.74),  # Joint 2: Shoulder Y [-100°, +100°]
    (-2.18, 2.18),  # Joint 3: Elbow Y [-125°, +125°]
    (-3.31, 3.31),  # Joint 4: Forearm X [-190°, +190°]
    (-2.09, 2.09),  # Joint 5: Wrist Y [-120°, +120°]
]

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
    """
    Damped Least Squares (Levenberg-Marquardt) IK solver for target position [x, y, z].
    """
    if q_init is None:
        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        q = np.array(q_init, dtype=float)

    damping = 0.015
    step_size = 0.45
    eps = 1e-6
    q_min = np.array([lim[0] for lim in JOINT_LIMITS])
    q_max = np.array([lim[1] for lim in JOINT_LIMITS])

    err = np.zeros(3)
    for _ in range(max_iter):
        T = forward_kinematics(q)
        curr_pos = T[:3, 3]
        err = target_pos - curr_pos
        err_norm = np.linalg.norm(err)
        if err_norm < tol:
            return q, True, err_norm

        # Numerical Jacobian 3x5
        J = np.zeros((3, 5))
        for j in range(5):
            q_p = q.copy()
            q_p[j] += eps
            p_p = forward_kinematics(q_p)[:3, 3]
            J[:, j] = (p_p - curr_pos) / eps

        # DLS pseudo-inverse
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
    joint_states_received = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.node = None
        self.cmd_pub = None
        self.latest_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

    def start_ros(self):
        rclpy.init(args=None)
        self.node = Node('denso_kinematics_gui')
        self.node.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.cmd_pub = self.node.create_publisher(String, '/denso/cmd', 10)

        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _js_cb(self, msg: JointState):
        name_map = {name: i for i, name in enumerate(JOINT_NAMES)}
        updated = False
        for i, name in enumerate(msg.name):
            if name in name_map and i < len(msg.position):
                idx = name_map[name]
                self.latest_joints[idx] = float(msg.position[i])
                updated = True
        if updated:
            self.joint_states_received.emit(list(self.latest_joints))

    def send_cmd(self, cmd_str: str):
        if self.cmd_pub:
            msg = String()
            msg.data = cmd_str
            self.cmd_pub.publish(msg)


# -------------------------------------------------------------
# MAIN DASHBOARD GUI
# -------------------------------------------------------------
class DensoKinematicsGUI(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.solved_ik_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.setWindowTitle("DENSO VS-6556 - Bảng Động Học Thuận & Nghịch (ROS 2)")
        self.resize(1150, 780)
        self.init_ui()
        self.apply_dark_theme()

        # Connect ROS signal
        self.bridge.joint_states_received.connect(self.on_joint_states)

        # Refresh timer (10Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_fk_display)
        self.timer.start(100)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(12)

        # --- HEADER ---
        header_layout = QHBoxLayout()
        title_box = QVBoxLayout()
        title_label = QLabel("DENSO VS-6556 KINEMATICS DASHBOARD")
        title_label.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title_label.setStyleSheet("color: #00e5ff; letter-spacing: 1px;")
        sub_label = QLabel("Mô phỏng & Giám sát: Động Học Thuận (FK) & Động Học Nghịch (IK) trong ROS 2")
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
        main_layout.addLayout(header_layout)

        # --- DIVIDER ---
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color: #3d3d52;")
        main_layout.addWidget(line)

        # --- TWO PANELS (FK & IK) ---
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.create_fk_panel())
        splitter.addWidget(self.create_ik_panel())
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        main_layout.addWidget(splitter, 1)

        # --- FOOTER / LOG BAR ---
        self.log_label = QLabel("Trạng thái: Đang kết nối tới topic /joint_states và /denso/cmd...")
        self.log_label.setStyleSheet("color: #00e676; font-size: 11px; padding: 4px;")
        main_layout.addWidget(self.log_label)

    # ---------------------------------------------------------
    # LEFT PANEL: FORWARD KINEMATICS (FK)
    # ---------------------------------------------------------
    def create_fk_panel(self):
        panel = QGroupBox("📍 ĐỘNG HỌC THUẬN (FORWARD KINEMATICS)")
        panel.setFont(QFont("Segoe UI", 11, QFont.Bold))
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        # 1. Joints Table
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

        # 2. Cartesian Pose Box
        lbl_cart = QLabel("Tọa Độ & Hướng Đầu Gắp (End-Effector / Flange):")
        lbl_cart.setStyleSheet("color: #50fa7b; font-weight: bold;")
        layout.addWidget(lbl_cart)

        cart_box = QGroupBox()
        cart_box.setStyleSheet("background-color: #21222c; border: 1px solid #44475a; border-radius: 8px;")
        grid = QGridLayout(cart_box)
        grid.setContentsMargins(12, 10, 12, 10)
        grid.setHorizontalSpacing(15)
        grid.setVerticalSpacing(8)

        # Labels for X, Y, Z
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

        # 3. Quick Joint Joggers
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
    # RIGHT PANEL: INVERSE KINEMATICS (IK)
    # ---------------------------------------------------------
    def create_ik_panel(self):
        panel = QGroupBox("🎯 ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS)")
        panel.setFont(QFont("Segoe UI", 11, QFont.Bold))
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        # 1. Target Cartesian Inputs
        lbl_target = QLabel("Nhập Tọa Độ Mục Tiêu Đầu Gắp (Target Pose):")
        lbl_target.setStyleSheet("color: #ff79c6; font-weight: bold;")
        layout.addWidget(lbl_target)

        inp_box = QGroupBox()
        inp_box.setStyleSheet("background-color: #21222c; border: 1px solid #44475a; border-radius: 8px;")
        grid = QGridLayout(inp_box)
        grid.setContentsMargins(12, 10, 12, 10)
        grid.setHorizontalSpacing(15)

        # Spinboxes for X, Y, Z (mm)
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

        # 2. Solve IK Button
        self.btn_solve = QPushButton("⚙️ GIẢI ĐỘNG HỌC NGHỊCH (SOLVE IK)")
        self.btn_solve.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_solve.setStyleSheet(
            "background-color: #00b4d8; color: #03045e; padding: 10px; border-radius: 6px; font-weight: bold;"
        )
        self.btn_solve.clicked.connect(self.on_solve_ik)
        layout.addWidget(self.btn_solve)

        # 3. IK Solution Table
        lbl_res = QLabel("Nghiệm Góc Khớp Tính Toán (IK Solution):")
        lbl_res.setStyleSheet("color: #8be9fd; font-weight: bold;")
        layout.addWidget(lbl_res)

        self.ik_table = QTableWidget(5, 3)
        self.ik_table.setHorizontalHeaderLabels(["Khớp", "Nghiệm (Rad)", "Nghiệm (Độ °)"])
        self.ik_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ik_table.verticalHeader().setVisible(False)
        self.ik_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.ik_table.setFixedHeight(175)

        for row in range(5):
            self.ik_table.setItem(row, 0, QTableWidgetItem(f"Joint {row+1}"))
            self.ik_table.setItem(row, 1, QTableWidgetItem("0.0000"))
            self.ik_table.setItem(row, 2, QTableWidgetItem("0.00°"))
            for col in range(3):
                self.ik_table.item(row, col).setTextAlignment(Qt.AlignCenter)
        layout.addWidget(self.ik_table)

        # Accuracy & Status Badge
        self.lbl_ik_status = QLabel("Trạng thái: Chưa giải | Sai số: 0.0 mm")
        self.lbl_ik_status.setStyleSheet("color: #f1fa8c; font-weight: bold;")
        layout.addWidget(self.lbl_ik_status)

        # 4. Action Buttons
        self.btn_execute = QPushButton("🚀 GỬI TỚI ROBOT TRONG RVIZ2")
        self.btn_execute.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_execute.setStyleSheet(
            "background-color: #06d6a0; color: #073b4c; padding: 11px; border-radius: 6px; font-weight: bold;"
        )
        self.btn_execute.clicked.connect(self.on_send_to_robot)
        layout.addWidget(self.btn_execute)

        # Quick Presets
        lbl_preset = QLabel("Vị trí mẫu nhanh (Presets):")
        lbl_preset.setStyleSheet("color: #a0a0b0;")
        layout.addWidget(lbl_preset)

        preset_layout = QHBoxLayout()
        presets = [
            ("🏠 Home 0°", [-370.0, 0.0, 695.0]),
            ("📦 Vị trí Gắp 1", [-250.0, 150.0, 550.0]),
            ("📍 Vị trí Gắp 2", [-280.0, -180.0, 520.0]),
            ("🔝 Nâng Cao", [-350.0, 0.0, 750.0]),
        ]
        for name, coords in presets:
            btn = QPushButton(name)
            btn.setStyleSheet("background-color: #383a59; color: #f8f8f2; padding: 5px;")
            btn.clicked.connect(lambda _, c=coords: self.load_preset(c))
            preset_layout.addWidget(btn)
        layout.addLayout(preset_layout)

        layout.addStretch()
        return panel

    # ---------------------------------------------------------
    # SLOTS & LOGIC
    # ---------------------------------------------------------
    def on_joint_states(self, joints):
        self.current_joints = list(joints)

    def update_fk_display(self):
        # Update Table
        for row in range(5):
            rad = self.current_joints[row]
            deg = np.rad2deg(rad)
            self.fk_table.item(row, 2).setText(f"{rad:+.4f}")
            self.fk_table.item(row, 3).setText(f"{deg:+.1f}°")

        # Update Cartesian Pose
        pose = get_cartesian_pose(self.current_joints)
        x_mm = pose['x'] * 1000.0
        y_mm = pose['y'] * 1000.0
        z_mm = pose['z'] * 1000.0
        r_mm = math.sqrt(x_mm**2 + y_mm**2 + z_mm**2)

        self.val_x.setText(f"{x_mm:+.1f} mm ({pose['x']:+.3f} m)")
        self.val_y.setText(f"{y_mm:+.1f} mm ({pose['y']:+.3f} m)")
        self.val_z.setText(f"{z_mm:+.1f} mm ({pose['z']:+.3f} m)")
        self.val_r.setText(f"{pose['roll']:+.1f}°")
        self.val_p.setText(f"{pose['pitch']:+.1f}°")
        self.val_yaw.setText(f"{pose['yaw']:+.1f}°")
        self.val_dist.setText(f"{r_mm:.1f} mm")

    def on_jog_slider_changed(self):
        jog_q = []
        for i in range(5):
            sl, lbl = self.jog_sliders[i]
            val = sl.value()
            lbl.setText(f"{val:+d}°")
            jog_q.append(np.deg2rad(val))
        
        # Calculate FK for jogged joints
        pose = get_cartesian_pose(jog_q)
        x_mm = pose['x'] * 1000.0
        y_mm = pose['y'] * 1000.0
        z_mm = pose['z'] * 1000.0
        self.log_label.setText(
            f"💡 Xem thử FK góc kéo: J=[{','.join([f'{sl.value()}°' for sl, _ in self.jog_sliders])}] "
            f"-> End-Effector: X={x_mm:.1f}mm, Y={y_mm:.1f}mm, Z={z_mm:.1f}mm"
        )

    def copy_current_to_target(self):
        pose = get_cartesian_pose(self.current_joints)
        self.sp_x.setValue(round(pose['x'] * 1000.0, 1))
        self.sp_y.setValue(round(pose['y'] * 1000.0, 1))
        self.sp_z.setValue(round(pose['z'] * 1000.0, 1))
        self.log_label.setText("📌 Đã sao chép tọa độ hiện tại của robot vào ô mục tiêu IK.")

    def load_preset(self, coords):
        self.sp_x.setValue(coords[0])
        self.sp_y.setValue(coords[1])
        self.sp_z.setValue(coords[2])
        self.on_solve_ik()

    def on_solve_ik(self):
        target_pos = np.array([
            self.sp_x.value() / 1000.0,
            self.sp_y.value() / 1000.0,
            self.sp_z.value() / 1000.0
        ])

        t0 = time.perf_counter()
        q_sol, success, err_norm = solve_inverse_kinematics(target_pos, q_init=self.current_joints)
        calc_ms = (time.perf_counter() - t0) * 1000.0

        self.solved_ik_joints = list(q_sol)
        err_mm = err_norm * 1000.0

        # Update IK Table
        for row in range(5):
            rad = q_sol[row]
            deg = np.rad2deg(rad)
            self.ik_table.item(row, 1).setText(f"{rad:+.4f}")
            self.ik_table.item(row, 2).setText(f"{deg:+.2f}°")

        if success:
            self.lbl_ik_status.setText(f"✅ GIẢI THÀNH CÔNG! Thời gian: {calc_ms:.1f}ms | Sai số: {err_mm:.2f} mm")
            self.lbl_ik_status.setStyleSheet("color: #50fa7b; font-weight: bold;")
            self.log_label.setText(f"✅ Đã tìm thấy nghiệm IK tối ưu cho tọa độ [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] m")
        else:
            self.lbl_ik_status.setText(f"⚠️ NGOÀI TẦM VỚI HOẶC GẦN ĐIỂM KỲ DỊ! Sai số: {err_mm:.2f} mm")
            self.lbl_ik_status.setStyleSheet("color: #ff5555; font-weight: bold;")
            self.log_label.setText("⚠️ Vị trí mục tiêu có thể vượt quá không gian làm việc hoặc gần điểm kỳ dị của cánh tay.")

    def on_send_to_robot(self):
        # Format: goto q1 q2 q3 q4 q5
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
