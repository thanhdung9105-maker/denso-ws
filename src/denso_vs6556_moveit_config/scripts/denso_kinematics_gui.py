#!/usr/bin/env python3
"""
DENSO VS-6556 DYNAMICS CONTROL & MONITORING DASHBOARD (ROS 2)
Bảng điều khiển & Giám sát Động Lực Học chuyên sâu:
- Tab 1: Bảng Thông Số Động Lực Học Thời Gian Thực (Nền Trắng Siêu Rõ Nét)
- Tab 2: Phân Tích Động Lực Học Nghịch (Inverse Dynamics & Torque Breakdown)
- Tab 3: Mô Phỏng Động Lực Học Thuận (Forward Dynamics & Real-time Simulation)
- Đồng bộ trực tiếp với ROS 2 (/joint_states, /denso/cmd, /denso/joint_dynamics_markers)
"""

import sys
import os
import math
import time
import threading
import csv
from datetime import datetime
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
    QProgressBar, QFrame, QSplitter, QTabWidget, QCheckBox,
    QComboBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette

# Import Dynamics Engine
sys.path.append(os.path.dirname(__file__))
from denso_dynamics_engine import (
    DensoDynamicsEngine, TORQUE_LIMITS, JOINT_LIMITS, AXES, DH_CONFIG
)

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
JOINT_DESCS = [
    "Base (Xoay trục Z)",
    "Vai (Gập nghiêng Y)",
    "Khuỷu tay (Gập Y)",
    "Cẳng tay (Xoay trục dọc / Roll)",
    "Cổ tay (Gập ngửa Y)"
]

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
        self.node = Node('denso_dynamics_gui')
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
# MAIN DYNAMICS DASHBOARD GUI
# -------------------------------------------------------------
class DensoKinematicsGUI(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.dynamics_engine = DensoDynamicsEngine()

        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_efforts = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Velocity and acceleration calculation cache
        self.prev_calc_time = time.monotonic()
        self.prev_calc_qd = np.zeros(5)

        # Forward dynamics real-time simulation state
        self.sim_active = False
        self.sim_q = np.zeros(5)
        self.sim_qd = np.zeros(5)
        self.sim_tau = np.zeros(5)
        self.dynamics_visible = True
        self.latest_fk = None

        self.setWindowTitle("DENSO VS-6556 - BẢNG ĐIỀU KHIỂN & GIÁM SÁT ĐỘNG HỌC & ĐỘNG LỰC HỌC (ROS 2)")
        self.resize(1340, 880)
        self.init_ui()
        self.apply_dark_theme()

        # Connect ROS signal
        self.bridge.joint_states_received.connect(self.on_joint_states)

        # Main Telemetry refresh timer (10Hz)
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
        title_label = QLabel("DENSO VS-6556 KINEMATICS & DYNAMICS MONITOR")
        title_label.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title_label.setStyleSheet("color: #00e5ff; letter-spacing: 1px;")
        sub_label = QLabel("Bảng Giám Sát Động Lực Học Thời Gian Thực, Phân Tích Thuận/Nghịch & Động Học D-H 5 Khớp (ROS 2)")
        sub_label.setFont(QFont("Segoe UI", 10))
        sub_label.setStyleSheet("color: #a0a0b0;")
        title_box.addWidget(title_label)
        title_box.addWidget(sub_label)
        header_layout.addLayout(title_box)

        header_layout.addStretch()

        self.status_badge = QLabel("● ROS 2 CONNECTED")
        self.status_badge.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.status_badge.setStyleSheet(
            "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
            "border-radius: 6px; padding: 6px 14px;"
        )
        header_layout.addWidget(self.status_badge)

        self.btn_toggle_dynamics = QPushButton("VECTOR LỰC RVIZ2: ĐANG HIỆN")
        self.btn_toggle_dynamics.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_toggle_dynamics.setStyleSheet(
            "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
            "border-radius: 6px; padding: 6px 14px;"
        )
        self.btn_toggle_dynamics.setToolTip("Bấm để ẨN hoặc HIỆN các vector mô-men xoắn tại các khớp trong RViz2")
        self.btn_toggle_dynamics.clicked.connect(self.on_toggle_dynamics)
        header_layout.addWidget(self.btn_toggle_dynamics)

        main_layout.addLayout(header_layout)

        # --- TABS CONTAINER ---
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("Segoe UI", 10, QFont.Bold))

        # Tab 1: Bảng thông số động lực học (Nền trắng, chạy tab riêng)
        tab_telemetry = self.create_telemetry_tab()
        self.tabs.addTab(tab_telemetry, "1. BẢNG THÔNG SỐ ĐỘNG LỰC HỌC")

        # Tab 2: Động lực học nghịch (Inverse Dynamics)
        tab_id = self.create_id_panel()
        self.tabs.addTab(tab_id, "2. PHÂN TÍCH ĐỘNG LỰC HỌC NGHỊCH")

        # Tab 3: Động lực học thuận (Forward Dynamics)
        tab_fd = self.create_fd_panel()
        self.tabs.addTab(tab_fd, "3. MÔ PHỎNG ĐỘNG LỰC HỌC THUẬN")

        # Tab 4: Động học D-H & Tọa độ TCP
        tab_dh = self.create_dh_tab()
        self.tabs.addTab(tab_dh, "4. ĐỘNG HỌC D-H & TỌA ĐỘ TCP")

        main_layout.addWidget(self.tabs, 1)

        # --- FOOTER / LOG BAR ---
        self.log_label = QLabel("Hệ thống sẵn sàng. Dữ liệu động học & động lực học được cập nhật thời gian thực ở chu kỳ 10Hz.")
        self.log_label.setStyleSheet("color: #00e676; font-size: 11px; padding: 4px;")
        main_layout.addWidget(self.log_label)

    # ---------------------------------------------------------
    # TAB 1: DEDICATED DYNAMICS TELEMETRY (WHITE BACKGROUND)
    # ---------------------------------------------------------
    def create_telemetry_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # 1. Top KPI Summary Cards (White Background)
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(12)

        def make_card(title, initial_val, subtext, val_color="#0f172a"):
            box = QFrame()
            box.setStyleSheet("""
                QFrame {
                    background-color: #ffffff;
                    border: 1px solid #cbd5e1;
                    border-radius: 8px;
                    padding: 8px 14px;
                }
            """)
            v = QVBoxLayout(box)
            v.setContentsMargins(4, 4, 4, 4)
            v.setSpacing(3)
            lbl_t = QLabel(title)
            lbl_t.setStyleSheet("color: #64748b; font-size: 11px; font-weight: bold; text-transform: uppercase;")
            lbl_v = QLabel(initial_val)
            lbl_v.setStyleSheet(f"color: {val_color}; font-size: 20px; font-weight: bold; font-family: 'Segoe UI', monospace;")
            lbl_s = QLabel(subtext)
            lbl_s.setStyleSheet("color: #94a3b8; font-size: 11px;")
            v.addWidget(lbl_t)
            v.addWidget(lbl_v)
            v.addWidget(lbl_s)
            return box, lbl_v, lbl_s

        self.card_tau_box, self.card_tau_val, self.card_tau_sub = make_card(
            "MÔ-MEN LỚN NHẤT (MAX TORQUE)", "0.0 N·m", "Khớp J2 định mức 100 N·m", "#0284c7"
        )
        self.card_load_box, self.card_load_val, self.card_load_sub = make_card(
            "TẢI MOTOR CAO NHẤT (% LOAD)", "0.0%", "Ngưỡng an toàn < 80%", "#10b981"
        )
        self.card_payload_box, self.card_payload_val, self.card_payload_sub = make_card(
            "TẢI TRỌNG ĐẦU GẮP (PAYLOAD)", "0.0 kg", "Khả năng chịu tải tối đa 6.5 kg", "#8b5cf6"
        )
        self.card_power_box, self.card_power_val, self.card_power_sub = make_card(
            "TỔNG CÔNG SUẤT (POWER)", "0.0 W", "P = Σ |τ_i · q̇_i| cơ học", "#d97706"
        )
        self.card_status_box, self.card_status_val, self.card_status_sub = make_card(
            "TRẠNG THÁI HỆ THỐNG (STATUS)", "● AN TOÀN", "Tất cả khớp hoạt động <50% định mức", "#059669"
        )

        cards_layout.addWidget(self.card_tau_box)
        cards_layout.addWidget(self.card_load_box)
        cards_layout.addWidget(self.card_payload_box)
        cards_layout.addWidget(self.card_power_box)
        cards_layout.addWidget(self.card_status_box)
        layout.addLayout(cards_layout)

        # 1b. Mini Cartesian TCP HUD Row
        hud_box = QFrame()
        hud_box.setStyleSheet("""
            QFrame {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                padding: 6px 14px;
            }
        """)
        hud_layout = QHBoxLayout(hud_box)
        hud_layout.setContentsMargins(6, 4, 6, 4)
        hud_layout.setSpacing(14)

        lbl_hud_icon = QLabel("📍 TỌA ĐỘ ĐẦU GẮP (TCP):")
        lbl_hud_icon.setStyleSheet("color: #0284c7; font-weight: bold; font-size: 11px;")
        hud_layout.addWidget(lbl_hud_icon)

        self.tab1_tcp_xyz = QLabel("X: +0.0 mm   Y: +0.0 mm   Z: +1000.0 mm")
        self.tab1_tcp_xyz.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.tab1_tcp_xyz.setStyleSheet("color: #0f172a; font-family: 'Segoe UI', monospace;")
        hud_layout.addWidget(self.tab1_tcp_xyz)

        hud_layout.addSpacing(8)

        self.tab1_tcp_r = QLabel("Tầm vươn R: 1000.0 mm (r_xy: 0.0 mm)")
        self.tab1_tcp_r.setFont(QFont("Segoe UI", 10))
        self.tab1_tcp_r.setStyleSheet("color: #475569;")
        hud_layout.addWidget(self.tab1_tcp_r)

        hud_layout.addStretch()

        self.tab1_tcp_rpy = QLabel("Hướng Roll: -90.0° | Pitch: -90.0° | Yaw: 0.0°")
        self.tab1_tcp_rpy.setFont(QFont("Segoe UI", 10))
        self.tab1_tcp_rpy.setStyleSheet("color: #64748b;")
        hud_layout.addWidget(self.tab1_tcp_rpy)

        layout.addWidget(hud_box)

        # 2. Main Telemetry Table (White Background)
        table_container = QGroupBox("BẢNG THEO DÕI ĐỘNG LỰC HỌC 5 KHỚP THỜI GIAN THỰC")
        table_container.setFont(QFont("Segoe UI", 11, QFont.Bold))
        table_container.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 10px;
                padding: 12px 10px 10px 10px;
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
        tbl_layout = QVBoxLayout(table_container)
        tbl_layout.setContentsMargins(6, 12, 6, 6)

        self.telemetry_table = QTableWidget(5, 10)
        self.telemetry_table.setHorizontalHeaderLabels([
            "Khớp", "Trục & Vai Trò", "Vị Trí Góc q", "Vận Tốc q̇", "Gia Tốc q̈",
            "Mô-Men τ (N·m)", "Giới Hạn τ_max", "Tải Động Cơ (%)", "Công Suất P", "Trạng Thái"
        ])
        h_header = self.telemetry_table.horizontalHeader()
        h_header.setSectionResizeMode(QHeaderView.Stretch)
        h_header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        h_header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        h_header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        h_header.setSectionResizeMode(6, QHeaderView.ResizeToContents)
        h_header.setSectionResizeMode(7, QHeaderView.Fixed)
        self.telemetry_table.setColumnWidth(7, 155)
        h_header.setSectionResizeMode(8, QHeaderView.ResizeToContents)
        h_header.setSectionResizeMode(9, QHeaderView.Fixed)
        self.telemetry_table.setColumnWidth(9, 130)
        self.telemetry_table.verticalHeader().setVisible(False)
        self.telemetry_table.verticalHeader().setDefaultSectionSize(48)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.telemetry_table.setFixedHeight(290)
        self.telemetry_table.setAlternatingRowColors(True)

        for row in range(5):
            self.telemetry_table.setRowHeight(row, 48)

        self.telemetry_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                color: #0f172a;
                gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                font-family: 'Segoe UI', 'DejaVu Sans', 'Ubuntu', sans-serif;
                font-size: 13px;
                selection-background-color: #f1f5f9;
            }
            QHeaderView::section {
                background-color: #f1f5f9;
                color: #0f172a;
                font-weight: bold;
                font-size: 13px;
                padding: 10px 6px;
                border: 1px solid #e2e8f0;
                border-top: none;
                border-left: none;
            }
            QTableWidget::item {
                padding: 6px 8px;
                color: #0f172a;
            }
            QTableWidget::item:alternate {
                background-color: #f8fafc;
            }
        """)

        self.telemetry_progress_bars = []
        self.telemetry_status_badges = []

        for row in range(5):
            # 0: Joint Name
            it_name = QTableWidgetItem(f"Joint {row+1}")
            it_name.setFont(QFont("Segoe UI", 12, QFont.Bold))
            it_name.setTextAlignment(Qt.AlignCenter)
            self.telemetry_table.setItem(row, 0, it_name)

            # 1: Axis & Role
            it_axis = QTableWidgetItem(JOINT_DESCS[row])
            it_axis.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.telemetry_table.setItem(row, 1, it_axis)

            # 2: Position q
            it_q = QTableWidgetItem("0.0° (0.000 rad)")
            it_q.setTextAlignment(Qt.AlignCenter)
            self.telemetry_table.setItem(row, 2, it_q)

            # 3: Velocity qd
            it_qd = QTableWidgetItem("0.0°/s (0.000 rad/s)")
            it_qd.setTextAlignment(Qt.AlignCenter)
            self.telemetry_table.setItem(row, 3, it_qd)

            # 4: Acceleration qdd
            it_qdd = QTableWidgetItem("0.0°/s² (0.00 rad/s²)")
            it_qdd.setTextAlignment(Qt.AlignCenter)
            self.telemetry_table.setItem(row, 4, it_qdd)

            # 5: Torque tau
            it_tau = QTableWidgetItem("0.00 N·m")
            it_tau.setTextAlignment(Qt.AlignCenter)
            it_tau.setFont(QFont("Segoe UI", 12, QFont.Bold))
            self.telemetry_table.setItem(row, 5, it_tau)

            # 6: Torque Limit
            it_lim = QTableWidgetItem(f"{TORQUE_LIMITS[row]:.0f} N·m")
            it_lim.setTextAlignment(Qt.AlignCenter)
            it_lim.setForeground(QColor("#64748b"))
            self.telemetry_table.setItem(row, 6, it_lim)

            # 7: Motor Load Gauge Widget (with ample vertical space)
            pb_container = QWidget()
            pb_container.setStyleSheet("background-color: transparent;")
            pb_box = QHBoxLayout(pb_container)
            pb_box.setContentsMargins(8, 0, 8, 0)
            pb_box.setSpacing(10)
            pb_box.setAlignment(Qt.AlignVCenter)

            pb = QProgressBar()
            pb.setRange(0, 100)
            pb.setValue(0)
            pb.setFixedHeight(18)
            pb.setTextVisible(False)
            pb.setStyleSheet("""
                QProgressBar {
                    background-color: #f1f5f9;
                    border: 1px solid #cbd5e1;
                    border-radius: 5px;
                }
                QProgressBar::chunk {
                    background-color: #10b981;
                    border-radius: 4px;
                }
            """)

            lbl_pct = QLabel("0.0%")
            lbl_pct.setFixedWidth(54)
            lbl_pct.setFixedHeight(26)
            lbl_pct.setFont(QFont("Segoe UI", 11, QFont.Bold))
            lbl_pct.setStyleSheet("color: #0f172a; font-size: 13px; font-weight: bold;")
            lbl_pct.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            pb_box.addWidget(pb)
            pb_box.addWidget(lbl_pct)
            self.telemetry_table.setCellWidget(row, 7, pb_container)
            self.telemetry_progress_bars.append((pb, lbl_pct))

            # 8: Mechanical/Electrical Power P (W)
            it_power = QTableWidgetItem("0.0 W")
            it_power.setTextAlignment(Qt.AlignCenter)
            it_power.setFont(QFont("Segoe UI", 11, QFont.Bold))
            it_power.setForeground(QColor("#0f172a"))
            self.telemetry_table.setItem(row, 8, it_power)

            # 9: Status Badge (Crisp, centered pill badge with ample row height)
            badge = QLabel("● AN TOÀN")
            badge.setAlignment(Qt.AlignCenter)
            badge.setFont(QFont("Segoe UI", 10, QFont.Bold))
            badge.setFixedHeight(28)
            badge.setMinimumWidth(115)
            badge.setStyleSheet("""
                background-color: #ecfdf5;
                color: #065f46;
                border: 1.5px solid #10b981;
                border-radius: 14px;
                font-weight: bold;
                font-size: 12px;
                padding: 0px 8px;
            """)
            badge_container = QWidget()
            badge_container.setStyleSheet("background-color: transparent;")
            badge_box = QHBoxLayout(badge_container)
            badge_box.setContentsMargins(6, 0, 6, 0)
            badge_box.setAlignment(Qt.AlignCenter)
            badge_box.addWidget(badge)
            self.telemetry_table.setCellWidget(row, 9, badge_container)
            self.telemetry_status_badges.append(badge)

        tbl_layout.addWidget(self.telemetry_table)
        layout.addWidget(table_container)

        # 3. Control & Quick Actions Bar
        ctrl_box = QGroupBox("ĐIỀU KHIỂN & VẬN HÀNH ROBOT THỜI GIAN THỰC")
        ctrl_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        ctrl_box.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 10px;
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

        ctrl_layout.addWidget(QLabel("Tải gắp (Payload kg):"))
        self.sp_tab1_payload = QDoubleSpinBox()
        self.sp_tab1_payload.setRange(0.0, 7.0)
        self.sp_tab1_payload.setValue(0.0)
        self.sp_tab1_payload.setSingleStep(0.5)
        self.sp_tab1_payload.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #f8fafc;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 6px 10px;
                font-weight: bold;
                font-size: 13px;
            }
        """)
        self.sp_tab1_payload.valueChanged.connect(self.on_tab1_payload_changed)
        ctrl_layout.addWidget(self.sp_tab1_payload)

        ctrl_layout.addSpacing(10)

        btn_demo = QPushButton("▶ Chạy Demo Vòng Lặp")
        btn_demo.setStyleSheet("background-color: #0284c7; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_demo.clicked.connect(lambda: self.bridge.send_cmd("demo"))
        ctrl_layout.addWidget(btn_demo)

        btn_once = QPushButton("⚡ Chạy 1 Chiều")
        btn_once.setStyleSheet("background-color: #475569; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_once.clicked.connect(lambda: self.bridge.send_cmd("once"))
        ctrl_layout.addWidget(btn_once)

        btn_stop = QPushButton("■ Dừng Robot")
        btn_stop.setStyleSheet("background-color: #dc2626; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_stop.clicked.connect(lambda: self.bridge.send_cmd("stop"))
        ctrl_layout.addWidget(btn_stop)

        btn_home = QPushButton("⌂ Về Gốc (Home 0°)")
        btn_home.setStyleSheet("background-color: #059669; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_home.clicked.connect(lambda: self.bridge.send_cmd("home"))
        ctrl_layout.addWidget(btn_home)

        btn_csv = QPushButton("📸 Xuất Báo Cáo CSV")
        btn_csv.setStyleSheet("background-color: #0d9488; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        btn_csv.clicked.connect(self.on_export_telemetry_csv)
        ctrl_layout.addWidget(btn_csv)

        ctrl_layout.addStretch()
        layout.addWidget(ctrl_box)

        layout.addStretch()
        return tab

    def on_tab1_payload_changed(self, val):
        self.bridge.send_cmd(f"payload {val:.2f}")
        self.sp_payload.setValue(val)
        self.log_label.setText(f"⚖️ Đã thiết lập khối lượng tải trọng đầu gắp: {val:.2f} kg")

    # ---------------------------------------------------------
    # TAB 2: INVERSE DYNAMICS (ID) - WHITE TABLES
    # ---------------------------------------------------------
    def create_id_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setSpacing(15)

        # Left Column: Inputs & Commands
        left_box = QGroupBox("📥 THÔNG SỐ ĐẦU VÀO ĐỘNG LỰC HỌC NGHỊCH")
        left_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        left_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(12, 16, 12, 12)

        lbl_q = QLabel("Vị trí (q), Vận tốc (q̇) và Gia tốc (q̈) tại 5 khớp:")
        lbl_q.setStyleSheet("color: #0284c7; font-weight: bold;")
        left_layout.addWidget(lbl_q)

        self.id_input_table = QTableWidget(5, 4)
        self.id_input_table.setHorizontalHeaderLabels(["Khớp", "q (độ)", "q̇ (rad/s)", "q̈ (rad/s²)"])
        self.id_input_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.id_input_table.verticalHeader().setVisible(False)
        self.id_input_table.setFixedHeight(175)
        self.id_input_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

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
        load_box.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; color: #0f172a;")
        load_grid = QGridLayout(load_box)
        load_grid.addWidget(QLabel("Khối lượng tải (kg):"), 0, 0)
        self.sp_payload = QDoubleSpinBox()
        self.sp_payload.setRange(0.0, 7.0)
        self.sp_payload.setValue(0.0)
        self.sp_payload.setSingleStep(0.5)
        self.sp_payload.setStyleSheet("background-color: #ffffff; color: #0f172a; border: 1px solid #cbd5e1; padding: 4px;")
        load_grid.addWidget(self.sp_payload, 0, 1)

        load_grid.addWidget(QLabel("Lực Fz (N):"), 0, 2)
        self.sp_fz = QDoubleSpinBox()
        self.sp_fz.setRange(-100.0, 100.0)
        self.sp_fz.setValue(0.0)
        self.sp_fz.setStyleSheet("background-color: #ffffff; color: #0f172a; border: 1px solid #cbd5e1; padding: 4px;")
        load_grid.addWidget(self.sp_fz, 0, 3)
        left_layout.addWidget(load_box)

        # Quick Actions
        btn_sync = QPushButton("📌 Lấy góc & vận tốc hiện tại từ Robot")
        btn_sync.setStyleSheet("background-color: #475569; color: white; padding: 7px; border-radius: 6px; font-weight: bold;")
        btn_sync.clicked.connect(self.on_id_sync_robot)
        left_layout.addWidget(btn_sync)

        btn_grav_comp = QPushButton("⚖️ Cân Bằng Trọng Lực Tĩnh (q̇=0, q̈=0)")
        btn_grav_comp.setStyleSheet("background-color: #0284c7; color: white; padding: 7px; border-radius: 6px; font-weight: bold;")
        btn_grav_comp.clicked.connect(self.on_id_gravity_comp)
        left_layout.addWidget(btn_grav_comp)

        # Quick Presets Box
        preset_box = QGroupBox("Tư Thế Mẫu Nhanh (Pose Presets):")
        preset_box.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; color: #0f172a;")
        preset_layout = QHBoxLayout(preset_box)
        preset_layout.setContentsMargins(6, 6, 6, 6)
        preset_layout.setSpacing(6)

        btn_p_home = QPushButton("⌂ Home 0°")
        btn_p_home.setStyleSheet("background-color: #059669; color: white; padding: 5px 6px; border-radius: 4px; font-weight: bold;")
        btn_p_home.clicked.connect(lambda: self.apply_id_preset([0.0, 0.0, 0.0, 0.0, 0.0]))
        preset_layout.addWidget(btn_p_home)

        btn_p_reach = QPushButton("↔ Vươn Ngang 90°")
        btn_p_reach.setStyleSheet("background-color: #0284c7; color: white; padding: 5px 6px; border-radius: 4px; font-weight: bold;")
        btn_p_reach.clicked.connect(lambda: self.apply_id_preset([0.0, 90.0, 0.0, 0.0, 0.0]))
        preset_layout.addWidget(btn_p_reach)

        btn_p_elbow = QPushButton("⌐ Gập Khuỷu 90°")
        btn_p_elbow.setStyleSheet("background-color: #475569; color: white; padding: 5px 6px; border-radius: 4px; font-weight: bold;")
        btn_p_elbow.clicked.connect(lambda: self.apply_id_preset([0.0, 0.0, 90.0, 0.0, 0.0]))
        preset_layout.addWidget(btn_p_elbow)

        btn_p_ready = QPushButton("⚡ Sẵn Sàng")
        btn_p_ready.setStyleSheet("background-color: #8b5cf6; color: white; padding: 5px 6px; border-radius: 4px; font-weight: bold;")
        btn_p_ready.clicked.connect(lambda: self.apply_id_preset([0.0, 30.0, 60.0, 0.0, 30.0]))
        preset_layout.addWidget(btn_p_ready)

        left_layout.addWidget(preset_box)

        self.btn_calc_id = QPushButton("⚡ TÍNH TOÁN MÔ-MEN XOẮN (INVERSE DYNAMICS)")
        self.btn_calc_id.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_calc_id.setStyleSheet("background-color: #059669; color: white; padding: 10px; border-radius: 6px; font-weight: bold;")
        self.btn_calc_id.clicked.connect(self.on_compute_id)
        left_layout.addWidget(self.btn_calc_id)

        self.chk_auto_marker = QCheckBox("🔴 Hiển thị Vector Mô-men Xoắn Trên RViz2")
        self.chk_auto_marker.setChecked(True)
        self.chk_auto_marker.setStyleSheet("color: #047857; font-weight: bold; margin-top: 5px;")
        left_layout.addWidget(self.chk_auto_marker)

        left_layout.addStretch()
        layout.addWidget(left_box, 1)

        # Right Column: Torque Outputs & Breakdown (White Background)
        right_box = QGroupBox("📊 KẾT QUẢ PHÂN RÃ MÔ-MEN XOẮN (TORQUE BREAKDOWN)")
        right_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(12, 16, 12, 12)

        self.id_res_table = QTableWidget(5, 7)
        self.id_res_table.setHorizontalHeaderLabels([
            "Khớp", "Quán tính (M·q̈)", "Coriolis (C·q̇)", "Trọng lực (g)", "Tải trọng", "Tổng τ (N·m)", "% Tải"
        ])
        self.id_res_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.id_res_table.verticalHeader().setVisible(False)
        self.id_res_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.id_res_table.setFixedHeight(180)
        self.id_res_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

        for i in range(5):
            for c in range(7):
                item = QTableWidgetItem("-" if c > 0 else f"Joint {i+1}")
                item.setTextAlignment(Qt.AlignCenter)
                self.id_res_table.setItem(i, c, item)
        right_layout.addWidget(self.id_res_table)

        # Progress bars for torque load
        lbl_bars = QLabel("Mức Độ Tải Trọng Động Cơ So Với Giới Hạn Tối Đa (Motor Load %):")
        lbl_bars.setStyleSheet("color: #0284c7; font-weight: bold; margin-top: 5px;")
        right_layout.addWidget(lbl_bars)

        self.load_bars = []
        bar_box = QGroupBox()
        bar_box.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px;")
        bar_grid = QGridLayout(bar_box)
        for i in range(5):
            bar_grid.addWidget(QLabel(f"J{i+1} (Max {TORQUE_LIMITS[i]:.0f}Nm):"), i, 0)
            pb = QProgressBar()
            pb.setRange(0, 100)
            pb.setValue(0)
            pb.setStyleSheet("""
                QProgressBar {
                    border: 1px solid #cbd5e1; border-radius: 4px; text-align: center;
                    background-color: #ffffff; color: #0f172a; font-weight: bold;
                }
                QProgressBar::chunk { background-color: #10b981; border-radius: 3px; }
            """)
            bar_grid.addWidget(pb, i, 1)
            self.load_bars.append(pb)
        right_layout.addWidget(bar_box)

        self.id_status_msg = QLabel("Chưa thực hiện tính toán.")
        self.id_status_msg.setStyleSheet("color: #64748b; font-style: italic;")
        right_layout.addWidget(self.id_status_msg)

        right_layout.addStretch()
        layout.addWidget(right_box, 1)

        return panel

    # ---------------------------------------------------------
    # TAB 3: FORWARD DYNAMICS (FD) - WHITE TABLES
    # ---------------------------------------------------------
    def create_fd_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setSpacing(15)

        # Left Column: Input Torques
        left_box = QGroupBox("📥 NHẬP MÔ-MEN XOẮN ĐẦU VÀO τ (INPUT TORQUES)")
        left_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        left_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(12, 16, 12, 12)

        lbl_desc = QLabel("Thiết lập mô-men xoắn τ (N·m) cho từng động cơ:")
        lbl_desc.setStyleSheet("color: #0284c7; font-weight: bold;")
        left_layout.addWidget(lbl_desc)

        self.fd_spinboxes = []
        for i in range(5):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"Joint {i+1} (Max {TORQUE_LIMITS[i]:.0f} N·m):"))
            sp = QDoubleSpinBox()
            sp.setRange(-float(TORQUE_LIMITS[i]), float(TORQUE_LIMITS[i]))
            sp.setValue(0.0)
            sp.setSingleStep(1.0)
            sp.setDecimals(1)
            sp.setStyleSheet("background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; padding: 4px; font-weight: bold;")
            h.addWidget(sp)
            self.fd_spinboxes.append(sp)
            left_layout.addLayout(h)

        btn_h = QHBoxLayout()
        btn_zero = QPushButton("0️⃣ Đặt Về 0 N·m")
        btn_zero.setStyleSheet("background-color: #475569; color: white; padding: 6px; border-radius: 6px;")
        btn_zero.clicked.connect(self.on_fd_zero_torques)
        btn_h.addWidget(btn_zero)

        btn_grav = QPushButton("⚖️ Giữ Cân Bằng Trọng Lực")
        btn_grav.setStyleSheet("background-color: #0284c7; color: white; padding: 6px; border-radius: 6px;")
        btn_grav.clicked.connect(self.on_fd_set_gravity_torques)
        btn_h.addWidget(btn_grav)
        left_layout.addLayout(btn_h)

        self.btn_calc_fd = QPushButton("⚙️ TÍNH TOÁN GIA TỐC GÓC q̈ (FORWARD DYNAMICS)")
        self.btn_calc_fd.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_calc_fd.setStyleSheet("background-color: #059669; color: white; padding: 10px; border-radius: 6px; font-weight: bold;")
        self.btn_calc_fd.clicked.connect(self.on_compute_fd)
        left_layout.addWidget(self.btn_calc_fd)

        # Simulation Section
        sim_box = QGroupBox("Mô Phỏng Phản Ứng Vật Lý Tương Tác (RViz2):")
        sim_box.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; color: #0f172a;")
        sim_layout = QVBoxLayout(sim_box)

        lbl_sim_desc = QLabel("Bấm 'Bắt đầu' để kéo thanh trượt mô-men và quan sát robot gia tốc trong RViz2:")
        lbl_sim_desc.setWordWrap(True)
        lbl_sim_desc.setStyleSheet("color: #64748b; font-size: 11px;")
        sim_layout.addWidget(lbl_sim_desc)

        sim_btn_h = QHBoxLayout()
        self.btn_start_sim = QPushButton("▶️ Bắt Đầu Mô Phỏng")
        self.btn_start_sim.setStyleSheet("background-color: #059669; color: white; padding: 7px; border-radius: 6px; font-weight: bold;")
        self.btn_start_sim.clicked.connect(self.on_start_simulation)
        sim_btn_h.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton("⏹️ Dừng Mô Phỏng")
        self.btn_stop_sim.setStyleSheet("background-color: #dc2626; color: white; padding: 7px; border-radius: 6px; font-weight: bold;")
        self.btn_stop_sim.setEnabled(False)
        self.btn_stop_sim.clicked.connect(self.on_stop_simulation)
        sim_btn_h.addWidget(self.btn_stop_sim)
        sim_layout.addLayout(sim_btn_h)

        left_layout.addWidget(sim_box)
        left_layout.addStretch()
        layout.addWidget(left_box, 1)

        # Right Column: Acceleration Results & Mass Matrix (White Background)
        right_box = QGroupBox("📊 KẾT QUẢ GIA TỐC GÓC q̈ & MA TRẬN QUÁN TÍNH M(q)")
        right_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(12, 16, 12, 12)

        lbl_acc = QLabel("Gia tốc góc tức thời tại các khớp (Joint Accelerations):")
        lbl_acc.setStyleSheet("color: #0284c7; font-weight: bold;")
        right_layout.addWidget(lbl_acc)

        self.fd_res_table = QTableWidget(5, 4)
        self.fd_res_table.setHorizontalHeaderLabels(["Khớp", "Gia tốc q̈ (rad/s²)", "Gia tốc q̈ (°/s²)", "Chiều Gia Tốc"])
        self.fd_res_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.fd_res_table.verticalHeader().setVisible(False)
        self.fd_res_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.fd_res_table.setFixedHeight(175)
        self.fd_res_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

        for i in range(5):
            self.fd_res_table.setItem(i, 0, QTableWidgetItem(f"Joint {i+1}"))
            self.fd_res_table.setItem(i, 1, QTableWidgetItem("0.000"))
            self.fd_res_table.setItem(i, 2, QTableWidgetItem("0.00°/s²"))
            self.fd_res_table.setItem(i, 3, QTableWidgetItem("Cân bằng"))
            for c in range(4):
                self.fd_res_table.item(i, c).setTextAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.fd_res_table)

        lbl_m = QLabel("Ma trận quán tính M(q) 5×5 (Inertia Matrix):")
        lbl_m.setStyleSheet("color: #8b5cf6; font-weight: bold; margin-top: 5px;")
        right_layout.addWidget(lbl_m)

        self.mass_table = QTableWidget(5, 5)
        self.mass_table.setHorizontalHeaderLabels(["M1", "M2", "M3", "M4", "M5"])
        self.mass_table.setVerticalHeaderLabels(["J1", "J2", "J3", "J4", "J5"])
        self.mass_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.mass_table.setFixedHeight(150)
        self.mass_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.mass_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 12px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

        for r in range(5):
            for c in range(5):
                item = QTableWidgetItem("0.0000")
                item.setTextAlignment(Qt.AlignCenter)
                self.mass_table.setItem(r, c, item)
        right_layout.addWidget(self.mass_table)

        self.lbl_kinetic_energy = QLabel("⚡ Động năng toàn robot (E_k): 0.0000 J")
        self.lbl_kinetic_energy.setStyleSheet("color: #8b5cf6; font-weight: bold; font-size: 12px; margin-top: 6px; padding: 6px; background-color: #f8fafc; border-radius: 4px; border: 1px solid #e2e8f0;")
        right_layout.addWidget(self.lbl_kinetic_energy)

        right_layout.addStretch()
        layout.addWidget(right_box, 1)

        return panel

    # ---------------------------------------------------------
    # TAB 4: KINEMATICS D-H & CARTESIAN TCP (WHITE BACKGROUND)
    # ---------------------------------------------------------
    def create_dh_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # 1. Top Cartesian HUD KPI Cards
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(12)

        def make_dh_card(title, initial_val, subtext, val_color="#0f172a"):
            box = QFrame()
            box.setStyleSheet("""
                QFrame {
                    background-color: #ffffff;
                    border: 1px solid #cbd5e1;
                    border-radius: 8px;
                    padding: 8px 14px;
                }
            """)
            v = QVBoxLayout(box)
            v.setContentsMargins(4, 4, 4, 4)
            v.setSpacing(3)
            lbl_t = QLabel(title)
            lbl_t.setStyleSheet("color: #64748b; font-size: 11px; font-weight: bold; text-transform: uppercase;")
            lbl_v = QLabel(initial_val)
            lbl_v.setStyleSheet(f"color: {val_color}; font-size: 17px; font-weight: bold; font-family: 'Segoe UI', monospace;")
            lbl_s = QLabel(subtext)
            lbl_s.setStyleSheet("color: #94a3b8; font-size: 11px;")
            v.addWidget(lbl_t)
            v.addWidget(lbl_v)
            v.addWidget(lbl_s)
            return box, lbl_v, lbl_s

        self.dh_card_pos_box, self.dh_card_pos_val, self.dh_card_pos_sub = make_dh_card(
            "VỊ TRÍ ĐẦU GẮP TCP (X, Y, Z)", "X: +0.0  Y: +0.0  Z: 1000.0", "Hệ quy chiếu Base (Đơn vị: mm)", "#0284c7"
        )
        self.dh_card_reach_box, self.dh_card_reach_val, self.dh_card_reach_sub = make_dh_card(
            "TẦM VƯƠN ROBOT (REACH)", "R = 1000.0 mm", "Bán kính phẳng r_xy = 0.0 mm", "#10b981"
        )
        self.dh_card_rpy_box, self.dh_card_rpy_val, self.dh_card_rpy_sub = make_dh_card(
            "HƯỚNG EULER ĐẦU GẮP (RPY)", "R: -90.0°  P: -90.0°  Y: 0.0°", "Góc Roll - Pitch - Yaw (độ)", "#8b5cf6"
        )
        self.dh_card_status_box, self.dh_card_status_val, self.dh_card_status_sub = make_dh_card(
            "KHÔNG GIAN LÀM VIỆC (WORKSPACE)", "● TRONG TẦM VƯƠN", "Bán kính R <= 1000 mm (An toàn)", "#059669"
        )

        cards_layout.addWidget(self.dh_card_pos_box)
        cards_layout.addWidget(self.dh_card_reach_box)
        cards_layout.addWidget(self.dh_card_rpy_box)
        cards_layout.addWidget(self.dh_card_status_box)
        layout.addLayout(cards_layout)

        # 2. Split Area: Left = DH Table, Right = 4x4 Matrix
        content_layout = QHBoxLayout()
        content_layout.setSpacing(12)

        # --- LEFT: DH Table Box ---
        left_box = QGroupBox("BẢNG THAM SỐ ĐỘNG HỌC DENAVIT - HARTENBERG (STANDARD DH)")
        left_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        left_box.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 10px;
                padding: 12px 10px 10px 10px;
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
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(6, 12, 6, 6)

        self.dh_table = QTableWidget(5, 7)
        self.dh_table.setHorizontalHeaderLabels([
            "Khớp", "Góc q (°)", "Tham Số θ_i", "Độ Dời d_i (m)", "Độ Dài a_i (m)", "Góc Xoắn α_i", "Đặc Điểm Trục Z_i"
        ])
        h_dh = self.dh_table.horizontalHeader()
        h_dh.setSectionResizeMode(QHeaderView.Stretch)
        h_dh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        h_dh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        h_dh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        h_dh.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        h_dh.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        h_dh.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        self.dh_table.verticalHeader().setVisible(False)
        self.dh_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.dh_table.setFixedHeight(275)
        self.dh_table.setAlternatingRowColors(True)

        for row in range(5):
            self.dh_table.setRowHeight(row, 48)

        self.dh_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                color: #0f172a;
                gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                font-family: 'Segoe UI', 'DejaVu Sans', sans-serif;
                font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9;
                color: #0f172a;
                font-weight: bold;
                font-size: 12px;
                padding: 8px 4px;
                border: 1px solid #e2e8f0;
            }
            QTableWidget::item { padding: 6px; color: #0f172a; }
            QTableWidget::item:alternate { background-color: #f8fafc; }
        """)

        for row in range(5):
            it_name = QTableWidgetItem(f"Joint {row+1}")
            it_name.setFont(QFont("Segoe UI", 11, QFont.Bold))
            it_name.setTextAlignment(Qt.AlignCenter)
            self.dh_table.setItem(row, 0, it_name)

            for col in range(1, 7):
                it = QTableWidgetItem("-")
                it.setTextAlignment(Qt.AlignCenter if col < 6 else Qt.AlignLeft | Qt.AlignVCenter)
                self.dh_table.setItem(row, col, it)

        left_layout.addWidget(self.dh_table)

        # Explanatory formula box
        dh_info_box = QFrame()
        dh_info_box.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; padding: 8px;")
        dh_info_layout = QVBoxLayout(dh_info_box)
        dh_info_layout.setContentsMargins(4, 4, 4, 4)
        dh_info_layout.setSpacing(4)

        lbl_dh_eq = QLabel("Công thức ma trận D-H chuẩn: A_i = Rot(Z, θ_i) · Trans(Z, d_i) · Trans(X, a_i) · Rot(X, α_i)")
        lbl_dh_eq.setStyleSheet("color: #0284c7; font-weight: bold; font-size: 11px;")
        dh_info_layout.addWidget(lbl_dh_eq)

        lbl_dh_sub = QLabel("• Thông số danh định: J1 (d1=0.335m, α1=-90°) | J2 (a2=0.270m) | J4 (d4=0.295m, α4=-90°) | J5 (a5=0.100m)\n"
                            "• Căn chỉnh 100% khớp với tư thế nến thẳng đứng (Candlestick / Vertical 0° Calibration) của robot thật.")
        lbl_dh_sub.setStyleSheet("color: #64748b; font-size: 11px;")
        dh_info_layout.addWidget(lbl_dh_sub)

        left_layout.addWidget(dh_info_box)
        content_layout.addWidget(left_box, 6)

        # --- RIGHT: 4x4 Matrix Box ---
        right_box = QGroupBox("MA TRẬN BIẾN ĐỔI ĐỒNG NHẤT 4×4")
        right_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_box.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 10px;
                padding: 12px 10px 10px 10px;
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
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(6, 12, 6, 6)

        # Matrix Selector Dropdown
        sel_box = QHBoxLayout()
        sel_box.addWidget(QLabel("Chọn Ma Trận:"))
        self.combo_matrix = QComboBox()
        self.combo_matrix.addItems([
            "Ma trận tổng thể T_0^5 (Base → TCP Flange)",
            "Ma trận mắt xích A_1 (Base → Joint 1)",
            "Ma trận mắt xích A_2 (Joint 1 → Joint 2)",
            "Ma trận mắt xích A_3 (Joint 2 → Joint 3)",
            "Ma trận mắt xích A_4 (Joint 3 → Joint 4)",
            "Ma trận mắt xích A_5 (Joint 4 → Joint 5)"
        ])
        self.combo_matrix.setStyleSheet("""
            QComboBox {
                background-color: #f8fafc;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 4px 8px;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        self.combo_matrix.currentIndexChanged.connect(self.on_matrix_combo_changed)
        sel_box.addWidget(self.combo_matrix, 1)
        right_layout.addLayout(sel_box)

        # 4x4 Table
        self.matrix_table = QTableWidget(4, 4)
        self.matrix_table.setHorizontalHeaderLabels(["nx (cột 1)", "oy (cột 2)", "az (cột 3)", "Vị trí P (m)"])
        self.matrix_table.setVerticalHeaderLabels(["X", "Y", "Z", "1"])
        self.matrix_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.matrix_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.matrix_table.setFixedHeight(210)
        self.matrix_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.matrix_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                color: #0f172a;
                gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 13px;
                font-weight: bold;
            }
            QHeaderView::section {
                background-color: #f1f5f9;
                color: #0f172a;
                font-weight: bold;
                border: 1px solid #e2e8f0;
            }
            QTableWidget::item { padding: 4px; }
        """)

        for r in range(4):
            for c in range(4):
                it = QTableWidgetItem("0.0000")
                it.setTextAlignment(Qt.AlignCenter)
                self.matrix_table.setItem(r, c, it)
        right_layout.addWidget(self.matrix_table)

        # Structure explanation
        mat_info = QLabel("Cấu trúc: [ Ma trận xoay R (3×3)  |  Vector vị trí P (3×1) ]\n"
                          "         [ 0         0        0   |  1                   ]")
        mat_info.setStyleSheet("color: #475569; font-family: 'Consolas', monospace; font-size: 11px; background-color: #f8fafc; padding: 6px; border-radius: 4px;")
        mat_info.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(mat_info)

        content_layout.addWidget(right_box, 5)
        layout.addLayout(content_layout)

        layout.addStretch()
        return tab

    def update_dh_tab(self, fk_res):
        if fk_res is None:
            return

        pos = fk_res['tcp_pos']
        rpy = fk_res['tcp_rpy']
        R_reach = fk_res['reach_R']
        r_xy = fk_res['reach_xy']

        # Update Top KPI Cards
        self.dh_card_pos_val.setText(f"X: {pos[0]*1000:+5.1f}  Y: {pos[1]*1000:+5.1f}  Z: {pos[2]*1000:5.1f}")
        self.dh_card_reach_val.setText(f"R = {R_reach*1000:5.1f} mm")
        self.dh_card_reach_sub.setText(f"Bán kính phẳng r_xy = {r_xy*1000:5.1f} mm")
        self.dh_card_rpy_val.setText(f"R: {rpy[0]:+5.1f}°  P: {rpy[1]:+5.1f}°  Y: {rpy[2]:+5.1f}°")

        if R_reach <= 1.001:
            self.dh_card_status_val.setText("● TRONG TẦM VƯƠN")
            self.dh_card_status_val.setStyleSheet("color: #059669; font-size: 17px; font-weight: bold;")
            self.dh_card_status_sub.setText("Bán kính R <= 1000 mm (An toàn)")
        else:
            self.dh_card_status_val.setText("▲ NGOÀI TẦM CHUẨN")
            self.dh_card_status_val.setStyleSheet("color: #d97706; font-size: 17px; font-weight: bold;")
            self.dh_card_status_sub.setText("Khoảng cách vượt quá chiều dài danh định")

        # Update DH Table
        dh_rows = fk_res['dh_table']
        for i, row_data in enumerate(dh_rows):
            # Col 1: q_deg
            self.dh_table.item(i, 1).setText(f"{row_data['q_deg']:+.1f}°")
            # Col 2: theta_deg
            self.dh_table.item(i, 2).setText(f"{row_data['theta_deg']:+.1f}° ({row_data['theta_rad']:+.3f})")
            # Col 3: d
            self.dh_table.item(i, 3).setText(f"{row_data['d']:.3f}")
            # Col 4: a
            self.dh_table.item(i, 4).setText(f"{row_data['a']:.3f}")
            # Col 5: alpha
            self.dh_table.item(i, 5).setText(f"{row_data['alpha_deg']:+.1f}°")
            # Col 6: desc
            self.dh_table.item(i, 6).setText(row_data['desc'])

        # Update 4x4 Matrix Table
        self.render_selected_matrix(fk_res)

    def on_matrix_combo_changed(self, idx):
        if self.latest_fk:
            self.render_selected_matrix(self.latest_fk)

    def render_selected_matrix(self, fk_res):
        idx = self.combo_matrix.currentIndex()
        if idx == 0:
            M = fk_res['T_total']
        else:
            M = fk_res['A_matrices'][idx - 1]

        for r in range(4):
            for c in range(4):
                val = M[r, c]
                if abs(val) < 1e-4:
                    val_str = "0.0000"
                else:
                    val_str = f"{val:+.4f}"
                it = self.matrix_table.item(r, c)
                it.setText(val_str)
                if c == 3 and r < 3:
                    it.setForeground(QColor("#0284c7"))  # Position in blue
                elif r == 3:
                    it.setForeground(QColor("#94a3b8"))  # Bottom row in gray
                else:
                    it.setForeground(QColor("#0f172a"))  # Rotation in dark

    def on_export_telemetry_csv(self):
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"denso_telemetry_{now_str}.csv"
        filepath = os.path.join(os.path.expanduser("~"), filename)

        q = self.current_joints
        qd = self.current_velocities
        now = time.monotonic()
        dt = max(1e-3, now - self.prev_calc_time)
        qdd = (np.array(qd) - self.prev_calc_qd) / dt
        payload = self.sp_tab1_payload.value()
        res = self.dynamics_engine.inverse_dynamics(q, qd, qdd, payload_mass=payload)
        fk = self.dynamics_engine.compute_fk_dh(q)

        lines = [
            f"# DENSO VS-6556 TELEMETRY SNAPSHOT - {datetime.now().isoformat()}",
            f"# TCP Position (m): X={fk['tcp_pos'][0]:.4f}, Y={fk['tcp_pos'][1]:.4f}, Z={fk['tcp_pos'][2]:.4f}",
            f"# TCP Euler RPY (deg): Roll={fk['tcp_rpy'][0]:.1f}, Pitch={fk['tcp_rpy'][1]:.1f}, Yaw={fk['tcp_rpy'][2]:.1f}",
            f"# Reach R (m): {fk['reach_R']:.4f}, Payload (kg): {payload:.2f}",
            "Joint,Name,q_deg,q_rad,qd_deg_s,qd_rad_s,qdd_rad_s2,tau_Nm,tau_limit_Nm,load_percent,power_W"
        ]

        for i in range(5):
            q_deg = float(np.degrees(q[i]))
            qd_deg = float(np.degrees(qd[i]))
            t_val = float(res['tau'][i])
            pct = float(res['percent_load'][i])
            p_val = abs(t_val * float(qd[i]))
            lines.append(
                f"Joint_{i+1},{JOINT_NAMES[i]},{q_deg:.2f},{q[i]:.4f},{qd_deg:.2f},{qd[i]:.4f},{qdd[i]:.3f},{t_val:.2f},{TORQUE_LIMITS[i]:.0f},{pct:.1f},{p_val:.2f}"
            )

        csv_content = "\n".join(lines)
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(csv_content)
        except Exception:
            pass

        try:
            clipboard = QApplication.clipboard()
            clipboard.setText(csv_content)
            self.log_label.setText(f"📸 Đã xuất dữ liệu đo ra file '{filepath}' và lưu vào Clipboard!")
        except Exception:
            self.log_label.setText(f"📸 Đã xuất dữ liệu đo ra file '{filepath}'!")

    def apply_id_preset(self, deg_list):
        for i in range(5):
            self.id_input_table.item(i, 1).setText(f"{deg_list[i]:.1f}")
            self.id_input_table.item(i, 2).setText("0.00")
            self.id_input_table.item(i, 3).setText("0.00")
        self.on_compute_id()
        self.log_label.setText(f"🎯 Đã nạp tư thế mẫu: {deg_list}° và tính xong mô-men.")

    # ---------------------------------------------------------
    # TAB 1 LIVE UPDATE (TELEMETRY)
    # ---------------------------------------------------------
    def update_telemetry_tab(self):
        q = np.array(self.current_joints, dtype=float)
        qd = np.array(self.current_velocities, dtype=float)

        now = time.monotonic()
        dt = max(1e-3, now - self.prev_calc_time)
        qdd = (qd - self.prev_calc_qd) / dt
        self.prev_calc_qd = np.array(qd)
        self.prev_calc_time = now

        # 1. Forward Kinematics (DH)
        fk_res = self.dynamics_engine.compute_fk_dh(q)
        self.latest_fk = fk_res

        # Update Tab 1 Mini Cartesian HUD
        pos = fk_res['tcp_pos']
        rpy = fk_res['tcp_rpy']
        self.tab1_tcp_xyz.setText(f"X: {pos[0]*1000:+6.1f} mm   Y: {pos[1]*1000:+6.1f} mm   Z: {pos[2]*1000:6.1f} mm")
        self.tab1_tcp_r.setText(f"Tầm vươn R: {fk_res['reach_R']*1000:6.1f} mm (r_xy: {fk_res['reach_xy']*1000:6.1f} mm)")
        self.tab1_tcp_rpy.setText(f"Hướng Roll: {rpy[0]:+5.1f}° | Pitch: {rpy[1]:+5.1f}° | Yaw: {rpy[2]:+5.1f}°")

        # 2. Inverse Dynamics for Telemetry
        payload = self.sp_tab1_payload.value()
        res = self.dynamics_engine.inverse_dynamics(q, qd, qdd, payload_mass=payload)
        tau = res['tau']
        pct_load = res['percent_load']

        max_tau_val = 0.0
        max_load_pct = 0.0
        total_power = 0.0

        for i in range(5):
            t_val = float(tau[i])
            pct = float(pct_load[i])
            p_val = abs(t_val * float(qd[i]))
            total_power += p_val

            if abs(t_val) > max_tau_val:
                max_tau_val = abs(t_val)
            if pct > max_load_pct:
                max_load_pct = pct

            # Col 2: Position q (Degrees first)
            q_deg = np.rad2deg(q[i])
            if abs(q_deg) < 0.05:
                q_deg = 0.0
            q_val = 0.0 if abs(q[i]) < 0.0005 else q[i]
            self.telemetry_table.item(i, 2).setText(f"{q_deg:+.1f}° ({q_val:+.2f} rad)")

            # Col 3: Velocity qd
            qd_deg = np.rad2deg(qd[i])
            if abs(qd_deg) < 0.05:
                qd_deg = 0.0
            qd_val = 0.0 if abs(qd[i]) < 0.0005 else qd[i]
            self.telemetry_table.item(i, 3).setText(f"{qd_deg:+.1f}°/s ({qd_val:+.2f} rad/s)")

            # Col 4: Acceleration qdd
            qdd_deg = np.rad2deg(qdd[i])
            if abs(qdd_deg) < 0.05:
                qdd_deg = 0.0
            qdd_val = 0.0 if abs(qdd[i]) < 0.0005 else qdd[i]
            self.telemetry_table.item(i, 4).setText(f"{qdd_deg:+.1f}°/s² ({qdd_val:+.1f} rad/s²)")

            # Col 5: Torque tau (Prevent -0.00 N·m glitch!)
            if abs(t_val) < 0.005:
                tau_str = "0.00 N·m"
            else:
                tau_str = f"{t_val:+.2f} N·m"
            tau_item = self.telemetry_table.item(i, 5)
            tau_item.setText(tau_str)
            if pct < 50.0:
                tau_item.setForeground(QColor("#047857"))  # Dark emerald
            elif pct < 80.0:
                tau_item.setForeground(QColor("#b45309"))  # Dark amber
            else:
                tau_item.setForeground(QColor("#b91c1c"))  # Dark red

            # Col 7: Progress bar & percent
            pb, lbl_pct = self.telemetry_progress_bars[i]
            val = int(min(100, max(0, pct)))
            pb.setValue(val)
            lbl_pct.setText(f"{pct:4.1f}%")

            if pct < 50.0:
                chunk_col = "#10b981"
                lbl_pct.setStyleSheet("color: #047857; font-weight: bold; font-size: 13px;")
            elif pct < 80.0:
                chunk_col = "#f59e0b"
                lbl_pct.setStyleSheet("color: #b45309; font-weight: bold; font-size: 13px;")
            else:
                chunk_col = "#ef4444"
                lbl_pct.setStyleSheet("color: #b91c1c; font-weight: bold; font-size: 13px;")

            pb.setStyleSheet(f"""
                QProgressBar {{
                    background-color: #f1f5f9;
                    border: 1px solid #cbd5e1;
                    border-radius: 5px;
                }}
                QProgressBar::chunk {{
                    background-color: {chunk_col};
                    border-radius: 4px;
                }}
            """)

            # Col 8: Power P (W)
            it_power = self.telemetry_table.item(i, 8)
            it_power.setText(f"{p_val:.1f} W")
            if p_val > 50.0:
                it_power.setForeground(QColor("#b45309"))
            else:
                it_power.setForeground(QColor("#0f172a"))

            # Col 9: Status Badge (Crisp, centered pill badge with ample row height)
            badge = self.telemetry_status_badges[i]
            if pct < 50.0:
                badge.setText("● AN TOÀN")
                badge.setStyleSheet("""
                    background-color: #ecfdf5; color: #065f46;
                    border: 1.5px solid #10b981; border-radius: 14px;
                    font-weight: bold; font-size: 12px; padding: 0px 8px;
                """)
            elif pct < 80.0:
                badge.setText("▲ CẢNH BÁO")
                badge.setStyleSheet("""
                    background-color: #fffbeb; color: #92400e;
                    border: 1.5px solid #f59e0b; border-radius: 14px;
                    font-weight: bold; font-size: 12px; padding: 0px 8px;
                """)
            else:
                badge.setText("■ QUÁ TẢI")
                badge.setStyleSheet("""
                    background-color: #fef2f2; color: #991b1b;
                    border: 1.5px solid #ef4444; border-radius: 14px;
                    font-weight: bold; font-size: 12px; padding: 0px 8px;
                """)

        # Update Top KPI Cards
        self.card_tau_val.setText(f"{max_tau_val:.1f} N·m")
        self.card_load_val.setText(f"{max_load_pct:.1f}%")
        self.card_payload_val.setText(f"{payload:.1f} kg")

        self.card_power_val.setText(f"{total_power:.1f} W")
        if total_power < 50.0:
            self.card_power_val.setStyleSheet("color: #059669; font-size: 20px; font-weight: bold;")
        elif total_power < 150.0:
            self.card_power_val.setStyleSheet("color: #d97706; font-size: 20px; font-weight: bold;")
        else:
            self.card_power_val.setStyleSheet("color: #dc2626; font-size: 20px; font-weight: bold;")

        if max_load_pct < 50.0:
            self.card_load_val.setStyleSheet("color: #10b981; font-size: 20px; font-weight: bold;")
            self.card_status_val.setText("● AN TOÀN")
            self.card_status_val.setStyleSheet("color: #059669; font-size: 20px; font-weight: bold;")
            self.card_status_sub.setText("Tất cả khớp hoạt động <50% định mức")
        elif max_load_pct < 80.0:
            self.card_load_val.setStyleSheet("color: #f59e0b; font-size: 20px; font-weight: bold;")
            self.card_status_val.setText("▲ TẢI TRUNG BÌNH")
            self.card_status_val.setStyleSheet("color: #d97706; font-size: 20px; font-weight: bold;")
            self.card_status_sub.setText("Mô-men trong ngưỡng an toàn (50-80%)")
        else:
            self.card_load_val.setStyleSheet("color: #ef4444; font-size: 20px; font-weight: bold;")
            self.card_status_val.setText("■ CẢNH BÁO QUÁ TẢI")
            self.card_status_val.setStyleSheet("color: #dc2626; font-size: 20px; font-weight: bold;")
            self.card_status_sub.setText("Có khớp vượt 80% mô-men định mức!")

        # 3. Update Tab 4 (DH Kinematics & Cartesian TCP)
        self.update_dh_tab(fk_res)

    # ---------------------------------------------------------
    # TAB 2 & 3 CALLBACKS
    # ---------------------------------------------------------
    def on_id_sync_robot(self):
        for i in range(5):
            self.id_input_table.item(i, 1).setText(f"{np.rad2deg(self.current_joints[i]):.1f}")
            self.id_input_table.item(i, 2).setText(f"{self.current_velocities[i]:.3f}")
            self.id_input_table.item(i, 3).setText("0.00")
        self.log_label.setText("📌 Đã đồng bộ vị trí và vận tốc hiện tại từ Robot vào Động Lực Học Nghịch.")

    def on_id_gravity_comp(self):
        for i in range(5):
            self.id_input_table.item(i, 2).setText("0.00")
            self.id_input_table.item(i, 3).setText("0.00")
        self.on_compute_id()
        self.log_label.setText("⚖️ Đã tính toán mô-men bù trọng lực tĩnh (Gravity Compensation).")

    def on_compute_id(self):
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
                pb_color = "#10b981"
            elif pct < 80:
                pb_color = "#f59e0b"
            else:
                pb_color = "#ef4444"
            pb.setStyleSheet(f"""
                QProgressBar {{
                    border: 1px solid #cbd5e1; border-radius: 4px; text-align: center;
                    background-color: #ffffff; color: #0f172a; font-weight: bold;
                }}
                QProgressBar::chunk {{ background-color: {pb_color}; border-radius: 3px; }}
            """)

        if res['is_overload']:
            self.id_status_msg.setText("⚠️ CẢNH BÁO QUÁ TẢI: Có khớp vượt quá 100% mô-men xoắn định mức!")
            self.id_status_msg.setStyleSheet("color: #dc2626; font-weight: bold; font-size: 12px;")
        else:
            self.id_status_msg.setText("✅ Trạng thái an toàn: Tất cả các khớp hoạt động trong ngưỡng cho phép.")
            self.id_status_msg.setStyleSheet("color: #059669; font-weight: bold; font-size: 12px;")

        if self.chk_auto_marker.isChecked():
            markers = self.dynamics_engine.build_marker_array(q, res['tau'], qdd, frame_id="world")
            self.bridge.publish_markers(markers)

        self.log_label.setText(f"⚡ Đã tính xong Động Lực Học Nghịch. Tổng Torque = {np.round(res['tau'], 2)} N·m")

    def on_fd_zero_torques(self):
        for sp in self.fd_spinboxes:
            sp.setValue(0.0)

    def on_fd_set_gravity_torques(self):
        g_tau = self.dynamics_engine.gravity_compensation(self.current_joints)
        for i in range(5):
            val = float(g_tau[i])
            limit = float(TORQUE_LIMITS[i])
            val_clipped = max(-limit, min(limit, val))
            self.fd_spinboxes[i].setValue(val_clipped)
        self.log_label.setText("⚖️ Đã thiết lập các thanh trượt mô-men xoắn bằng đúng giá trị bù trọng lực.")

    def on_compute_fd(self):
        tau = np.array([sp.value() for sp in self.fd_spinboxes])
        q = np.array(self.current_joints)
        qd = np.array(self.current_velocities)

        M = self.dynamics_engine.compute_mass_matrix(q)
        qdd = self.dynamics_engine.forward_dynamics(q, qd, tau)

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

        for r in range(5):
            for c in range(5):
                self.mass_table.item(r, c).setText(f"{M[r, c]:.4f}")

        ek = self.dynamics_engine.compute_kinetic_energy(q, qd)
        self.lbl_kinetic_energy.setText(f"⚡ Động năng toàn robot (E_k): {ek:.4f} J")

        markers = self.dynamics_engine.build_marker_array(q, tau, qdd, frame_id="world")
        self.bridge.publish_markers(markers)

        self.log_label.setText(f"⚙️ Đã giải xong Động Lực Học Thuận: q̈ = {np.round(qdd, 3)} rad/s²")

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

        cmd = f"goto {self.sim_q[0]:.4f} {self.sim_q[1]:.4f} {self.sim_q[2]:.4f} {self.sim_q[3]:.4f} {self.sim_q[4]:.4f}"
        self.bridge.send_cmd(cmd)

        for i in range(5):
            self.fd_res_table.item(i, 1).setText(f"{qdd[i]:+.3f}")
            self.fd_res_table.item(i, 2).setText(f"{np.rad2deg(qdd[i]):+.2f}°/s²")

        ek = self.dynamics_engine.compute_kinetic_energy(self.sim_q, self.sim_qd)
        self.lbl_kinetic_energy.setText(f"⚡ Động năng toàn robot (E_k): {ek:.4f} J")

        markers = self.dynamics_engine.build_marker_array(self.sim_q, tau, qdd, frame_id="world")
        self.bridge.publish_markers(markers)

    # ---------------------------------------------------------
    # CALLBACKS & TIMERS
    # ---------------------------------------------------------
    def on_joint_states(self, positions, velocities, efforts):
        self.current_joints = list(positions)
        self.current_velocities = list(velocities)
        self.current_efforts = list(efforts)

    def on_timer_tick(self):
        self.update_telemetry_tab()

    def on_toggle_dynamics(self):
        self.dynamics_visible = not self.dynamics_visible
        if self.dynamics_visible:
            self.btn_toggle_dynamics.setText("VECTOR LỰC RVIZ2: ĐANG HIỆN")
            self.btn_toggle_dynamics.setStyleSheet(
                "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
                "border-radius: 6px; padding: 6px 14px;"
            )
            self.bridge.send_cmd("dynamics_on")
            self.log_label.setText("Đã hiển thị các vector mô-men xoắn trong RViz2.")
        else:
            self.btn_toggle_dynamics.setText("VECTOR LỰC RVIZ2: ĐÃ ẨN")
            self.btn_toggle_dynamics.setStyleSheet(
                "background-color: #495057; color: #adb5bd; border: 1px solid #6c757d; "
                "border-radius: 6px; padding: 6px 14px;"
            )
            self.bridge.send_cmd("dynamics_off")
            self.log_label.setText("Đã ẩn các vector mô-men xoắn trong RViz2.")

    def apply_dark_theme(self):
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor("#1e1e24"))
        palette.setColor(QPalette.WindowText, QColor("#f8f8f2"))
        palette.setColor(QPalette.Base, QColor("#282a36"))
        palette.setColor(QPalette.AlternateBase, QColor("#1e1e24"))
        palette.setColor(QPalette.Text, QColor("#f8f8f2"))
        palette.setColor(QPalette.Button, QColor("#44475a"))
        palette.setColor(QPalette.ButtonText, QColor("#f8f8f2"))
        self.setPalette(palette)

        self.setStyleSheet("""
            QTabBar::tab {
                background: #282a36;
                color: #a0a0b0;
                padding: 9px 20px;
                border: 1px solid #44475a;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 4px;
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
