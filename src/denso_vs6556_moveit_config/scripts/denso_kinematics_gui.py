#!/usr/bin/env python3
"""
DENSO VS-6556 DYNAMICS CONTROL & MONITORING DASHBOARD (ROS 2)
Bảng điều khiển & Giám sát Động Lực Học chuyên sâu:
- Tab 1: Bảng Thông Số Động Lực Học Thời Gian Thực (Nền Trắng Siêu Rõ Nét)
- Tab 2: Phân Tích Động Lực Học Nghịch (Inverse Dynamics & Torque Breakdown)
- Tab 3: Mô Phỏng Động Lực Học Thuận (Forward Dynamics & Real-time Simulation)
- Tab 4: Động Học D-H & Ma Trận Biến Đổi (Kinematics & TCP Coordinates)
- Tab 5: Điều Khiển Tay Trực Tiếp, Ghi/Phát Quỹ Đạo & Workspace Visualizer
- Đồng bộ trực tiếp với ROS 2 (/joint_states, /denso/cmd, /denso/status, /denso/joint_dynamics_markers)
"""

import sys
import os
import math
import time
import threading
import json
import datetime
import numpy as np

# ROS 2 imports
import rclpy  # pyrefly: ignore [missing-import]
from rclpy.node import Node  # pyrefly: ignore [missing-import]
from sensor_msgs.msg import JointState  # pyrefly: ignore [missing-import]
from std_msgs.msg import String  # pyrefly: ignore [missing-import]
from visualization_msgs.msg import MarkerArray  # pyrefly: ignore [missing-import]
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # pyrefly: ignore [missing-import]
from builtin_interfaces.msg import Duration  # pyrefly: ignore [missing-import]

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QProgressBar, QFrame, QSplitter, QTabWidget, QCheckBox, QComboBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette
from typing import Any, Optional

def connect_signal(signal: Any, slot: Any) -> None:
    """Safely connect a Qt signal to a slot (avoids IDE stub discrepancy for QPushButton.clicked)."""
    signal.connect(slot)

def set_table_item_text(table: QTableWidget, row: int, col: int, text: str, color: Optional[QColor] = None) -> None:
    """Safely update or set a QTableWidgetItem text and optional foreground color."""
    it = table.item(row, col)
    if it is not None:
        it.setText(text)
        if color is not None:
            it.setForeground(color)
    else:
        new_it = QTableWidgetItem(text)
        if color is not None:
            new_it.setForeground(color)
        table.setItem(row, col, new_it)

def get_table_item_text(table: QTableWidget, row: int, col: int, default: str = "0.0") -> str:
    """Safely retrieve text from a QTableWidgetItem without NoneType error."""
    it = table.item(row, col)
    return it.text() if it is not None else default
try:
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Import Dynamics Engine
sys.path.append(os.path.dirname(__file__))
from denso_dynamics_engine import (
    DensoDynamicsEngine, TORQUE_LIMITS, JOINT_LIMITS, AXES, DH_PARAMS
)

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
JOINT_DESCS = [
    "Base (Xoay trục Z)",
    "Vai (Gập nghiêng Y)",
    "Khuỷu tay (Gập Y)",
    "Cẳng tay (Xoay trục dọc / Roll)",
    "Cổ tay (Gập ngửa Y)",
    "Mặt bích (Xoay trục Z / Flange Roll)"
]

# -------------------------------------------------------------
# ROS 2 THREAD & BRIDGE
# -------------------------------------------------------------
class RosBridge(QObject):
    joint_states_received: Any = pyqtSignal(list, list, list)

    def __init__(self):
        super().__init__()
        self.node = None
        self.cmd_pub = None
        self.marker_pub = None
        self.traj_pub = None
        self.latest_joints = [0.0] * len(JOINT_NAMES)
        self.latest_velocities = [0.0] * len(JOINT_NAMES)
        self.latest_efforts = [0.0] * len(JOINT_NAMES)

    def start_ros(self):
        rclpy.init(args=None)
        self.node = Node('denso_dynamics_gui')
        self.node.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.node.create_subscription(String, '/denso/status', self._status_cb, 10)
        self.latest_status = {}
        self.last_js_time = self.node.get_clock().now()
        self.cmd_pub = self.node.create_publisher(String, '/denso/cmd', 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, '/denso/joint_dynamics_markers', 10)
        self.traj_pub = self.node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

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
            try:
                self.last_js_time = self.node.get_clock().now()
                self.joint_states_received.emit(
                    list(self.latest_joints),
                    list(self.latest_velocities),
                    list(self.latest_efforts)
                )
            except (RuntimeError, AttributeError):
                pass

    def _status_cb(self, msg: String):
        try:
            self.latest_status = json.loads(msg.data)
        except Exception:
            pass

    def send_cmd(self, cmd_str: str):
        if self.cmd_pub:
            msg = String()
            msg.data = cmd_str
            self.cmd_pub.publish(msg)

    def send_trajectory(self, target_rads, duration_sec=1.5):
        if self.traj_pub:
            traj = JointTrajectory()
            traj.joint_names = list(JOINT_NAMES)
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in target_rads]
            pt.velocities = [0.0] * len(JOINT_NAMES)
            sec = int(duration_sec)
            nanosec = int((duration_sec - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj.points = [pt]
            self.traj_pub.publish(traj)

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

        self.current_joints = [0.0] * len(JOINT_NAMES)
        self.current_velocities = [0.0] * len(JOINT_NAMES)
        self.current_efforts = [0.0] * len(JOINT_NAMES)

        # Velocity and acceleration calculation cache
        self.prev_calc_time = time.monotonic()
        self.prev_calc_qd = np.zeros(len(JOINT_NAMES))

        # Forward dynamics real-time simulation state
        self.sim_active = False
        self.sim_q = np.zeros(len(JOINT_NAMES))
        self.sim_qd = np.zeros(len(JOINT_NAMES))
        self.sim_tau = np.zeros(len(JOINT_NAMES))
        self.dynamics_visible = True

        self.setWindowTitle("DENSO VS-6556 - BẢNG ĐIỀU KHIỂN & GIÁM SÁT ĐỘNG LỰC HỌC (ROS 2)")
        self.resize(1240, 820)
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

        # Trajectory recording state
        self.traj_recording = False
        self.traj_waypoints = []   # list of (time_sec, [5 angles rad])
        self.traj_rec_start = 0.0
        self.traj_play_thread = None

        # Status latency timer (1Hz update for connection badge)
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status_badge)
        self.status_timer.start(1000)
        # Cartesian IK Link 5 Control State
        self.last_ik_sol = None
        self.ik_target_pos = [15.0, 0.0, 900.0]

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(15, 12, 15, 12)
        main_layout.setSpacing(10)

        # --- HEADER ---
        header_layout = QHBoxLayout()
        title_box = QVBoxLayout()
        title_label = QLabel("DENSO VS-6556 DYNAMICS MONITOR & CONTROL")
        title_label.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title_label.setStyleSheet("color: #00e5ff; letter-spacing: 1px;")
        sub_label = QLabel("Bảng Giám Sát Thời Gian Thực & Phân Tích Động Lực Học Thuận/Nghịch tại 5 Khớp Robot (ROS 2)")
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
        connect_signal(self.btn_toggle_dynamics.clicked, self.on_toggle_dynamics)
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

        # Tab 4: Động học D-H & Tọa độ điểm cuối (Kinematics & D-H Matrix)
        tab_dh = self.create_dh_panel()
        self.tabs.addTab(tab_dh, "4. ĐỘNG HỌC D-H & TỌA ĐỘ TCP")

        # Tab 5: Điều Khiển Tay Trực Tiếp, Ghi/Phát Quỹ Đạo & Workspace Visualizer
        tab_manual = self.create_manual_panel()
        self.tabs.addTab(tab_manual, "5. ĐIỀU KHIỂN THỦ CÔNG & QUỸ ĐẠO")

        # Tab 6: Điều Khiển Tọa Độ Link 5 (Inverse Kinematics)
        tab_ik = self.create_ik_cartesian_panel()
        self.tabs.addTab(tab_ik, "6. ĐIỀU KHIỂN TỌA ĐỘ LINK 5 (IK)")

        main_layout.addWidget(self.tabs, 1)

        # --- FOOTER / LOG BAR ---
        self.log_label = QLabel("Hệ thống sẵn sàng. Dữ liệu động lực học được cập nhật thời gian thực ở chu kỳ 10Hz.")
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
        self.card_status_box, self.card_status_val, self.card_status_sub = make_card(
            "TRẠNG THÁI HỆ THỐNG (STATUS)", "● AN TOÀN", "Tất cả khớp hoạt động <50% định mức", "#059669"
        )

        cards_layout.addWidget(self.card_tau_box)
        cards_layout.addWidget(self.card_load_box)
        cards_layout.addWidget(self.card_payload_box)
        cards_layout.addWidget(self.card_status_box)
        layout.addLayout(cards_layout)

        # 2. Main Telemetry Table (White Background)
        table_container = QGroupBox("BẢNG THEO DÕI ĐỘNG LỰC HỌC 5 KHỚP THỜI GIAN THỰC")
        table_container.setFont(QFont("Segoe UI", 11, QFont.Bold))
        table_container.setStyleSheet("""
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                margin-top: 16px;
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
        tbl_layout = QVBoxLayout(table_container)
        tbl_layout.setContentsMargins(6, 12, 6, 6)

        self.telemetry_table = QTableWidget(len(JOINT_NAMES), 9)
        self.telemetry_table.setHorizontalHeaderLabels([
            "Khớp", "Trục & Vai Trò", "Vị Trí Góc q", "Vận Tốc q̇", "Gia Tốc q̈",
            "Mô-Men τ (N·m)", "Giới Hạn τ_max", "Tải Động Cơ (%)", "Trạng Thái"
        ])
        h_header = self.telemetry_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
            h_header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(6, QHeaderView.ResizeToContents)
            h_header.setSectionResizeMode(7, QHeaderView.Fixed)
            h_header.setSectionResizeMode(8, QHeaderView.Fixed)
        self.telemetry_table.setColumnWidth(7, 165)
        self.telemetry_table.setColumnWidth(8, 135)
        v_header = self.telemetry_table.verticalHeader()
        if v_header is not None:
            v_header.setVisible(False)
            v_header.setDefaultSectionSize(48)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.telemetry_table.setFixedHeight(345)
        self.telemetry_table.setAlternatingRowColors(True)

        for row in range(len(JOINT_NAMES)):
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

        for row in range(len(JOINT_NAMES)):
            # 0: Joint Name
            it_name = QTableWidgetItem(f"Joint {row+1}")
            it_name.setFont(QFont("Segoe UI", 12, QFont.Bold))
            it_name.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 0, it_name)

            # 1: Axis & Role
            it_axis = QTableWidgetItem(JOINT_DESCS[row])
            it_axis.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            self.telemetry_table.setItem(row, 1, it_axis)

            # 2: Position q
            it_q = QTableWidgetItem("0.0° (0.000 rad)")
            it_q.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 2, it_q)

            # 3: Velocity qd
            it_qd = QTableWidgetItem("0.0°/s (0.000 rad/s)")
            it_qd.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 3, it_qd)

            # 4: Acceleration qdd
            it_qdd = QTableWidgetItem("0.0°/s² (0.00 rad/s²)")
            it_qdd.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.telemetry_table.setItem(row, 4, it_qdd)

            # 5: Torque tau
            it_tau = QTableWidgetItem("0.00 N·m")
            it_tau.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            it_tau.setFont(QFont("Segoe UI", 12, QFont.Bold))
            self.telemetry_table.setItem(row, 5, it_tau)

            # 6: Torque Limit
            it_lim = QTableWidgetItem(f"{TORQUE_LIMITS[row]:.0f} N·m")
            it_lim.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            it_lim.setForeground(QColor("#64748b"))
            self.telemetry_table.setItem(row, 6, it_lim)

            # 7: Motor Load Gauge Widget (with ample vertical space)
            pb_container = QWidget()
            pb_container.setStyleSheet("background-color: transparent;")
            pb_box = QHBoxLayout(pb_container)
            pb_box.setContentsMargins(8, 0, 8, 0)
            pb_box.setSpacing(10)
            pb_box.setAlignment(Qt.AlignmentFlag.AlignVCenter)

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
            lbl_pct.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

            pb_box.addWidget(pb)
            pb_box.addWidget(lbl_pct)
            self.telemetry_table.setCellWidget(row, 7, pb_container)
            self.telemetry_progress_bars.append((pb, lbl_pct))

            # 8: Status Badge (Crisp, centered pill badge with ample row height)
            badge = QLabel("● AN TOÀN")
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
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
            badge_box.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge_box.addWidget(badge)
            self.telemetry_table.setCellWidget(row, 8, badge_container)
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
                margin-top: 16px;
                padding: 14px 14px 10px 14px;
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
        connect_signal(btn_demo.clicked, self.on_demo_all)
        ctrl_layout.addWidget(btn_demo)

        btn_once = QPushButton("⚡ Chạy 1 Chiều")
        btn_once.setStyleSheet("background-color: #475569; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_once.clicked, self.on_once_all)
        ctrl_layout.addWidget(btn_once)

        btn_stop = QPushButton("■ Dừng Robot")
        btn_stop.setStyleSheet("background-color: #dc2626; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_stop.clicked, self.on_stop_all)
        ctrl_layout.addWidget(btn_stop)

        btn_home = QPushButton("⌂ Về Gốc (Home 0°)")
        btn_home.setStyleSheet("background-color: #059669; color: white; padding: 7px 14px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_home.clicked, self.on_home_all)
        ctrl_layout.addWidget(btn_home)

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

        self.id_input_table = QTableWidget(len(JOINT_NAMES), 4)
        self.id_input_table.setHorizontalHeaderLabels(["Khớp", "q (độ)", "q̇ (rad/s)", "q̈ (rad/s²)"])
        h_header = self.id_input_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        v_header = self.id_input_table.verticalHeader()
        if v_header is not None:
            v_header.setVisible(False)
        self.id_input_table.setFixedHeight(210)
        self.id_input_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

        for i in range(len(JOINT_NAMES)):
            it0 = QTableWidgetItem(f"Joint {i+1}")
            it0.setFlags(Qt.ItemFlag.ItemIsEnabled)
            it0.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.id_input_table.setItem(i, 0, it0)
            for c in range(1, 4):
                it = QTableWidgetItem("0.0")
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.id_input_table.setItem(i, c, it)
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
        connect_signal(btn_sync.clicked, self.on_id_sync_robot)
        left_layout.addWidget(btn_sync)

        btn_grav_comp = QPushButton("⚖️ Cân Bằng Trọng Lực Tĩnh (q̇=0, q̈=0)")
        btn_grav_comp.setStyleSheet("background-color: #0284c7; color: white; padding: 7px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_grav_comp.clicked, self.on_id_gravity_comp)
        left_layout.addWidget(btn_grav_comp)

        self.btn_calc_id = QPushButton("⚡ TÍNH TOÁN MÔ-MEN XOẮN (INVERSE DYNAMICS)")
        self.btn_calc_id.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_calc_id.setStyleSheet("background-color: #059669; color: white; padding: 10px; border-radius: 6px; font-weight: bold;")
        connect_signal(self.btn_calc_id.clicked, self.on_compute_id)
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

        self.id_res_table = QTableWidget(len(JOINT_NAMES), 7)
        self.id_res_table.setHorizontalHeaderLabels([
            "Khớp", "Quán tính (M·q̈)", "Coriolis (C·q̇)", "Trọng lực (g)", "Tải trọng", "Tổng τ (N·m)", "% Tải"
        ])
        h_header = self.id_res_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        v_header = self.id_res_table.verticalHeader()
        if v_header is not None:
            v_header.setVisible(False)
        self.id_res_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.id_res_table.setFixedHeight(210)
        self.id_res_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

        for i in range(len(JOINT_NAMES)):
            for c in range(7):
                item = QTableWidgetItem("-" if c > 0 else f"Joint {i+1}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
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
        for i in range(len(JOINT_NAMES)):
            max_limit = TORQUE_LIMITS[i] if i < len(TORQUE_LIMITS) else 30.0
            bar_grid.addWidget(QLabel(f"J{i+1} (Max {max_limit:.0f}Nm):"), i, 0)
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
        for i in range(len(JOINT_NAMES)):
            max_limit = TORQUE_LIMITS[i] if i < len(TORQUE_LIMITS) else 30.0
            h = QHBoxLayout()
            h.addWidget(QLabel(f"Joint {i+1} (Max {max_limit:.0f} N·m):"))
            sp = QDoubleSpinBox()
            sp.setRange(-float(max_limit), float(max_limit))
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
        connect_signal(btn_zero.clicked, self.on_fd_zero_torques)
        btn_h.addWidget(btn_zero)

        btn_grav = QPushButton("⚖️ Giữ Cân Bằng Trọng Lực")
        btn_grav.setStyleSheet("background-color: #0284c7; color: white; padding: 6px; border-radius: 6px;")
        connect_signal(btn_grav.clicked, self.on_fd_set_gravity_torques)
        btn_h.addWidget(btn_grav)
        left_layout.addLayout(btn_h)

        self.btn_calc_fd = QPushButton("⚙️ TÍNH TOÁN GIA TỐC GÓC q̈ (FORWARD DYNAMICS)")
        self.btn_calc_fd.setFont(QFont("Segoe UI", 11, QFont.Bold))
        self.btn_calc_fd.setStyleSheet("background-color: #059669; color: white; padding: 10px; border-radius: 6px; font-weight: bold;")
        connect_signal(self.btn_calc_fd.clicked, self.on_compute_fd)
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
        connect_signal(self.btn_start_sim.clicked, self.on_start_simulation)
        sim_btn_h.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton("⏹️ Dừng Mô Phỏng")
        self.btn_stop_sim.setStyleSheet("background-color: #dc2626; color: white; padding: 7px; border-radius: 6px; font-weight: bold;")
        self.btn_stop_sim.setEnabled(False)
        connect_signal(self.btn_stop_sim.clicked, self.on_stop_simulation)
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

        self.fd_res_table = QTableWidget(len(JOINT_NAMES), 4)
        self.fd_res_table.setHorizontalHeaderLabels(["Khớp", "Gia tốc q̈ (rad/s²)", "Gia tốc q̈ (°/s²)", "Chiều Gia Tốc"])
        h_header = self.fd_res_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        v_header = self.fd_res_table.verticalHeader()
        if v_header is not None:
            v_header.setVisible(False)
        self.fd_res_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.fd_res_table.setFixedHeight(210)
        self.fd_res_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0;
                border: 1px solid #cbd5e1; border-radius: 4px; font-size: 13px;
            }
            QHeaderView::section {
                background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0;
            }
        """)

        for i in range(len(JOINT_NAMES)):
            texts = [f"Joint {i+1}", "0.000", "0.00°/s²", "Cân bằng"]
            for c, text in enumerate(texts):
                it = QTableWidgetItem(text)
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.fd_res_table.setItem(i, c, it)
        right_layout.addWidget(self.fd_res_table)

        lbl_m = QLabel(f"Ma trận quán tính M(q) {len(JOINT_NAMES)}×{len(JOINT_NAMES)} (Inertia Matrix):")
        lbl_m.setStyleSheet("color: #8b5cf6; font-weight: bold; margin-top: 5px;")
        right_layout.addWidget(lbl_m)

        self.mass_table = QTableWidget(len(JOINT_NAMES), len(JOINT_NAMES))
        self.mass_table.setHorizontalHeaderLabels([f"M{i+1}" for i in range(len(JOINT_NAMES))])
        self.mass_table.setVerticalHeaderLabels([f"J{i+1}" for i in range(len(JOINT_NAMES))])
        h_header = self.mass_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        self.mass_table.setFixedHeight(180)
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

        for r in range(len(JOINT_NAMES)):
            for c in range(len(JOINT_NAMES)):
                item = QTableWidgetItem("0.0000")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.mass_table.setItem(r, c, item)
        right_layout.addWidget(self.mass_table)

        right_layout.addStretch()
        layout.addWidget(right_box, 1)

        return panel

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

        payload = self.sp_tab1_payload.value()
        res = self.dynamics_engine.inverse_dynamics(q, qd, qdd, payload_mass=payload)
        tau = res['tau']
        pct_load = res['percent_load']

        max_tau_val = 0.0
        max_load_pct = 0.0

        for i in range(len(JOINT_NAMES)):
            t_val = float(tau[i])
            pct = float(pct_load[i])
            if abs(t_val) > max_tau_val:
                max_tau_val = abs(t_val)
            if pct > max_load_pct:
                max_load_pct = pct

            # Col 2: Position q (Degrees first)
            q_deg = np.rad2deg(q[i])
            if abs(q_deg) < 0.05:
                q_deg = 0.0
            q_val = 0.0 if abs(q[i]) < 0.0005 else q[i]
            set_table_item_text(self.telemetry_table, i, 2, f"{q_deg:+.1f}° ({q_val:+.2f} rad)")

            # Col 3: Velocity qd
            qd_deg = np.rad2deg(qd[i])
            if abs(qd_deg) < 0.05:
                qd_deg = 0.0
            qd_val = 0.0 if abs(qd[i]) < 0.0005 else qd[i]
            set_table_item_text(self.telemetry_table, i, 3, f"{qd_deg:+.1f}°/s ({qd_val:+.2f} rad/s)")

            # Col 4: Acceleration qdd
            qdd_deg = np.rad2deg(qdd[i])
            if abs(qdd_deg) < 0.05:
                qdd_deg = 0.0
            qdd_val = 0.0 if abs(qdd[i]) < 0.0005 else qdd[i]
            set_table_item_text(self.telemetry_table, i, 4, f"{qdd_deg:+.1f}°/s² ({qdd_val:+.1f} rad/s²)")

            # Col 5: Torque tau (Prevent -0.00 N·m glitch!)
            if abs(t_val) < 0.005:
                tau_str = "0.00 N·m"
            else:
                tau_str = f"{t_val:+.2f} N·m"
            if pct < 50.0:
                tau_col = QColor("#047857")  # Dark emerald
            elif pct < 80.0:
                tau_col = QColor("#b45309")  # Dark amber
            else:
                tau_col = QColor("#b91c1c")  # Dark red
            set_table_item_text(self.telemetry_table, i, 5, tau_str, tau_col)

            # Col 7: Progress bar & percent
            pb, lbl_pct = self.telemetry_progress_bars[i]
            val = int(min(100.0, max(0.0, float(pct))))
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

            # Col 8: Status Badge (Crisp, fully legible, no emoji squishing)
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

    # ---------------------------------------------------------
    # TAB 2 & 3 CALLBACKS
    # ---------------------------------------------------------
    def on_id_sync_robot(self):
        for i in range(len(JOINT_NAMES)):
            set_table_item_text(self.id_input_table, i, 1, f"{np.rad2deg(self.current_joints[i]):.1f}")
            set_table_item_text(self.id_input_table, i, 2, f"{self.current_velocities[i]:.3f}")
            set_table_item_text(self.id_input_table, i, 3, "0.00")
        self.log_label.setText("📌 Đã đồng bộ vị trí và vận tốc hiện tại từ Robot vào Động Lực Học Nghịch.")

    def on_id_gravity_comp(self):
        for i in range(len(JOINT_NAMES)):
            set_table_item_text(self.id_input_table, i, 2, "0.00")
            set_table_item_text(self.id_input_table, i, 3, "0.00")
        self.on_compute_id()
        self.log_label.setText("⚖️ Đã tính toán mô-men bù trọng lực tĩnh (Gravity Compensation).")

    def on_compute_id(self):
        try:
            q = np.array([np.deg2rad(float(get_table_item_text(self.id_input_table, i, 1))) for i in range(len(JOINT_NAMES))])
            qd = np.array([float(get_table_item_text(self.id_input_table, i, 2)) for i in range(len(JOINT_NAMES))])
            qdd = np.array([float(get_table_item_text(self.id_input_table, i, 3)) for i in range(len(JOINT_NAMES))])
        except ValueError:
            self.log_label.setText("❌ Lỗi định dạng số trong bảng nhập thông số khớp!")
            return

        payload = self.sp_payload.value()
        fz = self.sp_fz.value()
        f_ext = np.array([0.0, 0.0, fz]) if abs(fz) > 1e-3 else None

        res = self.dynamics_engine.inverse_dynamics(q, qd, qdd, payload_mass=payload, f_ext=f_ext)

        for i in range(len(JOINT_NAMES)):
            set_table_item_text(self.id_res_table, i, 1, f"{res['M_qdd'][i]:+.2f}")
            set_table_item_text(self.id_res_table, i, 2, f"{res['C_qd'][i]:+.2f}")
            set_table_item_text(self.id_res_table, i, 3, f"{res['g'][i]:+.2f}")
            set_table_item_text(self.id_res_table, i, 4, f"{res['payload'][i]:+.2f}")
            set_table_item_text(self.id_res_table, i, 5, f"{res['tau'][i]:+.2f}")
            pct = res['percent_load'][i]
            set_table_item_text(self.id_res_table, i, 6, f"{pct:.1f}%")

            # Update progress bar
            pb = self.load_bars[i]
            val = int(min(100.0, float(pct)))
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
            markers = self.dynamics_engine.build_marker_array(q, res['tau'], qdd, frame_id="base_link")
            self.bridge.publish_markers(markers)

        self.log_label.setText(f"⚡ Đã tính xong Động Lực Học Nghịch. Tổng Torque = {np.round(res['tau'], 2)} N·m")

    def on_fd_zero_torques(self):
        for sp in self.fd_spinboxes:
            sp.setValue(0.0)

    def on_fd_set_gravity_torques(self):
        g_tau = self.dynamics_engine.gravity_compensation(self.current_joints)
        for i in range(len(JOINT_NAMES)):
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

        for i in range(len(JOINT_NAMES)):
            rad_s2 = qdd[i]
            deg_s2 = np.rad2deg(rad_s2)
            set_table_item_text(self.fd_res_table, i, 1, f"{rad_s2:+.3f}")
            set_table_item_text(self.fd_res_table, i, 2, f"{deg_s2:+.2f}°/s²")
            if abs(rad_s2) < 0.05:
                direction = "Cân bằng"
            elif rad_s2 > 0:
                direction = "Gia tốc dương (+)"
            else:
                direction = "Gia tốc âm (-)"
            set_table_item_text(self.fd_res_table, i, 3, direction)

        for r in range(len(JOINT_NAMES)):
            for c in range(len(JOINT_NAMES)):
                set_table_item_text(self.mass_table, r, c, f"{M[r, c]:.4f}")

        markers = self.dynamics_engine.build_marker_array(q, tau, qdd, frame_id="base_link")
        self.bridge.publish_markers(markers)

        self.log_label.setText(f"⚙️ Đã giải xong Động Lực Học Thuận: q̈ = {np.round(qdd, 3)} rad/s²")

    def on_start_simulation(self):
        self.sim_active = True
        self.sim_q = np.array(self.current_joints, dtype=float)
        self.sim_qd = np.zeros(len(JOINT_NAMES))
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

        cmd = "goto " + " ".join(f"{val:.4f}" for val in self.sim_q)
        self.bridge.send_cmd(cmd)

        for i in range(len(JOINT_NAMES)):
            set_table_item_text(self.fd_res_table, i, 1, f"{qdd[i]:+.3f}")
            set_table_item_text(self.fd_res_table, i, 2, f"{np.rad2deg(qdd[i]):+.2f}°/s²")

        markers = self.dynamics_engine.build_marker_array(self.sim_q, tau, qdd, frame_id="base_link")
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
        self.update_dh_tab()
        self.update_ik_tab()

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


    # ---------------------------------------------------------
    # SYNCHRONIZED ACTION HANDLERS (RVIZ + GAZEBO)
    # ---------------------------------------------------------
    def on_home_all(self):
        self.bridge.send_cmd("home")
        self.bridge.send_trajectory([0.0] * len(JOINT_NAMES), duration_sec=1.5)
        self.log_label.setText("⌂ Đang đưa robot về Home 0° (Đồng bộ RViz2 & Gazebo)...")

    def on_stop_all(self):
        self.bridge.send_cmd("stop")
        # Hold current position in Gazebo
        self.bridge.send_trajectory(list(self.current_joints), duration_sec=0.2)
        self.log_label.setText("■ Đã dừng giữ robot tại vị trí hiện tại.")

    def on_demo_all(self):
        self.bridge.send_cmd("demo")
        self.log_label.setText("▶ Bắt đầu chu trình chạy lặp Link 1 -> Link 6 (Đồng bộ RViz2 & Gazebo).")
        self._start_gazebo_sequence()

    def on_once_all(self):
        self.bridge.send_cmd("once")
        self.log_label.setText("⚡ Chạy 1 chiều Link 1 -> Link 6 rồi giữ vị trí.")
        self._start_gazebo_sequence(loop=False)

    def _start_gazebo_sequence(self, loop=True):
        seq = [
            ([0.785, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0),
            ([0.785, 0.785, 0.0, 0.0, 0.0, 0.0], 2.0),
            ([0.785, 0.785, 0.785, 0.0, 0.0, 0.0], 2.0),
            ([0.785, 0.785, 0.785, 1.57, 0.0, 0.0], 2.0),
            ([0.785, 0.785, 0.785, 1.57, 0.87, 0.0], 2.0),
            ([0.785, 0.785, 0.785, 1.57, 0.87, 1.57], 2.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2.5),
        ]
        def run_seq():
            while True:
                for target, dur in seq:
                    self.bridge.send_trajectory(target, dur)
                    time.sleep(dur + 1.0)
                if not loop:
                    break
        threading.Thread(target=run_seq, daemon=True).start()

    def send_pose_sync(self, target_joints, name=""):
        self.bridge.send_trajectory(target_joints, duration_sec=1.5)
        # Also publish manual angles to mock controller if needed
        self.log_label.setText(f"🎯 Đã gửi tư thế '{name}' tới Robot (Đồng bộ RViz2 & Gazebo)...")

    # ---------------------------------------------------------
    # TAB 4: KINEMATICS D-H & TCP COORDINATES
    # ---------------------------------------------------------
    def create_dh_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # 1. Top KPI Summary Cards (White Background)
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
            lbl_v.setStyleSheet(f"color: {val_color}; font-size: 18px; font-weight: bold; font-family: 'Segoe UI', monospace;")
            lbl_s = QLabel(subtext)
            lbl_s.setStyleSheet("color: #94a3b8; font-size: 11px;")
            v.addWidget(lbl_t)
            v.addWidget(lbl_v)
            v.addWidget(lbl_s)
            return box, lbl_v, lbl_s

        self.dh_card_xyz_box, self.dh_card_xyz_val, self.dh_card_xyz_sub = make_dh_card(
            "TỌA ĐỘ POINT P & MẶT BÍCH (X, Y, Z)", "P: [+15, 0, +900] mm", "Mặt bích: [+15, 0, +980] mm", "#0284c7"
        )
        self.dh_card_reach_box, self.dh_card_reach_val, self.dh_card_reach_sub = make_dh_card(
            "TẦM VƯƠN R (MAX R=653.42 mm)", "R_p: 15.0 / 653.4 mm", "Tầm vươn: 2.3% | R_flange: 980.1 mm", "#10b981"
        )
        self.dh_card_rpy_box, self.dh_card_rpy_val, self.dh_card_rpy_sub = make_dh_card(
            "GÓC EULER ROLL-PITCH-YAW", "R: 0.0° | P: 0.0° | Y: 0.0°", "Hướng vector trục mặt bích gá", "#8b5cf6"
        )
        self.dh_card_ws_box, self.dh_card_ws_val, self.dh_card_ws_sub = make_dh_card(
            "VÙNG LÀM VIỆC (WORKSPACE ENVELOPE)", "● TRONG VÙNG", "Trạng thái an toàn chuẩn Denso", "#059669"
        )

        cards_layout.addWidget(self.dh_card_xyz_box)
        cards_layout.addWidget(self.dh_card_reach_box)
        cards_layout.addWidget(self.dh_card_rpy_box)
        cards_layout.addWidget(self.dh_card_ws_box)
        layout.addLayout(cards_layout)

        # 2. Main Middle Splitter: Left = DH Table, Right = 4x4 Homogeneous Transform
        mid_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left Box: Extended DH Parameters Table matching RViz URDF
        dh_box = QGroupBox("📋 BẢNG THAM SỐ ĐỘNG HỌC D-H MỞ RỘNG (EXTENDED D-H - CHUẨN XÁC TỪNG KHÂU RVIZ)")
        dh_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        dh_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        dh_layout = QVBoxLayout(dh_box)
        dh_layout.setContentsMargins(10, 14, 10, 10)

        self.dh_table = QTableWidget(7, 8)
        self.dh_table.setHorizontalHeaderLabels([
            "Khâu (Link)", "Liên kết TF", "Góc θi (Live)", "di (m)", "ai (m)", "bi (m) [Lệch Y]", "Trục quay", "Tọa độ RViz [X, Y, Z] (mm)"
        ])
        h_header = self.dh_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        self.dh_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.dh_table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                color: #0f172a;
                gridline-color: #e2e8f0;
                font-size: 12px;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
            }
            QHeaderView::section {
                background-color: #f1f5f9;
                color: #0f172a;
                font-weight: bold;
                padding: 6px;
                border: 1px solid #cbd5e1;
            }
        """)

        # Extended D-H nominal values for Denso VS-6556 matching ROS 2 URDF & RViz
        DH_NOMINALS = [
            ("link_1", "base_link -> link_1", "0.0°", "0.2050", "0.0000", "0.0000", "Z (Yaw)",    "[   0.0,    0.0, +205.0]"),
            ("link_2", "link_1 -> link_2",    "0.0°", "0.1300", "-0.0750", "-0.0745", "-Y (Pitch)", "[ -75.0,  -74.5, +335.0]"),
            ("link_3", "link_2 -> link_3",    "0.0°", "0.2700", "0.0000", "+0.0065", "-Y (Pitch)", "[ -75.0,  -68.0, +605.0]"),
            ("link_4", "link_3 -> link_4",    "0.0°", "0.1090", "+0.0900", "+0.0680", "Z (Roll)",   "[ +15.0,    0.0, +714.0]"),
            ("link_5", "link_4 -> link_5",    "0.0°", "0.1860", "0.0000", "0.0000", "-Y (Pitch)", "[ +15.0,    0.0, +900.0]"),
            ("link_6", "link_5 -> link_6",    "0.0°", "0.0620", "0.0000", "0.0000", "Z (Roll)",   "[ +15.0,    0.0, +962.0]"),
            ("flange", "link_6 -> flange",    "0.0°", "0.0280", "0.0000", "0.0000", "Z (TCP)",    "[ +15.0,    0.0, +990.0]"),
        ]
        for r, row_data in enumerate(DH_NOMINALS):
            for c, val in enumerate(row_data):
                it = QTableWidgetItem(val)
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.dh_table.setItem(r, c, it)
        dh_layout.addWidget(self.dh_table)

        # Drawing Specs Reference Footer
        spec_lbl = QLabel("📐 <b>Ma trận D-H Mở Rộng:</b> Tích hợp độ lệch phương ngang <b>b</b> (trục Y) của cụm động cơ vai (-74.5mm) & khuỷu (+6.5mm), đưa tọa độ từng link khớp 100% với RViz/TF Tree.")
        spec_lbl.setStyleSheet("color: #475569; font-size: 11px; padding: 4px; background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 4px;")
        dh_layout.addWidget(spec_lbl)
        mid_splitter.addWidget(dh_box)

        # Right Box: 4x4 Homogeneous Transformation Matrix
        tf_box = QGroupBox("📐 MA TRẬN BIẾN ĐỔI ĐỒNG NHẤT 4x4 (HOMOGENEOUS TRANSFORM)")
        tf_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        tf_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        tf_layout = QVBoxLayout(tf_box)
        tf_layout.setContentsMargins(10, 14, 10, 10)

        sel_h = QHBoxLayout()
        sel_h.addWidget(QLabel("Chọn mắt xích:"))
        self.combo_matrix = QComboBox()
        self.combo_matrix.addItems([
            "Ma trận tích lũy T0_Flange (Base -> Flange TCP: Z=990mm)",
            "Ma trận tích lũy T0_link6 (Base -> Link 6 Cổ tay xoay: Z=962mm)",
            "Ma trận tích lũy T0_link5 (Base -> Link 5 / Tâm cổ tay Point P: Z=900mm)",
            "Ma trận tích lũy T0_link4 (Base -> Link 4 Cẳng tay: Z=714mm)",
            "Ma trận tích lũy T0_link3 (Base -> Link 3 Khớp khuỷu: Z=605mm)",
            "Ma trận tích lũy T0_link2 (Base -> Link 2 Khớp vai: Z=335mm)",
            "Ma trận tích lũy T0_link1 (Base -> Link 1 Mặt bích J1: Z=205mm)",
            "Ma trận cục bộ D-H H_link1 (Base -> Link 1: d=0.205m)",
            "Ma trận cục bộ D-H H_link2 (Link 1 -> Link 2: a=-0.075, b=-0.0745, d=0.130)",
            "Ma trận cục bộ D-H H_link3 (Link 2 -> Link 3: b=+0.0065, d=0.270)",
            "Ma trận cục bộ D-H H_link4 (Link 3 -> Link 4: a=+0.090, b=+0.068, d=0.109)",
            "Ma trận cục bộ D-H H_link5 (Link 4 -> Link 5: d=0.186m)",
            "Ma trận cục bộ D-H H_link6 (Link 5 -> Link 6: d=0.06204m)",
            "Ma trận cục bộ D-H H_flange (Link 6 -> Flange TCP: d=0.028m)",
        ])
        self.combo_matrix.setStyleSheet("background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; padding: 4px; font-weight: bold;")
        sel_h.addWidget(self.combo_matrix)
        sel_h.addStretch()
        tf_layout.addLayout(sel_h)

        self.matrix_table = QTableWidget(4, 4)
        self.matrix_table.setHorizontalHeaderLabels(["nx (Col 1)", "oy (Col 2)", "az (Col 3)", "p (Col 4)"])
        self.matrix_table.setVerticalHeaderLabels(["X", "Y", "Z", "1"])
        h_header = self.matrix_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        v_header = self.matrix_table.verticalHeader()
        if v_header is not None:
            v_header.setSectionResizeMode(QHeaderView.Stretch)
        self.matrix_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.matrix_table.setStyleSheet("""
            QTableWidget {
                background-color: #f8fafc;
                color: #0f172a;
                gridline-color: #cbd5e1;
                font-family: monospace;
                font-size: 13px;
                font-weight: bold;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
            }
            QHeaderView::section {
                background-color: #e2e8f0;
                color: #0f172a;
                font-weight: bold;
                border: 1px solid #cbd5e1;
            }
        """)
        for r in range(4):
            for c in range(4):
                val = "1.0000" if r == c else "0.0000"
                it = QTableWidgetItem(val)
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.matrix_table.setItem(r, c, it)
        tf_layout.addWidget(self.matrix_table)
        mid_splitter.addWidget(tf_box)

        layout.addWidget(mid_splitter, 1)

        # 3. Bottom Box: Quick Preset Poses (Sync RViz2 & Gazebo)
        preset_box = QGroupBox("🎯 TƯ THẾ MẪU CHUẨN ĐỒNG BỘ RVIZ2 & GAZEBO (QUICK POSES)")
        preset_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        preset_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        p_layout = QHBoxLayout(preset_box)
        p_layout.setContentsMargins(10, 10, 10, 10)
        p_layout.setSpacing(10)

        poses = [
            ("⌂ Home (0°)", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "#059669"),
            ("🎯 Sẵn Sàng (Ready)", [0.0, 0.5, 0.8, 0.0, -0.5, 0.0], "#0284c7"),
            ("📏 Vươn Ngang 90°", [0.0, 0.785, 0.785, 0.0, 0.0, 0.0], "#475569"),
            ("📐 Gập Khuỷu 90°", [0.0, 0.0, 1.57, 0.0, 0.0, 0.0], "#64748b"),
            ("🔄 Gắp Thấp (Pick Low)", [0.0, 1.05, 0.25, 0.0, -0.7, 0.0], "#7c3aed"),
        ]
        for title, joints, color in poses:
            btn = QPushButton(title)
            btn.setStyleSheet(f"background-color: {color}; color: white; padding: 8px 14px; border-radius: 6px; font-weight: bold;")
            connect_signal(btn.clicked, lambda ch, j=joints, n=title: self.send_pose_sync(j, n))
            p_layout.addWidget(btn)

        layout.addWidget(preset_box)
        return panel

    def update_dh_tab(self):
        q = np.array(self.current_joints, dtype=float)
        # Computes forward kinematics directly via verified Extended D-H chain (RViz URDF matched)
        ext_transforms, ext_matrices_H = self.dynamics_engine.get_extended_dh_transforms(q)
        point_P_T = ext_transforms[5]   # Link 5 (Point P)
        flange_T  = ext_transforms[6]   # Flange TCP

        # Point P Position (mm) - Center of J4, J5, J6
        p_x = point_P_T[0, 3] * 1000.0
        p_y = point_P_T[1, 3] * 1000.0
        p_z = point_P_T[2, 3] * 1000.0
        r_xy_p = math.sqrt(p_x**2 + p_y**2)
        reach_R_p = math.sqrt(p_x**2 + p_y**2 + p_z**2)

        # Flange Position (mm) - View A Mounting Face (+80mm)
        f_x = flange_T[0, 3] * 1000.0
        f_y = flange_T[1, 3] * 1000.0
        f_z = flange_T[2, 3] * 1000.0
        r_xy_f = math.sqrt(f_x**2 + f_y**2)
        reach_R_f = math.sqrt(f_x**2 + f_y**2 + f_z**2)

        # Euler angles from Flange rotation matrix
        R = flange_T[:3, :3]
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        if sy > 1e-6:
            roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
            pitch = math.degrees(math.atan2(-R[2, 0], sy))
            yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        else:
            roll = math.degrees(math.atan2(-R[1, 2], R[1, 1]))
            pitch = math.degrees(math.atan2(-R[2, 0], sy))
            yaw = 0.0

        # Update Top KPI Cards
        self.dh_card_xyz_val.setText(f"P: [{p_x:+.0f}, {p_y:+.0f}, {p_z:+.0f}] mm")
        self.dh_card_xyz_sub.setText(f"Mặt bích: [{f_x:+.0f}, {f_y:+.0f}, {f_z:+.0f}] mm")

        pct_reach = (r_xy_p / 653.42) * 100.0
        self.dh_card_reach_val.setText(f"R_p: {r_xy_p:5.1f} / 653.4 mm")
        self.dh_card_reach_sub.setText(f"Tầm vươn: {pct_reach:.1f}% | R_flange: {reach_R_f:.1f} mm")

        self.dh_card_rpy_val.setText(f"R: {roll:+5.1f}° | P: {pitch:+5.1f}° | Y: {yaw:+5.1f}°")

        # Workspace boundary check based on Drawing R653.42 mm
        if r_xy_p > 653.42:
            self.dh_card_ws_val.setText("⛔ VƯỢT QUÁ VÙNG (OVER)")
            self.dh_card_ws_val.setStyleSheet("color: #dc2626; font-size: 16px; font-weight: bold;")
            self.dh_card_ws_sub.setText(f"Vượt giới hạn R653.42mm (+{r_xy_p - 653.42:.1f}mm)")
        elif r_xy_p > 600.0:
            self.dh_card_ws_val.setText("⚠ GẦN BIÊN WORKSPACE")
            self.dh_card_ws_val.setStyleSheet("color: #e11d48; font-size: 16px; font-weight: bold;")
            self.dh_card_ws_sub.setText(f"Cách biên {653.42 - r_xy_p:.1f} mm")
        else:
            self.dh_card_ws_val.setText("● TRONG VÙNG AN TOÀN")
            self.dh_card_ws_val.setStyleSheet("color: #059669; font-size: 16px; font-weight: bold;")
            self.dh_card_ws_sub.setText(f"Tầm vươn an toàn ({pct_reach:.1f}%)")

        # Update Live θ and Live Coordinates in Extended DH table (Rows 0..6 = link_1..flange)
        for i in range(7):
            if i < 6:
                q_deg = math.degrees(q[i])
                if abs(q_deg) < 0.05:
                    q_deg = 0.0
                set_table_item_text(self.dh_table, i, 2, f"q{i+1} = {q_deg:+5.1f}°")
            else:
                set_table_item_text(self.dh_table, i, 2, "Cố định (Fixed)")

            # Live [X, Y, Z] from cumulative transform ext_transforms[i+1] (0 is base_link)
            T_i = ext_transforms[i+1]
            pos_x, pos_y, pos_z = T_i[0, 3] * 1000.0, T_i[1, 3] * 1000.0, T_i[2, 3] * 1000.0
            set_table_item_text(self.dh_table, i, 7, f"[{pos_x:+6.1f}, {pos_y:+6.1f}, {pos_z:+6.1f}]")

        # Update 4x4 Transformation Matrix based on selected dropdown
        c_idx = self.combo_matrix.currentIndex()
        if c_idx == 0:
            selected_T = flange_T
        elif c_idx == 1:
            selected_T = ext_transforms[6]   # link_6
        elif c_idx == 2:
            selected_T = point_P_T           # link_5
        elif c_idx == 3:
            selected_T = ext_transforms[4]   # link_4
        elif c_idx == 4:
            selected_T = ext_transforms[3]   # link_3
        elif c_idx == 5:
            selected_T = ext_transforms[2]   # link_2
        elif c_idx == 6:
            selected_T = ext_transforms[1]   # link_1
        elif c_idx == 7:
            selected_T = ext_matrices_H[0]   # H_link1
        elif c_idx == 8:
            selected_T = ext_matrices_H[1]   # H_link2
        elif c_idx == 9:
            selected_T = ext_matrices_H[2]   # H_link3
        elif c_idx == 10:
            selected_T = ext_matrices_H[3]   # H_link4
        elif c_idx == 11:
            selected_T = ext_matrices_H[4]   # H_link5
        elif c_idx == 12:
            selected_T = ext_matrices_H[5]   # H_link6
        elif c_idx == 13:
            selected_T = ext_matrices_H[6]   # H_flange
        else:
            selected_T = flange_T

        for r in range(4):
            for c in range(4):
                val = selected_T[r, c]
                if abs(val) < 0.0001:
                    val = 0.0
                fmt = f"{val:+7.4f}" if c < 3 else (f"{val*1000:+7.1f} mm" if r < 3 else f"{val:.0f}")
                set_table_item_text(self.matrix_table, r, c, fmt)

    def _update_status_badge(self):
        """Update connection badge with latency and controller mode."""
        try:
            # Latency: time since last /joint_states message
            now_ns = self.bridge.node.get_clock().now().nanoseconds
            last_ns = self.bridge.last_js_time.nanoseconds
            latency_ms = (now_ns - last_ns) / 1e6

            status = self.bridge.latest_status
            mode = status.get('mode', 'unknown')
            step = status.get('demo_step', 0)
            payload = status.get('payload_kg', 0.0)

            if latency_ms < 500:
                mode_str = {'demo': f'DEMO (Bước {step+1}/6)', 'idle': 'IDLE', 'external': 'MOVEIT CTRL'}.get(mode, mode.upper())
                self.status_badge.setText(f"● ROS2 /{mode_str} | {latency_ms:.0f}ms | {payload:.1f}kg")
                self.status_badge.setStyleSheet(
                    "background-color: #1b4332; color: #74c69d; border: 1px solid #40916c; "
                    "border-radius: 6px; padding: 6px 14px;"
                )
            else:
                self.status_badge.setText(f"⚠ MẤT KẾT NỐI ({latency_ms:.0f}ms)")
                self.status_badge.setStyleSheet(
                    "background-color: #7f1d1d; color: #fca5a5; border: 1px solid #dc2626; "
                    "border-radius: 6px; padding: 6px 14px;"
                )
        except Exception:
            pass

    # ---------------------------------------------------------
    # TAB 5: MANUAL JOINT CONTROL + TRAJECTORY RECORDER + WORKSPACE
    # ---------------------------------------------------------
    def create_manual_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(12)

        # ── LEFT: Joint Sliders + Send ──────────────────────────
        left_box = QGroupBox("🎮 ĐIỀU KHIỂN THỦ CÔNG TỪNG KHỚP (MANUAL JOINT CONTROL)")
        left_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        left_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(12, 16, 12, 12)
        left_layout.setSpacing(8)

        JOINT_LABELS = [
            "J1 Base (Z) ±170°", "J2 Vai (Y) -120°~+135°",
            "J3 Khuỷu (Y) -119°~+166°", "J4 Cẳng (Z) ±190°",
            "J5 Cổ Tay (Y) ±120°", "J6 Mặt Bích (Z) ±360°"
        ]
        SLIDER_MINS  = [-170, -120, -119, -190, -120, -360]
        SLIDER_MAXS  = [ 170,  135,  166,  190,  120,  360]

        self.manual_sliders = []
        self.manual_val_labels = []

        for i in range(len(JOINT_NAMES)):
            row = QHBoxLayout()
            lbl = QLabel(JOINT_LABELS[i])
            lbl.setFixedWidth(185)
            lbl.setStyleSheet("color: #0f172a; font-size: 12px; font-weight: bold;")
            row.addWidget(lbl)

            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(SLIDER_MINS[i] * 10, SLIDER_MAXS[i] * 10)
            sl.setValue(0)
            sl.setStyleSheet("""
                QSlider::groove:horizontal { height: 6px; background: #e2e8f0; border-radius: 3px; }
                QSlider::sub-page:horizontal { background: #0284c7; border-radius: 3px; }
                QSlider::handle:horizontal { width: 16px; height: 16px; margin: -5px 0;
                    background: #0284c7; border-radius: 8px; border: 2px solid white; }
            """)
            self.manual_sliders.append(sl)

            val_lbl = QLabel("0.0°")
            val_lbl.setFixedWidth(62)
            val_lbl.setFont(QFont("Segoe UI", 12, QFont.Bold))
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val_lbl.setStyleSheet("color: #0284c7; font-weight: bold;")
            self.manual_val_labels.append(val_lbl)

            # Live update: slider → label → FK preview
            sl.valueChanged.connect(lambda v, idx=i, lbl=val_lbl: self._on_slider_moved(idx, v, lbl))

            row.addWidget(sl, 1)
            row.addWidget(val_lbl)
            left_layout.addLayout(row)

        # Live FK XYZ preview
        self.manual_fk_label = QLabel("TCP: P=[+15, 0, +900] mm  |  R_xy=15mm  |  Roll=0° Pitch=0° Yaw=0°")
        self.manual_fk_label.setStyleSheet(
            "color: #0284c7; font-size: 11px; font-weight: bold; background: #f0f9ff; "
            "border: 1px solid #bae6fd; border-radius: 6px; padding: 6px;"
        )
        self.manual_fk_label.setWordWrap(True)
        left_layout.addWidget(self.manual_fk_label)

        # Duration spinbox
        dur_row = QHBoxLayout()
        dur_row.addWidget(QLabel("Thời gian chạy (giây):"))
        self.manual_dur_spin = QDoubleSpinBox()
        self.manual_dur_spin.setRange(0.3, 10.0)
        self.manual_dur_spin.setValue(2.0)
        self.manual_dur_spin.setSingleStep(0.5)
        self.manual_dur_spin.setDecimals(1)
        self.manual_dur_spin.setStyleSheet("background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; padding: 4px;")
        dur_row.addWidget(self.manual_dur_spin)
        dur_row.addStretch()
        left_layout.addLayout(dur_row)

        # Action buttons
        btn_row = QHBoxLayout()
        btn_sync = QPushButton("🔄 Đồng Bộ Từ Robot")
        btn_sync.setStyleSheet("background-color: #64748b; color: white; padding: 8px 12px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_sync.clicked, self._on_manual_sync)
        btn_row.addWidget(btn_sync)

        btn_zero = QPushButton("⌂ Về 0°")
        btn_zero.setStyleSheet("background-color: #059669; color: white; padding: 8px 12px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_zero.clicked, self._on_manual_zero)
        btn_row.addWidget(btn_zero)

        btn_send = QPushButton("🚀 GỬI TƯ THẾ")
        btn_send.setFont(QFont("Segoe UI", 11, QFont.Bold))
        btn_send.setStyleSheet("background-color: #0284c7; color: white; padding: 10px 16px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_send.clicked, self._on_manual_send)
        btn_row.addWidget(btn_send)
        left_layout.addLayout(btn_row)

        left_layout.addStretch()
        layout.addWidget(left_box, 3)

        # ── RIGHT: Trajectory Recorder ──────────────────────────
        right_box = QGroupBox("🎞️ GHI & PHÁT QUỸ ĐẠO (TRAJECTORY RECORD & PLAYBACK)")
        right_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(12, 16, 12, 12)
        right_layout.setSpacing(8)

        # Record controls
        rec_row = QHBoxLayout()
        self.btn_rec = QPushButton("🔴 BẮT ĐẦU GHI")
        self.btn_rec.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_rec.setStyleSheet("background-color: #dc2626; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        connect_signal(self.btn_rec.clicked, self._on_rec_start)
        rec_row.addWidget(self.btn_rec)

        self.btn_rec_stop = QPushButton("⏹ DỪNG")
        self.btn_rec_stop.setStyleSheet("background-color: #64748b; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        self.btn_rec_stop.setEnabled(False)
        connect_signal(self.btn_rec_stop.clicked, self._on_rec_stop)
        rec_row.addWidget(self.btn_rec_stop)
        right_layout.addLayout(rec_row)

        self.rec_status_label = QLabel("● Chưa ghi. Nhấn BẮT ĐẦU GHI khi robot đang di chuyển.")
        self.rec_status_label.setStyleSheet("color: #64748b; font-size: 11px; padding: 4px;")
        self.rec_status_label.setWordWrap(True)
        right_layout.addWidget(self.rec_status_label)

        # Save/Load/Play controls
        sl_row = QHBoxLayout()
        btn_save = QPushButton("💾 LƯU YAML")
        btn_save.setStyleSheet("background-color: #0284c7; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_save.clicked, self._on_traj_save)
        sl_row.addWidget(btn_save)

        btn_load = QPushButton("📂 NẠP YAML")
        btn_load.setStyleSheet("background-color: #475569; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_load.clicked, self._on_traj_load)
        sl_row.addWidget(btn_load)
        right_layout.addLayout(sl_row)

        play_row = QHBoxLayout()
        self.btn_play = QPushButton("▶ PHÁT LẠI")
        self.btn_play.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.btn_play.setStyleSheet("background-color: #059669; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        connect_signal(self.btn_play.clicked, self._on_traj_play)
        play_row.addWidget(self.btn_play)

        btn_clear = QPushButton("🗑️ XÓA")
        btn_clear.setStyleSheet("background-color: #ef4444; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        connect_signal(btn_clear.clicked, self._on_traj_clear)
        play_row.addWidget(btn_clear)
        right_layout.addLayout(play_row)

        # Trajectory info table
        self.traj_table = QTableWidget(0, 3)
        self.traj_table.setHorizontalHeaderLabels(["#", "Thời Gian (s)", "Vị Trí Khớp (°)"])
        h_header = self.traj_table.horizontalHeader()
        if h_header is not None:
            h_header.setSectionResizeMode(QHeaderView.Stretch)
        self.traj_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.traj_table.setStyleSheet("""
            QTableWidget { background-color: #f8fafc; color: #0f172a; gridline-color: #e2e8f0;
                font-size: 11px; border: 1px solid #cbd5e1; border-radius: 4px; }
            QHeaderView::section { background-color: #f1f5f9; color: #0f172a; font-weight: bold; border: 1px solid #e2e8f0; }
        """)
        self.traj_table.setMaximumHeight(180)
        right_layout.addWidget(self.traj_table)

        right_layout.addStretch()
        layout.addWidget(right_box, 2)

        return panel

    # ---------------------------------------------------------
    # MANUAL JOINT SLIDER CALLBACKS
    # ---------------------------------------------------------
    def _on_slider_moved(self, joint_idx: int, slider_val: int, val_lbl: QLabel):
        deg = slider_val / 10.0
        val_lbl.setText(f"{deg:+.1f}°")
        # Live FK preview
        q = [s.value() / 10.0 for s in self.manual_sliders]
        q_rad = [math.radians(v) for v in q]
        try:
            transforms = self.dynamics_engine.get_joint_transforms(q_rad)
            T_flange = transforms[6]
            px, py, pz = T_flange[0, 3]*1000, T_flange[1, 3]*1000, T_flange[2, 3]*1000
            r_xy = math.sqrt(px**2 + py**2)
            R = T_flange[:3, :3]
            sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
            if sy > 1e-6:
                roll  = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                pitch = math.degrees(math.atan2(-R[2, 0], sy))
                yaw   = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            else:
                roll, pitch, yaw = math.degrees(math.atan2(-R[1, 2], R[1, 1])), math.degrees(math.atan2(-R[2, 0], sy)), 0.0
            self.manual_fk_label.setText(
                f"TCP: P=[{px:+.0f}, {py:+.0f}, {pz:+.0f}] mm  |  R_xy={r_xy:.1f}mm  |  "
                f"Roll={roll:+.1f}°  Pitch={pitch:+.1f}°  Yaw={yaw:+.1f}°"
            )
        except Exception:
            pass

    def _on_manual_sync(self):
        for i, sl in enumerate(self.manual_sliders):
            deg = math.degrees(self.current_joints[i])
            sl.setValue(int(deg * 10))
        self.log_label.setText("🔄 Đã đồng bộ slider từ vị trí robot hiện tại.")

    def _on_manual_zero(self):
        for sl in self.manual_sliders:
            sl.setValue(0)
        self.log_label.setText("⌂ Đã đặt tất cả slider về 0°.")

    def _on_manual_send(self):
        q_deg = [sl.value() / 10.0 for sl in self.manual_sliders]
        q_rad = [math.radians(d) for d in q_deg]
        dur = self.manual_dur_spin.value()
        self.bridge.send_trajectory(q_rad, duration_sec=dur)
        self.log_label.setText(f"🚀 Gửi tư thế: {[f'{d:.1f}°' for d in q_deg]} trong {dur:.1f}s")

    # ---------------------------------------------------------
    # TRAJECTORY RECORDER CALLBACKS
    # ---------------------------------------------------------
    def _on_rec_start(self):
        self.traj_recording = True
        self.traj_waypoints = []
        self.traj_rec_start = time.monotonic()
        self.btn_rec.setEnabled(False)
        self.btn_rec_stop.setEnabled(True)
        self.rec_status_label.setText("🔴 ĐANG GHI... Di chuyển robot, mỗi bản cập nhật /joint_states sẽ được lưu.")
        self.rec_status_label.setStyleSheet("color: #dc2626; font-size: 11px; padding: 4px; font-weight: bold;")
        self.log_label.setText("🔴 Bắt đầu ghi quỹ đạo từ /joint_states...")

    def _on_rec_stop(self):
        self.traj_recording = False
        self.btn_rec.setEnabled(True)
        self.btn_rec_stop.setEnabled(False)
        n = len(self.traj_waypoints)
        dur = self.traj_waypoints[-1][0] if n > 0 else 0.0
        self.rec_status_label.setText(f"✅ Đã ghi xong: {n} waypoints trong {dur:.1f}s")
        self.rec_status_label.setStyleSheet("color: #059669; font-size: 11px; padding: 4px; font-weight: bold;")
        self._refresh_traj_table()
        self.log_label.setText(f"✅ Ghi xong! {n} waypoints, thời gian: {dur:.1f}s")

    def _refresh_traj_table(self):
        n = len(self.traj_waypoints)
        self.traj_table.setRowCount(n)
        for i, (t, q) in enumerate(self.traj_waypoints):
            self.traj_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self.traj_table.setItem(i, 1, QTableWidgetItem(f"{t:.2f}"))
            q_str = ", ".join(f"{math.degrees(v):.1f}°" for v in q)
            self.traj_table.setItem(i, 2, QTableWidgetItem(q_str))

    def _on_traj_save(self):
        from PyQt5.QtWidgets import QFileDialog
        if not self.traj_waypoints:
            self.log_label.setText("⚠ Chưa có quỹ đạo để lưu!")
            return
        save_dir = os.path.expanduser("~/denso_ws/trajectories")
        os.makedirs(save_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = os.path.join(save_dir, f"trajectory_{stamp}.yaml")
        path, _ = QFileDialog.getSaveFileName(self, "Lưu Quỹ Đạo", default_name, "YAML Files (*.yaml)")
        if not path:
            return
        data = {
            'joint_names': list(JOINT_NAMES),
            'waypoints': [{'time': float(t), 'positions': [float(v) for v in q]} for t, q in self.traj_waypoints]
        }
        import yaml
        try:
            with open(path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            self.log_label.setText(f"💾 Đã lưu {len(self.traj_waypoints)} waypoints → {path}")
        except Exception as e:
            self.log_label.setText(f"❌ Lỗi lưu file: {e}")

    def _on_traj_load(self):
        from PyQt5.QtWidgets import QFileDialog
        import yaml
        save_dir = os.path.expanduser("~/denso_ws/trajectories")
        os.makedirs(save_dir, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(self, "Nạp Quỹ Đạo", save_dir, "YAML Files (*.yaml)")
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            self.traj_waypoints = [(wp['time'], wp['positions']) for wp in data.get('waypoints', [])]
            self._refresh_traj_table()
            self.rec_status_label.setText(f"📂 Đã nạp: {len(self.traj_waypoints)} waypoints từ {os.path.basename(path)}")
            self.rec_status_label.setStyleSheet("color: #0284c7; font-size: 11px; padding: 4px; font-weight: bold;")
            self.log_label.setText(f"📂 Nạp quỹ đạo: {len(self.traj_waypoints)} waypoints từ {path}")
        except Exception as e:
            self.log_label.setText(f"❌ Lỗi nạp file: {e}")

    def _on_traj_play(self):
        if not self.traj_waypoints:
            self.log_label.setText("⚠ Chưa có quỹ đạo để phát!")
            return
        self.log_label.setText(f"▶ Đang phát {len(self.traj_waypoints)} waypoints...")
        def play_thread():
            for i, (t, q) in enumerate(self.traj_waypoints):
                if i == 0:
                    self.bridge.send_trajectory(q, duration_sec=max(0.5, t))
                    time.sleep(max(0.5, t))
                else:
                    dt = t - self.traj_waypoints[i-1][0]
                    self.bridge.send_trajectory(q, duration_sec=max(0.3, dt))
                    time.sleep(max(0.3, dt))
        threading.Thread(target=play_thread, daemon=True).start()

    def _on_traj_clear(self):
        self.traj_waypoints = []
        self.traj_table.setRowCount(0)
        self.rec_status_label.setText("● Quỹ đạo đã được xóa.")
        self.rec_status_label.setStyleSheet("color: #64748b; font-size: 11px; padding: 4px;")
        self.log_label.setText("🗑️ Đã xóa toàn bộ quỹ đạo.")


    # ---------------------------------------------------------
    # TAB 6: CARTESIAN INVERSE KINEMATICS (IK)
    # ---------------------------------------------------------
    def create_ik_cartesian_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        # Left Column: Inputs & Commands
        left_box = QGroupBox("🎯 NHẬP TỌA ĐỘ MỤC TIÊU LINK 5 (CARTESIAN TARGET)")
        left_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        left_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(12, 14, 12, 12)
        left_layout.setSpacing(10)

        # Mode row
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Khâu mục tiêu:"))
        self.gui_ik_mode = QComboBox()
        self.gui_ik_mode.addItems([
            "Khâu 5 - Cổ tay (Point P, Z_home=900mm)",
            "Mặt Bích / End-Effector (Flange, Z_home=980mm)"
        ])
        self.gui_ik_mode.setStyleSheet("background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; padding: 4px; font-weight: bold;")
        self.gui_ik_mode.currentIndexChanged.connect(self._on_gui_ik_calc)
        mode_row.addWidget(self.gui_ik_mode)
        left_layout.addLayout(mode_row)

        # Grid XYZ
        grid = QGridLayout()
        grid.setSpacing(8)

        self.gui_spin_x = QDoubleSpinBox()
        self.gui_spin_x.setRange(-800.0, 800.0)
        self.gui_spin_x.setValue(15.0)
        self.gui_spin_x.setSingleStep(10.0)

        self.gui_spin_y = QDoubleSpinBox()
        self.gui_spin_y.setRange(-800.0, 800.0)
        self.gui_spin_y.setValue(0.0)
        self.gui_spin_y.setSingleStep(10.0)

        self.gui_spin_z = QDoubleSpinBox()
        self.gui_spin_z.setRange(-200.0, 1100.0)
        self.gui_spin_z.setValue(900.0)
        self.gui_spin_z.setSingleStep(10.0)

        self.gui_spin_dur = QDoubleSpinBox()
        self.gui_spin_dur.setRange(0.5, 10.0)
        self.gui_spin_dur.setValue(2.0)
        self.gui_spin_dur.setSingleStep(0.5)
        self.gui_spin_dur.setSuffix(" s")

        for spin in [self.gui_spin_x, self.gui_spin_y, self.gui_spin_z]:
            spin.setStyleSheet("background-color: #f8fafc; color: #0284c7; border: 1px solid #cbd5e1; border-radius: 4px; padding: 4px; font-weight: bold;")
            spin.valueChanged.connect(self._on_gui_ik_calc)

        self.gui_spin_dur.setStyleSheet("background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; border-radius: 4px; padding: 4px; font-weight: bold;")

        grid.addWidget(QLabel("Tọa độ X (mm):"), 0, 0)
        grid.addWidget(self.gui_spin_x, 0, 1)
        grid.addWidget(QLabel("Tọa độ Y (mm):"), 1, 0)
        grid.addWidget(self.gui_spin_y, 1, 1)
        grid.addWidget(QLabel("Tọa độ Z (mm):"), 2, 0)
        grid.addWidget(self.gui_spin_z, 2, 1)
        grid.addWidget(QLabel("Thời gian chạy:"), 3, 0)
        grid.addWidget(self.gui_spin_dur, 3, 1)
        left_layout.addLayout(grid)

        # Quick presets
        pre_layout = QHBoxLayout()
        for name, coords in [("Home", (15, 0, 900)), ("Back (Bàn Vẽ)", (-380, 0, 180)), ("Front", (400, 0, 500)), ("Left", (0, 380, 500)), ("Right", (0, -380, 500))]:
            btn = QPushButton(name)
            btn.setStyleSheet("background-color: #f1f5f9; color: #0f172a; border: 1px solid #cbd5e1; padding: 5px 8px; border-radius: 4px; font-weight: bold;")
            btn.clicked.connect(lambda _, c=coords: self._set_gui_ik_coords(*c))
            pre_layout.addWidget(btn)
        left_layout.addLayout(pre_layout)

        # Read actual button
        btn_read = QPushButton("📍 Đọc Tọa Độ Robot Hiện Tại")
        btn_read.setStyleSheet("background-color: #64748b; color: white; padding: 8px; border-radius: 6px; font-weight: bold;")
        btn_read.clicked.connect(self._on_gui_ik_read_actual)
        left_layout.addWidget(btn_read)

        # Execute button
        btn_exec = QPushButton("🚀 CHUYỂN ĐỘNG (GỬI TRAJECTORY)")
        btn_exec.setStyleSheet("background-color: #0284c7; color: white; padding: 12px; border-radius: 6px; font-weight: bold; font-size: 13px;")
        btn_exec.clicked.connect(self._on_gui_ik_execute)
        left_layout.addWidget(btn_exec)

        left_layout.addStretch()
        layout.addWidget(left_box, 2)

        # Right Column: Results & Collision Check
        right_box = QGroupBox("📋 KẾT QUẢ TÍNH NGHỊCH HỌC & KIỂM TRA AN TOÀN")
        right_box.setFont(QFont("Segoe UI", 11, QFont.Bold))
        right_box.setStyleSheet("background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; color: #0f172a;")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(12, 14, 12, 12)
        right_layout.setSpacing(10)

        # Status badge
        self.gui_ik_status_badge = QLabel("✅ SẴN SÀNG")
        self.gui_ik_status_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.gui_ik_status_badge.setStyleSheet("background-color: #ecfdf5; color: #065f46; border: 1.5px solid #10b981; border-radius: 6px; padding: 8px; font-weight: bold;")
        right_layout.addWidget(self.gui_ik_status_badge)

        # IK table
        self.gui_ik_table = QTableWidget(len(JOINT_NAMES), 5)
        self.gui_ik_table.setHorizontalHeaderLabels(["Khớp", "Mục Tiêu (°)", "Mục Tiêu (rad)", "Hiện Tại (°)", "Độ Lệch Δ(°)"])
        h_hdr = self.gui_ik_table.horizontalHeader()
        if h_hdr:
            h_hdr.setSectionResizeMode(QHeaderView.Stretch)
        self.gui_ik_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.gui_ik_table.setFixedHeight(210)
        self.gui_ik_table.setStyleSheet("background-color: #ffffff; color: #0f172a; gridline-color: #e2e8f0; font-size: 12px;")
        for r in range(len(JOINT_NAMES)):
            it = QTableWidgetItem(f"Joint {r+1}")
            it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            it.setFont(QFont("Segoe UI", 10, QFont.Bold))
            self.gui_ik_table.setItem(r, 0, it)
            for c in range(1, 5):
                it_c = QTableWidgetItem("0.0")
                it_c.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.gui_ik_table.setItem(r, c, it_c)

        right_layout.addWidget(self.gui_ik_table)
        right_layout.addStretch()
        layout.addWidget(right_box, 3)

        self.last_gui_ik_sol = [0.0] * len(JOINT_NAMES)
        # Calculate initial
        QTimer.singleShot(200, self._on_gui_ik_calc)
        return panel

    def _set_gui_ik_coords(self, x, y, z):
        self.gui_spin_x.blockSignals(True)
        self.gui_spin_y.blockSignals(True)
        self.gui_spin_z.blockSignals(True)
        self.gui_spin_x.setValue(x)
        self.gui_spin_y.setValue(y)
        self.gui_spin_z.setValue(z)
        self.gui_spin_x.blockSignals(False)
        self.gui_spin_y.blockSignals(False)
        self.gui_spin_z.blockSignals(False)
        self._on_gui_ik_calc()

    def _on_gui_ik_read_actual(self):
        use_flange = (self.gui_ik_mode.currentIndex() == 1)
        fk = self.dynamics_engine.compute_forward_kinematics(self.current_joints, use_flange=use_flange)
        p = fk["position_mm"]
        self._set_gui_ik_coords(p[0], p[1], p[2])
        self.log_label.setText(f"📍 Đã đọc tọa độ thực tế: X={p[0]:.1f}, Y={p[1]:.1f}, Z={p[2]:.1f} mm")

    def _on_gui_ik_calc(self):
        x = self.gui_spin_x.value() / 1000.0
        y = self.gui_spin_y.value() / 1000.0
        z = self.gui_spin_z.value() / 1000.0
        use_flange = (self.gui_ik_mode.currentIndex() == 1)

        q_sol, ok, iters, err_m = self.dynamics_engine.inverse_kinematics(
            [x, y, z], q_seed=self.current_joints, use_flange=use_flange
        )
        if q_sol is not None:
            self.last_gui_ik_sol = list(q_sol)
            is_col, col_reason = self.dynamics_engine.check_collision(q_sol)
            if is_col:
                self.gui_ik_status_badge.setText(f"⚠️ {col_reason}")
                self.gui_ik_status_badge.setStyleSheet("background-color: #fef2f2; color: #991b1b; border: 1.5px solid #ef4444; border-radius: 6px; padding: 8px; font-weight: bold;")
            elif ok:
                self.gui_ik_status_badge.setText(f"✅ HỘI TỤ THÀNH CÔNG (Sai số: {err_m*1000.0:.2f} mm | An toàn)")
                self.gui_ik_status_badge.setStyleSheet("background-color: #ecfdf5; color: #065f46; border: 1.5px solid #10b981; border-radius: 6px; padding: 8px; font-weight: bold;")
            else:
                self.gui_ik_status_badge.setText(f"⚠️ NGOÀI TẦM VỚI (Sai số: {err_m*1000.0:.1f} mm)")
                self.gui_ik_status_badge.setStyleSheet("background-color: #fffbeb; color: #92400e; border: 1.5px solid #f59e0b; border-radius: 6px; padding: 8px; font-weight: bold;")

            for i in range(len(JOINT_NAMES)):
                t_deg = math.degrees(q_sol[i])
                c_deg = math.degrees(self.current_joints[i])
                d_deg = abs(t_deg - c_deg)
                set_table_item_text(self.gui_ik_table, i, 1, f"{c_deg:+6.1f}°")
                set_table_item_text(self.gui_ik_table, i, 2, f"{t_deg:+6.1f}°")
                set_table_item_text(self.gui_ik_table, i, 3, f"{d_deg:5.1f}°")
                st = "OK" if not is_col else "CẢNH BÁO"
                set_table_item_text(self.gui_ik_table, i, 4, st)
        else:
            self.gui_ik_status_badge.setText(f"⛔ NGOÀI VÙNG LÀM VIỆC (Không tìm thấy nghiệm IK)")
            self.gui_ik_status_badge.setStyleSheet("background-color: #fee2e2; color: #b91c1c; border: 1.5px solid #ef4444; border-radius: 6px; padding: 8px; font-weight: bold;")
            for i in range(len(JOINT_NAMES)):
                c_deg = math.degrees(self.current_joints[i])
                set_table_item_text(self.gui_ik_table, i, 1, f"{c_deg:+6.1f}°")
                set_table_item_text(self.gui_ik_table, i, 2, "N/A")
                set_table_item_text(self.gui_ik_table, i, 3, "N/A")
                set_table_item_text(self.gui_ik_table, i, 4, "LỖI")
            t_deg = math.degrees(q_sol[i])
            c_deg = math.degrees(self.current_joints[i])
            d_deg = abs(t_deg - c_deg)
            self.gui_ik_table.item(i, 1).setText(f"{t_deg:+.2f}°")
            self.gui_ik_table.item(i, 2).setText(f"{q_sol[i]:+.3f}")
            self.gui_ik_table.item(i, 3).setText(f"{c_deg:+.2f}°")
            self.gui_ik_table.item(i, 4).setText(f"{d_deg:.2f}°")

    def _on_gui_ik_execute(self):
        # Recalculate directly to guarantee freshest values
        x = self.gui_spin_x.value() / 1000.0
        y = self.gui_spin_y.value() / 1000.0
        z = self.gui_spin_z.value() / 1000.0
        use_flange = (self.gui_ik_mode.currentIndex() == 1)

        q_sol, ok, _, err_m = self.dynamics_engine.inverse_kinematics(
            [x, y, z], q_seed=self.current_joints, use_flange=use_flange
        )
        is_col, col_reason = self.dynamics_engine.check_collision(q_sol)
        if is_col:
            self.log_label.setText(f"❌ KHÔNG THỂ CHẠY: {col_reason}")
            return

        dur = self.gui_spin_dur.value()
        self.bridge.send_trajectory(q_sol, duration_sec=dur)
        self.log_label.setText(f"🚀 Đã gửi quỹ đạo chuyển động tới tọa độ Link 5 (Thời gian: {dur:.1f}s)")

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
