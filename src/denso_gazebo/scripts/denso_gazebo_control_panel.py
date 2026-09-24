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
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "denso_vs6556_moveit_config", "scripts"))
try:
    from denso_dynamics_engine import DensoDynamicsEngine
except ImportError:
    sys.path.append("/home/dung/denso_ws/src/denso_vs6556_moveit_config/scripts")
    from denso_dynamics_engine import DensoDynamicsEngine

from PyQt5.QtWidgets import (
    QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView, QComboBox, QGridLayout,
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QFrame, QRadioButton, QButtonGroup, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

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
        'axis': 'Trục X',
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
    }
]

JOINT_NAMES_VI = [
    "J1 Đế (Base Yaw)",
    "J2 Vai (Shoulder Pitch)",
    "J3 Khuỷu (Elbow Pitch)",
    "J4 Cẳng tay (Roll)",
    "J5 Cổ tay (Wrist Pitch)"
]

PRESETS = {
    '🏠 Về Home (0°)': [0.0, 0.0, 0.0, 0.0, 0.0],
    '🎯 Sẵn sàng (Ready)': [0.0, 30.0, 45.0, 0.0, 30.0],
    '📦 Vị trí Gắp (Pick)': [45.0, 45.0, 60.0, 0.0, -30.0],
    '📥 Vị trí Đặt (Place)': [-45.0, 30.0, 75.0, 0.0, -15.0],
    '🔭 Vươn xa (Reach)': [0.0, 15.0, 30.0, 0.0, 0.0],
}


class GazeboRosBridge:
    def __init__(self):
        self.node = None
        self.traj_pub = None
        self.latest_positions = [0.0] * 5
        self.latest_velocities = [0.0] * 5
        self.latest_efforts = [0.0] * 5
        self.last_msg_time = 0.0
        self.lock = threading.Lock()

    def start(self):
        rclpy.init(args=None)
        self.node = Node('denso_gazebo_control_panel')

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

        def _spin_worker():
            try:
                rclpy.spin(self.node)
            except Exception:
                pass
        threading.Thread(target=_spin_worker, daemon=True).start()

    def _joint_state_cb(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(JOINT_NAMES)}
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in name_to_idx:
                    idx = name_to_idx[name]
                    if i < len(msg.position):
                        self.latest_positions[idx] = float(msg.position[i])
                    if i < len(msg.velocity):
                        self.latest_velocities[idx] = float(msg.velocity[i])
                    if i < len(msg.effort):
                        self.latest_efforts[idx] = float(msg.effort[i])
            self.last_msg_time = time.time()

    def get_latest_state(self):
        with self.lock:
            return (
                list(self.latest_positions),
                list(self.latest_velocities),
                list(self.latest_efforts),
                self.last_msg_time
            )

    def send_trajectory(self, target_rads, duration_sec=2.0):
        if not self.traj_pub:
            return
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = [float(r) for r in target_rads]
        
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        
        traj.points = [point]
        self.traj_pub.publish(traj)


class DensoGazeboPanel(QMainWindow):
    def __init__(self, bridge: GazeboRosBridge):
        super().__init__()
        self.bridge = bridge
        self.is_deg_mode = True
        self.live_drag = False
        self.target_rads = [0.0] * 5
        self.actual_rads = [0.0] * 5
        self.actual_efforts = [0.0] * 5
        self.updating_widgets = False
        self.engine = DensoDynamicsEngine()
        self.ik_target_pos = [15.0, 0.0, 900.0]
        self.last_ik_sol = None

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
        self.resize(1180, 820)
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

        # 2. Tabs Container
        self.tabs = QTabWidget(self)
        
        tab_ik = self.create_ik_tab()
        self.tabs.addTab(tab_ik, "🎯 1. NHẬP VỊ TRÍ LINK 5 (CARTESIAN IK)")

        tab_joints = self.create_joints_tab()
        self.tabs.addTab(tab_joints, "🎛️ 2. ĐIỀU KHIỂN TỪNG KHỚP (SLIDERS)")

        main_layout.addWidget(self.tabs, 1)

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

    def create_ik_tab(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        # ── CỘT TRÁI: NHẬP TỌA ĐỘ VÀ ĐIỀU KHIỂN (40%) ──
        left_box = QGroupBox("🎯 BẢNG NHẬP VỊ TRÍ MỤC TIÊU (CARTESIAN TARGET)")
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(14, 16, 14, 14)
        left_layout.setSpacing(12)

        # Khâu mục tiêu: Flange hay Point P
        mode_row = QHBoxLayout()
        lbl_mode = QLabel("Khâu mục tiêu:")
        lbl_mode.setStyleSheet("font-weight: bold;")
        self.ik_target_mode = QComboBox()
        self.ik_target_mode.addItems([
            "Khâu 5 - Cổ tay (Point P, Z_home=900mm)",
            "Mặt Bích / End-Effector (Flange, Z_home=980mm)"
        ])
        self.ik_target_mode.setStyleSheet("background-color: #0b1120; color: #38bdf8; border: 1px solid #334155; padding: 4px; font-weight: bold;")
        mode_row.addWidget(lbl_mode)
        mode_row.addWidget(self.ik_target_mode)
        left_layout.addLayout(mode_row)

        # Grid Tọa độ X, Y, Z, Duration
        grid_in = QGridLayout()
        grid_in.setSpacing(8)

        lbl_x = QLabel("Tọa độ X (mm):")
        lbl_x.setStyleSheet("font-weight: bold;")
        self.spin_ik_x = QDoubleSpinBox()
        self.spin_ik_x.setRange(-800.0, 800.0)
        self.spin_ik_x.setValue(15.0)
        self.spin_ik_x.setSingleStep(10.0)

        lbl_y = QLabel("Tọa độ Y (mm):")
        lbl_y.setStyleSheet("font-weight: bold;")
        self.spin_ik_y = QDoubleSpinBox()
        self.spin_ik_y.setRange(-800.0, 800.0)
        self.spin_ik_y.setValue(0.0)
        self.spin_ik_y.setSingleStep(10.0)

        lbl_z = QLabel("Tọa độ Z (mm):")
        lbl_z.setStyleSheet("font-weight: bold;")
        self.spin_ik_z = QDoubleSpinBox()
        self.spin_ik_z.setRange(-200.0, 1100.0)
        self.spin_ik_z.setValue(900.0)
        self.spin_ik_z.setSingleStep(10.0)

        lbl_dur = QLabel("Thời gian di chuyển:")
        lbl_dur.setStyleSheet("font-weight: bold;")
        self.spin_ik_dur = QDoubleSpinBox()
        self.spin_ik_dur.setRange(0.5, 10.0)
        self.spin_ik_dur.setValue(2.0)
        self.spin_ik_dur.setSingleStep(0.5)
        self.spin_ik_dur.setSuffix(" s")

        grid_in.addWidget(lbl_x, 0, 0)
        grid_in.addWidget(self.spin_ik_x, 0, 1)
        grid_in.addWidget(lbl_y, 1, 0)
        grid_in.addWidget(self.spin_ik_y, 1, 1)
        grid_in.addWidget(lbl_z, 2, 0)
        grid_in.addWidget(self.spin_ik_z, 2, 1)
        grid_in.addWidget(lbl_dur, 3, 0)
        grid_in.addWidget(self.spin_ik_dur, 3, 1)
        left_layout.addLayout(grid_in)

        # Presets Cartesian
        lbl_pre = QLabel("GỢI Ý VỊ TRÍ MẪU TRONG GAZEBO:")
        lbl_pre.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold; margin-top: 4px;")
        left_layout.addWidget(lbl_pre)

        grid_p = QGridLayout()
        grid_p.setSpacing(6)
        
        btn_home = QPushButton("🏠 Home")
        btn_home.setStyleSheet("background-color: #1e293b; padding: 6px;")
        btn_home.clicked.connect(self._preset_home)
        grid_p.addWidget(btn_home, 0, 0)

        btn_front = QPushButton("➡️ Phía Trước")
        btn_front.setStyleSheet("background-color: #1e293b; padding: 6px;")
        btn_front.clicked.connect(self._preset_front)
        grid_p.addWidget(btn_front, 0, 1)

        btn_right = QPushButton("➡️ Bên Phải")
        btn_right.setStyleSheet("background-color: #1e293b; padding: 6px;")
        btn_right.clicked.connect(self._preset_right)
        grid_p.addWidget(btn_right, 1, 0)

        btn_left = QPushButton("⬅️ Bên Trái")
        btn_left.setStyleSheet("background-color: #1e293b; padding: 6px;")
        btn_left.clicked.connect(self._preset_left)
        grid_p.addWidget(btn_left, 1, 1)
        left_layout.addLayout(grid_p)

        # Nút đọc tọa độ hiện tại và Preview IK
        b_row = QHBoxLayout()
        b_row.setSpacing(8)
        btn_read_cur = QPushButton("📥 Lấy Tọa Độ Hiện Tại")
        btn_read_cur.setStyleSheet("background-color: #334155; padding: 8px;")
        btn_read_cur.clicked.connect(self._read_current_cartesian)
        b_row.addWidget(btn_read_cur)

        btn_calc = QPushButton("🧮 TÍNH TOÁN IK (PREVIEW)")
        btn_calc.setStyleSheet("background-color: #0d9488; padding: 8px;")
        btn_calc.clicked.connect(self._calc_ik)
        b_row.addWidget(btn_calc)
        left_layout.addLayout(b_row)

        self.btn_exec_ik = QPushButton("🚀 CHUYỂN ĐỘNG ROBOT TRONG GAZEBO")
        self.btn_exec_ik.setFixedHeight(48)
        self.btn_exec_ik.setStyleSheet("background-color: #16a34a; font-size: 14px; font-weight: bold; border-radius: 8px;")
        self.btn_exec_ik.clicked.connect(self._exec_ik_motion)
        left_layout.addWidget(self.btn_exec_ik)

        # Status Card
        self.ik_status_card = QFrame()
        self.ik_status_card.setStyleSheet("background-color: #0b1120; border-radius: 6px; padding: 8px; border-left: 4px solid #0284c7;")
        v_st = QVBoxLayout(self.ik_status_card)
        v_st.setContentsMargins(6, 6, 6, 6)
        v_st.setSpacing(4)
        self.lbl_ik_status = QLabel("● Sẵn sàng nhập tọa độ và giải động học nghịch cho Gazebo.")
        self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #38bdf8;")
        self.lbl_ik_info = QLabel("Tầm vươn tối đa: ~733 mm (Flange) / ~653 mm (Point P)")
        self.lbl_ik_info.setStyleSheet("font-size: 11px; color: #94a3b8;")
        v_st.addWidget(self.lbl_ik_status)
        v_st.addWidget(self.lbl_ik_info)
        left_layout.addWidget(self.ik_status_card)

        left_layout.addStretch()
        layout.addWidget(left_box, 4)

        # ── CỘT PHẢI: BẢNG GÓC JOINT THỜI GIAN THỰC (60%) ──
        right_box = QGroupBox("📊 BẢNG GÓC CÁC JOINT TRONG GAZEBO THỜI GIAN THỰC")
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(14, 16, 14, 14)
        right_layout.setSpacing(10)

        # KPI Cards (Rõ ràng, không bị cắt text)
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(10)
        self.card_cur = self._make_kpi_card("VỊ TRÍ GAZEBO", "[+0, +0, +0] mm", "#4ade80")
        self.card_tar = self._make_kpi_card("VỊ TRÍ MỤC TIÊU", "[+0, +0, +0] mm", "#38bdf8")
        self.card_dist = self._make_kpi_card("KHOẢNG CÁCH ΔD", "ΔD: 0.0 mm", "#facc15")
        cards_layout.addWidget(self.card_cur)
        cards_layout.addWidget(self.card_tar)
        cards_layout.addWidget(self.card_dist)
        right_layout.addLayout(cards_layout)

        # Bảng 5 cột chuẩn, rộng rãi, không bị dấu ...
        self.ik_table = QTableWidget(5, 5)
        self.ik_table.setHorizontalHeaderLabels([
            "Khớp (Joint)", "Hiện Tại (Gazebo)", "Mục Tiêu (IK)", "Sai Lệch Δq", "Lực (Nm)"
        ])
        
        h_h = self.ik_table.horizontalHeader()
        h_h.setSectionResizeMode(0, QHeaderView.Interactive)
        h_h.setSectionResizeMode(1, QHeaderView.Stretch)
        h_h.setSectionResizeMode(2, QHeaderView.Stretch)
        h_h.setSectionResizeMode(3, QHeaderView.Interactive)
        h_h.setSectionResizeMode(4, QHeaderView.Interactive)
        
        self.ik_table.setColumnWidth(0, 165)
        self.ik_table.setColumnWidth(3, 110)
        self.ik_table.setColumnWidth(4, 110)
        
        self.ik_table.verticalHeader().setVisible(False)
        self.ik_table.verticalHeader().setDefaultSectionSize(36)
        
        self.ik_table.setStyleSheet("""
            QTableWidget {
                background-color: #0b1120;
                color: #e2e8f0;
                gridline-color: #1e293b;
                font-family: 'Segoe UI', Consolas, monospace;
                font-size: 13px;
                border: 1px solid #1e293b;
                border-radius: 6px;
            }
            QHeaderView::section {
                background-color: #1e293b;
                color: #38bdf8;
                font-weight: bold;
                border: 1px solid #334155;
                padding: 6px;
                height: 32px;
            }
        """)
        right_layout.addWidget(self.ik_table, 1)

        lbl_foot = QLabel("💡 Trạng thái: Khi robot di chuyển trong Gazebo, các cột góc thực tế và sai lệch Δq sẽ cập nhật liên tục.")
        lbl_foot.setStyleSheet("color: #94a3b8; font-size: 11px; font-style: italic;")
        right_layout.addWidget(lbl_foot)

        layout.addWidget(right_box, 6)
        return panel

    def _preset_home(self):
        use_flange = (self.ik_target_mode.currentIndex() == 1)
        self.spin_ik_x.setValue(15.0)
        self.spin_ik_y.setValue(0.0)
        self.spin_ik_z.setValue(980.0 if use_flange else 900.0)
        self._calc_ik()

    def _preset_front(self):
        self.spin_ik_x.setValue(350.0)
        self.spin_ik_y.setValue(0.0)
        self.spin_ik_z.setValue(600.0)
        self._calc_ik()

    def _preset_right(self):
        self.spin_ik_x.setValue(300.0)
        self.spin_ik_y.setValue(-250.0)
        self.spin_ik_z.setValue(550.0)
        self._calc_ik()

    def _preset_left(self):
        self.spin_ik_x.setValue(300.0)
        self.spin_ik_y.setValue(250.0)
        self.spin_ik_z.setValue(550.0)
        self._calc_ik()

    def _read_current_cartesian(self):
        use_flange = (self.ik_target_mode.currentIndex() == 1)
        transforms, _ = self.engine.get_dh_transforms(self.actual_rads)
        idx = 5 if use_flange else 4
        cur_cart = transforms[idx][:3, 3] * 1000.0
        self.spin_ik_x.setValue(round(float(cur_cart[0]), 1))
        self.spin_ik_y.setValue(round(float(cur_cart[1]), 1))
        self.spin_ik_z.setValue(round(float(cur_cart[2]), 1))
        self._calc_ik()

    def _calc_ik(self):
        tx = self.spin_ik_x.value()
        ty = self.spin_ik_y.value()
        tz = self.spin_ik_z.value()
        self.ik_target_pos = [tx, ty, tz]
        target_m = [tx / 1000.0, ty / 1000.0, tz / 1000.0]
        use_flange = (self.ik_target_mode.currentIndex() == 1)

        q_sol, success, iters, err_m = self.engine.inverse_kinematics(
            target_m, q_seed=self.actual_rads, use_flange=use_flange
        )
        self.last_ik_sol = q_sol
        err_mm = err_m * 1000.0

        if success:
            self.ik_status_card.setStyleSheet("background-color: #064e3b; border-radius: 6px; padding: 8px; border-left: 4px solid #10b981;")
            self.lbl_ik_status.setText(f"✅ Đã tìm thấy nghiệm IK sau {iters} bước lặp! (Sai số: {err_mm:.3f} mm)")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #a7f3d0;")
            deg_sol_str = ", ".join([f"{math.degrees(a):+.1f}°" for a in q_sol])
            self.lbl_ik_info.setText(f"Góc mục tiêu: [{deg_sol_str}]")
            self.card_tar.val_lbl.setText(f"[{tx:+.0f}, {ty:+.0f}, {tz:+.0f}] mm")
        else:
            self.ik_status_card.setStyleSheet("background-color: #7f1d1d; border-radius: 6px; padding: 8px; border-left: 4px solid #ef4444;")
            self.lbl_ik_status.setText(f"⚠️ Vị trí ngoài tầm với an toàn! (Sai số: {err_mm:.1f} mm)")
            self.lbl_ik_status.setStyleSheet("font-weight: bold; font-size: 12px; color: #fecaca;")
            self.lbl_ik_info.setText("Vui lòng nhập vị trí nằm trong tầm vươn của cánh tay robot.")

        # Cập nhật hiển thị bảng ngay lập tức
        self._refresh_ik_table()

    def _exec_ik_motion(self):
        if self.last_ik_sol is None:
            self._calc_ik()
        if self.last_ik_sol is not None:
            dur = self.spin_ik_dur.value()
            self.bridge.send_trajectory(self.last_ik_sol, duration_sec=dur)

    def _refresh_ik_table(self):
        if not hasattr(self, 'ik_table'):
            return

        use_flange = (self.ik_target_mode.currentIndex() == 1)
        transforms, _ = self.engine.get_dh_transforms(self.actual_rads)
        idx = 5 if use_flange else 4
        cur_cart = transforms[idx][:3, 3] * 1000.0

        self.card_cur.val_lbl.setText(f"[{cur_cart[0]:+4.0f}, {cur_cart[1]:+4.0f}, {cur_cart[2]:+4.0f}] mm")

        if self.last_ik_sol is not None:
            dist = float(np.linalg.norm(np.array(self.ik_target_pos) - cur_cart))
            self.card_dist.val_lbl.setText(f"ΔD: {dist:5.1f} mm")
            if dist < 3.0:
                self.card_dist.val_lbl.setStyleSheet("color: #4ade80; font-size: 15px; font-weight: bold; font-family: monospace;")
            else:
                self.card_dist.val_lbl.setStyleSheet("color: #facc15; font-size: 15px; font-weight: bold; font-family: monospace;")
        else:
            self.card_dist.val_lbl.setText("ΔD: --- mm")

        def set_item(row, col, text, color=None):
            it = self.ik_table.item(row, col)
            if it is None:
                it = QTableWidgetItem(text)
                it.setTextAlignment(Qt.AlignCenter if col > 0 else Qt.AlignLeft | Qt.AlignVCenter)
                self.ik_table.setItem(row, col, it)
            else:
                it.setText(text)
            if color:
                it.setForeground(color)

        for i in range(5):
            cur_deg = math.degrees(self.actual_rads[i])
            cur_rad = self.actual_rads[i]

            set_item(i, 0, JOINT_NAMES_VI[i])
            set_item(i, 1, f"{cur_deg:+6.1f}° ({cur_rad:+5.2f})", QColor("#4ade80"))

            if self.last_ik_sol is not None:
                tar_rad = self.last_ik_sol[i]
                tar_deg = math.degrees(tar_rad)
                diff = abs(tar_deg - cur_deg)
                set_item(i, 2, f"{tar_deg:+6.1f}° ({tar_rad:+5.2f})", QColor("#38bdf8"))

                diff_col = QColor("#4ade80") if diff < 0.8 else QColor("#facc15")
                set_item(i, 3, f"{diff:5.2f}°", diff_col)
            else:
                set_item(i, 2, "---")
                set_item(i, 3, "---")

            set_item(i, 4, f"{self.actual_efforts[i]:+5.1f} Nm", QColor("#94a3b8"))

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

        for i in range(5):
            self.spin_boxes[i].valueChanged.connect(lambda val, idx=i: self._on_spin_changed(idx, val))
            self.sliders[i].valueChanged.connect(lambda val, idx=i: self._on_slider_changed(idx, val))

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
        for i in range(5):
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
        for i in range(5):
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

        # Cập nhật telemetry Tab 2 (Joints)
        for i in range(5):
            if self.is_deg_mode:
                pos_str = f'Gazebo: {math.degrees(pos[i]):+.2f}°'
            else:
                pos_str = f'Gazebo: {pos[i]:+.3f} rad'

            err = abs(self.target_rads[i] - pos[i])
            color = '#4ade80' if err < 0.02 else '#38bdf8'
            self.actual_labels[i].setText(pos_str)
            self.actual_labels[i].setStyleSheet(f'color: {color}; font-weight: bold; font-size: 12px;')

            eff_val = eff[i]
            limit = JOINT_INFO[i]['effort_limit']
            eff_pct = min(100.0, (abs(eff_val) / limit) * 100.0)
            eff_color = '#94a3b8' if eff_pct < 60 else '#facc15' if eff_pct < 85 else '#ef4444'
            self.effort_labels[i].setText(f'Lực: {eff_val:+.1f} Nm ({eff_pct:.0f}%)')
            self.effort_labels[i].setStyleSheet(f'color: {eff_color}; font-size: 11px;')

        # Cập nhật telemetry Tab 1 (Cartesian IK & Table)
        self._refresh_ik_table()


def main():
    bridge = GazeboRosBridge()
    bridge.start()

    app = QApplication(sys.argv)
    window = DensoGazeboPanel(bridge)
    window.show()
    ret = app.exec_()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(ret)

if __name__ == '__main__':
    main()
