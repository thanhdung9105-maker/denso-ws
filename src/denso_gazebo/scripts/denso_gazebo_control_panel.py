#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 GAZEBO INTERACTIVE CONTROL PANEL
Bảng nhập và điều khiển trực tiếp các khớp robot trong môi trường Gazebo Sim.
Hỗ trợ:
- Nhập giá trị số trực tiếp (Độ ° / Radian)
- Thanh trượt Slider tương tác mượt mà
- Chế độ Live Drag (kéo slider robot trong Gazebo chuyển động theo tức thì)
- Đọc phản hồi thực tế từ Gazebo (/joint_states: vị trí, vận tốc, mô-men lực)
- Các tư thế mẫu (Home, Ready, Pick, Place, Reach, Hold)
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

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QGroupBox, QFrame, QRadioButton, QButtonGroup, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

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

PRESETS = {
    '🏠 Về Home (0°)': [0.0, 0.0, 0.0, 0.0, 0.0],
    '🎯 Sẵn sàng (Ready)': [0.0, 30.0, 45.0, 0.0, 30.0],
    '📦 Vị trí Gắp (Pick)': [45.0, 45.0, 60.0, 0.0, -30.0],
    '📥 Vị trí Đặt (Place)': [-45.0, 30.0, 75.0, 0.0, -15.0],
    '🔭 Vươn xa (Reach)': [0.0, 15.0, 30.0, 0.0, 0.0],
}

class GazeboRosBridge(QObject):
    joint_states_signal = pyqtSignal(list, list, list)
    connection_signal = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.node = None
        self.traj_pub = None
        self.latest_positions = [0.0] * 5
        self.latest_velocities = [0.0] * 5
        self.latest_efforts = [0.0] * 5
        self.is_connected = False
        self.last_msg_time = 0.0

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
        updated = False
        for i, name in enumerate(msg.name):
            if name in name_to_idx:
                idx = name_to_idx[name]
                if i < len(msg.position):
                    self.latest_positions[idx] = float(msg.position[i])
                    updated = True
                if i < len(msg.velocity):
                    self.latest_velocities[idx] = float(msg.velocity[i])
                if i < len(msg.effort):
                    self.latest_efforts[idx] = float(msg.effort[i])

        if updated:
            self.last_msg_time = time.time()
            if not self.is_connected:
                self.is_connected = True
                self.connection_signal.emit(True)
            self.joint_states_signal.emit(
                list(self.latest_positions),
                list(self.latest_velocities),
                list(self.latest_efforts)
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

        self.init_ui()
        self.setup_connections()

        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._check_connection)
        self.watchdog.start(1000)

    def init_ui(self):
        self.setWindowTitle('DENSO VS-6556 - Bảng Nhập & Điều Khiển Trực Tiếp Gazebo')
        self.resize(880, 720)
        self.setMinimumWidth(780)

        self.setStyleSheet('''
            QMainWindow { background-color: #1a1a24; }
            QWidget { color: #e2e8f0; font-family: 'Segoe UI', Arial, sans-serif; }
            QGroupBox {
                background-color: #242434;
                border: 1px solid #383850;
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
                border: 1px solid #475569;
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
        ''')

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(16, 16, 16, 16)

        # 1. Header Banner
        header = QFrame(self)
        header.setStyleSheet('background-color: #0f172a; border-radius: 8px; padding: 6px;')
        h_layout = QHBoxLayout(header)
        
        title_lbl = QLabel('🤖 BẢNG ĐIỀU KHIỂN ROBOT TRỰC TIẾP TRONG GAZEBO', self)
        title_lbl.setStyleSheet('font-size: 16px; font-weight: bold; color: #38bdf8;')
        h_layout.addWidget(title_lbl)
        h_layout.addStretch()

        self.status_badge = QLabel('🔴 Chờ kết nối Gazebo...', self)
        self.status_badge.setStyleSheet('background-color: #7f1d1d; color: #fecaca; padding: 4px 12px; border-radius: 12px; font-weight: bold; font-size: 12px;')
        h_layout.addWidget(self.status_badge)
        main_layout.addWidget(header)

        # 2. Options Bar
        options_box = QFrame(self)
        options_box.setStyleSheet('background-color: #242434; border: 1px solid #383850; border-radius: 8px;')
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

        main_layout.addWidget(options_box)

        # 3. 5 Joints Strip
        joints_group = QGroupBox('CÁC KHỚP ROBOT (NHẬP SỐ HOẶC KÉO THANH TRƯỢT)', self)
        joints_layout = QVBoxLayout(joints_group)
        joints_layout.setSpacing(10)
        joints_layout.setContentsMargins(14, 16, 14, 14)

        self.spin_boxes = []
        self.sliders = []
        self.actual_labels = []
        self.effort_labels = []

        for i, info in enumerate(JOINT_INFO):
            row = QFrame(self)
            row.setStyleSheet('background-color: #1e1e2d; border-radius: 6px; padding: 4px;')
            r_layout = QHBoxLayout(row)
            r_layout.setContentsMargins(8, 4, 8, 4)

            lbl_name = QLabel(f"{info['name']}<br><span style='color: #94a3b8; font-size:11px;'>{info['axis']}</span>", self)
            lbl_name.setFixedWidth(170)
            r_layout.addWidget(lbl_name)

            spin = QDoubleSpinBox(self)
            spin.setDecimals(2)
            spin.setFixedWidth(105)
            r_layout.addWidget(spin)
            self.spin_boxes.append(spin)

            slider = QSlider(Qt.Horizontal, self)
            slider.setRange(-1000, 1000)
            slider.setValue(0)
            r_layout.addWidget(slider, stretch=2)
            self.sliders.append(slider)

            fb_frame = QFrame(self)
            fb_frame.setFixedWidth(185)
            fb_layout = QVBoxLayout(fb_frame)
            fb_layout.setContentsMargins(4, 0, 4, 0)
            fb_layout.setSpacing(2)

            act_lbl = QLabel('Gazebo: 0.00°', self)
            act_lbl.setStyleSheet('color: #4ade80; font-weight: bold; font-size: 12px;')
            eff_lbl = QLabel('Lực: 0.0 Nm', self)
            eff_lbl.setStyleSheet('color: #94a3b8; font-size: 11px;')

            fb_layout.addWidget(act_lbl)
            fb_layout.addWidget(eff_lbl)
            r_layout.addWidget(fb_frame)

            self.actual_labels.append(act_lbl)
            self.effort_labels.append(eff_lbl)

            joints_layout.addWidget(row)

        main_layout.addWidget(joints_group)

        # 4. Action Buttons
        action_layout = QHBoxLayout()

        self.btn_send = QPushButton('🚀 GỬI LỆNH ĐẾN GAZEBO', self)
        self.btn_send.setFixedHeight(44)
        self.btn_send.setStyleSheet('QPushButton { background-color: #16a34a; font-size: 14px; font-weight: bold; border-radius: 8px; } QPushButton:hover { background-color: #15803d; }')
        action_layout.addWidget(self.btn_send, stretch=2)

        self.btn_read = QPushButton('🔄 Đọc từ Gazebo', self)
        self.btn_read.setFixedHeight(44)
        self.btn_read.setStyleSheet('QPushButton { background-color: #475569; font-size: 13px; font-weight: bold; border-radius: 8px; } QPushButton:hover { background-color: #334155; }')
        action_layout.addWidget(self.btn_read, stretch=1)

        self.btn_stop = QPushButton('🛑 Dừng / Hold', self)
        self.btn_stop.setFixedHeight(44)
        self.btn_stop.setStyleSheet('QPushButton { background-color: #dc2626; font-size: 13px; font-weight: bold; border-radius: 8px; } QPushButton:hover { background-color: #b91c1c; }')
        action_layout.addWidget(self.btn_stop, stretch=1)

        main_layout.addLayout(action_layout)

        # 5. Presets
        preset_box = QGroupBox('TƯ THẾ CÓ SẴN (QUICK PRESETS)', self)
        preset_layout = QHBoxLayout(preset_box)
        preset_layout.setContentsMargins(10, 12, 10, 10)
        preset_layout.setSpacing(8)

        for name, angles in PRESETS.items():
            btn = QPushButton(name, self)
            btn.setStyleSheet('QPushButton { background-color: #334155; font-size: 12px; padding: 6px 12px; } QPushButton:hover { background-color: #0284c7; }')
            btn.clicked.connect(lambda checked, a=angles: self.apply_preset(a))
            preset_layout.addWidget(btn)

        main_layout.addWidget(preset_box)

        self.update_unit_display()

    def setup_connections(self):
        self.rb_deg.toggled.connect(self.update_unit_display)
        self.cb_live.toggled.connect(self._toggle_live_drag)
        self.btn_send.clicked.connect(self.send_to_gazebo)
        self.btn_read.clicked.connect(self.read_actual_as_target)
        self.btn_stop.clicked.connect(self.stop_and_hold)

        for i in range(5):
            self.spin_boxes[i].valueChanged.connect(lambda val, idx=i: self._on_spin_changed(idx, val))
            self.sliders[i].valueChanged.connect(lambda val, idx=i: self._on_slider_changed(idx, val))

        self.bridge.joint_states_signal.connect(self._on_joint_states_update)
        self.bridge.connection_signal.connect(self._on_connection_changed)

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

    def _on_slider_changed(self, idx, slider_val):
        if self.updating_widgets:
            return
        self.updating_widgets = True
        info = JOINT_INFO[idx]
        ratio = (slider_val + 1000) / 2000.0
        rad_val = info['rad_min'] + ratio * (info['rad_max'] - info['rad_min'])
        self.target_rads[idx] = rad_val

        display_val = math.degrees(rad_val) if self.is_deg_mode else rad_val
        self.spin_boxes[idx].setValue(display_val)
        self.updating_widgets = False

        if self.live_drag:
            self.bridge.send_trajectory(self.target_rads, duration_sec=0.08)

    def apply_preset(self, deg_list):
        self.updating_widgets = True
        for i in range(5):
            rad = math.radians(deg_list[i])
            self.target_rads[i] = rad
            val = deg_list[i] if self.is_deg_mode else rad
            self.spin_boxes[i].setValue(val)
            self._sync_slider_from_target(i)
        self.updating_widgets = False

        dur = self.sp_duration.value()
        self.bridge.send_trajectory(self.target_rads, duration_sec=dur)

    def send_to_gazebo(self):
        dur = self.sp_duration.value()
        self.bridge.send_trajectory(self.target_rads, duration_sec=dur)

    def read_actual_as_target(self):
        self.updating_widgets = True
        for i in range(5):
            self.target_rads[i] = self.actual_rads[i]
            val = math.degrees(self.actual_rads[i]) if self.is_deg_mode else self.actual_rads[i]
            self.spin_boxes[i].setValue(val)
            self._sync_slider_from_target(i)
        self.updating_widgets = False

    def stop_and_hold(self):
        self.read_actual_as_target()
        self.bridge.send_trajectory(self.actual_rads, duration_sec=0.05)

    def _on_joint_states_update(self, pos, vel, eff):
        self.actual_rads = pos
        self.actual_efforts = eff

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

    def _on_connection_changed(self, connected):
        if connected:
            self.status_badge.setText('🟢 ĐÃ KẾT NỐI GAZEBO SIM')
            self.status_badge.setStyleSheet('background-color: #14532d; color: #bbf7d0; padding: 4px 12px; border-radius: 12px; font-weight: bold; font-size: 12px;')
        else:
            self.status_badge.setText('🔴 MẤT KẾT NỐI GAZEBO')
            self.status_badge.setStyleSheet('background-color: #7f1d1d; color: #fecaca; padding: 4px 12px; border-radius: 12px; font-weight: bold; font-size: 12px;')

    def _check_connection(self):
        if time.time() - self.bridge.last_msg_time > 2.0 and self.bridge.is_connected:
            self.bridge.is_connected = False
            self.bridge.connection_signal.emit(False)


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
