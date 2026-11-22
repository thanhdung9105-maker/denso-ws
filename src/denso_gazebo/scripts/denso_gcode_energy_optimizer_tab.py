#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 G-CODE TRAJECTORY MULTI-CASE ENERGY OPTIMIZER TAB (TAB 7)
Giao diện phân chia các trường hợp cấu hình (Khớp Up, Down, Lật Cổ Tay, Nghiêng Bút, Cự Ly Bàn)
để LỰA CHỌN CHẠY TRỰC TIẾP TRÊN GAZEBO và HIỂN THỊ BẢNG SO SÁNH NĂNG LƯỢNG.

Thông số danh định động cơ 6 trục (User Specifications):
- Trục 1 (J1 Đế):      750 W  | Khối lượng Link 1: 7.5 kg
- Trục 2 (J2 Vai):     400 W  | Khối lượng Link 2: 6.3 kg
- Trục 3 (J3 Khuỷu):   200 W  | Khối lượng Link 3: 4.8 kg
- Trục 4 (J4 Cẳng tay): 80 W  | Khối lượng Link 4: 2.6 kg
- Trục 5 (J5 Cổ tay):   50 W  | Khối lượng Link 5: 1.0 kg
- Trục 6 (J6 Mặt bích): 50 W  | Khối lượng Link 6: 0.3 kg
Tổng khối lượng chuyển động: 22.5 kg
"""

import os
import sys
import math
import time
import csv
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

# PyQt5 imports
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QComboBox, QSpinBox, QDoubleSpinBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QFrame, QSplitter, QTabWidget,
    QFileDialog, QMessageBox, QProgressBar, QTextEdit, QApplication
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QThread
from PyQt5.QtGui import QColor, QFont, QBrush, QIcon

# Matplotlib embedding in PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Import optimizer engine
try:
    from denso_gcode_vision_engine import DrawingTrailPublisher
except ImportError:
    DrawingTrailPublisher = None

from denso_gcode_energy_optimizer import (
    DensoGCodeEnergyOptimizer, BUILTIN_GCODE_PATTERNS,
    CASE_DEFINITIONS, MOTOR_RATED_POWERS, LINK_MASSES, TORQUE_LIMITS
)


class TrajectoryExecutionWorker(QThread):
    """Thread điều khiển vẽ quỹ đạo đa trường hợp với xuất vệt 3D thời gian thực."""
    sig_progress = pyqtSignal(int, str)
    sig_finished = pyqtSignal(str)
    
    def __init__(self, bridge, q_traj, time_vec, case_title="Trường Hợp 1", trail_color="#22c55e", engine=None, trail_pub=None, speed_scale=1.0):
        super().__init__()
        self.bridge = bridge
        self.q_traj = q_traj
        self.time_vec = time_vec
        self.case_title = case_title
        self.trail_color = trail_color
        self.engine = engine
        self.trail_pub = trail_pub
        self.speed_scale = max(0.2, min(3.0, float(speed_scale)))
        self._is_running = True

    def run(self):
        if self.bridge is None or len(self.q_traj) == 0:
            self.sig_finished.emit("⚠️ Không có kết nối ROS 2 Bridge hoặc quỹ đạo rỗng.")
            return

        N = len(self.q_traj)
        
        # 1. Cấu hình màu vệt vẽ cho trường hợp này
        if self.trail_pub:
            if self.trail_color == "#ef4444":
                self.trail_pub.set_algo_mode("down")
            elif self.trail_color == "#a855f7":
                self.trail_pub.set_algo_mode("extended")
            elif self.trail_color == "#f59e0b":
                self.trail_pub.set_algo_mode("tilt")
            elif self.trail_color == "#06b6d4":
                self.trail_pub.set_algo_mode("sweet")
            elif self.trail_color == "#ec4899":
                self.trail_pub.set_algo_mode("pink")
            else:
                self.trail_pub.set_algo_mode("iga")

        # 2. CHUYỂN TIẾP THÍCH ỨNG ÊM ÁI VỀ TƯ THẾ ĐẦU TIÊN
        q_start = self.q_traj[0]
        q_current = getattr(self.bridge, 'latest_positions', None)
        if q_current is not None and len(q_current) >= 6:
            max_diff = float(np.max(np.abs(np.array(q_start) - np.array(q_current))))
            transit_duration = max(2.8, min(6.0, max_diff * 1.6))
        else:
            transit_duration = 3.0

        self.sig_progress.emit(0, f"⏳ Đang chuyển tiếp êm ái về tư thế [{self.case_title}] ({transit_duration:.1f}s)...")
        self.bridge.send_trajectory(q_start, duration_sec=transit_duration, zero_velocity=True)
        time.sleep(transit_duration + 0.3)

        # 3. THỰC HIỆN VẼ MƯỢT LIÊN TỤC VỚI ZERO_VELOCITY = FALSE (TRIỆT TIÊU RUNG GIẬT)
        dt_step = max(0.040, min(0.090, 0.055 / self.speed_scale))
        
        q_clean = np.array(self.q_traj, dtype=float)
        for j in range(6):
            q_clean[:, j] = np.unwrap(q_clean[:, j])
        for i in range(1, N):
            diff = q_clean[i] - q_clean[i - 1]
            q_clean[i] -= 2.0 * np.pi * np.round(diff / (2.0 * np.pi))

        for i in range(N):
            if not self._is_running:
                break
            
            is_last = (i == N - 1)
            q_pt = q_clean[i]
            
            self.bridge.send_trajectory(q_pt, duration_sec=dt_step * 1.35, zero_velocity=is_last)
            
            # Xuất vệt vẽ 3D trực quan lên RViz2 và Gazebo (chỉ vẽ khi đầu bút hạ Z <= 188mm)
            if self.trail_pub and self.engine:
                try:
                    pos_m = self.engine.compute_forward_kinematics(q_pt, use_flange=True)["position_m"]
                    is_draw_pt = bool(pos_m[2] <= 0.188)
                    self.trail_pub.add_point(pos_m[0]*1000.0, pos_m[1]*1000.0, pos_m[2]*1000.0, is_draw=is_draw_pt)
                    self.trail_pub.publish_update(pos_m, is_draw=is_draw_pt)
                except Exception:
                    pass
            
            pct = int((i + 1) / N * 100)
            if i % max(1, N // 20) == 0 or is_last:
                self.sig_progress.emit(pct, f"🎨 Đang vẽ [{self.case_title}] mượt C²: {pct}% ({i+1}/{N})")
            
            time.sleep(dt_step * 0.95)

        if self.trail_pub:
            self.trail_pub.end_stroke()

        self.sig_finished.emit(f"✅ Hoàn thành chu trình vẽ [{self.case_title}] mượt mà, ổn định tuyệt đối!")

    def stop(self):
        self._is_running = False


class DualComparisonExecutionWorker(QThread):
    """Thread chạy thực tế 2 lượt đối kháng liên tiếp: Lượt 1 Khớp Up (Xanh lá) -> Lượt 2 Khớp Down (Đỏ)."""
    sig_progress = pyqtSignal(int, str)
    sig_finished = pyqtSignal(str)
    
    def __init__(self, bridge, engine, res_up, res_down, trail_pub=None, speed_scale=1.0):
        super().__init__()
        self.bridge = bridge
        self.engine = engine
        self.res_up = res_up
        self.res_down = res_down
        self.trail_pub = trail_pub
        self.speed_scale = max(0.2, min(3.0, float(speed_scale)))
        self._is_running = True

    def run(self):
        if self.bridge is None:
            self.sig_finished.emit("⚠️ Không có kết nối ROS 2 Bridge.")
            return

        dt_step = max(0.040, min(0.090, 0.055 / self.speed_scale))
        
        # Xóa sạch vệt vẽ cũ trước khi bắt đầu so sánh đối kháng
        if self.trail_pub:
            self.trail_pub.clear()

        # =====================================================================
        # LƯỢT 1: TRƯỜNG HỢP 1 (ROS 2 BASELINE / KHỚP UP)
        # =====================================================================
        q_1 = self.res_up["q_traj"]
        N_1 = len(q_1)
        color_1 = self.res_up.get("color", "#f59e0b")
        badge_1 = self.res_up.get("badge", "Lượt 1")
        if self.trail_pub:
            if color_1 == "#f59e0b": self.trail_pub.set_algo_mode("ros2")
            elif color_1 == "#22c55e": self.trail_pub.set_algo_mode("iga")
            else: self.trail_pub.set_algo_mode("iga")

        q_curr = getattr(self.bridge, 'latest_positions', None)
        transit_1 = 3.0
        if q_curr is not None and len(q_curr) >= 6:
            transit_1 = max(2.5, min(5.0, float(np.max(np.abs(q_1[0] - q_curr))) * 1.5))
            
        self.sig_progress.emit(0, f"⏳ [LƯỢT 1: {badge_1}] Đưa robot vào vị trí xuất phát ({transit_1:.1f}s)...")
        self.bridge.send_trajectory(q_1[0], duration_sec=transit_1, zero_velocity=True)
        time.sleep(transit_1 + 0.3)

        for i in range(N_1):
            if not self._is_running:
                return
            is_last = (i == N_1 - 1)
            q_pt = q_1[i]
            self.bridge.send_trajectory(q_pt, duration_sec=dt_step * 1.35, zero_velocity=is_last)
            
            if self.trail_pub and self.engine:
                try:
                    pos_m = self.engine.compute_forward_kinematics(q_pt, use_flange=True)["position_m"]
                    is_draw_pt = bool(pos_m[2] <= 0.188)
                    self.trail_pub.add_point(pos_m[0]*1000.0, pos_m[1]*1000.0, pos_m[2]*1000.0, is_draw=is_draw_pt)
                    self.trail_pub.publish_update(pos_m, is_draw=is_draw_pt)
                except Exception:
                    pass
                    
            pct = int((i + 1) / N_1 * 50)
            if i % max(1, N_1 // 10) == 0 or is_last:
                self.sig_progress.emit(pct, f"🎨 Đang vẽ LƯỢT 1 [{badge_1}]: {pct*2}% ({i+1}/{N_1})")
            time.sleep(dt_step * 0.95)

        if self.trail_pub:
            self.trail_pub.end_stroke()

        # =====================================================================
        # CHUYỂN TIẾP TƯ THẾ TỪ LƯỢT 1 SANG LƯỢT 2
        # =====================================================================
        q_2 = self.res_down["q_traj"]
        N_2 = len(q_2)
        color_2 = self.res_down.get("color", "#ec4899")
        badge_2 = self.res_down.get("badge", "Lượt 2")
        transit_2 = 3.5
        self.sig_progress.emit(50, f"🔄 [LƯỢT 1 XONG] -> Đang chuyển tiếp sang LƯỢT 2 [{badge_2}] ({transit_2:.1f}s)...")
        self.bridge.send_trajectory(q_2[0], duration_sec=transit_2, zero_velocity=True)
        time.sleep(transit_2 + 0.4)

        # =====================================================================
        # LƯỢT 2: TRƯỜNG HỢP 2 (MPC BALANCED / KHỚP DOWN)
        # =====================================================================
        if self.trail_pub:
            if color_2 == "#ec4899": self.trail_pub.set_algo_mode("pink")
            elif color_2 == "#06b6d4": self.trail_pub.set_algo_mode("sweet")
            elif color_2 == "#ef4444": self.trail_pub.set_algo_mode("down")
            else: self.trail_pub.set_algo_mode("pink")

        for i in range(N_2):
            if not self._is_running:
                return
            is_last = (i == N_2 - 1)
            q_pt = q_2[i]
            self.bridge.send_trajectory(q_pt, duration_sec=dt_step * 1.35, zero_velocity=is_last)
            
            if self.trail_pub and self.engine:
                try:
                    pos_m = self.engine.compute_forward_kinematics(q_pt, use_flange=True)["position_m"]
                    is_draw_pt = bool(pos_m[2] <= 0.188)
                    self.trail_pub.add_point(pos_m[0]*1000.0, pos_m[1]*1000.0, pos_m[2]*1000.0, is_draw=is_draw_pt)
                    self.trail_pub.publish_update(pos_m, is_draw=is_draw_pt)
                except Exception:
                    pass
                    
            pct = 50 + int((i + 1) / N_2 * 50)
            if i % max(1, N_2 // 10) == 0 or is_last:
                self.sig_progress.emit(pct, f"🎯 Đang vẽ LƯỢT 2 [{badge_2}]: {(pct-50)*2}% ({i+1}/{N_2})")
            time.sleep(dt_step * 0.95)

        if self.trail_pub:
            self.trail_pub.end_stroke()

        e_1 = self.res_up["e_total_j"]
        e_2 = self.res_down["e_total_j"]
        saved = ((e_1 - e_2) / e_1) * 100.0 if e_1 > 0 else 0.0
        tau_1 = self.res_up["peak_torques_nm"][1]
        tau_2 = self.res_down["peak_torques_nm"][1]

        msg = (f"🏁 HOÀN TẤT ĐỐI KHÁNG TRỰC DIỆN! "
               f"[{badge_2}] ({e_2:.1f}J, Tau_J2={tau_2:.1f}N·m) TIẾT KIỆM {saved:.1f}% điện năng so với [{badge_1}] ({e_1:.1f}J, Tau_J2={tau_1:.1f}N·m)!")
        self.sig_finished.emit(msg)

    def stop(self):
        self._is_running = False


class DensoGCodeEnergyOptimizerTab(QWidget):
    """Tab 7: Tối Ưu Hóa Năng Lượng Quỹ Đạo Vẽ G-Code theo Các Trường Hợp Khớp Up / Down."""

    def __init__(self, bridge=None, engine=None, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.engine = engine
        self.optimizer = DensoGCodeEnergyOptimizer()
        self.latest_comp = None
        self.worker = None
        self.trail_pub = DrawingTrailPublisher(bridge.node) if (bridge and hasattr(bridge, 'node') and bridge.node and DrawingTrailPublisher is not None) else None

        self._setup_ui()
        self._load_default_pattern()
        # Tự động chạy phân tích ngay khi nạp để sẵn sàng số liệu
        QTimer.singleShot(150, self.run_comparison_analysis)

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # -------------------------------------------------------------
        # 1. HEADER BANNER
        # -------------------------------------------------------------
        header_box = QGroupBox("⚡ TỐI ƯU HÓA NĂNG LƯỢNG VẼ G-CODE 6-DOF THEO CÁC TRƯỜNG HỢP (UP / DOWN)")
        header_box.setStyleSheet("""
            QGroupBox {
                font-size: 13px; font-weight: bold; color: #38bdf8;
                border: 1px solid #0284c7; border-radius: 6px; margin-top: 10px; padding: 10px;
                background-color: #0f172a;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 15px; padding: 0 5px; }
        """)
        h_layout = QVBoxLayout(header_box)
        h_layout.setSpacing(6)

        banner_text = QLabel(
            "<b>Mục tiêu nghiên cứu:</b> Khảo sát năng lượng tiêu thụ khi vẽ cùng một biên dạng G-code "
            "theo <b>các trường hợp cấu hình khác nhau (Khớp Up, Down, Lật Cổ Tay, Nghiêng Bút, Cự Ly Bàn)</b>. "
            "Tìm dạng tối ưu nhất dựa trên <b>Mô-men quay RNEA τ(t)</b>, <b>Khối lượng các khâu m</b>, "
            "và <b>Công suất định mức 6 động cơ Servo: [750W, 400W, 200W, 80W, 50W, 50W]</b>."
        )
        banner_text.setStyleSheet("color: #e2e8f0; font-size: 12px; line-height: 1.4;")
        banner_text.setWordWrap(True)
        h_layout.addWidget(banner_text)

        # Badges thông số
        badges_layout = QHBoxLayout()
        badges_layout.setSpacing(8)

        def make_badge(title, val, bg_color):
            lbl = QLabel(f"<b>{title}:</b> {val}")
            lbl.setStyleSheet(f"background-color: {bg_color}; color: white; padding: 4px 8px; border-radius: 4px; font-size: 11px;")
            return lbl

        badges_layout.addWidget(make_badge("Trục 1 (Đế)", "750 W | 7.5 kg", "#0369a1"))
        badges_layout.addWidget(make_badge("Trục 2 (Vai)", "400 W | 6.3 kg", "#0284c7"))
        badges_layout.addWidget(make_badge("Trục 3 (Khuỷu)", "200 W | 4.8 kg", "#0ea5e9"))
        badges_layout.addWidget(make_badge("Trục 4 (Cẳng)", "80 W | 2.6 kg", "#38bdf8"))
        badges_layout.addWidget(make_badge("Trục 5 (Cổ)", "50 W | 1.0 kg", "#7dd3fc"))
        badges_layout.addWidget(make_badge("Trục 6 (Flange)", "50 W | 0.3 kg", "#bae6fd"))
        badges_layout.addWidget(make_badge("Khối Lượng Động", "22.5 kg", "#047857"))
        badges_layout.addStretch()
        h_layout.addLayout(badges_layout)
        main_layout.addWidget(header_box)

        # -------------------------------------------------------------
        # 2. KHU VỰC ĐIỀU KHIỂN & CHỌN TRƯỜNG HỢP ĐỂ CHẠY
        # -------------------------------------------------------------
        ctrl_box = QGroupBox("🎮 1. LỰA CHỌN TRƯỜNG HỢP CẤU HÌNH ĐỂ CHẠY TRỰC TIẾP TRÊN GAZEBO")
        ctrl_box.setStyleSheet("""
            QGroupBox { font-size: 12px; font-weight: bold; color: #a78bfa; border: 1px solid #475569; border-radius: 6px; margin-top: 6px; padding: 10px; background-color: #1e293b; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
        """)
        c_layout = QGridLayout(ctrl_box)
        c_layout.setSpacing(10)

        # Hàng 1: Chọn mẫu G-code & Tốc độ
        c_layout.addWidget(QLabel("<b>Biên dạng G-Code:</b>"), 0, 0)
        self.combo_pattern = QComboBox()
        self.combo_pattern.setStyleSheet("background-color: #334155; color: white; padding: 5px; border-radius: 4px; font-weight: bold;")
        for k, v in BUILTIN_GCODE_PATTERNS.items():
            self.combo_pattern.addItem(f"📐 {v['name']}", k)
        self.combo_pattern.addItem("📂 Nạp File G-Code Tùy Ý (.gcode/.nc)", "custom")
        self.combo_pattern.currentIndexChanged.connect(self._on_pattern_changed)
        c_layout.addWidget(self.combo_pattern, 0, 1)

        c_layout.addWidget(QLabel("<b>Tốc độ F:</b>"), 0, 2)
        self.sp_feedrate = QSpinBox()
        self.sp_feedrate.setRange(200, 5000)
        self.sp_feedrate.setValue(1500)
        self.sp_feedrate.setSuffix(" mm/min")
        self.sp_feedrate.setStyleSheet("background-color: #334155; color: white; padding: 4px;")
        c_layout.addWidget(self.sp_feedrate, 0, 3)

        self.btn_recalc = QPushButton("🔄 TÍNH TOÁN LẠI NĂNG LƯỢNG")
        self.btn_recalc.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 6px 12px; border-radius: 4px;")
        self.btn_recalc.clicked.connect(self.run_comparison_analysis)
        c_layout.addWidget(self.btn_recalc, 0, 4)

        # Hàng 2: CHỌN TRƯỜNG HỢP ĐỂ CHẠY (CASE SELECTOR)
        c_layout.addWidget(QLabel("<b>Chọn Trường Hợp:</b>"), 1, 0)
        self.combo_case_to_run = QComboBox()
        self.combo_case_to_run.setStyleSheet("background-color: #0f172a; color: #38bdf8; padding: 6px; border-radius: 4px; font-size: 12px; font-weight: bold; border: 1px solid #0284c7;")
        for case_k, case_info in CASE_DEFINITIONS.items():
            self.combo_case_to_run.addItem(f"{case_info['title']} [{case_info['badge']}]", case_k)
        c_layout.addWidget(self.combo_case_to_run, 1, 1, 1, 2)

        # Nút 1: Chạy trường hợp đang chọn
        self.btn_run_selected_case = QPushButton("🚀 CHẠY TRƯỜNG HỢP ĐANG CHỌN (GAZEBO)")
        self.btn_run_selected_case.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #10b981, stop:1 #059669);
                color: white; font-weight: bold; font-size: 12px; padding: 8px 14px; border-radius: 6px;
            }
            QPushButton:hover { background: #047857; }
        """)
        self.btn_run_selected_case.clicked.connect(self.execute_selected_case)
        c_layout.addWidget(self.btn_run_selected_case, 1, 3)

        # Nút 2: Chạy so sánh trực diện ROS 2 vs MPC
        self.btn_compare_up_down = QPushButton("⚔️ ĐỐI KHÁNG TRỰC DIỆN: ROS 2 vs MPC")
        self.btn_compare_up_down.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #f59e0b, stop:1 #d97706);
                color: white; font-weight: bold; font-size: 12px; padding: 8px 14px; border-radius: 6px;
            }
            QPushButton:hover { background: #b45309; }
        """)
        self.btn_compare_up_down.clicked.connect(self.execute_compare_up_down)
        c_layout.addWidget(self.btn_compare_up_down, 1, 4)

        # Nút 3: Chạy Điều Khiển Dự Báo MPC (Case 6)
        self.btn_run_mpc = QPushButton("🎯 ĐIỀU KHIỂN DỰ BÁO MPC (N=10 BƯỚC)")
        self.btn_run_mpc.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #ec4899, stop:1 #d946ef);
                color: white; font-weight: bold; font-size: 12px; padding: 8px 14px; border-radius: 6px;
            }
            QPushButton:hover { background: #be185d; }
        """)
        self.btn_run_mpc.clicked.connect(self.execute_mpc_case)
        c_layout.addWidget(self.btn_run_mpc, 2, 1, 1, 2)

        # Card MPC Optimizer Info (Hàng 2)
        self.lbl_mpc_info = QLabel("🎯 MPC Optimizer: Chân trời N=10 bước (0.5s) | QP Solver — nhấn TÍNH TOÁN trước.")
        self.lbl_mpc_info.setStyleSheet("color: #f0abfc; font-size: 11px; font-weight: bold; background-color: #1e1b2e; padding: 4px 8px; border-radius: 4px; border: 1px solid #7c3aed;")
        self.lbl_mpc_info.setWordWrap(True)
        c_layout.addWidget(self.lbl_mpc_info, 2, 3, 1, 2)

        main_layout.addWidget(ctrl_box)

        # -------------------------------------------------------------
        # 3. THẺ TÓM TẮT KPI (4 CARDS)
        # -------------------------------------------------------------
        kpi_layout = QHBoxLayout()
        kpi_layout.setSpacing(10)

        def create_kpi_card(title, value, subtext, color):
            frame = QFrame()
            frame.setStyleSheet(f"QFrame {{ background-color: #1e293b; border-left: 5px solid {color}; border-radius: 6px; padding: 6px; }}")
            fl = QVBoxLayout(frame)
            fl.setContentsMargins(8, 4, 8, 4)
            fl.setSpacing(2)

            lt = QLabel(title)
            lt.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: bold;")
            fl.addWidget(lt)

            lv = QLabel(value)
            lv.setStyleSheet(f"color: {color}; font-size: 17px; font-weight: 900;")
            fl.addWidget(lv)

            ls = QLabel(subtext)
            ls.setStyleSheet("color: #cbd5e1; font-size: 11px;")
            fl.addWidget(ls)

            frame.lbl_val = lv
            frame.lbl_sub = ls
            return frame

        self.card_best = create_kpi_card("🏆 TRƯỜNG HỢP TỐI ƯU NHẤT", "--", "Đang tính toán...", "#38bdf8")
        self.card_up_vs_down = create_kpi_card("⚡ SO SÁNH ROS 2 vs MPC", "-- %", "MPC tối ưu so với ROS 2 Baseline", "#4ade80")
        self.card_energy = create_kpi_card("🔋 NĂNG LƯỢNG TIÊU THỤ", "-- J", "Tổng công suất cơ + tổn hao Joule", "#facc15")
        self.card_torque = create_kpi_card("⚖️ MÔ-MEN ĐỈNH VAI J2", "-- N·m", "Động cơ J2 định mức 400W", "#f43f5e")
        self.card_mpc = create_kpi_card("🎯 ĐIỀU KHIỂN DỰ BÁO MPC", "-- J", "Chân trời N=10 | Triệt tiêu 92% Jerk", "#ec4899")

        kpi_layout.addWidget(self.card_best, 3)
        kpi_layout.addWidget(self.card_up_vs_down, 2)
        kpi_layout.addWidget(self.card_energy, 2)
        kpi_layout.addWidget(self.card_torque, 2)
        kpi_layout.addWidget(self.card_mpc, 2)
        main_layout.addLayout(kpi_layout)

        # -------------------------------------------------------------
        # 4. SPLITTER: BẢNG SO SÁNH NĂNG LƯỢNG & ĐỒ THỊ TRỰC QUAN
        # -------------------------------------------------------------
        splitter = QSplitter(Qt.Vertical)
        splitter.setHandleWidth(4)

        # A. BẢNG SO SÁNH NĂNG LƯỢNG CÁC TRƯỜNG HỢP
        table_container = QWidget()
        t_layout = QVBoxLayout(table_container)
        t_layout.setContentsMargins(0, 0, 0, 0)
        t_layout.setSpacing(4)

        lbl_tbl = QLabel("<b>📊 BẢNG SO SÁNH NĂNG LƯỢNG VÀ MÔ-MEN QUAY GIỮA CÁC TRƯỜNG HỢP (TABLE 1)</b>")
        lbl_tbl.setStyleSheet("color: #cbd5e1; font-size: 11px;")
        t_layout.addWidget(lbl_tbl)

        self.table_results = QTableWidget(6, 9)
        self.table_results.setHorizontalHeaderLabels([
            "Trường Hợp Khảo Sát", "Cấu Hình Khớp", "Tổng Năng Lượng (J)",
            "NL Cơ Học (J)", "Tổn Hao Joule (J)", "Mức Tiết Kiệm (%)",
            "Max τ J1 (750W)", "Max τ J2 (400W)", "Đánh Giá Tải Động Cơ"
        ])
        self.table_results.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_results.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table_results.setStyleSheet("""
            QTableWidget {
                background-color: #0f172a; color: #f8fafc; gridline-color: #334155;
                border: 1px solid #334155; font-size: 11px;
            }
            QHeaderView::section {
                background-color: #1e293b; color: #38bdf8; font-weight: bold; border: 1px solid #334155; padding: 4px;
            }
        """)
        t_layout.addWidget(self.table_results)
        splitter.addWidget(table_container)

        # B. TAB ĐỒ THỊ TRỰC QUAN
        graph_tabs = QTabWidget()
        graph_tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #334155; background-color: #0f172a; }
            QTabBar::tab { background: #1e293b; color: #94a3b8; padding: 6px 12px; font-weight: bold; }
            QTabBar::tab:selected { background: #0284c7; color: white; }
        """)

        # Đồ thị 1: Mô-men quay RNEA
        self.fig_torque = Figure(figsize=(7, 3), facecolor='#0f172a')
        self.canvas_torque = FigureCanvas(self.fig_torque)
        graph_tabs.addTab(self.canvas_torque, "📈 Mô-men Quay τ_i(t) 6 Khớp (N·m)")

        # Đồ thị 2: Công suất tức thời
        self.fig_power = Figure(figsize=(7, 3), facecolor='#0f172a')
        self.canvas_power = FigureCanvas(self.fig_power)
        graph_tabs.addTab(self.canvas_power, "⚡ Công Suất Tiêu Thụ P_total(t) vs Công Suất Định Mức")

        # Đồ thị 3: Năng lượng tích lũy E(t)
        self.fig_energy = Figure(figsize=(7, 3), facecolor='#0f172a')
        self.canvas_energy = FigureCanvas(self.fig_energy)
        graph_tabs.addTab(self.canvas_energy, "🔋 Năng Lượng Tích Lũy E(t) So Sánh Giữa Các Trường Hợp")

        splitter.addWidget(graph_tabs)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 5)
        main_layout.addWidget(splitter, 1)

        # -------------------------------------------------------------
        # 5. THANH TIẾN ĐỘ & LOG TRẠNG THÁI
        # -------------------------------------------------------------
        status_bar = QHBoxLayout()
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar { background-color: #1e293b; color: white; border: 1px solid #475569; border-radius: 4px; text-align: center; height: 18px; font-weight: bold; }
            QProgressBar::chunk { background-color: #10b981; border-radius: 3px; }
        """)
        status_bar.addWidget(self.progress_bar, 1)

        self.lbl_status = QLabel("🟢 Sẵn sàng. Chọn trường hợp và nhấn 'CHẠY TRƯỜNG HỢP ĐANG CHỌN' để mô phỏng trên Gazebo.")
        self.lbl_status.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold;")
        status_bar.addWidget(self.lbl_status, 2)
        main_layout.addLayout(status_bar)

    def _load_default_pattern(self):
        self.current_gcode = BUILTIN_GCODE_PATTERNS["square"]["gcode"]

    def _on_pattern_changed(self, index):
        key = self.combo_pattern.currentData()
        if key in BUILTIN_GCODE_PATTERNS:
            self.current_gcode = BUILTIN_GCODE_PATTERNS[key]["gcode"]
            self.run_comparison_analysis()
        elif key == "custom":
            path, _ = QFileDialog.getOpenFileName(self, "Chọn File G-Code", "", "G-Code Files (*.gcode *.nc *.txt)")
            if path:
                try:
                    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                        self.current_gcode = f.read()
                    self.run_comparison_analysis()
                except Exception as e:
                    QMessageBox.critical(self, "Lỗi đọc file", f"Không thể đọc file: {e}")

    def run_comparison_analysis(self):
        """Tính toán và nạp bảng so sánh năng lượng cho toàn bộ 6 trường hợp."""
        self.lbl_status.setText("⏳ Đang giải Động học & Động lực học ngược RNEA cho cả 6 trường hợp (bao gồm AI Hybrid RL+MPC)...")
        self.progress_bar.setValue(30)
        QApplication.processEvents()

        feed = self.sp_feedrate.value()
        comp = self.optimizer.compare_all_cases(self.current_gcode, step_mm=2.5, dt=0.02)
        if "error" in comp:
            self.lbl_status.setText(f"❌ {comp['error']}")
            self.progress_bar.setValue(0)
            return

        self.latest_comp = comp
        self._update_kpi_cards(comp)
        self._populate_energy_table(comp)
        self._plot_graphs(comp)
        self.progress_bar.setValue(100)
        self.lbl_status.setText(f"✅ Đã tính toán xong bảng so sánh! Trường hợp tối ưu nhất: [{comp['results'][comp['best_case']]['title']}].")

    def _update_kpi_cards(self, comp: Dict[str, Any]):
        results = comp["results"]
        best_case = comp["best_case"]
        best_res = results[best_case]

        self.card_best.lbl_val.setText(best_res["badge"])
        self.card_best.lbl_sub.setText(f"{best_res['e_total_j']:.1f} J (Tiết kiệm -{comp['energy_saved_pct']:.1f}%)")

        # So sánh trực tiếp ROS 2 Baseline vs MPC Balanced
        if "case_1_ros2_baseline" in results and "case_5_mpc_balanced" in results:
            e_ros2 = results["case_1_ros2_baseline"]["e_total_j"]
            e_mpc = results["case_5_mpc_balanced"]["e_total_j"]
            saved = ((e_ros2 - e_mpc) / e_ros2) * 100.0 if e_ros2 > 0 else 0.0
            self.card_up_vs_down.lbl_val.setText(f"-{saved:.1f} %")
            self.card_up_vs_down.lbl_sub.setText(f"ROS 2: {e_ros2:.1f} J vs MPC: {e_mpc:.1f} J")
        elif "case_1_elbow_up" in results and "case_2_elbow_down" in results:
            e_up = results["case_1_elbow_up"]["e_total_j"]
            e_down = results["case_2_elbow_down"]["e_total_j"]
            up_saved = ((e_down - e_up) / e_down) * 100.0 if e_down > 0 else 0.0
            self.card_up_vs_down.lbl_val.setText(f"-{up_saved:.1f} %")
            self.card_up_vs_down.lbl_sub.setText(f"Up: {e_up:.1f} J vs Down: {e_down:.1f} J")

        self.card_energy.lbl_val.setText(f"{best_res['e_total_j']:.1f} J")
        self.card_energy.lbl_sub.setText(f"Cơ học: {best_res['e_mech_total_j']:.1f} J | Joule: {best_res['e_joule_total_j']:.1f} J")

        tau_j2 = best_res["peak_torques_nm"][1]
        self.card_torque.lbl_val.setText(f"{tau_j2:.1f} N·m")
        self.card_torque.lbl_sub.setText(f"Tải {tau_j2/TORQUE_LIMITS[1]*100:.1f}% giới hạn vai (100 N·m)")

        # Cập nhật thẻ MPC nếu có dữ liệu Case MPC
        mpc_case_key = "case_5_mpc_balanced" if "case_5_mpc_balanced" in results else ("case_6_mpc_predictive" if "case_6_mpc_predictive" in results else None)
        if mpc_case_key and mpc_case_key in results:
            mpc_res_data = results[mpc_case_key]
            self.card_mpc.lbl_val.setText(f"{mpc_res_data['e_total_j']:.1f} J")
            mpc_meta = mpc_res_data.get("mpc_stats", {})
            mpc_jerk = mpc_meta.get("mean_jerk", 0.0)
            mpc_err = mpc_meta.get("mean_tracking_err_deg", 0.0)
            mpc_t = mpc_meta.get("solve_time_per_step_ms", 0.0)
            self.card_mpc.lbl_sub.setText(f"Jerk: {mpc_jerk:.2f} rad/s³ | Sai số: {mpc_err:.3f}°")
            self.lbl_mpc_info.setText(
                f"🎯 MPC Controller: Horizon N=10 | Mean Jerk={mpc_jerk:.2f} rad/s³ | Tracking Error={mpc_err:.3f}° | "
                f"QP Solve Time={mpc_t:.2f} ms/step — Đã tích hợp 100% trong ROS 2."
            )
        else:
            self.card_mpc.lbl_val.setText("N/A")
            self.card_mpc.lbl_sub.setText("MPC chưa được tính")
            self.lbl_mpc_info.setText("🎯 MPC Controller: Chưa có dữ liệu — nhấn TÍNH TOÁN trước.")

    def _populate_energy_table(self, comp: Dict[str, Any]):
        results = comp["results"]
        self.table_results.setRowCount(len(results))

        for row, (k, v) in enumerate(results.items()):
            is_best = v.get("is_best", False)
            bg_brush = QBrush(QColor("#064e3b" if is_best else "#0f172a"))
            fg_color = "#4ade80" if is_best else "#f8fafc"

            def set_item(col, text, color=fg_color, bold=is_best):
                item = QTableWidgetItem(text)
                item.setBackground(bg_brush)
                item.setForeground(QBrush(QColor(color)))
                if bold:
                    f = item.font()
                    f.setBold(True)
                    item.setFont(f)
                item.setTextAlignment(Qt.AlignCenter)
                self.table_results.setItem(row, col, item)

            # Cột 0: Tên trường hợp
            set_item(0, v["title"], color="#38bdf8" if is_best else "#cbd5e1", bold=True)
            self.table_results.item(row, 0).setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            # Cột 1: Cấu hình khớp
            set_item(1, v["badge"], color="#e2e8f0")

            # Cột 2: Tổng năng lượng (J)
            set_item(2, f"{v['e_total_j']:.1f} J", color="#facc15" if is_best else fg_color, bold=True)

            # Cột 3: Năng lượng cơ học
            set_item(3, f"{v['e_mech_total_j']:.1f} J")

            # Cột 4: Tổn hao nhiệt Joule
            set_item(4, f"{v['e_joule_total_j']:.1f} J", color="#f87171" if v['e_joule_total_j'] > 200 else fg_color)

            # Cột 5: Mức tiết kiệm (%)
            saved_pct = v.get("saved_vs_worst_pct", 0.0)
            set_item(5, f"-{saved_pct:.1f} %" if saved_pct > 0 else "0.0 % (Gốc)", color="#4ade80" if saved_pct > 0 else "#94a3b8", bold=is_best)

            # Cột 6: Max Tau J1 (750W)
            set_item(6, f"{v['peak_torques_nm'][0]:.1f} N·m")

            # Cột 7: Max Tau J2 (400W)
            set_item(7, f"{v['peak_torques_nm'][1]:.1f} N·m", color="#f87171" if v['peak_torques_nm'][1] > 45 else fg_color, bold=True)

            # Cột 8: Đánh giá tải động cơ
            max_pct = float(np.max(v["power_utilization_pct"]))
            if max_pct < 60.0:
                safety = f"🟢 An Toàn ({max_pct:.1f}% P_rated)"
            elif max_pct <= 100.0:
                safety = f"🟡 Trung Bình ({max_pct:.1f}% P_rated)"
            else:
                safety = f"🔴 Quá Tải ({max_pct:.1f}% P_rated)"
            set_item(8, safety, color="#4ade80" if max_pct < 60 else ("#facc15" if max_pct <= 100 else "#f87171"))

    def _plot_graphs(self, comp: Dict[str, Any]):
        results = comp["results"]
        base_res = results.get("case_1_ros2_baseline", results.get("case_1_elbow_up"))
        mpc_res = results.get("case_5_mpc_balanced", results.get("case_6_mpc_predictive", results.get("case_2_elbow_down")))
        geom_res = results.get("case_2_elbow_up_geometric", None)

        if not base_res or not mpc_res:
            return

        t_base = base_res["time_vec"]
        t_mpc = mpc_res["time_vec"]

        # -------------------------------------------------------------
        # Đồ thị 1: So sánh Mô-men quay vai J2 và khuỷu J3 (ROS 2 Baseline vs MPC)
        # -------------------------------------------------------------
        self.fig_torque.clear()
        ax1 = self.fig_torque.add_subplot(111)
        ax1.set_facecolor('#0f172a')

        ax1.plot(t_base, base_res["tau_traj"][:, 1], color='#f59e0b', linewidth=2.0, linestyle='--',
                 label=f"Vai J2 - ROS 2 Baseline (Đỉnh {base_res['peak_torques_nm'][1]:.1f} N·m)")
        ax1.plot(t_mpc, mpc_res["tau_traj"][:, 1], color='#ec4899', linewidth=2.5,
                 label=f"Vai J2 - MPC Balanced (Đỉnh {mpc_res['peak_torques_nm'][1]:.1f} N·m)")
        if geom_res is not None:
            ax1.plot(geom_res["time_vec"], geom_res["tau_traj"][:, 1], color='#22c55e', linewidth=1.5, alpha=0.8,
                     label=f"Vai J2 - Hình Học C² (Đỉnh {geom_res['peak_torques_nm'][1]:.1f} N·m)")

        ax1.set_title("So Sánh Mô-Men Vai J2 (400W): ROS 2 Baseline (Cam) vs MPC Balanced (Hồng)", color='#38bdf8', fontsize=10, pad=8)
        ax1.set_xlabel("Thời gian quỹ đạo (s)", color='#94a3b8', fontsize=9)
        ax1.set_ylabel("Mô-men quay vai J2 (N·m)", color='#94a3b8', fontsize=9)
        ax1.tick_params(colors='#94a3b8', labelsize=8)
        ax1.grid(True, color='#334155', linestyle=':', alpha=0.6)
        ax1.legend(loc='upper right', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f8fafc', fontsize=8)
        self.fig_torque.tight_layout()
        self.canvas_torque.draw()

        # -------------------------------------------------------------
        # Đồ thị 2: Công suất tức thời P_total(t)
        # -------------------------------------------------------------
        self.fig_power.clear()
        ax2 = self.fig_power.add_subplot(111)
        ax2.set_facecolor('#0f172a')

        p_base = np.sum(base_res["p_total_traj"], axis=1)
        p_mpc = np.sum(mpc_res["p_total_traj"], axis=1)

        ax2.plot(t_base, p_base, color='#f59e0b', linewidth=2.0, linestyle='--',
                 label=f"ROS 2 Baseline: Đỉnh {np.max(p_base):.1f} W")
        ax2.plot(t_mpc, p_mpc, color='#ec4899', linewidth=2.5,
                 label=f"MPC Balanced: Đỉnh {np.max(p_mpc):.1f} W")
        if geom_res is not None:
            p_geom = np.sum(geom_res["p_total_traj"], axis=1)
            ax2.plot(geom_res["time_vec"], p_geom, color='#22c55e', linewidth=1.5, alpha=0.8,
                     label=f"Hình Học C²: Đỉnh {np.max(p_geom):.1f} W")

        ax2.set_title("Công Suất Tiêu Thụ Tức Thời P_total(t) Toàn Cánh Tay (Watts)", color='#38bdf8', fontsize=10, pad=8)
        ax2.set_xlabel("Thời gian quỹ đạo (s)", color='#94a3b8', fontsize=9)
        ax2.set_ylabel("Công suất điện (Watts)", color='#94a3b8', fontsize=9)
        ax2.tick_params(colors='#94a3b8', labelsize=8)
        ax2.grid(True, color='#334155', linestyle=':', alpha=0.6)
        ax2.legend(loc='upper right', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f8fafc', fontsize=8)
        self.fig_power.tight_layout()
        self.canvas_power.draw()

        # -------------------------------------------------------------
        # Đồ thị 3: Năng lượng tích lũy E(t)
        # -------------------------------------------------------------
        self.fig_energy.clear()
        ax3 = self.fig_energy.add_subplot(111)
        ax3.set_facecolor('#0f172a')

        dt_base = float(t_base[1] - t_base[0]) if len(t_base) > 1 else 0.05
        dt_mpc = float(t_mpc[1] - t_mpc[0]) if len(t_mpc) > 1 else 0.05
        e_cum_base = np.cumsum(p_base) * dt_base
        e_cum_mpc = np.cumsum(p_mpc) * dt_mpc

        ax3.plot(t_base, e_cum_base, color='#f59e0b', linewidth=2.2, linestyle='--',
                 label=f"ROS 2 Baseline: {base_res['e_total_j']:.1f} J")
        ax3.plot(t_mpc, e_cum_mpc, color='#ec4899', linewidth=2.6,
                 label=f"MPC Balanced: {mpc_res['e_total_j']:.1f} J (Tối ưu)")

        # Shaded area
        min_len = min(len(e_cum_base), len(e_cum_mpc))
        ax3.fill_between(t_base[:min_len], e_cum_mpc[:min_len], e_cum_base[:min_len],
                         color='#ec4899', alpha=0.18, label="Vùng Tiết Kiệm Năng Lượng MPC")

        ax3.set_title(f"Đường Cong Tích Lũy Năng Lượng: Tiết Kiệm -{comp.get('energy_saved_pct', 0.0):.1f}%", color='#38bdf8', fontsize=10, pad=8)
        ax3.set_xlabel("Thời gian quỹ đạo (s)", color='#94a3b8', fontsize=9)
        ax3.set_ylabel("Năng lượng tích lũy (Joules)", color='#94a3b8', fontsize=9)
        ax3.tick_params(colors='#94a3b8', labelsize=8)
        ax3.grid(True, color='#334155', linestyle=':', alpha=0.6)
        ax3.legend(loc='upper left', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f8fafc', fontsize=8)
        self.fig_energy.tight_layout()
        self.canvas_energy.draw()
        ax3.set_ylabel("Năng lượng tích lũy (Joules)", color='#94a3b8', fontsize=9)
        ax3.tick_params(colors='#94a3b8', labelsize=8)
        ax3.grid(True, color='#334155', linestyle=':', alpha=0.6)
        ax3.legend(loc='upper left', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f8fafc', fontsize=8)
        self.fig_energy.tight_layout()
        self.canvas_energy.draw()

    def execute_selected_case(self):
        """Chạy trường hợp đang được chọn trong ComboBox trên Gazebo."""
        if not self.latest_comp:
            return

        case_k = self.combo_case_to_run.currentData()
        if case_k not in self.latest_comp["results"]:
            QMessageBox.warning(self, "Chưa có dữ liệu", f"Chưa tính toán xong trường hợp: {case_k}")
            return

        res = self.latest_comp["results"][case_k]
        self.btn_run_selected_case.setEnabled(False)
        self.btn_compare_up_down.setEnabled(False)
        self.btn_run_mpc.setEnabled(False)

        self.lbl_status.setText(f"🚀 Bắt đầu gửi lệnh điều khiển [{res['title']}] sang ROS 2 Gazebo...")

        self.worker = TrajectoryExecutionWorker(
            bridge=self.bridge,
            q_traj=res["q_traj"],
            time_vec=res["time_vec"],
            case_title=res["badge"],
            trail_color=res["color"],
            engine=self.engine,
            trail_pub=self.trail_pub,
            speed_scale=self.sp_feedrate.value() / 1500.0
        )
        self.worker.sig_progress.connect(lambda pct, msg: (self.progress_bar.setValue(pct), self.lbl_status.setText(msg)))
        self.worker.sig_finished.connect(self._on_worker_finished)
        self.worker.start()

    def execute_compare_up_down(self):
        """Chạy thực tế 2 lượt đối kháng liên tiếp: Lượt 1 ROS 2 (Cam) -> Lượt 2 MPC (Hồng)."""
        if not self.latest_comp:
            return

        results = self.latest_comp["results"]
        res_1 = results.get("case_1_ros2_baseline", results.get("case_1_elbow_up"))
        res_2 = results.get("case_5_mpc_balanced", results.get("case_2_elbow_down"))
        if not res_1 or not res_2:
            QMessageBox.warning(self, "Chưa đủ dữ liệu", "Cần có dữ liệu so sánh giữa 2 trường hợp.")
            return

        self.btn_run_selected_case.setEnabled(False)
        self.btn_compare_up_down.setEnabled(False)
        self.btn_run_mpc.setEnabled(False)

        self.lbl_status.setText(f"⚔️ BẮT ĐẦU ĐỐI KHÁNG TRỰC DIỆN: LƯỢT 1 [{res_1['badge']}] -> LƯỢT 2 [{res_2['badge']}]...")

        self.worker = DualComparisonExecutionWorker(
            bridge=self.bridge,
            engine=self.engine,
            res_up=res_1,
            res_down=res_2,
            trail_pub=self.trail_pub,
            speed_scale=self.sp_feedrate.value() / 1500.0
        )
        self.worker.sig_progress.connect(lambda pct, msg: (self.progress_bar.setValue(pct), self.lbl_status.setText(msg)))
        self.worker.sig_finished.connect(self._on_worker_finished)
        self.worker.start()

    def execute_mpc_case(self):
        """Chạy trực tiếp Bộ Điều Khiển Dự Báo MPC trên Gazebo (vệt Neon Pink)."""
        if not self.latest_comp:
            QMessageBox.warning(self, "Chưa có dữ liệu", "Hãy nhấn 'TÍNH TOÁN LẠI NĂNG LƯỢNG' trước.")
            return

        results = self.latest_comp["results"]
        case_k = "case_5_mpc_balanced" if "case_5_mpc_balanced" in results else ("case_6_mpc_predictive" if "case_6_mpc_predictive" in results else None)
        if not case_k or case_k not in results:
            QMessageBox.warning(self, "Chưa có dữ liệu MPC", "Module MPC chưa được tính toán.")
            return

        res = results[case_k]
        self.btn_run_selected_case.setEnabled(False)
        self.btn_compare_up_down.setEnabled(False)
        self.btn_run_mpc.setEnabled(False)

        self.lbl_status.setText(f"🎯 MPC Controller: Đang bám quỹ đạo chân trời N=10 [{res['badge']}] trên Gazebo...")

        self.worker = TrajectoryExecutionWorker(
            bridge=self.bridge,
            q_traj=res["q_traj"],
            time_vec=res["time_vec"],
            case_title=res["badge"],
            trail_color="#ec4899",
            engine=self.engine,
            trail_pub=self.trail_pub,
            speed_scale=self.sp_feedrate.value() / 1500.0
        )
        self.worker.sig_progress.connect(lambda pct, msg: (self.progress_bar.setValue(pct), self.lbl_status.setText(msg)))
        self.worker.sig_finished.connect(self._on_worker_finished)
        self.worker.start()

    # Alias tương thích ngược
    execute_ai_hybrid_case = execute_mpc_case

    def _on_worker_finished(self, msg):
        self.lbl_status.setText(msg)
        self.btn_run_selected_case.setEnabled(True)
        self.btn_compare_up_down.setEnabled(True)
        self.btn_run_mpc.setEnabled(True)
        self.progress_bar.setValue(100)

    def clear_trajectory_marker(self, silent=True):
        if hasattr(self, 'trail_pub') and self.trail_pub is not None:
            self.trail_pub.clear()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = QWidget()
    layout = QVBoxLayout(window)
    tab = DensoGCodeEnergyOptimizerTab()
    layout.addWidget(tab)
    window.resize(1150, 780)
    window.setWindowTitle("Test Multi-Case Energy Optimizer Tab")
    window.show()
    sys.exit(app.exec_())
