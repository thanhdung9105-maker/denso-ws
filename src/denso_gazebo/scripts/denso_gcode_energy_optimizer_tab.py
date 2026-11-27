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
import hashlib
import json
import pickle
from dataclasses import dataclass
from pathlib import Path
from uuid import uuid4
from typing import Dict, List, Optional, Tuple, Any

import numpy as np

# PyQt5 imports
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QComboBox, QSpinBox, QDoubleSpinBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QFrame, QSplitter, QTabWidget,
    QFileDialog, QMessageBox, QProgressBar, QTextEdit, QApplication, QScrollArea
)
from PyQt5.QtCore import QByteArray, QObject, Qt, QTimer, QThread, QUrl, QUrlQuery, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QBrush, QIcon
from PyQt5.QtNetwork import (
    QHttpMultiPart, QHttpPart, QNetworkAccessManager, QNetworkProxy, QNetworkReply, QNetworkRequest
)

# Matplotlib embedding in PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Import optimizer engine
try:
    from denso_gcode_vision_engine import DrawingTrailPublisher, ImageGCodeProcessor
except ImportError:
    DrawingTrailPublisher = None

from denso_gcode_joint_table import format_joint_gcode
from denso_mpc_trajectory_controller import DensoPostureMPCOptimizer

from denso_gcode_energy_optimizer import (
    DensoGCodeEnergyOptimizer, BUILTIN_GCODE_PATTERNS,
    CASE_DEFINITIONS, MOTOR_RATED_POWERS, LINK_MASSES, TORQUE_LIMITS
)


AMSTAR_URL = "http://192.168.1.249"
AMSTAR_AXES = "ABCUVW"


@dataclass(frozen=True)
class AmstarState:
    angles_deg: tuple
    job_state: str
    filename: str
    file_position: int
    eventtime: float
    motion_angles_deg: tuple | None = None


class AmstarStateMonitor(QObject):
    """Read command and motion feedback from AmstarRobot."""
    updated = pyqtSignal(object)
    gcode_received = pyqtSignal(str, str)
    MAX_REFERENCE_BYTES = 2 * 1024 * 1024

    def __init__(self, parent=None):
        super().__init__(parent)
        self.network = QNetworkAccessManager(self)
        self.network.setProxy(QNetworkProxy(QNetworkProxy.NoProxy))
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self._poll)
        self.deadline = QTimer(self)
        self.deadline.setSingleShot(True)
        self.deadline.timeout.connect(self._abort_pending)
        self.network.finished.connect(self._receive)
        self._reply = None
        self._connected = None
        self._gcode_reply = None
        self._gcode_filename = ""
        self._next_gcode_filename = ""
        self.gcode_deadline = QTimer(self)
        self.gcode_deadline.setSingleShot(True)
        self.gcode_deadline.timeout.connect(self._abort_gcode)

    def start(self):
        self.timer.start(0)

    @staticmethod
    def parse_state(data):
        result = data["result"]
        status = result["status"]
        move = status["gcode_move"]
        position = move["gcode_position"]
        indices = [move["axis_map"][axis] for axis in AMSTAR_AXES]
        if any(type(index) is not int or index < 0 for index in indices) or len(set(indices)) != 6:
            raise ValueError("Invalid Amstar axis map")
        angles = tuple(float(position[index]) for index in indices)
        eventtime = float(result["eventtime"])
        if not all(math.isfinite(value) for value in (*angles, eventtime)):
            raise ValueError("Non-finite Amstar telemetry")
        job = status.get("print_stats", {})
        sdcard = status.get("virtual_sdcard", {})
        # ROBOT_STATE reports these same six G-code angles to two decimal places.
        angles = tuple(round(angle, 2) for angle in angles)
        motion = None
        report = status.get("motion_report")
        live_position = report.get("live_position") if isinstance(report, dict) else None
        if isinstance(live_position, (list, tuple)) and len(live_position) > max(indices):
            try:
                values = tuple(float(live_position[index]) for index in indices)
                if all(math.isfinite(value) for value in values):
                    motion = values
            except (TypeError, ValueError, OverflowError):
                pass
        return AmstarState(angles, job.get("state") or "", job.get("filename") or "",
                           int(sdcard.get("file_position") or 0), eventtime, motion)

    def _poll(self):
        if self._reply is not None:
            return
        url = QUrl(AMSTAR_URL + "/printer/objects/query?"
                   "gcode_move=gcode_position,axis_map&motion_report=live_position"
                   "&print_stats=state,filename&virtual_sdcard=file_position")
        self._reply = self.network.get(self._request(url))
        self.deadline.start(1500)

    @staticmethod
    def _request(url):
        request = QNetworkRequest(url)
        request.setAttribute(QNetworkRequest.RedirectPolicyAttribute, QNetworkRequest.ManualRedirectPolicy)
        api_key = os.environ.get("AMSTAR_MOONRAKER_API_KEY")
        if api_key:
            request.setRawHeader(b"X-Api-Key", api_key.encode("utf-8"))
        return request

    def request_gcode(self, filename):
        parts = filename.split("/")
        if (not filename.lower().endswith((".gcode", ".nc"))
                or any(part in ("", ".", "..") for part in parts) or "\\" in filename):
            return
        if self._gcode_reply is not None:
            if filename != self._gcode_filename:
                self._next_gcode_filename = filename
            return
        url = QUrl(AMSTAR_URL)
        url.setPath("/server/files/gcodes/" + filename)
        self._gcode_filename = filename
        self._gcode_reply = self.network.get(self._request(url))
        self._gcode_reply.downloadProgress.connect(self._check_gcode_size)
        self.gcode_deadline.start(5000)

    def _check_gcode_size(self, received, total):
        if max(received, total) > self.MAX_REFERENCE_BYTES:
            self._abort_gcode()

    def _abort_gcode(self):
        if self._gcode_reply is not None and not self._gcode_reply.isFinished():
            self._gcode_reply.abort()

    def _receive_gcode(self, reply):
        self.gcode_deadline.stop()
        filename = self._gcode_filename
        self._gcode_reply = None
        self._gcode_filename = ""
        if (reply.error() == QNetworkReply.NoError
                and reply.attribute(QNetworkRequest.HttpStatusCodeAttribute) == 200
                and reply.bytesAvailable() <= self.MAX_REFERENCE_BYTES):
            self.gcode_received.emit(filename, bytes(reply.readAll()).decode("utf-8", errors="replace"))
        reply.deleteLater()
        if self._next_gcode_filename:
            filename, self._next_gcode_filename = self._next_gcode_filename, ""
            self.request_gcode(filename)

    def _abort_pending(self):
        reply = self._reply
        if reply is not None and not reply.isFinished():
            reply.abort()

    def _receive(self, reply):
        if reply is self._gcode_reply:
            self._receive_gcode(reply)
            return
        if reply is not self._reply:
            reply.deleteLater()
            return
        self.deadline.stop()
        self._reply = None
        state = None
        if reply.error() == QNetworkReply.NoError:
            try:
                state = self.parse_state(json.loads(bytes(reply.readAll())))
            except (ValueError, KeyError, TypeError, IndexError, OverflowError, AttributeError):
                pass
        reply.deleteLater()
        connected = state is not None
        if connected or self._connected is not False:
            self.updated.emit(state)
        self._connected = connected
        self.timer.start(100 if connected else 1000)



def _prepare_timed_joint_path(q_traj, time_vec, speed_scale=1.0, engine=None):
    """Build a monotonic, limit-aware time base and smooth joint velocities."""
    q = np.asarray(q_traj, dtype=float).copy()
    if q.ndim != 2 or q.shape[1] != 6 or not len(q):
        raise ValueError("Quỹ đạo khớp phải có dạng N x 6 và không được rỗng.")
    for joint_idx in range(6):
        q[:, joint_idx] = np.unwrap(q[:, joint_idx])

    n = len(q)
    if n == 1:
        return q, np.zeros(1, dtype=float), np.zeros_like(q)

    t_src = np.asarray(time_vec, dtype=float).reshape(-1) if time_vec is not None else np.empty(0)
    if (len(t_src) != n or not np.all(np.isfinite(t_src))
            or np.any(np.diff(t_src) <= 0.0)):
        t_src = np.arange(n, dtype=float) * 0.055

    scale = max(0.2, min(3.0, float(speed_scale)))
    dt = np.maximum(np.diff(t_src) / scale, 0.025)
    dq = np.abs(np.diff(q, axis=0))
    qd_max = np.array([4.5815, 4.1888, 5.2360, 5.2360, 5.2360, 7.3304])
    qdd_max = np.array([15.0, 12.0, 15.0, 20.0, 20.0, 25.0])
    dt = np.maximum(dt, np.max(dq / (0.82 * qd_max[None, :]), axis=1))
    # Giới hạn tốc độ TCP theo F của giao diện (mặc định 1500 mm/min).
    # Lịch MPC chỉ nhìn q nên có thể yêu cầu 5 mm trong 50 ms (100 mm/s),
    # nhanh hơn feedrate; khi đó Gazebo không bám kịp và nét bị dồn/rối.
    if engine is not None and n > 1:
        positions = np.asarray([
            engine.compute_forward_kinematics(qi, use_flange=True)["position_m"]
            for qi in q
        ], dtype=float)
        cart_dist_mm = np.linalg.norm(np.diff(positions, axis=0), axis=1) * 1000.0
        tcp_speed_mm_s = 1500.0 / 60.0 * scale
        dt = np.maximum(dt, cart_dist_mm / max(tcp_speed_mm_s, 1.0))
    t = np.concatenate(([0.0], np.cumsum(dt)))

    def joint_velocities(times):
        v = np.zeros_like(q)
        if n > 2:
            v[1:-1] = (q[2:] - q[:-2]) / (times[2:, None] - times[:-2, None])
        elif n == 2:
            v[0] = v[1] = (q[1] - q[0]) / max(times[1] - times[0], 1e-6)
        v[0] = 0.0
        v[-1] = 0.0
        return v

    if n >= 3:
        qd = joint_velocities(t)
        qdd = np.gradient(qd, t, axis=0)
        accel_ratio = float(np.max(np.abs(qdd) / (0.82 * qdd_max[None, :])))
        if accel_ratio > 1.0:
            t *= math.sqrt(accel_ratio)
    qd = joint_velocities(t)
    return q, t, qd


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

        # Một lần chạy một bộ vệt; không cộng dồn các lượt MPC cũ cùng màu.
        if self.trail_pub:
            self.trail_pub.clear()

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
                self.trail_pub.set_algo_mode("pink")

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

        # Gửi cả quỹ đạo một lần. Phát từng điểm bằng JointTrajectory mới sẽ
        # ghi đè spline đang chạy ở arm_controller, làm đầu bút hụt và rối nét.
        try:
            q_clean, t_exec, qd_exec = _prepare_timed_joint_path(
                self.q_traj, self.time_vec, self.speed_scale
            )
            self.bridge.send_multi_point_trajectory(
                [(q_clean[i], float(t_exec[i])) for i in range(N)],
                velocities=qd_exec.tolist(),
            )
        except Exception as exc:
            self.sig_finished.emit(f"❌ Không gửi được quỹ đạo liên tục [{self.case_title}]: {exc}")
            return

        start_time = time.monotonic()
        for i in range(N):
            if not self._is_running:
                break
            due_time = start_time + 0.05 + float(t_exec[i])
            while self._is_running:
                remaining = due_time - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.01, remaining))
            if not self._is_running:
                break

            q_pt = q_clean[i]
            # Vệt được đồng bộ với thời điểm waypoint đến, không vẽ trước khi robot chạy.
            if self.trail_pub and self.engine:
                try:
                    pos_m = self.engine.compute_forward_kinematics(q_pt, use_flange=True)["position_m"]
                    is_draw_pt = bool(pos_m[2] <= 0.188)
                    self.trail_pub.add_point(pos_m[0] * 1000.0, pos_m[1] * 1000.0, pos_m[2] * 1000.0, is_draw=is_draw_pt)
                    self.trail_pub.publish_update(pos_m, is_draw=is_draw_pt)
                except Exception:
                    pass

            pct = int((i + 1) / N * 100)
            if i % max(1, N // 20) == 0 or i == N - 1:
                self.sig_progress.emit(pct, f"🎨 Đang vẽ [{self.case_title}] (MPC spline): {pct}% ({i+1}/{N})")

        if self.trail_pub:
            self.trail_pub.end_stroke()

        if self._is_running:
            self.sig_finished.emit(f"✅ Hoàn thành chu trình vẽ [{self.case_title}] theo spline MPC.")
        else:
            self.sig_finished.emit(f"⏹ Đã dừng chu trình vẽ [{self.case_title}].")

    def stop(self):
        self._is_running = False
        q_hold = getattr(self.bridge, "latest_positions", None)
        if q_hold is not None and len(q_hold) >= 6:
            self.bridge.send_trajectory(q_hold[:6], duration_sec=0.2, zero_velocity=True)


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
            else: self.trail_pub.set_algo_mode("ros2")

        q_curr = getattr(self.bridge, 'latest_positions', None)
        transit_1 = 3.0
        if q_curr is not None and len(q_curr) >= 6:
            transit_1 = max(2.5, min(5.0, float(np.max(np.abs(q_1[0] - q_curr))) * 1.5))
            
        self.sig_progress.emit(0, f"⏳ [LƯỢT 1: {badge_1}] Đưa robot vào vị trí xuất phát ({transit_1:.1f}s)...")
        self.bridge.send_trajectory(q_1[0], duration_sec=transit_1, zero_velocity=True)
        time.sleep(transit_1 + 0.3)

        q_1, t_1, qd_1 = _prepare_timed_joint_path(
            q_1, self.res_up.get("time_vec"), self.speed_scale
        )
        try:
            self.bridge.send_multi_point_trajectory(
                [(q_1[i], float(t_1[i])) for i in range(N_1)], velocities=qd_1.tolist()
            )
        except Exception as exc:
            self.sig_finished.emit(f"❌ Không gửi được quỹ đạo lượt 1: {exc}")
            return
        start_1 = time.monotonic()
        for i in range(N_1):
            if not self._is_running:
                return
            due = start_1 + 0.05 + float(t_1[i])
            while self._is_running:
                remaining = due - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.01, remaining))
            if not self._is_running:
                self.sig_finished.emit("⏹ Đã dừng lượt vẽ so sánh.")
                return
            q_pt = q_1[i]

            if self.trail_pub and self.engine:
                try:
                    pos_m = self.engine.compute_forward_kinematics(q_pt, use_flange=True)["position_m"]
                    is_draw_pt = bool(pos_m[2] <= 0.188)
                    self.trail_pub.add_point(pos_m[0] * 1000.0, pos_m[1] * 1000.0, pos_m[2] * 1000.0, is_draw=is_draw_pt)
                    self.trail_pub.publish_update(pos_m, is_draw=is_draw_pt)
                except Exception:
                    pass

            pct = int((i + 1) / N_1 * 50)
            if i % max(1, N_1 // 10) == 0 or i == N_1 - 1:
                self.sig_progress.emit(pct, f"🎨 Đang vẽ LƯỢT 1 [{badge_1}]: {pct * 2}% ({i+1}/{N_1})")

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

        q_2, t_2, qd_2 = _prepare_timed_joint_path(
            q_2, self.res_down.get("time_vec"), self.speed_scale
        )
        try:
            self.bridge.send_multi_point_trajectory(
                [(q_2[i], float(t_2[i])) for i in range(N_2)], velocities=qd_2.tolist()
            )
        except Exception as exc:
            self.sig_finished.emit(f"❌ Không gửi được quỹ đạo lượt 2: {exc}")
            return
        start_2 = time.monotonic()
        for i in range(N_2):
            if not self._is_running:
                return
            due = start_2 + 0.05 + float(t_2[i])
            while self._is_running:
                remaining = due - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.01, remaining))
            if not self._is_running:
                self.sig_finished.emit("⏹ Đã dừng lượt vẽ so sánh.")
                return
            q_pt = q_2[i]

            if self.trail_pub and self.engine:
                try:
                    pos_m = self.engine.compute_forward_kinematics(q_pt, use_flange=True)["position_m"]
                    is_draw_pt = bool(pos_m[2] <= 0.188)
                    self.trail_pub.add_point(pos_m[0] * 1000.0, pos_m[1] * 1000.0, pos_m[2] * 1000.0, is_draw=is_draw_pt)
                    self.trail_pub.publish_update(pos_m, is_draw=is_draw_pt)
                except Exception:
                    pass

            pct = 50 + int((i + 1) / N_2 * 50)
            if i % max(1, N_2 // 10) == 0 or i == N_2 - 1:
                self.sig_progress.emit(pct, f"🎯 Đang vẽ LƯỢT 2 [{badge_2}]: {(pct - 50) * 2}% ({i+1}/{N_2})")

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
        q_hold = getattr(self.bridge, "latest_positions", None)
        if q_hold is not None and len(q_hold) >= 6:
            self.bridge.send_trajectory(q_hold[:6], duration_sec=0.2, zero_velocity=True)


class ComparisonCalculationWorker(QThread):
    """Thread tính toán toàn diện 6 trường hợp Động lực học & MPC để UI không bao giờ bị đơ / xám màn."""
    sig_finished = pyqtSignal(dict)
    sig_error = pyqtSignal(str)

    def __init__(self, optimizer, gcode, step_mm=2.5, dt=0.02):
        super().__init__()
        self.optimizer = optimizer
        self.gcode = gcode
        self.step_mm = step_mm
        self.dt = dt

    def run(self):
        try:
            comp = self.optimizer.compare_all_cases(self.gcode, step_mm=self.step_mm, dt=self.dt)
            if "error" in comp:
                self.sig_error.emit(str(comp["error"]))
            else:
                self.sig_finished.emit(comp)
        except Exception as e:
            self.sig_error.emit(str(e))


class SquarePostureWorker(QThread):
    completed = pyqtSignal(object, str)
    progress = pyqtSignal(str, int)
    partial = pyqtSignal(str, object)
    def __init__(self, optimizer, gcode, spacing, roll_range, z_draw_thresh=182.0, copper_scale=1.0):
        super().__init__()
        self.optimizer, self.gcode = optimizer, gcode
        self.spacing, self.roll_range = float(spacing), roll_range
        self.z_draw_thresh = float(z_draw_thresh)
        self.copper_scale = float(copper_scale)
    def run(self):
        try:
            result = _load_posture_cache(
                self.gcode, self.spacing, self.roll_range, self.z_draw_thresh, self.copper_scale
            )
            if result is None:
                result = _calculate_posture_result(
                    self.optimizer, self.gcode, self.spacing, self.roll_range,
                    z_draw_thresh=self.z_draw_thresh,
                    copper_scale=self.copper_scale,
                    progress_callback=lambda msg, pct: self.progress.emit(msg, pct),
                    result_callback=lambda stage, items: self.partial.emit(stage, items),
                )
            self.completed.emit(result, "")
        except Exception as e:
            self.completed.emit(None, str(e))



POSTURE_CACHE_VERSION = "ofoegbu2026-empc-6dof-v16"


def _posture_cache_key(gcode, spacing_mm, roll_range, z_draw_thresh=182.0, copper_scale=1.0):
    digest = hashlib.sha256()
    digest.update(POSTURE_CACHE_VERSION.encode("utf-8"))
    digest.update(str(round(float(spacing_mm), 4)).encode("ascii"))
    digest.update(str(round(float(z_draw_thresh), 4)).encode("ascii"))
    digest.update(str(round(float(copper_scale), 4)).encode("ascii"))
    digest.update(repr(tuple(float(v) for v in roll_range)).encode("ascii"))
    # Image G-code embeds a wall-clock timestamp in its header. Ignore only
    # that non-motion metadata so the same image/settings reuse the same cache.
    canonical_gcode = "\n".join(
        line for line in gcode.splitlines()
        if not line.strip().startswith("; Generated Date:")
    )
    digest.update(canonical_gcode.encode("utf-8"))
    source_root = Path(__file__).resolve().parent
    source_files = (
        source_root / "denso_mpc_trajectory_controller.py",
        source_root / "denso_gcode_energy_optimizer.py",
        source_root / "denso_robot_energy_parameters.py",
        source_root.parent / "denso_vs6556_moveit_config" / "scripts" / "denso_dynamics_engine.py",
    )
    for source in source_files:
        try:
            digest.update(source.read_bytes())
        except OSError:
            digest.update(str(source).encode("utf-8"))
    return digest.hexdigest()


def _posture_cache_file(gcode, spacing_mm, roll_range, z_draw_thresh=182.0, copper_scale=1.0):
    cache_dir = Path(os.path.expanduser("~/.cache/denso_gazebo/postures"))
    return cache_dir / (_posture_cache_key(gcode, spacing_mm, roll_range, z_draw_thresh, copper_scale) + ".pkl")


def _load_posture_cache(gcode, spacing_mm, roll_range, z_draw_thresh=182.0, copper_scale=1.0):
    z_thresh = float(z_draw_thresh)
    c_scale = float(copper_scale)
    path = _posture_cache_file(gcode, spacing_mm, roll_range, z_thresh, c_scale)
    try:
        with path.open("rb") as cache_file:
            payload = pickle.load(cache_file)
        result = payload.get("result")
        if payload.get("version") != POSTURE_CACHE_VERSION or not result:
            return None
        strokes = ImageGCodeProcessor().parse_gcode(
            gcode, point_spacing_mm=float(spacing_mm), z_draw_thresh=z_thresh
        )
        result = dict(result)
        result.update(
            strokes=strokes,
            source_gcode=gcode,
            spacing_mm=float(spacing_mm),
            z_draw_thresh=z_thresh,
            copper_scale=c_scale,
            point_count=sum(len(st.points) for st in strokes),
            cache_key=path.stem,
        )
        return result
    except (OSError, EOFError, pickle.PickleError, AttributeError, ValueError, TypeError):
        return None


def _save_posture_cache(gcode, spacing_mm, roll_range, result, z_draw_thresh=182.0, copper_scale=1.0):
    path = _posture_cache_file(gcode, spacing_mm, roll_range, z_draw_thresh, copper_scale)
    path.parent.mkdir(parents=True, exist_ok=True)
    saved = dict(result)
    saved.pop("strokes", None)
    saved.pop("cache_key", None)
    tmp = path.with_suffix(f".{os.getpid()}.tmp")
    try:
        with tmp.open("wb") as cache_file:
            pickle.dump({"version": POSTURE_CACHE_VERSION, "result": saved}, cache_file, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(str(tmp), str(path))
    finally:
        try:
            tmp.unlink(missing_ok=True)
        except OSError:
            pass



def _posture_checkpoint_file(gcode, spacing_mm, roll_range, z_draw_thresh=182.0, copper_scale=1.0):
    return _posture_cache_file(gcode, spacing_mm, roll_range, z_draw_thresh, copper_scale).with_suffix(".progress.pkl")


def _load_posture_checkpoint(gcode, spacing_mm, roll_range, z_draw_thresh=182.0, copper_scale=1.0):
    try:
        with _posture_checkpoint_file(gcode, spacing_mm, roll_range, z_draw_thresh, copper_scale).open("rb") as f:
            payload = pickle.load(f)
        return payload if payload.get("version") == POSTURE_CACHE_VERSION else {}
    except (OSError, EOFError, pickle.PickleError, AttributeError, ValueError, TypeError):
        return {}


def _save_posture_checkpoint(gcode, spacing_mm, roll_range, checkpoint, z_draw_thresh=182.0, copper_scale=1.0):
    path = _posture_checkpoint_file(gcode, spacing_mm, roll_range, z_draw_thresh, copper_scale)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(f".{os.getpid()}.tmp")
    try:
        with tmp.open("wb") as f:
            pickle.dump({"version": POSTURE_CACHE_VERSION, **checkpoint}, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(str(tmp), str(path))
    finally:
        try:
            tmp.unlink(missing_ok=True)
        except OSError:
            pass


def _calculate_posture_result(
    optimizer, gcode, spacing_mm, roll_range, progress_callback=None,
    z_draw_thresh=182.0, result_callback=None, copper_scale=1.0,
):
    z_thresh = float(z_draw_thresh)
    c_scale = float(copper_scale)
    if hasattr(optimizer, "copper_scale"):
        optimizer.copper_scale = max(0.1, min(5.0, c_scale))
    strokes = ImageGCodeProcessor().parse_gcode(
        gcode, point_spacing_mm=float(spacing_mm), z_draw_thresh=z_thresh
    )
    pts = [[p.x * 1e-3, p.y * 1e-3, p.z * 1e-3, p.is_draw] for st in strokes for p in st.points]
    optimizer.set_j6_roll_range(*roll_range)
    checkpoint = _load_posture_checkpoint(gcode, spacing_mm, roll_range, z_thresh, c_scale)
    result = optimizer.optimize_and_select_best_posture(
        pts, dt=0.09, progress_callback=progress_callback, checkpoint=checkpoint,
        checkpoint_callback=lambda state: _save_posture_checkpoint(gcode, spacing_mm, roll_range, state, z_thresh, c_scale),
        result_callback=result_callback,
    )
    if result:
        result.update(
            strokes=strokes,
            source_gcode=gcode,
            spacing_mm=float(spacing_mm),
            z_draw_thresh=z_thresh,
            copper_scale=c_scale,
            point_count=len(pts),
            cache_key=_posture_cache_file(gcode, spacing_mm, roll_range, z_thresh, c_scale).stem,
        )
        _save_posture_cache(gcode, spacing_mm, roll_range, result, z_thresh, c_scale)
        try:
            _posture_checkpoint_file(gcode, spacing_mm, roll_range, z_thresh, c_scale).unlink(missing_ok=True)
        except OSError:
            pass
    return result


class BuiltinPosturePrecacheWorker(QThread):
    progress = pyqtSignal(str, int)
    partial = pyqtSignal(str, str, object)
    case_ready = pyqtSignal(str, object)
    finished_all = pyqtSignal()

    def __init__(self, spacing_mm, roll_range, patterns, dynamics_engine=None, z_draw_thresh=182.0, copper_scale=1.0):
        super().__init__()
        self.spacing_mm = float(spacing_mm)
        self.roll_range = tuple(float(v) for v in roll_range)
        self.patterns = list(patterns)
        self.dynamics_engine = dynamics_engine
        self.z_draw_thresh = float(z_draw_thresh)
        self.copper_scale = float(copper_scale)
        self._priority_key = None

    def prioritize(self, pattern_key):
        # The active calculation is allowed to finish; then the selected shape is
        # moved to the front of the remaining background queue.
        self._priority_key = pattern_key

    def run(self):
        pending = list(enumerate(self.patterns, 1))
        total = max(1, len(pending))
        processed = 0
        try:
            optimizer = DensoPostureMPCOptimizer(dynamics_engine=self.dynamics_engine, copper_scale=self.copper_scale)
            while pending:
                selected_index = 0
                requested_key = self._priority_key
                if requested_key:
                    for candidate_index, (_, (candidate_key, _)) in enumerate(pending):
                        if candidate_key == requested_key:
                            selected_index = candidate_index
                            break
                    self._priority_key = None
                original_index, (key, pattern) = pending.pop(selected_index)
                processed += 1
                gcode = pattern["gcode"]
                cached = _load_posture_cache(
                    gcode, self.spacing_mm, self.roll_range, self.z_draw_thresh, self.copper_scale
                )
                if cached is None:
                    self.progress.emit(
                        f"Đang quét sẵn {pattern['name']} ({processed}/{total})…",
                        int(100 * (processed - 1) / total),
                    )
                    cached = _calculate_posture_result(
                        optimizer, gcode, self.spacing_mm, self.roll_range,
                        z_draw_thresh=self.z_draw_thresh,
                        copper_scale=self.copper_scale,
                        progress_callback=lambda msg, pct, done=processed, n=total, title=pattern["name"]: self.progress.emit(
                            f"{title} — {msg}", int(100 * ((done - 1) + pct / 100.0) / n)
                        ),
                        result_callback=lambda stage, items, pattern_key=key: self.partial.emit(pattern_key, stage, items),
                    )
                if cached:
                    self.case_ready.emit(key, cached)
        except Exception as exc:
            self.progress.emit(f"Tiền quét bị gián đoạn: {exc}", 0)
        finally:
            self.finished_all.emit()


class DensoGCodeEnergyOptimizerTab(QWidget):
    """Tab 7: Tối Ưu Hóa Năng Lượng Quỹ Đạo Vẽ G-Code theo Các Trường Hợp Khớp Up / Down."""
    amstar_job_prepared = pyqtSignal(str, str)

    def __init__(self, bridge=None, engine=None, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.engine = engine
        self.optimizer = DensoGCodeEnergyOptimizer()
        self.latest_comp = None
        self.worker = None
        self._calc_worker = None
        self._amstar_network = QNetworkAccessManager(self)
        self._amstar_network.setProxy(QNetworkProxy(QNetworkProxy.NoProxy))
        self._amstar_reply = None
        self._amstar_prev_sample = None
        self._amstar_last_telemetry = None
        self.trail_pub = DrawingTrailPublisher(bridge.node) if (bridge and hasattr(bridge, 'node') and bridge.node and DrawingTrailPublisher is not None) else None

        self._setup_ui()
        self._load_default_pattern()
        self._precache_worker = None
        self._precached_keys = set()
        self._pose_scan_pending = False
        # Warm persistent caches in the background; UI stays responsive.
        QTimer.singleShot(0, self._start_builtin_posture_precache)

    def _setup_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        self.main_scroll = QScrollArea(self)
        self.main_scroll.setWidgetResizable(True)
        self.main_scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        content.setStyleSheet("background-color: #0b1120;")
        main_layout = QVBoxLayout(content)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)

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
            "<b>Mục tiêu:</b> Giữ cùng đường vẽ; chạy MPC N=10 trên từng nghiệm IK khả thi, "
            "rồi xếp hạng theo năng lượng điện ước tính sau MPC."
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

        guide_box = QGroupBox("HƯỚNG DẪN NHANH — THỰC HIỆN THEO THỨ TỰ")
        guide_box.setStyleSheet("""
            QGroupBox { color: #7dd3fc; font-weight: bold; border: 1px solid #155e75;
                border-radius: 6px; margin-top: 12px; padding: 10px 8px 8px; background: #082f49; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; background: #0b1120; }
            QTableWidget { background: #0f172a; color: #e2e8f0; gridline-color: #334155;
                border: 1px solid #334155; }
            QHeaderView::section { background: #1e293b; color: #7dd3fc; padding: 4px; font-weight: bold; }
        """)
        guide_layout = QVBoxLayout(guide_box)
        self.guide_table = QTableWidget(5, 2)
        self.guide_table.setHorizontalHeaderLabels(["Bước", "Thao tác"])
        self.guide_table.verticalHeader().setVisible(False)
        self.guide_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.guide_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.guide_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.guide_table.setSelectionMode(QTableWidget.NoSelection)
        self.guide_table.setFocusPolicy(Qt.NoFocus)
        self.guide_table.setFixedHeight(156)
        guide_steps = [
            ("1. Chọn đường", "Chọn G-code mẫu hoặc chọn ảnh Tab 5 (Logo BK, bánh răng, DENSO, xoắn ốc, trái tim)."),
            ("2. Đặt thông số", "Chọn mật độ điểm và khoảng quét J6 (min, max, bước)."),
            ("3. Quét nghiệm", "Bấm QUÉT CÁC TƯ THẾ; chờ bảng kết quả xuất hiện."),
            ("4. Chọn tư thế", "Chọn một hàng trong bảng để nạp hoặc chạy thử."),
            ("5. Chạy robot", "Dùng các nút điều khiển ở cuối mục 3; Home và xóa vệt nằm ở Tab 5."),
        ]
        for row, (step, action) in enumerate(guide_steps):
            self.guide_table.setItem(row, 0, QTableWidgetItem(step))
            self.guide_table.setItem(row, 1, QTableWidgetItem(action))
            self.guide_table.setRowHeight(row, 25)
        guide_layout.addWidget(self.guide_table)
        main_layout.addWidget(guide_box)

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
        for title, filename in (
            ("🎓 Logo Bách Khoa (Tab 5)", "logoBK.png"),
            ("⚙️ Bánh răng cơ khí (Tab 5)", "gear_pattern.png"),
            ("🤖 Logo DENSO Robot (Tab 5)", "denso_robot_logo.png"),
            ("🌀 Hoa xoắn ốc (Tab 5)", "spiral_flower.png"),
            ("❤️ Trái tim nghệ thuật (Tab 5)", "heart_shape.png"),
        ):
            self.combo_pattern.addItem(title, f"tab5_image:{filename}")
        self.combo_pattern.addItem("📂 Nạp File G-Code Tùy Ý (.gcode/.nc)", "custom")
        self.combo_pattern.currentIndexChanged.connect(self._on_pattern_changed)
        c_layout.addWidget(self.combo_pattern, 0, 1)

        c_layout.addWidget(QLabel("<b>Tốc độ F:</b>"), 0, 2)
        self.sp_feedrate = QSpinBox()
        self.sp_feedrate.setRange(200, 5000)
        self.sp_feedrate.setValue(1200)
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
        ctrl_box.hide()

        pose_box = QGroupBox("3. QUÉT VÀ CHỌN TƯ THẾ (RNEA + MPC)")
        pose_box.setStyleSheet("QGroupBox { color: #38bdf8; font-weight: bold; border: 1px solid #334155; border-radius: 6px; margin-top: 13px; padding: 12px 8px 8px; background: #0f172a; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; background: #0b1120; }")
        pose = QGridLayout(pose_box)
        self.pose_box = pose_box
        self.pose_layout = pose
        pose.addWidget(QLabel("Hình G-code"),0,0)
        pose.addWidget(self.combo_pattern,0,1,1,2)
        pose.addWidget(QLabel("J6 roll từ"),0,3)
        self.pose_j6_min=QDoubleSpinBox(); self.pose_j6_min.setRange(-180,180); self.pose_j6_min.setValue(-45); self.pose_j6_min.setSuffix("°")
        self.pose_j6_max=QDoubleSpinBox(); self.pose_j6_max.setRange(-180,180); self.pose_j6_max.setValue(45); self.pose_j6_max.setSuffix("°")
        self.pose_j6_step=QDoubleSpinBox(); self.pose_j6_step.setRange(1,180); self.pose_j6_step.setValue(45); self.pose_j6_step.setSuffix("°")
        self.pose_spacing=QDoubleSpinBox(); self.pose_spacing.setRange(1,10); self.pose_spacing.setValue(5); self.pose_spacing.setSuffix(" mm")
        pose.addWidget(self.pose_j6_min,0,4)
        pose.addWidget(QLabel("đến"),0,5); pose.addWidget(self.pose_j6_max,0,6)
        pose.addWidget(QLabel("bước"),0,7); pose.addWidget(self.pose_j6_step,0,8)
        pose.addWidget(QLabel("Mật độ"),0,9); pose.addWidget(self.pose_spacing,0,10)
        self.btn_pose_scan=QPushButton("QUÉT CÁC TƯ THẾ"); self.btn_pose_scan.clicked.connect(self.run_square_posture_analysis); pose.addWidget(self.btn_pose_scan,0,11)
        self.pose_combo=QComboBox(); pose.addWidget(self.pose_combo,1,0,1,5)
        self.btn_pose_apply=QPushButton("NẠP QUỸ ĐẠO + TƯ THẾ SANG TAB 5"); self.btn_pose_apply.clicked.connect(self.apply_selected_posture_to_tab5); self.btn_pose_apply.setEnabled(False); pose.addWidget(self.btn_pose_apply,1,5,1,4)
        self.btn_export_pose_csv=QPushButton("📥 XUẤT CSV THỰC NGHIỆM")
        self.btn_export_pose_csv.setToolTip("Xuất bảng tổng hợp các tư thế và chuỗi thời gian (q, q_dot, tau, công suất, manipulability w) ra CSV.")
        self.btn_export_pose_csv.setStyleSheet("QPushButton { background: #0f766e; color: white; font-weight: bold; padding: 6px 8px; } QPushButton:hover { background: #0d9488; } QPushButton:disabled { background: #1e293b; color: #94a3b8; }")
        self.btn_export_pose_csv.setEnabled(False)
        self.btn_export_pose_csv.clicked.connect(self._on_export_csv_clicked)
        pose.addWidget(self.btn_export_pose_csv,1,9,1,3)

        self.btn_pose_run_selected=QPushButton("▶ CHẠY TƯ THẾ ĐANG CHỌN")
        self.btn_pose_run_selected.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 8px;")
        self.btn_pose_run_selected.setEnabled(False)
        self.btn_pose_run_selected.clicked.connect(self._run_selected_posture_in_tab5)
        pose.addWidget(self.btn_pose_run_selected,2,0,1,6)
        self.btn_pose_run_best=QPushButton("🏆 CHẠY TƯ THẾ TỐI ƯU (HẠNG 1)")
        self.btn_pose_run_best.setStyleSheet("background-color: #9333ea; color: white; font-weight: bold; padding: 8px;")
        self.btn_pose_run_best.setEnabled(False)
        self.btn_pose_run_best.clicked.connect(self._run_best_posture_in_tab5)
        pose.addWidget(self.btn_pose_run_best,2,6,1,6)

        self.pose_table=QTableWidget(0,16); self.pose_table.setHorizontalHeaderLabels([
            "Hạng","Case tư thế 6-DOF","q1°","q2°","q3°","q4°","q5°","q6°",
            "IK E (J)","EMPC E (J)","Tiết kiệm %","Jerk","Tải đỉnh %","Độ linh hoạt w","Lỗi đường mm","Lệch hướng °"
        ])
        self.pose_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.pose_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.pose_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.pose_table.setSelectionMode(QTableWidget.SingleSelection)
        self.pose_table.itemSelectionChanged.connect(self._on_pose_table_row_selected)
        self.pose_table.setMinimumHeight(240)
        self.pose_table.setMaximumHeight(350)
        self.pose_table.setStyleSheet("""
            QTableWidget { background-color: #0b1120; color: #e2e8f0; gridline-color: #334155;
                border: 1px solid #334155; selection-background-color: #075985; selection-color: white; }
            QTableWidget::item { padding: 5px; }
            QHeaderView::section { background-color: #1e293b; color: #7dd3fc; font-weight: bold;
                border: 1px solid #334155; padding: 6px; }
        """)
        pose.addWidget(self.pose_table,3,0,1,12)
        pose.setRowStretch(3, 0)
        self.pose_status=QLabel("Quét đa nghiệm IK 6-DOF + Economic SQP-RTI MPC (Ofoegbu & Adeleke 2026) để xếp hạng tư thế tối ưu năng lượng nhất."); self.pose_status.setWordWrap(True); pose.addWidget(self.pose_status,4,0,1,12)
        self._pose_result = None
        self.btn_amstar_upload = QPushButton("GỬI & CHẠY TRÊN AMSTAR")
        self.btn_amstar_upload.setStyleSheet(
            "QPushButton { background: #047857; color: white; font-weight: bold; padding: 8px; }"
            "QPushButton:hover { background: #059669; }"
            "QPushButton:disabled { background: #1e293b; color: #94a3b8; }"
        )
        self.btn_amstar_upload.setEnabled(False)
        self.btn_amstar_upload.clicked.connect(self._upload_selected_to_amstar)
        pose.addWidget(self.btn_amstar_upload, 5, 0, 1, 3)
        self.btn_amstar_home = QPushButton("VỀ HOME AMSTAR (0°)")
        self.btn_amstar_home.setToolTip("Đưa sáu khớp của AmstarRobot về 0° bằng macro MOV_ZEROALL_POS.")
        self.btn_amstar_home.setStyleSheet(
            "QPushButton { background: #334155; color: white; font-weight: bold; padding: 8px; }"
            "QPushButton:hover { background: #475569; }"
            "QPushButton:disabled { background: #1e293b; color: #94a3b8; }"
        )
        self.btn_amstar_home.clicked.connect(self._return_amstar_home)
        pose.addWidget(self.btn_amstar_home, 5, 3, 1, 2)
        lbl_copper = QLabel("Hiệu chuẩn I²R:")
        lbl_copper.setToolTip("Hệ số hiệu chuẩn thực nghiệm tổn hao đồng c_j·τ_j² khi đối chiếu công suất AmstarRobot thực tế.")
        pose.addWidget(lbl_copper, 5, 5, 1, 1)
        self.sp_copper_calib = QDoubleSpinBox()
        self.sp_copper_calib.setRange(0.20, 3.00)
        self.sp_copper_calib.setSingleStep(0.05)
        self.sp_copper_calib.setDecimals(2)
        self.sp_copper_calib.setValue(1.00)
        self.sp_copper_calib.setSuffix("x")
        self.sp_copper_calib.setToolTip("1.00x = thông số danh định DENSO VS-6556; điều chỉnh khi hiệu chuẩn với dòng/công suất thực trên Amstar.")
        pose.addWidget(self.sp_copper_calib, 5, 6, 1, 1)
        self.amstar_status = QLabel("AmstarRobot · 192.168.1.249 · Tải tệp và chạy ngay khi robot sẵn sàng")
        self.amstar_status.setTextFormat(Qt.PlainText)
        self.amstar_status.setWordWrap(True)
        pose.addWidget(self.amstar_status, 5, 7, 1, 5)
        self.pose_combo.currentIndexChanged.connect(self._update_amstar_buttons)
        for spinbox in (self.pose_spacing, self.pose_j6_min, self.pose_j6_max, self.pose_j6_step, self.sp_copper_calib):
            spinbox.valueChanged.connect(self._update_amstar_buttons)
        main_layout.addWidget(pose_box)


        # -------------------------------------------------------------
        # 4. THẺ TÓM TẮT KPI (4 CARDS)
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
        for card in (self.card_best, self.card_up_vs_down, self.card_energy, self.card_torque, self.card_mpc):
            card.hide()

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
        splitter.hide()
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
        for button in (self.btn_run_selected_case, self.btn_compare_up_down, self.btn_run_mpc):
            button.setEnabled(False)
        self.progress_bar.hide()
        self.lbl_status.hide()
        main_layout.addLayout(status_bar)
        self.main_scroll.setWidget(content)
        root_layout.addWidget(self.main_scroll)
        self.main_layout = main_layout

    def attach_drawing_controls(self, controls):
        """Embed the drawing controller at the bottom of section 3."""
        if controls is not None:
            controls.setTitle("🎮 ĐIỀU KHIỂN VẼ VÀ THEO DÕI")
            controls.setStyleSheet("QGroupBox { color: #38bdf8; font-size: 11px; font-weight: bold; border: 1px solid #334155; border-radius: 7px; margin-top: 10px; padding: 10px 8px 8px; background: #0f172a; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; background: #0b1120; } QLabel { color: #cbd5e1; }")
            self.pose_layout.addWidget(controls, 6, 0, 1, 12)

    def _current_pose_roll_range(self):
        return (self.pose_j6_min.value(), self.pose_j6_max.value(), self.pose_j6_step.value())

    def _current_copper_scale(self) -> float:
        return float(self.sp_copper_calib.value()) if hasattr(self, "sp_copper_calib") else 1.0

    def _current_z_draw_thresh(self) -> float:
        try:
            target, _ = self._find_tab5_drawing_widget()
            if target is not None and hasattr(target, "sp_z_draw"):
                return float(target.sp_z_draw.value()) + 2.0
        except Exception:
            pass
        return 182.0

    def _collect_precache_patterns(self):
        key = self.combo_pattern.currentData()
        if key in BUILTIN_GCODE_PATTERNS:
            # Prepare all built-in shapes in the background; process the active
            # one first so it can be selected as soon as its cache is ready.
            patterns = list(BUILTIN_GCODE_PATTERNS.items())
            patterns.sort(key=lambda entry: entry[0] != key)
            return patterns
        if isinstance(key, str) and key.startswith("tab5_image:") and self.current_gcode.strip():
            return [(key, {"name": key.split(":", 1)[1], "gcode": self.current_gcode})]
        return []

    def _start_builtin_posture_precache(self):
        if self._precache_worker is not None and self._precache_worker.isRunning():
            return
        patterns = self._collect_precache_patterns()
        selected_key = self.combo_pattern.currentData()
        patterns.sort(key=lambda entry: entry[0] != selected_key)
        self._precached_pattern_keys = {key for key, _ in patterns}
        self._precache_worker = BuiltinPosturePrecacheWorker(
            self.pose_spacing.value(), self._current_pose_roll_range(), patterns, self.engine,
            z_draw_thresh=self._current_z_draw_thresh(),
            copper_scale=self._current_copper_scale(),
        )
        self._precache_worker.progress.connect(self._on_precache_progress)
        self._precache_worker.partial.connect(self._on_builtin_posture_partial)
        self._precache_worker.case_ready.connect(self._on_builtin_posture_cached)
        self._precache_worker.finished_all.connect(self._on_builtin_precache_finished)
        self._precache_worker.start()

    def _on_precache_progress(self, message, percent):
        self.pose_status.setText(message)

    def _on_builtin_posture_partial(self, pattern_key, stage, items):
        if self.combo_pattern.currentData() != pattern_key:
            return
        current_key = _posture_cache_key(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
            self._current_z_draw_thresh(), self._current_copper_scale(),
        )
        if self._active_pose_cache_key != current_key:
            self._active_pose_cache_key = current_key
            self._pose_partial_items = []
            self._show_pose_scan_placeholder()
        self._on_pose_partial_result(stage, items)

    def _on_builtin_posture_cached(self, pattern_key, result):
        self._precached_keys.add(pattern_key)
        if self.combo_pattern.currentData() != pattern_key:
            return
        current_key = _posture_cache_key(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
            self._current_z_draw_thresh(), self._current_copper_scale(),
        )
        if result.get("cache_key") != current_key:
            return
        self._on_square_posture_done(result, "")
        self.pose_status.setText("Đã hiện kết quả quét sẵn từ cache; có thể chọn tư thế ngay.")

    def _on_builtin_precache_finished(self):
        if self._pose_scan_pending:
            self._pose_scan_pending = False
            self.run_square_posture_analysis()

    def _show_pose_cache_if_ready(self):
        cached = _load_posture_cache(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
            self._current_z_draw_thresh(), self._current_copper_scale(),
        )
        if cached is None:
            return False
        self._on_square_posture_done(cached, "")
        if cached.get("all_ranked_postures"):
            self.pose_status.setText("Đã nạp ngay các tư thế MPC đã tính sẵn cho hình đang chọn.")
        return True

    def run_square_posture_analysis(self):
        if getattr(self, "_pose_worker", None) and self._pose_worker.isRunning():
            self._pose_scan_pending = True
            self.pose_status.setText("Đang hoàn tất lượt quét trước; mẫu đang chọn sẽ được quét ngay sau đó.")
            return
        if self._show_pose_cache_if_ready():
            return
        selected_key = self.combo_pattern.currentData()
        if (
            self._precache_worker is not None
            and self._precache_worker.isRunning()
            and selected_key in getattr(self, "_precached_pattern_keys", set())
            and abs(getattr(self._precache_worker, "copper_scale", 1.0) - self._current_copper_scale()) < 1e-6
        ):
            self._precache_worker.prioritize(selected_key)
            self._pose_scan_pending = True
            self.pose_status.setText("Đang ưu tiên hình đang chọn; các tư thế sẽ hiện dần và bảng đầy đủ tự nạp khi xong.")
            return
        self.btn_pose_scan.setEnabled(False); self.btn_pose_apply.setEnabled(False)
        self.btn_export_pose_csv.setEnabled(False)
        self.btn_pose_run_selected.setEnabled(False); self.btn_pose_run_best.setEnabled(False)
        self._pose_result=None; self.pose_table.setRowCount(0); self.pose_combo.clear()
        z_thresh = self._current_z_draw_thresh()
        c_scale = self._current_copper_scale()
        self._active_pose_cache_key = _posture_cache_key(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(), z_thresh, c_scale
        )
        opt=DensoPostureMPCOptimizer(dynamics_engine=self.engine, copper_scale=c_scale)
        roll=self._current_pose_roll_range()
        self.pose_status.setText("Đang sàng lọc khả năng IK và chạy MPC đầy đủ cho từng nhánh tư thế khả thi…")
        self._pose_partial_items = []
        self._pose_worker=SquarePostureWorker(
            opt, self.current_gcode, self.pose_spacing.value(), roll,
            z_draw_thresh=z_thresh, copper_scale=c_scale,
        )
        self._pose_worker.completed.connect(self._on_square_posture_done)
        self._pose_worker.progress.connect(self._on_pose_scan_progress)
        self._pose_worker.partial.connect(self._on_pose_partial_result)
        self._show_pose_scan_placeholder()
        self._pose_worker.start()

    def _show_pose_scan_placeholder(self):
        self.pose_table.setRowCount(1)
        self.pose_table.setItem(0, 0, QTableWidgetItem("…"))
        self.pose_table.setItem(0, 1, QTableWidgetItem("Đang tìm các nhánh tư thế IK 6-DOF…"))
        for col in range(2, self.pose_table.columnCount()):
            self.pose_table.setItem(0, col, QTableWidgetItem("—"))
        self.pose_status.setText("Đang tìm tư thế khả thi; các nghiệm đầu tiên sẽ hiện ngay bên dưới…")

    @staticmethod
    def _format_manipulability_cell(item: Dict[str, Any]) -> Tuple[str, bool]:
        w_min = float(item.get("min_manipulability", 0.0))
        kappa = float(item.get("max_condition_number", 0.0))
        warn = bool(item.get("singularity_warning", False))
        prefix = "⚠ " if warn else ""
        if kappa > 0.0:
            return f"{prefix}{w_min:.3f} (κ={kappa:.0f})", warn
        return f"{prefix}{w_min:.3f}", warn

    def _on_pose_partial_result(self, stage, items):
        if not items or self._active_pose_cache_key != _posture_cache_key(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
            self._current_z_draw_thresh(), self._current_copper_scale(),
        ):
            return
        if stage == "screening":
            self._pose_partial_items = [dict(item, partial_preview=True) for item in items]
        elif stage == "shortlist":
            self._pose_partial_items = [dict(item, partial_preview=True) for item in items]
        elif stage == "detailed":
            # Keep provisional rows until their corresponding full-resolution MPC
            # result arrives, then replace them by posture family.
            current = {item["posture"].key.split("_j6_")[0]: item for item in self._pose_partial_items}
            for item in items:
                current[item["posture"].key.split("_j6_")[0]] = dict(item, partial_preview=False)
            self._pose_partial_items = list(current.values())

        rows = sorted(
            self._pose_partial_items,
            key=lambda item: (float(item.get("energy_joules_raw", item.get("energy_joules", float("inf")))), item["posture"].key),
        )[:4]
        self.pose_table.setRowCount(len(rows))
        for row, item in enumerate(rows):
            q = np.asarray(item["q_traj"], dtype=float)
            q_mid = np.rad2deg(q[len(q) // 2])
            provisional = item.get("partial_preview", True)
            energy_label = (
                "Đang tính MPC…" if provisional else
                (f"{item['energy_joules']:.1f} (IK)" if not item.get("mpc_feasible", True) else f"{item['energy_joules']:.1f}")
            )
            savings_label = (
                "Đang tính…" if provisional else
                ("MPC không khả thi" if not item.get("mpc_feasible", True) else f"{item.get('energy_savings_pct', 0.0):+.1f}%")
            )
            manip_label, manip_warn = self._format_manipulability_cell(item)
            vals = [
                row + 1,
                item["posture"].badge + (" · sơ bộ" if provisional else ""),
                *[f"{value:+.1f}" for value in q_mid],
                f"{item.get('ik_energy_joules', item.get('energy_joules', 0.0)):.1f}",
                energy_label,
                savings_label,
                f"{item.get('jerk_score', 0.0):.1f}",
                f"{item.get('max_torque_pct', 0.0):.1f}",
                manip_label,
                f"{item.get('max_path_error_mm', 0.0):.4f}",
                f"{item.get('max_orientation_error_deg', 0.0):.2f}",
            ]
            for col, value in enumerate(vals):
                cell = QTableWidgetItem(str(value))
                if provisional:
                    cell.setForeground(QBrush(QColor("#fbbf24")))
                elif col == 13 and manip_warn:
                    cell.setForeground(QBrush(QColor("#fbbf24")))
                self.pose_table.setItem(row, col, cell)
        if stage == "screening":
            self.pose_status.setText(f"Đã tìm thấy {len(rows)} tư thế sơ bộ; đang tiếp tục sàng lọc và đánh giá MPC…")
        elif stage == "shortlist":
            self.pose_status.setText(f"Đã chọn {len(rows)} tư thế khác biệt; đang tính MPC, kết quả sẽ cập nhật từng hàng…")
        else:
            complete = sum(not item.get("partial_preview", True) for item in rows)
            self.pose_status.setText(f"MPC đã hoàn tất {complete}/{len(rows)} tư thế; các hàng còn lại đang được tính…")

    def _on_pose_scan_progress(self, message, percent):
        self.pose_status.setText(message)
        if hasattr(self, "pose_progress"):
            self.pose_progress.setValue(max(0, min(100, int(percent))))

    def _on_square_posture_done(self,result,error):
        self.btn_pose_scan.setEnabled(True)
        current_key = _posture_cache_key(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
            self._current_z_draw_thresh(), self._current_copper_scale(),
        )
        result_key = (result or {}).get("cache_key")
        if result_key and result_key != current_key:
            self.pose_status.setText("Đã bỏ kết quả của mẫu cũ; đang chuyển sang mẫu đang chọn.")
            if self._pose_scan_pending:
                self._pose_scan_pending = False
                QTimer.singleShot(0, self.run_square_posture_analysis)
            return
        if error:
            self.pose_status.setText("Lỗi quét: " + error)
            if self._pose_scan_pending:
                self._pose_scan_pending = False
                QTimer.singleShot(0, self.run_square_posture_analysis)
            return
        if self._pose_scan_pending:
            self._pose_scan_pending = False
            QTimer.singleShot(0, self.run_square_posture_analysis)
        if not result or not result.get("all_ranked_postures"):
            self.pose_table.setRowCount(0)
            self.pose_combo.clear()
            self.btn_pose_apply.setEnabled(False)
            self.btn_export_pose_csv.setEnabled(False)
            self.btn_pose_run_selected.setEnabled(False)
            self.btn_pose_run_best.setEnabled(False)
            self.pose_status.setText((result or {}).get(
                "status_message",
                "Chưa có kết quả MPC hợp lệ. Hãy kiểm tra khả năng bám đường và giới hạn khớp.",
            ))
            return
        self._pose_result=result; ranked=result["all_ranked_postures"]; self.pose_table.setRowCount(len(ranked))
        self.pose_combo.blockSignals(True)
        self.pose_combo.clear()
        for row,item in enumerate(ranked):
            q=np.asarray(item["q_traj"]); j4=math.degrees(float(q[len(q)//2,3])); j5=math.degrees(float(q[len(q)//2,4]))
            vs_base = item.get("vs_vertical_baseline_pct", item["energy_savings_pct"])
            manip_label, manip_warn = self._format_manipulability_cell(item)
            self.pose_combo.addItem(
                f"#{item['rank']} {item['posture'].badge} | {item.get('motion_source', 'EMPC')} {item['energy_joules']:.1f}J "
                f"(IK {item['ik_energy_joules']:.1f}J, EMPC {item['energy_savings_pct']:+.1f}%, chuẩn thẳng {vs_base:+.1f}%, w={item.get('min_manipulability', 0.0):.3f})", item
            )
            q_mid=np.rad2deg(q[len(q)//2])
            energy_display = f"{item['energy_joules']:.1f}" if item.get("mpc_feasible", True) else f"{item['energy_joules']:.1f} (IK)"
            vals=[item["rank"],item["posture"].badge,*[f"{v:+.1f}" for v in q_mid],
                  f"{item['ik_energy_joules']:.1f}",energy_display,
                  f"{vs_base:+.1f}% (MPC {item['energy_savings_pct']:+.1f}%)",f"{item['jerk_score']:.1f}",
                  f"{item['max_torque_pct']:.1f}",manip_label,f"{item['max_path_error_mm']:.4f}",
                  f"{item['max_orientation_error_deg']:.2f}"]
            for col,val in enumerate(vals):
                cell = QTableWidgetItem(str(val))
                if row == 0:
                    cell.setBackground(QBrush(QColor("#064e3b")))
                    cell.setForeground(QBrush(QColor("#fbbf24" if (col == 13 and manip_warn) else "#4ade80")))
                    f_bold = cell.font(); f_bold.setBold(True); cell.setFont(f_bold)
                elif col == 13:
                    cell.setForeground(QBrush(QColor("#fbbf24" if manip_warn else "#4ade80")))
                self.pose_table.setItem(row,col,cell)
        self.pose_combo.blockSignals(False)
        if len(ranked) > 0:
            self.pose_table.blockSignals(True)
            self.pose_table.selectRow(0)
            self.pose_table.blockSignals(False)
        self.btn_pose_apply.setEnabled(True)
        self.btn_export_pose_csv.setEnabled(True)
        self.btn_pose_run_selected.setEnabled(True)
        self.btn_pose_run_best.setEnabled(True)
        self._update_amstar_buttons()
        best=ranked[0]
        base_v_j = result.get("baseline_vertical_ik_joules", best["ik_energy_joules"])
        vs_base_best = best.get("vs_vertical_baseline_pct", best["energy_savings_pct"])
        sing_note = (
            f"Cảnh báo suy biến cổ tay J5 (w_min={best.get('min_manipulability', 0.0):.3f})"
            if best.get("singularity_warning")
            else f"Tránh xa suy biến cổ tay (w_min={best.get('min_manipulability', 0.0):.3f}, κ={best.get('max_condition_number', 0.0):.1f})"
        )
        self.pose_status.setText(
            f"🏆 Đã quét {result.get('total_candidate_evaluations', 0)} tổ hợp (17 họ IK 6-DOF × dải J6) trên {result['point_count']} điểm → "
            f"Xếp hạng {len(ranked)} tư thế q1–q6 khác biệt rõ rệt (sai số mũi bút {best['max_path_error_mm']:.4f} mm, {sing_note}). "
            f"Tối ưu nhất Hạng #1 [{best['posture'].badge}]: E_EMPC = {best['energy_joules']:.1f} J "
            f"(giảm {vs_base_best:+.1f}% so với IK bút thẳng đứng chuẩn {base_v_j:.1f} J; riêng EMPC tiết kiệm {best['energy_savings_pct']:+.1f}% so với IK cùng tư thế {best['ik_energy_joules']:.1f} J)."
        )

    def _on_pose_table_row_selected(self):
        row = self.pose_table.currentRow()
        if 0 <= row < self.pose_combo.count():
            self.pose_combo.setCurrentIndex(row)

    def _current_amstar_posture(self):
        result = self._pose_result
        selected = self.pose_combo.currentData()
        if not result or not selected:
            return None
        key = _posture_cache_key(
            self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
            self._current_z_draw_thresh(), self._current_copper_scale(),
        )
        if result.get("cache_key") != key or selected.get("q_traj") is None:
            return None
        return result, selected

    def _on_export_csv_clicked(self):
        if not self._pose_result or not self._pose_result.get("all_ranked_postures"):
            self.pose_status.setText("Hãy quét các tư thế trước khi xuất CSV thực nghiệm.")
            return
        default_dir = os.path.expanduser("~/denso_ws/output")
        os.makedirs(default_dir, exist_ok=True)
        chosen_dir = QFileDialog.getExistingDirectory(self, "Chọn thư mục lưu CSV thực nghiệm", default_dir)
        if not chosen_dir:
            return
        try:
            summary_path, ts_path = self.export_posture_experiment_csv(chosen_dir)
            self.pose_status.setText(
                f"📥 Đã xuất 2 file CSV thực nghiệm: {os.path.basename(summary_path)} & {os.path.basename(ts_path)} tại {chosen_dir}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "Lỗi xuất CSV", f"Không thể ghi CSV thực nghiệm: {exc}")

    def export_posture_experiment_csv(self, output_dir: Optional[str] = None) -> Tuple[str, str]:
        """Xuất bảng xếp hạng tư thế 6-DOF và chuỗi thời gian động lực học ra 2 file CSV chuẩn báo cáo khoa học."""
        if not self._pose_result or not self._pose_result.get("all_ranked_postures"):
            raise ValueError("Chưa có dữ liệu kết quả quét tư thế 6-DOF để xuất CSV.")

        out_dir = Path(output_dir or os.path.expanduser("~/denso_ws/output"))
        out_dir.mkdir(parents=True, exist_ok=True)
        ts_tag = time.strftime("%Y%m%d_%H%M%S")
        pattern_key = str(self.combo_pattern.currentData() or "custom").replace(":", "_").replace("/", "_")
        summary_file = out_dir / f"posture_summary_{pattern_key}_{ts_tag}.csv"
        timeseries_file = out_dir / f"posture_timeseries_{pattern_key}_{ts_tag}.csv"

        ranked = self._pose_result["all_ranked_postures"]
        strokes = self._pose_result.get("strokes", ())
        cart_points = [(p.x, p.y, p.z, int(bool(p.is_draw))) for st in strokes for p in st.points]
        c_scale = float(self._pose_result.get("copper_scale", self._current_copper_scale()))
        copper_coeffs = (
            np.array([1.2, 1.5, 1.8, 2.2, 2.5, 2.5], dtype=float)
            / np.array([750.0, 400.0, 200.0, 80.0, 50.0, 50.0], dtype=float)
        ) * c_scale

        with summary_file.open("w", newline="", encoding="utf-8-sig") as f_sum:
            writer = csv.writer(f_sum)
            writer.writerow([
                "rank", "case_key", "posture_badge", "motion_source",
                "q1_mid_deg", "q2_mid_deg", "q3_mid_deg", "q4_mid_deg", "q5_mid_deg", "q6_mid_deg",
                "ik_energy_j", "empc_energy_j", "vs_vertical_baseline_pct", "mpc_savings_pct",
                "mechanical_energy_j", "copper_loss_j", "setup_transit_j",
                "jerk_score_rad2_s5", "max_torque_pct",
                "min_manipulability_w", "mean_manipulability_w", "max_condition_kappa", "singularity_warning",
                "max_path_error_mm", "max_orientation_error_deg", "copper_scale",
            ])
            for item in ranked:
                q_arr = np.asarray(item["q_traj"], dtype=float)
                q_mid = np.rad2deg(q_arr[len(q_arr) // 2])
                writer.writerow([
                    item.get("rank", 0),
                    item["posture"].key,
                    item["posture"].badge,
                    item.get("motion_source", "EMPC"),
                    *[round(float(v), 3) for v in q_mid],
                    item.get("ik_energy_joules", 0.0),
                    item.get("energy_joules", 0.0),
                    item.get("vs_vertical_baseline_pct", 0.0),
                    item.get("energy_savings_pct", 0.0),
                    item.get("mechanical_energy_j", 0.0),
                    item.get("copper_loss_energy_j", 0.0),
                    item.get("setup_energy_joules", 0.0),
                    item.get("jerk_score", 0.0),
                    item.get("max_torque_pct", 0.0),
                    item.get("min_manipulability", 0.0),
                    item.get("mean_manipulability", 0.0),
                    item.get("max_condition_number", 0.0),
                    int(bool(item.get("singularity_warning", False))),
                    item.get("max_path_error_mm", 0.0),
                    item.get("max_orientation_error_deg", 0.0),
                    round(c_scale, 3),
                ])

        with timeseries_file.open("w", newline="", encoding="utf-8-sig") as f_ts:
            writer = csv.writer(f_ts)
            writer.writerow([
                "rank", "case_key", "posture_badge", "step", "time_s",
                "x_mm", "y_mm", "z_mm", "is_draw",
                "q1_deg", "q2_deg", "q3_deg", "q4_deg", "q5_deg", "q6_deg",
                "qd1_rad_s", "qd2_rad_s", "qd3_rad_s", "qd4_rad_s", "qd5_rad_s", "qd6_rad_s",
                "tau1_nm", "tau2_nm", "tau3_nm", "tau4_nm", "tau5_nm", "tau6_nm",
                "p_elec_w", "e_cum_j", "manipulability_w",
            ])
            for item in ranked:
                q_arr = np.asarray(item["q_traj"], dtype=float)
                qd_arr = np.asarray(item.get("qd_traj", np.zeros_like(q_arr)), dtype=float)
                tau_arr = np.asarray(item.get("tau_traj", np.zeros_like(q_arr)), dtype=float)
                t_vec = np.asarray(item.get("time_vec", np.arange(len(q_arr)) * 0.08), dtype=float)
                manip_arr = np.asarray(
                    item.get("manipulability_traj", np.full(len(q_arr), item.get("min_manipulability", 0.0))),
                    dtype=float,
                )
                p_elec_vec = np.sum(np.abs(tau_arr * qd_arr) + copper_coeffs[None, :] * (tau_arr ** 2), axis=1)
                e_cum = np.zeros(len(q_arr), dtype=float)
                if len(q_arr) > 1:
                    dt_vec = np.diff(t_vec)
                    e_cum[1:] = np.cumsum(0.5 * (p_elec_vec[:-1] + p_elec_vec[1:]) * dt_vec)
                q_deg = np.rad2deg(q_arr)
                for k in range(len(q_arr)):
                    cx, cy, cz, cdraw = cart_points[k] if k < len(cart_points) else (0.0, 0.0, 0.0, 0)
                    writer.writerow([
                        item.get("rank", 0),
                        item["posture"].key,
                        item["posture"].badge,
                        k,
                        round(float(t_vec[k]), 4),
                        round(float(cx), 3),
                        round(float(cy), 3),
                        round(float(cz), 3),
                        cdraw,
                        *[round(float(v), 3) for v in q_deg[k]],
                        *[round(float(v), 4) for v in qd_arr[k]],
                        *[round(float(v), 4) for v in tau_arr[k]],
                        round(float(p_elec_vec[k]), 4),
                        round(float(e_cum[k]), 4),
                        round(float(manip_arr[k]) if k < len(manip_arr) else 0.0, 4),
                    ])

        return str(summary_file), str(timeseries_file)

    def evaluate_amstar_rnea_state(self, state: Optional[AmstarState]) -> Optional[Dict[str, Any]]:
        """Tính mô-men RNEA, công suất điện ước lượng P_est và sai số bám từ phản hồi góc thực AmstarRobot."""
        if state is None:
            self._amstar_prev_sample = None
            return None
        eng = self.engine or getattr(self.optimizer, "engine", None)
        if eng is None:
            return None

        angles_deg = state.motion_angles_deg if state.motion_angles_deg is not None else state.angles_deg
        q_rad = np.deg2rad(np.asarray(angles_deg, dtype=float))
        if q_rad.shape != (6,) or not np.all(np.isfinite(q_rad)):
            return None

        qd_rad = np.zeros(6, dtype=float)
        if self._amstar_prev_sample is not None:
            prev_q, prev_t = self._amstar_prev_sample
            dt_live = float(state.eventtime) - float(prev_t)
            if 0.01 <= dt_live <= 2.0:
                qd_rad = np.clip((q_rad - prev_q) / dt_live, -7.5, 7.5)
        self._amstar_prev_sample = (q_rad.copy(), float(state.eventtime))

        friction_b = np.array([0.85, 1.10, 0.75, 0.30, 0.22, 0.15], dtype=float)
        friction_fc = np.array([1.20, 1.60, 1.00, 0.40, 0.28, 0.18], dtype=float)
        c_scale = self._current_copper_scale()
        copper_c = (
            np.array([1.2, 1.5, 1.8, 2.2, 2.5, 2.5], dtype=float)
            / np.array([750.0, 400.0, 200.0, 80.0, 50.0, 50.0], dtype=float)
        ) * c_scale

        tau_live = eng.rnea(q_rad, qd_rad, np.zeros(6, dtype=float)) + friction_b * qd_rad + friction_fc * np.tanh(qd_rad / 0.05)
        p_mech_w = float(np.sum(np.abs(tau_live * qd_rad)))
        p_copper_w = float(np.sum(copper_c * (tau_live ** 2)))
        p_elec_w = p_mech_w + p_copper_w
        manip = (
            eng.compute_6dof_manipulability(q_rad, use_flange=True)
            if hasattr(eng, "compute_6dof_manipulability")
            else eng.compute_manipulability(q_rad, use_flange=True)
        )

        tracking_err_deg = None
        selected = self.pose_combo.currentData() if hasattr(self, "pose_combo") else None
        if isinstance(selected, dict) and selected.get("q_traj") is not None:
            q_ref = np.asarray(selected["q_traj"], dtype=float)
            if q_ref.ndim == 2 and q_ref.shape[1] == 6 and len(q_ref) > 0:
                dists = np.linalg.norm(np.rad2deg(q_ref) - np.asarray(angles_deg, dtype=float)[None, :], axis=1)
                tracking_err_deg = float(np.min(dists))

        metrics = {
            "angles_deg": tuple(round(float(v), 2) for v in angles_deg),
            "tau_nm": tau_live,
            "tau_j2_nm": float(tau_live[1]),
            "peak_tau_nm": float(np.max(np.abs(tau_live))),
            "p_elec_w": p_elec_w,
            "p_copper_w": p_copper_w,
            "manipulability_w": float(manip["manipulability"]),
            "singularity_warning": bool(manip.get("is_near_singularity", False)),
            "tracking_err_deg": tracking_err_deg,
            "job_state": state.job_state or "standby",
        }
        self._amstar_last_telemetry = metrics
        return metrics

    def update_amstar_telemetry(self, state: Optional[AmstarState]):
        """Cập nhật dòng giám sát RNEA & công suất trực tuyến từ phản hồi AmstarRobot."""
        if self._amstar_reply is not None:
            return
        if state is None:
            self._amstar_prev_sample = None
            return
        metrics = self.evaluate_amstar_rnea_state(state)
        if metrics is None:
            return
        err_txt = f" | Δq_MPC={metrics['tracking_err_deg']:.2f}°" if metrics["tracking_err_deg"] is not None else ""
        warn_txt = " ⚠Gần suy biến" if metrics["singularity_warning"] else ""
        self.amstar_status.setText(
            f"Amstar [{metrics['job_state']}] · P_ước={metrics['p_elec_w']:.2f}W (I²R={metrics['p_copper_w']:.2f}W) · "
            f"|τ_J2|={abs(metrics['tau_j2_nm']):.1f}N·m · w={metrics['manipulability_w']:.3f}{warn_txt}{err_txt}"
        )

    def _update_amstar_buttons(self):
        available = self._amstar_reply is None
        self.btn_amstar_upload.setEnabled(available and self._current_amstar_posture() is not None)
        self.btn_amstar_home.setEnabled(available)

    def _upload_selected_to_amstar(self):
        if self._amstar_reply is not None:
            return
        selection = self._current_amstar_posture()
        if selection is None:
            self.amstar_status.setText("Hãy quét và chọn tư thế cho hình với thông số hiện tại.")
            return

        result, selected = selection
        draw_flags = [point.is_draw for stroke in result.get("strokes", ()) for point in stroke.points]
        try:
            q = np.asarray(selected["q_traj"], dtype=float)
        except (TypeError, ValueError, OverflowError):
            self.amstar_status.setText("Dãy góc khớp không hợp lệ. Hãy quét lại tư thế.")
            return
        if not draw_flags or not any(draw_flags) or q.shape != (len(draw_flags), 6) or not np.isfinite(q).all():
            self.amstar_status.setText("Quỹ đạo phải có nét vẽ và đủ sáu góc hợp lệ cho từng điểm.")
            return
        angles_deg = np.rad2deg(q)
        gcode = format_joint_gcode(zip(angles_deg, draw_flags))
        rank = int(selected["rank"])
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"gcode_6link_{timestamp}_hang{rank}_{uuid4().hex[:8]}.gcode"
        self.amstar_job_prepared.emit(filename, gcode)
        multipart = QHttpMultiPart(QHttpMultiPart.FormDataType)
        for field, value in (("root", "gcodes"), ("print", "false")):
            part = QHttpPart()
            part.setRawHeader(b"Content-Disposition", f'form-data; name="{field}"'.encode("ascii"))
            part.setBody(QByteArray(value.encode("ascii")))
            multipart.append(part)
        file_part = QHttpPart()
        disposition = f'form-data; name="file"; filename="{filename}"'
        file_part.setRawHeader(b"Content-Disposition", disposition.encode("ascii"))
        file_part.setRawHeader(b"Content-Type", b"application/octet-stream")
        file_part.setBody(QByteArray(gcode.encode("ascii")))
        multipart.append(file_part)

        request = self._amstar_request(QUrl(AMSTAR_URL + "/server/files/upload"))
        reply = self._amstar_network.post(request, multipart)
        multipart.setParent(reply)
        self.amstar_status.setText(f"Đang gửi tư thế hạng {rank} ({len(draw_flags)} điểm) tới AmstarRobot…")
        self._watch_amstar_reply(reply, 60000, self._finish_amstar_upload, filename)

    def _amstar_request(self, url):
        request = QNetworkRequest(url)
        request.setRawHeader(b"Accept", b"application/json")
        request.setRawHeader(b"Connection", b"close")
        request.setAttribute(QNetworkRequest.RedirectPolicyAttribute, QNetworkRequest.ManualRedirectPolicy)
        api_key = os.environ.get("AMSTAR_MOONRAKER_API_KEY")
        if api_key:
            request.setRawHeader(b"X-Api-Key", api_key.encode("utf-8"))
        return request

    def _watch_amstar_reply(self, reply, timeout_ms, callback, *args):
        self._amstar_reply = reply
        self._update_amstar_buttons()
        timer = QTimer(reply)
        timer.setSingleShot(True)
        timer.timeout.connect(reply.abort)
        reply.finished.connect(timer.stop)
        reply.finished.connect(lambda: callback(reply, *args))
        timer.start(timeout_ms)

    def _read_amstar_reply(self, reply):
        status = reply.attribute(QNetworkRequest.HttpStatusCodeAttribute)
        error = reply.error()
        try:
            data = json.loads(bytes(reply.readAll()))
        except ValueError:
            data = {}
        if not isinstance(data, dict):
            data = {}
        detail = data.get("error")
        detail = detail.get("message") if isinstance(detail, dict) else None
        if status in (401, 403):
            problem = "Moonraker từ chối truy cập. Kiểm tra quyền hoặc khóa API."
        elif error == QNetworkReply.OperationCanceledError:
            problem = "Hết thời gian chờ phản hồi."
        elif error != QNetworkReply.NoError:
            problem = detail[:240] if isinstance(detail, str) and detail else reply.errorString()
        else:
            problem = None
        reply.deleteLater()
        return status, data, problem

    def _set_amstar_done(self, message):
        self.amstar_status.setText(message)
        self._amstar_reply = None
        self._update_amstar_buttons()

    def _finish_amstar_upload(self, reply, filename):
        status, data, problem = self._read_amstar_reply(reply)
        result = data.get("result", data)
        item = result.get("item", {}) if isinstance(result, dict) else {}
        if problem:
            self._set_amstar_done(f"Chưa xác nhận tải lên {filename}. {problem}")
            return
        if (status != 201 or not isinstance(item, dict) or item.get("root") != "gcodes"
                or item.get("path") != filename or result.get("print_started") is not False
                or result.get("print_queued") is not False):
            self._set_amstar_done(f"Chưa xác nhận lưu tệp. Kiểm tra {filename} trong G-Code Files.")
            return

        # Klipper may replace a paused job, so check immediately before starting.
        reply = self._query_amstar_status()
        self.amstar_status.setText(f"Đã lưu {filename}. Đang kiểm tra robot trước khi chạy…")
        self._watch_amstar_reply(reply, 8000, self._check_amstar_ready, filename)

    def _query_amstar_status(self, check_motion=False):
        query = "webhooks=state&print_stats=state"
        if check_motion:
            query += "&idle_timeout=state"
        url = QUrl(f"{AMSTAR_URL}/printer/objects/query?{query}")
        return self._amstar_network.get(self._amstar_request(url))

    def _amstar_readiness_problem(self, reply, check_motion=False):
        status, data, problem = self._read_amstar_reply(reply)
        if problem or status != 200:
            return f"Chưa kiểm tra được trạng thái robot. {problem or ''}".rstrip()
        result = data.get("result")
        objects = result.get("status") if isinstance(result, dict) else None
        if not isinstance(objects, dict):
            return "Robot chưa trả về trạng thái hợp lệ."
        webhooks = objects.get("webhooks")
        print_stats = objects.get("print_stats")
        machine = webhooks.get("state") if isinstance(webhooks, dict) else None
        job = print_stats.get("state") if isinstance(print_stats, dict) else None
        if job == "printing":
            return "Robot đang chạy tệp."
        if job == "paused":
            return "Robot đang tạm dừng một tệp."
        if machine != "ready" or job not in ("standby", "complete", "cancelled"):
            return "Robot chưa báo sẵn sàng."
        if check_motion:
            idle_timeout = objects.get("idle_timeout")
            motion = idle_timeout.get("state") if isinstance(idle_timeout, dict) else None
            if motion == "Printing":
                return "Robot đang thực hiện chuyển động."
            if motion not in ("Ready", "Idle"):
                return "Chưa xác nhận robot đã dừng chuyển động."
        return None

    def _check_amstar_ready(self, reply, filename):
        problem = self._amstar_readiness_problem(reply)
        if problem:
            self._set_amstar_done(f"Đã lưu {filename}. {problem} Tệp mới chưa được chạy.")
            return

        url = QUrl(AMSTAR_URL + "/printer/print/start")
        query = QUrlQuery()
        query.addQueryItem("filename", filename)
        url.setQuery(query)
        request = self._amstar_request(url)
        request.setHeader(QNetworkRequest.ContentTypeHeader, "application/x-www-form-urlencoded")
        reply = self._amstar_network.post(request, QByteArray())
        self.amstar_status.setText(f"Đang yêu cầu AmstarRobot chạy {filename}…")
        self._watch_amstar_reply(reply, 15000, self._finish_amstar_start, filename)

    def _finish_amstar_start(self, reply, filename):
        status, data, problem = self._read_amstar_reply(reply)
        if status == 200 and problem is None and data.get("result") == "ok":
            self._set_amstar_done(f"AmstarRobot đã xác nhận bắt đầu chạy {filename}.")
        else:
            detail = f" {problem}" if problem else ""
            self._set_amstar_done(
                f"Đã lưu {filename}. Chưa xác nhận robot đã chạy.{detail}"
                " Kiểm tra trạng thái trên AmstarRobot trước khi bấm lại."
            )

    def _return_amstar_home(self):
        if self._amstar_reply is not None:
            return
        reply = self._query_amstar_status(check_motion=True)
        self.amstar_status.setText("Đang kiểm tra AmstarRobot trước khi về Home…")
        self._watch_amstar_reply(reply, 8000, self._start_amstar_home)

    def _start_amstar_home(self, reply):
        problem = self._amstar_readiness_problem(reply, check_motion=True)
        if problem:
            self._set_amstar_done(f"{problem} Chưa gửi lệnh về Home.")
            return
        request = self._amstar_request(QUrl(AMSTAR_URL + "/printer/gcode/script"))
        request.setHeader(QNetworkRequest.ContentTypeHeader, "application/json")
        # Wait for Klipper's motion queue before reporting completion.
        body = QByteArray(json.dumps({"script": "MOV_ZEROALL_POS\nM400"}).encode("utf-8"))
        reply = self._amstar_network.post(request, body)
        self.amstar_status.setText("Đã gửi lệnh về Home Amstar (0°). Đang chờ robot xác nhận…")
        self._watch_amstar_reply(reply, 90000, self._finish_amstar_home)

    def _finish_amstar_home(self, reply):
        status, data, problem = self._read_amstar_reply(reply)
        if status == 200 and problem is None and data.get("result") == "ok":
            self._set_amstar_done("AmstarRobot đã hoàn tất lệnh về Home (0°).")
        else:
            detail = f" {problem}" if problem else ""
            self._set_amstar_done(f"Chưa xác nhận hoàn tất lệnh về Home.{detail} Kiểm tra trạng thái trên AmstarRobot.")

    def _find_tab5_drawing_widget(self):
        win = self.window()
        tab_widgets = ([win] if isinstance(win, QTabWidget) else []) + win.findChildren(QTabWidget)
        for tabs in tab_widgets:
            for i in range(tabs.count()):
                target=tabs.widget(i)
                if target.__class__.__name__=="ImageGCodeDrawingTab":
                    return target, tabs
        return None, None

    def _transfer_posture_to_tab5(self, item, start_drawing=False):
        if not item or not self._pose_result:
            self.pose_status.setText("Hãy quét và chọn một tư thế trước.")
            return False
        target, root_tabs = self._find_tab5_drawing_widget()
        if target is None:
            self.pose_status.setText("Không tìm thấy Tab 5.")
            return False
        try:
            target.apply_precomputed_posture(
                self._pose_result, item, self.current_gcode, self.pose_spacing.value()
            )
            if start_drawing:
                root_tabs.setCurrentWidget(target)
                target._draw_selected_posture_mpc()
                self.pose_status.setText("Đã nạp tư thế và bắt đầu vẽ; Tab 5 đang hiển thị tiến trình.")
            else:
                self.pose_status.setText("Đã chuyển G-code, mật độ điểm và quỹ đạo khớp sang Tab 5.")
            return True
        except Exception as e:
            self.pose_status.setText("Không thể chuyển sang Tab 5: "+str(e))
            return False

    def apply_selected_posture_to_tab5(self):
        self._transfer_posture_to_tab5(self.pose_combo.currentData())

    def _run_selected_posture_in_tab5(self):
        self._transfer_posture_to_tab5(self.pose_combo.currentData(), start_drawing=True)

    def _run_best_posture_in_tab5(self):
        if not self._pose_result:
            self.pose_status.setText("Hãy quét các tư thế trước.")
            return
        item=self._pose_result.get("best_posture")
        if item is None:
            self.pose_status.setText("Không tìm thấy tư thế tối ưu khả thi.")
            return
        for i in range(self.pose_combo.count()):
            if self.pose_combo.itemData(i) is item:
                self.pose_combo.setCurrentIndex(i)
                break
        self._transfer_posture_to_tab5(item, start_drawing=True)

    def _load_default_pattern(self):
        self.current_gcode = BUILTIN_GCODE_PATTERNS["square"]["gcode"]

    def _on_pattern_changed(self, index):
        key = self.combo_pattern.currentData()
        self._pose_result = None
        self.pose_table.setRowCount(0)
        self.pose_combo.clear()
        self.btn_pose_apply.setEnabled(False)
        self.btn_export_pose_csv.setEnabled(False)
        self.btn_pose_run_selected.setEnabled(False)
        self.btn_pose_run_best.setEnabled(False)
        if key in BUILTIN_GCODE_PATTERNS:
            self.current_gcode = BUILTIN_GCODE_PATTERNS[key]["gcode"]
            self._pose_result = None
            self.pose_table.setRowCount(0)
            self.pose_combo.clear()
            self.btn_pose_apply.setEnabled(False)
            self.btn_export_pose_csv.setEnabled(False)
            self.pose_status.setText("Đã chọn mẫu. Đang nạp cache riêng của hình này…")
            if not self._show_pose_cache_if_ready():
                if self._precache_worker is not None and self._precache_worker.isRunning():
                    self._precache_worker.prioritize(key)
                    self._active_pose_cache_key = _posture_cache_key(
                        self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
                        self._current_z_draw_thresh(), self._current_copper_scale(),
                    )
                    self._pose_partial_items = []
                    self._show_pose_scan_placeholder()
                    self.pose_status.setText("Đang ưu tiên tính hình này; kết quả sẽ hiện dần rồi được nạp đầy đủ vào cache.")
                else:
                    self.pose_status.setText("Hình chưa có cache; bấm QUÉT CÁC TƯ THẾ để tính và lưu cho lần sau.")
        elif isinstance(key, str) and key.startswith("tab5_image:"):
            sample_name = key.split(":", 1)[1]
            target, _ = self._find_tab5_drawing_widget()
            if target is None:
                self.pose_status.setText("Không tìm thấy Tab 5 để tạo đường G-code từ ảnh mẫu.")
                return
            if getattr(target, "worker", None) and target.worker.isRunning():
                self.pose_status.setText("Robot đang vẽ ở Tab 5. Chờ hoàn tất rồi mới đổi mẫu.")
                return
            sample_index = target.combo_samples.findData(sample_name)
            if sample_index < 0:
                self.pose_status.setText(f"Không tìm thấy ảnh mẫu {sample_name} trong Tab 5.")
                return
            target.combo_samples.blockSignals(True)
            target.combo_samples.setCurrentIndex(sample_index)
            target.combo_samples.blockSignals(False)
            target._on_sample_changed(sample_index)
            gcode_text = target.txt_gcode.toPlainText()
            if not gcode_text.strip():
                self.pose_status.setText(f"Tab 5 chưa tạo được G-code cho mẫu {sample_name}.")
                return
            self.current_gcode = gcode_text
            self._pose_result = None
            self.pose_table.setRowCount(0)
            self.pose_combo.clear()
            self.btn_pose_apply.setEnabled(False)
            self.btn_export_pose_csv.setEnabled(False)
            self.btn_pose_run_selected.setEnabled(False)
            self.btn_pose_run_best.setEnabled(False)
            self.pose_status.setText(f"Đã tạo đường G-code từ {sample_name} theo thiết lập Tab 5.")
            if not self._show_pose_cache_if_ready():
                if key in getattr(self, "_precached_pattern_keys", set()) and self._precache_worker is not None and self._precache_worker.isRunning():
                    self._active_pose_cache_key = _posture_cache_key(
                        self.current_gcode, self.pose_spacing.value(), self._current_pose_roll_range(),
                        self._current_z_draw_thresh(), self._current_copper_scale(),
                    )
                    self._pose_partial_items = []
                    self._show_pose_scan_placeholder()
                    self.pose_status.setText(self.pose_status.text() + " Đang quét sẵn; các nghiệm sẽ hiện dần khi tìm được.")
                else:
                    self.pose_status.setText(self.pose_status.text() + " G-code này chưa có cache; quét lần đầu để lưu kết quả.")
        elif key == "custom":
            path, _ = QFileDialog.getOpenFileName(self, "Chọn File G-Code", "", "G-Code Files (*.gcode *.nc *.txt)")
            if path:
                try:
                    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                        self.current_gcode = f.read()
                    self._pose_result = None
                    self.pose_table.setRowCount(0)
                    self.pose_combo.clear()
                    self.btn_pose_apply.setEnabled(False)
                    self.btn_export_pose_csv.setEnabled(False)
                    self.pose_status.setText("Đã nạp G-code. Kiểm tra cache tư thế…")
                    if not self._show_pose_cache_if_ready():
                        self.pose_status.setText("G-code mới chưa có cache; hãy bấm QUÉT CÁC TƯ THẾ.")
                except Exception as e:
                    QMessageBox.critical(self, "Lỗi đọc file", f"Không thể đọc file: {e}")

    def run_comparison_analysis(self):
        """Tính toán và nạp bảng so sánh năng lượng cho toàn bộ 6 trường hợp dưới nền (Non-blocking)."""
        if self._calc_worker is not None and self._calc_worker.isRunning():
            self.lbl_status.setText("⏳ Bảng năng lượng đang được tính; quét tư thế MPC vẫn dùng nút riêng.")
            return
        self.lbl_status.setText("⏳ Đang tính bảng năng lượng 6 trường hợp...")
        self.progress_bar.setValue(5)
        for button in (self.btn_run_selected_case, self.btn_compare_up_down, self.btn_run_mpc):
            button.setEnabled(False)

        self._calc_worker = ComparisonCalculationWorker(self.optimizer, self.current_gcode, step_mm=2.5, dt=0.02)
        self._calc_worker.sig_finished.connect(self._on_comparison_finished)
        self._calc_worker.sig_error.connect(self._on_comparison_error)
        self._calc_worker.start()

    def _on_comparison_finished(self, comp: Dict[str, Any]):
        self.latest_comp = comp
        self._update_kpi_cards(comp)
        self._populate_energy_table(comp)
        self._plot_graphs(comp)
        self.progress_bar.setValue(100)
        best_title = comp['results'][comp['best_case']]['title']
        self.lbl_status.setText(f"✅ Đã tính toán xong bảng so sánh! Trường hợp tối ưu nhất: [{best_title}].")
        self.btn_run_selected_case.setEnabled(True)
        self.btn_compare_up_down.setEnabled(True)
        has_mpc = any(k in comp.get("results", {}) for k in ("case_4_mpc_balanced", "case_5_mpc_balanced"))
        self.btn_run_mpc.setEnabled(has_mpc)

    def _on_comparison_error(self, err_msg: str):
        self.lbl_status.setText(f"❌ {err_msg}")
        self.progress_bar.setValue(0)

    def _update_kpi_cards(self, comp: Dict[str, Any]):
        results = comp["results"]
        best_case = comp["best_case"]
        best_res = results[best_case]

        self.card_best.lbl_val.setText(best_res["badge"])
        self.card_best.lbl_sub.setText(f"{best_res['e_total_j']:.1f} J (Tiết kiệm -{comp['energy_saved_pct']:.1f}%)")

        # So sánh trực tiếp ROS 2 Baseline vs MPC Balanced
        mpc_balanced_k = "case_4_mpc_balanced" if "case_4_mpc_balanced" in results else ("case_5_mpc_balanced" if "case_5_mpc_balanced" in results else None)
        if "case_1_ros2_baseline" in results and mpc_balanced_k and mpc_balanced_k in results:
            e_ros2 = results["case_1_ros2_baseline"]["e_total_j"]
            e_mpc = results[mpc_balanced_k]["e_total_j"]
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
        mpc_case_key = "case_4_mpc_balanced" if "case_4_mpc_balanced" in results else ("case_5_mpc_balanced" if "case_5_mpc_balanced" in results else ("case_6_mpc_predictive" if "case_6_mpc_predictive" in results else None))
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
        mpc_res = results.get("case_4_mpc_balanced", results.get("case_5_mpc_balanced", results.get("case_6_mpc_predictive")))

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
            self.lbl_status.setText("Chưa có bảng năng lượng; đang khởi động tính toán. Quét tư thế hình vuông vẫn dùng nút riêng.")
            self.run_comparison_analysis()
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
            self.lbl_status.setText("Chưa có bảng năng lượng; đang khởi động tính toán.")
            self.run_comparison_analysis()
            return

        results = self.latest_comp["results"]
        res_1 = results.get("case_1_ros2_baseline", results.get("case_1_elbow_up"))
        res_2 = results.get("case_4_mpc_balanced", results.get("case_5_mpc_balanced", results.get("case_2_elbow_down")))
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
            self.lbl_status.setText("Chưa có bảng năng lượng MPC; đang khởi động tính toán. Vui lòng chạy lại sau khi hoàn tất.")
            self.run_comparison_analysis()
            return

        results = self.latest_comp["results"]
        case_k = "case_4_mpc_balanced" if "case_4_mpc_balanced" in results else ("case_5_mpc_balanced" if "case_5_mpc_balanced" in results else None)
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
