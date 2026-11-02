#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 G-CODE & VISION DRAWING TRAJECTORY ENGINE
Tích hợp xử lý ảnh, phân rã điểm theo độ dày/khoảng cách tùy chỉnh (Point Spacing),
chuyển đổi thành G-code chuẩn, giải động học nghịch (IK), điều khiển cánh tay di chuyển
và để lại vệt vẽ (Drawing Trail) 3D trong RViz2 và 2D trên Giao diện tương tác.
"""

import os
import sys
import math
import time
import json
import threading
from typing import List, Tuple, Dict, Any, Optional

import numpy as np
import cv2

# ROS 2 Message imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point, PoseStamped, Pose
    from visualization_msgs.msg import Marker, MarkerArray
    from nav_msgs.msg import Path
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from std_msgs.msg import Header, ColorRGBA
except ImportError:
    Node = object
    Point = object
    Marker = object
    MarkerArray = object
    Path = object

# PyQt5 imports
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
    QDoubleSpinBox, QSpinBox, QComboBox, QCheckBox, QFileDialog,
    QTextEdit, QProgressBar, QGroupBox, QSplitter, QFrame,
    QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox, QTabWidget, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPointF, QRectF, QThread
from PyQt5.QtGui import (
    QColor, QPen, QBrush, QFont, QPainter, QImage, QPixmap,
    QPainterPath, QLinearGradient
)

# Robot Dynamics & Kinematics
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "denso_vs6556_moveit_config", "scripts"))
try:
    from denso_dynamics_engine import DensoDynamicsEngine, JOINT_NAMES, JOINT_LIMITS
except ImportError:
    sys.path.append("/home/dung/denso_ws/src/denso_vs6556_moveit_config/scripts")
    from denso_dynamics_engine import DensoDynamicsEngine, JOINT_NAMES, JOINT_LIMITS

# 5-Link G-code & Joint Table imports
try:
    from denso_gcode_joint_table import (
        GCodeJointAnglesTableWidget,
        export_records_to_gcode,
        extract_records_from_strokes,
        GCodePointRecord
    )
except ImportError:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from denso_gcode_joint_table import (
        GCodeJointAnglesTableWidget,
        export_records_to_gcode,
        extract_records_from_strokes,
        GCodePointRecord
    )


# =============================================================================
# DATA STRUCTURES
# =============================================================================
class CartesianPoint:
    def __init__(self, x: float, y: float, z: float, is_draw: bool = True, feedrate: float = 1200.0):
        self.x = float(x)  # mm
        self.y = float(y)  # mm
        self.z = float(z)  # mm
        self.is_draw = bool(is_draw)
        self.feedrate = float(feedrate)
        self.q_sol: Optional[np.ndarray] = None
        self.is_valid: bool = True
        self.err_mm: float = 0.0

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def dist_to(self, other: "CartesianPoint") -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)


class Stroke:
    def __init__(self, is_draw: bool = True):
        self.points: List[CartesianPoint] = []
        self.is_draw = is_draw

    def add_point(self, pt: CartesianPoint):
        self.points.append(pt)

    def length(self) -> float:
        if len(self.points) < 2:
            return 0.0
        total = 0.0
        for i in range(len(self.points) - 1):
            total += self.points[i].dist_to(self.points[i + 1])
        return total


# =============================================================================
# IMAGE PROCESSING & G-CODE VISION PROCESSOR
# =============================================================================
class ImageGCodeProcessor:
    """
    Xử lý hình ảnh thành các đường nét và phân rã thành các điểm cách đều
    theo độ dày/khoảng cách điểm (Point Spacing) do người dùng nhập.
    Ánh xạ lên mặt phẳng vẽ trong không gian thao tác của robot Denso.
    """
    def __init__(self):
        self.original_image: Optional[np.ndarray] = None
        self.processed_mask: Optional[np.ndarray] = None
        self.strokes: List[Stroke] = []
        self.gcode_text: str = ""
        self.summary: Dict[str, Any] = {}

    def load_image(self, file_path: str) -> bool:
        if not os.path.exists(file_path):
            return False
        img = cv2.imread(file_path)
        if img is None:
            return False
        self.original_image = img
        return True

    def set_image(self, img_array: np.ndarray):
        self.original_image = img_array.copy()

    def process(
        self,
        point_spacing_mm: float = 1.5,
        threshold_mode: str = "canny",
        canny_low: int = 50,
        canny_high: int = 150,
        binary_thresh: int = 127,
        invert: bool = True,
        blur_ksize: int = 3,
        approx_epsilon: float = 0.8,
        mode: str = "contours",
        hatch_spacing_mm: float = 3.0,
        hatch_angle_deg: float = 45.0,
        center_x: float = -380.0,
        center_y: float = 0.0,
        z_draw: float = 180.0,
        z_lift: float = 205.0,
        target_width_mm: float = 200.0,
        target_height_mm: float = 200.0,
        keep_aspect: bool = True,
        rotation_deg: float = 0.0,
        draw_feed: float = 1200.0,
        travel_feed: float = 2500.0
    ) -> List[Stroke]:
        if self.original_image is None:
            return []

        # 1. Chuyển sang ảnh xám
        if len(self.original_image.shape) == 3:
            gray = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = self.original_image.copy()

        h_img, w_img = gray.shape

        # 2. Đảo màu nếu cần
        if invert:
            gray = cv2.bitwise_not(gray)

        # 3. Làm mượt lọc nhiễu Gaussian
        if blur_ksize > 1:
            k = blur_ksize if blur_ksize % 2 == 1 else blur_ksize + 1
            blurred = cv2.GaussianBlur(gray, (k, k), 0)
        else:
            blurred = gray

        # 4. Tách biên / Ngưỡng nhị phân
        if threshold_mode == "canny":
            # Đối với mẫu thử hình học và logo (có độ tương phản rõ rệt):
            # Tự động trích xuất Otsu threshold để lấy chính xác 1 đường viền duy nhất (triệt tiêu 100% nét đôi)
            _, binary_otsu = cv2.threshold(
                blurred, 0, 255,
                (cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY) + cv2.THRESH_OTSU
            )
            cnts_otsu, _ = cv2.findContours(binary_otsu, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            valid_otsu = [c for c in cnts_otsu if cv2.arcLength(c, False) >= 8.0]
            if len(valid_otsu) >= 1 and len(valid_otsu) <= 45:
                binary = binary_otsu
            else:
                binary = cv2.Canny(blurred, canny_low, canny_high)
        elif threshold_mode == "adaptive":
            binary = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2
            )
        elif threshold_mode == "otsu":
            _, binary = cv2.threshold(blurred, 0, 255, (cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY) + cv2.THRESH_OTSU)
        else:
            _, binary = cv2.threshold(blurred, binary_thresh, 255, cv2.THRESH_BINARY)

        self.processed_mask = binary.copy()

        # 5. Rút trích đường viền (Contours)
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if not contours:
            return []

        # Lọc bỏ nhiễu quá ngắn
        filtered_contours = []
        for cnt in contours:
            if cv2.arcLength(cnt, False) >= 8.0:
                if approx_epsilon > 0.0:
                    cnt_approx = cv2.approxPolyDP(cnt, approx_epsilon, False)
                    filtered_contours.append(cnt_approx)
                else:
                    filtered_contours.append(cnt)

        if not filtered_contours:
            return []

        # 6. Tìm hộp giới hạn (Bounding Box)
        all_pts = np.vstack([c.reshape(-1, 2) for c in filtered_contours])
        u_min, v_min = np.min(all_pts, axis=0)
        u_max, v_max = np.max(all_pts, axis=0)

        bb_w = max(1.0, float(u_max - u_min))
        bb_h = max(1.0, float(v_max - v_min))
        u_center = (u_min + u_max) / 2.0
        v_center = (v_min + v_max) / 2.0

        # Tính tỷ lệ thu phóng (Scale factor sang mm)
        scale_x = target_width_mm / bb_w
        scale_y = target_height_mm / bb_h
        if keep_aspect:
            scale = min(scale_x, scale_y)
            scale_x = scale
            scale_y = scale

        rad_rot = math.radians(rotation_deg)
        cos_r = math.cos(rad_rot)
        sin_r = math.sin(rad_rot)

        # Hàm biến đổi từ pixel (u, v) sang tọa độ robot Denso (X, Y)
        # Robot X: hướng tiến (trục v đảo chiều: đỉnh ảnh là tiến xa hơn), Y: hướng trái (trục u)
        def pixel_to_robot_xy(u: float, v: float) -> Tuple[float, float]:
            dx_mm = (v_center - v) * scale_y
            dy_mm = (u - u_center) * scale_x
            rx = dx_mm * cos_r - dy_mm * sin_r
            ry = dx_mm * sin_r + dy_mm * cos_r
            return (center_x + rx, center_y + ry)

        # 7. Phân rã đường nét thành các điểm cách đều theo point_spacing_mm
        raw_strokes: List[List[Tuple[float, float]]] = []
        spacing = max(0.2, float(point_spacing_mm))

        for cnt in filtered_contours:
            pts_2d = cnt.reshape(-1, 2)
            if len(pts_2d) < 2:
                continue

            mm_pts = [pixel_to_robot_xy(p[0], p[1]) for p in pts_2d]

            sampled_stroke: List[Tuple[float, float]] = [mm_pts[0]]
            accum_dist = 0.0

            for i in range(len(mm_pts) - 1):
                p1 = mm_pts[i]
                p2 = mm_pts[i + 1]
                seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                if seg_len < 1e-4:
                    continue

                ux = (p2[0] - p1[0]) / seg_len
                uy = (p2[1] - p1[1]) / seg_len

                curr_pos = 0.0
                while curr_pos + (spacing - accum_dist) <= seg_len:
                    step = spacing - accum_dist
                    curr_pos += step
                    sampled_stroke.append((p1[0] + ux * curr_pos, p1[1] + uy * curr_pos))
                    accum_dist = 0.0
                accum_dist += (seg_len - curr_pos)

            if len(sampled_stroke) > 1:
                if math.hypot(sampled_stroke[-1][0] - mm_pts[-1][0], sampled_stroke[-1][1] - mm_pts[-1][1]) > (spacing * 0.4):
                    sampled_stroke.append(mm_pts[-1])
                raw_strokes.append(sampled_stroke)

        # 8. Xử lý Chế độ Hatching nếu bật
        if mode == "hatching" and hatch_spacing_mm >= 1.0:
            hatch_strokes = self._generate_hatching(
                binary, hatch_spacing_mm, hatch_angle_deg,
                pixel_to_robot_xy, spacing
            )
            raw_strokes.extend(hatch_strokes)

        if not raw_strokes:
            return []

        # 9. Tối ưu hóa thứ tự nét vẽ (Greedy Nearest Neighbor / TSP)
        optimized_strokes = self._optimize_stroke_order(raw_strokes)

        # 10. Chuyển đổi thành danh sách đối tượng Stroke hoàn chỉnh
        strokes_result: List[Stroke] = []
        total_points = 0
        draw_len = 0.0
        rapid_len = 0.0

        prev_end: Optional[CartesianPoint] = None

        for strk_pts in optimized_strokes:
            if not strk_pts:
                continue

            start_x, start_y = strk_pts[0]
            if prev_end is not None:
                rapid_stroke = Stroke(is_draw=False)
                rapid_stroke.add_point(CartesianPoint(prev_end.x, prev_end.y, z_lift, is_draw=False, feedrate=travel_feed))
                rapid_stroke.add_point(CartesianPoint(start_x, start_y, z_lift, is_draw=False, feedrate=travel_feed))
                strokes_result.append(rapid_stroke)
                rapid_len += prev_end.dist_to(CartesianPoint(start_x, start_y, z_lift))

            draw_stroke = Stroke(is_draw=True)
            draw_stroke.add_point(CartesianPoint(start_x, start_y, z_draw, is_draw=True, feedrate=draw_feed))

            for k in range(1, len(strk_pts)):
                px, py = strk_pts[k]
                draw_stroke.add_point(CartesianPoint(px, py, z_draw, is_draw=True, feedrate=draw_feed))

            strokes_result.append(draw_stroke)
            total_points += len(draw_stroke.points)
            draw_len += draw_stroke.length()
            prev_end = draw_stroke.points[-1]

        if prev_end is not None:
            final_rapid = Stroke(is_draw=False)
            final_rapid.add_point(CartesianPoint(prev_end.x, prev_end.y, z_lift, is_draw=False, feedrate=travel_feed))
            strokes_result.append(final_rapid)

        self.strokes = strokes_result
        self.gcode_text = self._build_gcode(
            strokes_result, draw_feed, travel_feed, z_draw, z_lift
        )

        est_time_sec = (draw_len / (draw_feed / 60.0)) + (rapid_len / (travel_feed / 60.0)) + (len(strokes_result) * 0.3)

        self.summary = {
            "total_strokes": len([s for s in strokes_result if s.is_draw]),
            "total_points": total_points,
            "draw_length_mm": round(draw_len, 1),
            "rapid_length_mm": round(rapid_len, 1),
            "point_spacing_mm": spacing,
            "estimated_time_sec": round(est_time_sec, 1),
            "z_draw": z_draw,
            "z_lift": z_lift,
            "center_x": center_x,
            "center_y": center_y,
            "width_mm": round(target_width_mm, 1),
            "height_mm": round(target_height_mm, 1)
        }

        return strokes_result

    def _optimize_stroke_order(self, raw_strokes: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
        if len(raw_strokes) <= 1:
            return raw_strokes

        remaining = list(raw_strokes)
        ordered: List[List[Tuple[float, float]]] = []

        curr_stroke = remaining.pop(0)
        ordered.append(curr_stroke)
        curr_pt = curr_stroke[-1]

        while remaining:
            best_idx = 0
            best_dist = float("inf")
            reverse_best = False

            for idx, s in enumerate(remaining):
                d_start = math.hypot(curr_pt[0] - s[0][0], curr_pt[1] - s[0][1])
                d_end = math.hypot(curr_pt[0] - s[-1][0], curr_pt[1] - s[-1][1])

                if d_start < best_dist:
                    best_dist = d_start
                    best_idx = idx
                    reverse_best = False
                if d_end < best_dist:
                    best_dist = d_end
                    best_idx = idx
                    reverse_best = True

            next_stroke = remaining.pop(best_idx)
            if reverse_best:
                next_stroke.reverse()
            ordered.append(next_stroke)
            curr_pt = next_stroke[-1]

        return ordered

    def _generate_hatching(
        self, binary_mask: np.ndarray, hatch_spacing_mm: float,
        angle_deg: float, px_to_robot_fn, point_spacing: float
    ) -> List[List[Tuple[float, float]]]:
        h, w = binary_mask.shape
        step_px = max(2, int(hatch_spacing_mm * (w / 120.0)))
        strokes: List[List[Tuple[float, float]]] = []

        for y in range(0, h, step_px):
            in_seg = False
            seg_start = 0
            for x in range(w):
                val = binary_mask[y, x]
                if val > 128 and not in_seg:
                    in_seg = True
                    seg_start = x
                elif val <= 128 and in_seg:
                    in_seg = False
                    if (x - seg_start) >= 5:
                        p1 = px_to_robot_fn(seg_start, y)
                        p2 = px_to_robot_fn(x, y)
                        strokes.append([p1, p2])
            if in_seg and (w - seg_start) >= 5:
                p1 = px_to_robot_fn(seg_start, y)
                p2 = px_to_robot_fn(w - 1, y)
                strokes.append([p1, p2])

        return strokes

    def _build_gcode(
        self, strokes: List[Stroke], draw_feed: float,
        travel_feed: float, z_draw: float, z_lift: float
    ) -> str:
        lines = [
            "; ===========================================================",
            "; DENSO VS-6556 INDUSTRIAL ROBOT G-CODE GENERATOR",
            f"; Generated Date: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"; Units: Millimeters (G21) | Mode: Absolute Coordinates (G90)",
            f"; Drawing Plane Z = {z_draw:.2f} mm | Lift Clearance Z = {z_lift:.2f} mm",
            f"; Draw Feedrate = {draw_feed:.1f} mm/min | Rapid Feed = {travel_feed:.1f} mm/min",
            "; ===========================================================",
            "G21 ; Set units to millimeters",
            "G90 ; Absolute positioning mode",
            f"G00 Z{z_lift:.2f} F{travel_feed:.1f} ; Safe lift height",
            ""
        ]

        stroke_cnt = 0
        for s in strokes:
            if not s.points:
                continue

            if not s.is_draw:
                pt = s.points[-1]
                lines.append(f"G00 X{pt.x:.2f} Y{pt.y:.2f} Z{pt.z:.2f} F{travel_feed:.1f} ; Travel move")
            else:
                stroke_cnt += 1
                lines.append(f"; --- Stroke #{stroke_cnt} ({len(s.points)} points) ---")
                p0 = s.points[0]
                lines.append(f"G01 X{p0.x:.2f} Y{p0.y:.2f} Z{p0.z:.2f} F{draw_feed:.1f} ; Plunge/Pen Down")
                for pt in s.points[1:]:
                    lines.append(f"G01 X{pt.x:.2f} Y{pt.y:.2f} Z{pt.z:.2f}")

        lines.extend([
            "",
            "; --- Final Retract & Park ---",
            f"G00 Z{z_lift + 30.0:.2f} F{travel_feed:.1f} ; High clearance",
            "G00 X-380.00 Y0.00 Z220.00 ; Park at safe ready pose over drawing table",
            "M30 ; Program End",
            ""
        ])

        return "\n".join(lines)

    def parse_gcode(self, gcode_str: str, point_spacing_mm: float = 1.5, z_draw_thresh: float = 185.0) -> List[Stroke]:
        strokes: List[Stroke] = []
        curr_x, curr_y, curr_z = -380.0, 0.0, 205.0
        curr_feed = 1200.0
        modal_cmd = "G00"
        current_stroke: Optional[Stroke] = None
        spacing = max(float(point_spacing_mm), 0.5)

        lines = gcode_str.splitlines()

        for line in lines:
            line = line.strip()
            if not line or line.startswith(";") or line.startswith("("):
                continue

            if ";" in line:
                line = line.split(";")[0].strip()

            tokens = line.upper().split()
            if not tokens:
                continue

            cmd = ""
            params: Dict[str, float] = {}

            for tok in tokens:
                if tok.startswith("G") or tok.startswith("M"):
                    cmd = tok
                elif len(tok) > 1 and tok[0] in "XYZFIJR":
                    try:
                        params[tok[0]] = float(tok[1:])
                    except ValueError:
                        pass

            if cmd in ["G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"]:
                modal_cmd = cmd
            elif cmd != "":
                # Non-motion G/M command (e.g. G21, G90, M30)
                if not any(k in params for k in ("X", "Y", "Z", "I", "J", "R")):
                    continue

            if not any(k in params for k in ("X", "Y", "Z", "I", "J", "R")):
                continue

            prev_x, prev_y, prev_z = curr_x, curr_y, curr_z
            target_x = params.get("X", curr_x)
            target_y = params.get("Y", curr_y)
            target_z = params.get("Z", curr_z)
            if "F" in params:
                curr_feed = params["F"]

            active_cmd = modal_cmd
            is_drawing = (target_z <= z_draw_thresh)

            if active_cmd in ["G0", "G00"]:
                if current_stroke and current_stroke.is_draw:
                    strokes.append(current_stroke)
                    current_stroke = None
                if current_stroke is None:
                    current_stroke = Stroke(is_draw=False)
                current_stroke.add_point(
                    CartesianPoint(target_x, target_y, target_z, is_draw=False, feedrate=curr_feed)
                )

            elif active_cmd in ["G1", "G01"]:
                if current_stroke and (current_stroke.is_draw != is_drawing):
                    strokes.append(current_stroke)
                    current_stroke = None
                if current_stroke is None:
                    current_stroke = Stroke(is_draw=is_drawing)

                dist_xy = math.hypot(target_x - prev_x, target_y - prev_y)
                if is_drawing and (prev_z <= z_draw_thresh) and dist_xy > spacing * 1.25:
                    if not current_stroke.points:
                        current_stroke.add_point(
                            CartesianPoint(prev_x, prev_y, prev_z, is_draw=True, feedrate=curr_feed)
                        )
                    num_steps = max(1, int(math.ceil(dist_xy / spacing)))
                    for step in range(1, num_steps + 1):
                        u = step / float(num_steps)
                        ix = prev_x + u * (target_x - prev_x)
                        iy = prev_y + u * (target_y - prev_y)
                        iz = prev_z + u * (target_z - prev_z)
                        current_stroke.add_point(
                            CartesianPoint(ix, iy, iz, is_draw=True, feedrate=curr_feed)
                        )
                else:
                    current_stroke.add_point(
                        CartesianPoint(target_x, target_y, target_z, is_draw=is_drawing, feedrate=curr_feed)
                    )

            elif active_cmd in ["G2", "G02", "G3", "G03"]:
                cw = active_cmd in ["G2", "G02"]
                if current_stroke and (current_stroke.is_draw != is_drawing):
                    strokes.append(current_stroke)
                    current_stroke = None
                if current_stroke is None:
                    current_stroke = Stroke(is_draw=is_drawing)
                if not current_stroke.points:
                    current_stroke.add_point(
                        CartesianPoint(prev_x, prev_y, prev_z, is_draw=is_drawing, feedrate=curr_feed)
                    )

                i_off = params.get("I", 0.0)
                j_off = params.get("J", 0.0)
                cx = prev_x + i_off
                cy = prev_y + j_off
                radius = math.hypot(prev_x - cx, prev_y - cy)

                if radius < 1e-6 and "R" in params:
                    r_val = abs(params["R"])
                    dx = target_x - prev_x
                    dy = target_y - prev_y
                    chord = math.hypot(dx, dy)
                    if 1e-6 < chord <= 2.0 * r_val:
                        h = math.sqrt(max(0.0, r_val**2 - (chord / 2.0) ** 2))
                        mx = 0.5 * (prev_x + target_x)
                        my = 0.5 * (prev_y + target_y)
                        nx = -dy / chord
                        ny = dx / chord
                        sign = -1.0 if cw else 1.0
                        if params["R"] < 0:
                            sign = -sign
                        cx = mx + sign * h * nx
                        cy = my + sign * h * ny
                        radius = r_val

                if radius > 1e-6:
                    start_ang = math.atan2(prev_y - cy, prev_x - cx)
                    end_ang = math.atan2(target_y - cy, target_x - cx)
                    full_circle = math.hypot(target_x - prev_x, target_y - prev_y) < 1e-3
                    if cw:
                        sweep = -2.0 * math.pi if full_circle else (end_ang - start_ang)
                        if sweep >= -1e-6:
                            sweep -= 2.0 * math.pi
                    else:
                        sweep = 2.0 * math.pi if full_circle else (end_ang - start_ang)
                        if sweep <= 1e-6:
                            sweep += 2.0 * math.pi

                    arc_len = abs(sweep) * radius
                    num_steps = max(12, int(math.ceil(arc_len / spacing)))
                    for step in range(1, num_steps + 1):
                        u = step / float(num_steps)
                        ang = start_ang + u * sweep
                        ix = cx + radius * math.cos(ang)
                        iy = cy + radius * math.sin(ang)
                        iz = prev_z + u * (target_z - prev_z)
                        current_stroke.add_point(
                            CartesianPoint(ix, iy, iz, is_draw=is_drawing, feedrate=curr_feed)
                        )
                else:
                    current_stroke.add_point(
                        CartesianPoint(target_x, target_y, target_z, is_draw=is_drawing, feedrate=curr_feed)
                    )

            curr_x, curr_y, curr_z = target_x, target_y, target_z

        if current_stroke and current_stroke.points:
            strokes.append(current_stroke)

        self.strokes = strokes
        self.gcode_text = gcode_str
        return strokes


# =============================================================================
# KINEMATICS & COLLISION SAFETY VALIDATOR
# =============================================================================
class KinematicsValidator:
    """
    Kiểm tra tính khả thi động học nghịch (IK) và an toàn va chạm của toàn bộ
    các điểm trên đường quỹ đạo vẽ trước khi gửi tới robot thật/Gazebo.
    """
    def __init__(self, engine: DensoDynamicsEngine):
        self.engine = engine

    def validate(self, strokes: List[Stroke]) -> Dict[str, Any]:
        total_pts = 0
        valid_pts = 0
        invalid_pts = 0
        collision_pts = 0
        q_solutions: List[Optional[np.ndarray]] = []

        # Khởi tạo góc khớp sẵn sàng với đầu bút hướng vuông góc xuống mặt bàn (6 khớp)
        q_seed = np.array([0.0, 0.908, 1.035, 0.0, 1.199, 0.0])

        for s in strokes:
            for pt in s.points:
                total_pts += 1
                pos_m = [pt.x / 1000.0, pt.y / 1000.0, pt.z / 1000.0]

                # Bút luôn hướng thẳng đứng vuông góc xuống mặt giấy (pitch = pi)
                q_sol, ok, iters, err_m = self.engine.inverse_kinematics(
                    pos_m, q_seed=q_seed, use_flange=True, mode="hybrid",
                    tool_pitch_rad=math.pi
                )

                pt.err_mm = err_m * 1000.0

                if ok and q_sol is not None:
                    is_col, col_reason = self.engine.check_collision(q_sol)
                    if not is_col:
                        pt.is_valid = True
                        pt.q_sol = q_sol
                        valid_pts += 1
                        q_seed = q_sol.copy()
                        q_solutions.append(q_sol)
                    else:
                        pt.is_valid = False
                        collision_pts += 1
                        q_solutions.append(None)
                else:
                    pt.is_valid = False
                    invalid_pts += 1
                    q_solutions.append(None)

        pct = (valid_pts / total_pts * 100.0) if total_pts > 0 else 0.0
        is_safe = (valid_pts == total_pts and total_pts > 0)

        return {
            "is_safe": is_safe,
            "total_points": total_pts,
            "valid_points": valid_pts,
            "invalid_points": invalid_pts,
            "collision_points": collision_pts,
            "feasibility_pct": round(pct, 1),
            "solutions": q_solutions
        }


# =============================================================================
# ROS 2 DRAWING TRAIL & PATH MANAGER (RViz2 Markers)
# =============================================================================

# =============================================================================
# GAZEBO SIM DIRECT DRAWING TRAIL & INK MANAGER (LIGHTWEIGHT & REAL-TIME)
# =============================================================================
class GazeboDrawingTrailPublisher:
    """
    Quản lý và xuất bản vệt vẽ lớn, rõ nét và tức thời trong Gazebo Sim
    bằng cách phát sóng gz.msgs.Marker_V qua Gazebo Transport service '/marker_array'.
    Sử dụng luồng nền bất đồng bộ (Non-blocking Async Worker Thread) kết hợp
    LINE_LIST (nối nét) + POINTS (độ đậm hạt mực) + SPHERE (ngòi bút phát sáng),
    đảm bảo 100% real-time, không gây giật lag simulation hay nghẽn bộ nhớ.
    """
    def __init__(self, world_name: str = "denso_world"):
        self.world_name = world_name
        self.gz_node = None
        self.gz_marker_v_pb2 = None
        self.gz_boolean_pb2 = None
        self.is_gz_connected = False

        self.completed_strokes: List[List[Tuple[float, float, float]]] = []
        self.current_stroke: List[Tuple[float, float, float]] = []
        self.current_tip: Optional[Tuple[float, float, float, bool]] = None
        
        self.paper_z = 0.1765  # Cao hơn mặt giấy 0.1745m đúng 2mm, triệt tiêu hoàn toàn z-fighting
        self.point_size = 2.0  # Kích thước hạt mực mảnh, sắc nét không bị nhòe khối
        self.lock = threading.Lock()
        
        self._is_running = True
        self._need_clear = False
        self._dirty_event = threading.Event()
        
        self._init_gz()
        
        # Khởi động luồng nền cập nhật Gazebo Sim
        self._worker_thread = threading.Thread(target=self._gz_worker_loop, daemon=True)
        self._worker_thread.start()

    def _init_gz(self):
        try:
            if "GZ_PARTITION" not in os.environ:
                os.environ["GZ_PARTITION"] = "denso_ws"
            if "GZ_IP" not in os.environ:
                os.environ["GZ_IP"] = "127.0.0.1" 
            import ctypes
            libs_to_load = [
                "/opt/ros/jazzy/opt/gz_utils_vendor/lib/libgz-utils2.so.2",
                "/opt/ros/jazzy/opt/gz_math_vendor/lib/libgz-math7.so.7",
                "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/libgz-msgs10.so.10",
                "/opt/ros/jazzy/opt/gz_transport_vendor/lib/libgz-transport13.so.13"
            ]
            for lp in libs_to_load:
                if os.path.exists(lp):
                    try:
                        ctypes.CDLL(lp, mode=ctypes.RTLD_GLOBAL)
                    except Exception:
                        pass

            for p in ["/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python", "/opt/ros/jazzy/opt/gz_transport_vendor/lib/python"]:
                if p not in sys.path:
                    sys.path.append(p)
            from gz.transport13 import Node as GzNode
            from gz.msgs10 import marker_v_pb2, boolean_pb2
            self.gz_node = GzNode()
            self.gz_marker_v_pb2 = marker_v_pb2
            self.gz_boolean_pb2 = boolean_pb2
            self.is_gz_connected = True
        except Exception:
            self.is_gz_connected = False

    def set_thickness(self, thickness_mm: float):
        with self.lock:
            self.point_size = max(1.5, min(4.0, float(thickness_mm) * 0.8))
            self._dirty_event.set()

    def add_point(self, x_mm: float, y_mm: float, z_mm: float, is_draw: bool):
        if not self.is_gz_connected:
            return

        x_m = float(x_mm / 1000.0)
        y_m = float(y_mm / 1000.0)
        z_m = max(self.paper_z, float(z_mm / 1000.0)) if is_draw else float(z_mm / 1000.0)

        with self.lock:
            self.current_tip = (x_m, y_m, z_m, is_draw)
            if is_draw:
                self.current_stroke.append((x_m, y_m, z_m))
            else:
                if len(self.current_stroke) >= 2:
                    self.completed_strokes.append(list(self.current_stroke))
                self.current_stroke = []
            self._dirty_event.set()

    def end_stroke(self):
        with self.lock:
            if len(self.current_stroke) >= 2:
                self.completed_strokes.append(list(self.current_stroke))
            self.current_stroke = []
            self._dirty_event.set()

    def clear(self):
        with self.lock:
            self.completed_strokes = []
            self.current_stroke = []
            self.current_tip = None
            self._need_clear = True
            self._dirty_event.set()

    def update_pen_tip(self, x_m: float, y_m: float, z_m: float, is_draw: bool = False):
        with self.lock:
            self.current_tip = (float(x_m), float(y_m), float(z_m), bool(is_draw))
            self._dirty_event.set()

    def _gz_worker_loop(self):
        time.sleep(0.5)
        while self._is_running:
            self._dirty_event.wait(timeout=0.033)
            self._dirty_event.clear()

            if not self.is_gz_connected or not self.gz_node:
                continue

            with self.lock:
                need_clear = self._need_clear
                self._need_clear = False
                strokes = list(self.completed_strokes)
                cur_stroke = list(self.current_stroke)
                tip = self.current_tip
                pt_size = self.point_size

            if need_clear:
                try:
                    mv_clear = self.gz_marker_v_pb2.Marker_V()
                    m_del = mv_clear.marker.add()
                    m_del.action = m_del.DELETE_ALL
                    self.gz_node.request("/marker_array", mv_clear, self.gz_marker_v_pb2.Marker_V, self.gz_boolean_pb2.Boolean, 500)
                except Exception:
                    pass

            if not strokes and not cur_stroke and tip is None:
                continue

            try:
                mv = self.gz_marker_v_pb2.Marker_V()
                all_strokes = strokes + ([cur_stroke] if len(cur_stroke) >= 1 else [])
                total_segments = sum(max(0, len(s) - 1) for s in all_strokes)

                if total_segments > 0:
                    # 1. Đoạn nối đường (LINE_LIST)
                    m_line = mv.marker.add()
                    m_line.ns = "denso_gazebo_trail"
                    m_line.id = 1
                    m_line.action = m_line.ADD_MODIFY
                    m_line.type = m_line.LINE_LIST
                    m_line.scale.x = 1.0
                    m_line.scale.y = 1.0
                    m_line.scale.z = 1.0
                    for mat in [m_line.material.ambient, m_line.material.diffuse]:
                        mat.r = 0.0; mat.g = 0.90; mat.b = 1.0; mat.a = 1.0

                    # 2. Hạt mực đậm (POINTS)
                    m_pts = mv.marker.add()
                    m_pts.ns = "denso_gazebo_trail"
                    m_pts.id = 2
                    m_pts.action = m_pts.ADD_MODIFY
                    m_pts.type = m_pts.POINTS
                    m_pts.scale.x = float(pt_size)
                    m_pts.scale.y = float(pt_size)
                    m_pts.scale.z = float(pt_size)
                    for mat in [m_pts.material.ambient, m_pts.material.diffuse]:
                        mat.r = 0.0; mat.g = 0.90; mat.b = 1.0; mat.a = 1.0

                    for s in all_strokes:
                        for idx in range(len(s)):
                            pt_msg = m_pts.point.add()
                            pt_msg.x, pt_msg.y, pt_msg.z = s[idx]
                            if idx < len(s) - 1:
                                pa = m_line.point.add()
                                pa.x, pa.y, pa.z = s[idx]
                                pb = m_line.point.add()
                                pb.x, pb.y, pb.z = s[idx+1]

                # 3. Con trỏ ngòi bút Stylus Pen Tip (SPHERE)
                if tip is not None:
                    tx, ty, tz, is_drawing = tip
                    m_tip = mv.marker.add()
                    m_tip.ns = "denso_gazebo_trail"
                    m_tip.id = 9999
                    m_tip.action = m_tip.ADD_MODIFY
                    m_tip.type = m_tip.SPHERE
                    m_tip.scale.x = 0.006
                    m_tip.scale.y = 0.006
                    m_tip.scale.z = 0.006
                    m_tip.pose.position.x = float(tx)
                    m_tip.pose.position.y = float(ty)
                    m_tip.pose.position.z = float(tz)
                    if is_drawing:
                        for mat in [m_tip.material.ambient, m_tip.material.diffuse]:
                            mat.r = 0.0; mat.g = 1.0; mat.b = 0.4; mat.a = 1.0
                    else:
                        for mat in [m_tip.material.ambient, m_tip.material.diffuse]:
                            mat.r = 1.0; mat.g = 0.75; mat.b = 0.1; mat.a = 0.9

                if len(mv.marker) > 0:
                    self.gz_node.request("/marker_array", mv, self.gz_marker_v_pb2.Marker_V, self.gz_boolean_pb2.Boolean, 500)
            except Exception:
                pass


class DrawingTrailPublisher:
    """
    Quản lý và xuất bản vệt di chuyển (Drawing Trail) 3D cho RViz2 bằng
    visualization_msgs/msg/MarkerArray và nav_msgs/msg/Path.
    - Phân biệt rõ nét vẽ (Pen Down: Cyan sáng, độ dày tùy chỉnh)
      và nét nhấc bút di chuyển (Pen Up: không để lại mực).
    """
    def __init__(self, node: Any = None):
        self.node = node
        self.marker_pub = node.create_publisher(MarkerArray, "/denso/drawing_trail", 10)
        self.path_pub = node.create_publisher(Path, "/denso/drawing_path", 10)

        self.stroke_markers: List[Marker] = []
        self.current_drawing_points: List[Point] = []
        self.active_stroke_id = 100
        self.trail_thickness_m = 0.003  # 3mm mặc định
        self.gz_trail = GazeboDrawingTrailPublisher()

        self.lock = threading.Lock()

    def set_thickness(self, thickness_mm: float):
        with self.lock:
            self.trail_thickness_m = max(0.001, thickness_mm / 1000.0)
        if hasattr(self, 'gz_trail') and self.gz_trail:
            self.gz_trail.set_thickness(thickness_mm)

    def add_point(self, x_mm: float, y_mm: float, z_mm: float, is_draw: bool):
        p = Point()
        p.x = float(x_mm / 1000.0)
        p.y = float(y_mm / 1000.0)
        p.z = float(z_mm / 1000.0)

        if hasattr(self, 'gz_trail') and self.gz_trail:
            self.gz_trail.add_point(x_mm, y_mm, z_mm, is_draw)

        with self.lock:
            if is_draw:
                self.current_drawing_points.append(p)
            else:
                if len(self.current_drawing_points) >= 2:
                    self._commit_current_stroke()
                self.current_drawing_points = []

    def end_stroke(self):
        if hasattr(self, 'gz_trail') and self.gz_trail:
            self.gz_trail.end_stroke()
        with self.lock:
            if len(self.current_drawing_points) >= 2:
                self._commit_current_stroke()
            self.current_drawing_points = []

    def _commit_current_stroke(self):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "denso_drawing_trail"
        m.id = self.active_stroke_id
        self.active_stroke_id += 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = self.trail_thickness_m

        # Màu sắc vệt vẽ: Cyan sáng rực rỡ (0.0, 0.94, 1.0)
        m.color.r = 0.0
        m.color.g = 0.94
        m.color.b = 1.0
        m.color.a = 0.95

        m.points = list(self.current_drawing_points)
        self.stroke_markers.append(m)

    def publish_update(self, current_pos_m: Optional[Tuple[float, float, float]] = None, is_draw: bool = False):
        if current_pos_m is not None and hasattr(self, 'gz_trail') and self.gz_trail:
            self.gz_trail.update_pen_tip(current_pos_m[0], current_pos_m[1], current_pos_m[2], is_draw=is_draw)
        if not self.node:
            return

        ma = MarkerArray()
        now = self.node.get_clock().now().to_msg()

        with self.lock:
            # 1. Các nét đã hoàn thành
            for sm in self.stroke_markers:
                sm.header.stamp = now
                ma.markers.append(sm)

            # 2. Nét đang vẽ dở (Live in-progress stroke)
            if len(self.current_drawing_points) >= 2:
                m_live = Marker()
                m_live.header.frame_id = "base_link"
                m_live.header.stamp = now
                m_live.ns = "denso_drawing_trail"
                m_live.id = 99
                m_live.type = Marker.LINE_STRIP
                m_live.action = Marker.ADD
                m_live.scale.x = self.trail_thickness_m
                m_live.color.r = 0.0
                m_live.color.g = 0.94
                m_live.color.b = 1.0
                m_live.color.a = 0.95
                m_live.points = list(self.current_drawing_points)
                ma.markers.append(m_live)

            # 3. Điểm con trỏ đầu bút (Pen Tip Indicator)
            if current_pos_m is not None:
                m_tip = Marker()
                m_tip.header.frame_id = "base_link"
                m_tip.header.stamp = now
                m_tip.ns = "denso_pen_tip"
                m_tip.id = 1
                m_tip.type = Marker.SPHERE
                m_tip.action = Marker.ADD
                m_tip.pose.position.x = float(current_pos_m[0])
                m_tip.pose.position.y = float(current_pos_m[1])
                m_tip.pose.position.z = float(current_pos_m[2])
                m_tip.scale.x = 0.006
                m_tip.scale.y = 0.006
                m_tip.scale.z = 0.006

                if is_draw:
                    m_tip.color.r = 0.1
                    m_tip.color.g = 1.0
                    m_tip.color.b = 0.4
                    m_tip.color.a = 0.95
                else:
                    m_tip.color.r = 1.0
                    m_tip.color.g = 0.7
                    m_tip.color.b = 0.1
                    m_tip.color.a = 0.8

                ma.markers.append(m_tip)

        self.marker_pub.publish(ma)

    def clear(self):
        if hasattr(self, 'gz_trail') and self.gz_trail:
            self.gz_trail.clear()
        """Xóa sạch vệt vẽ trong RViz2 ngay lập tức."""
        with self.lock:
            self.stroke_markers = []
            self.current_drawing_points = []
            self.active_stroke_id = 100

        ma = MarkerArray()
        m_del = Marker()
        m_del.action = Marker.DELETEALL
        ma.markers.append(m_del)
        self.marker_pub.publish(ma)


# =============================================================================
# TRAJECTORY EXECUTION THREAD (Điều khiển cánh tay di chuyển theo điểm)
# =============================================================================
class TrajectoryExecutionWorker(QThread):
    """
    Tiến trình chạy nền thực thi tuần tự quỹ đạo vẽ, truyền dữ liệu tới
    bộ điều khiển /arm_controller/joint_trajectory trong ROS 2,
    cập nhật tiến trình và kích hoạt tạo vệt vẽ thời gian thực.
    """
    sig_progress = pyqtSignal(int, int, float, float, float, float, bool, list)  # curr, total, pct, x, y, z, is_draw, q_target
    sig_status = pyqtSignal(str, str)                                       # status_msg, level ('info', 'ok', 'warn', 'error')
    sig_finished = pyqtSignal()

    def __init__(self, bridge, engine: DensoDynamicsEngine, trail_pub: DrawingTrailPublisher):
        super().__init__()
        self.bridge = bridge
        self.engine = engine
        self.trail_pub = trail_pub

        self.strokes: List[Stroke] = []
        self.speed_scale: float = 1.0

        self.is_running = False
        self.is_paused = False
        self.pause_cond = threading.Condition()

    def load_strokes(self, strokes: List[Stroke], speed_scale: float = 1.0):
        self.strokes = strokes
        self.speed_scale = max(0.1, min(3.0, speed_scale))

    def pause(self):
        with self.pause_cond:
            self.is_paused = True
            if self.bridge and hasattr(self.bridge, 'latest_positions'):
                self.bridge.send_trajectory(self.bridge.latest_positions, duration_sec=0.2, zero_velocity=True)
            self.sig_status.emit("⏸️ ĐÃ TẠM DỪNG VẼ (Đang giữ nguyên vị trí)", "warn")

    def resume(self):
        with self.pause_cond:
            self.is_paused = False
            self.pause_cond.notify_all()
            self.sig_status.emit("▶️ TIẾP TỤC VẼ", "ok")

    def cancel(self):
        self.is_running = False
        with self.pause_cond:
            self.is_paused = False
            self.pause_cond.notify_all()
        if self.bridge and hasattr(self.bridge, 'latest_positions'):
            self.bridge.send_trajectory(self.bridge.latest_positions, duration_sec=0.2, zero_velocity=True)
        self.sig_status.emit("⏹️ ĐÃ HỦY THỰC THI (Dừng an toàn)", "warn")

    def run(self):
        if not self.strokes or not self.bridge:
            self.sig_status.emit("❌ Không có quỹ đạo hoặc chưa kết nối ROS 2!", "error")
            self.sig_finished.emit()
            return

        self.is_running = True
        self.is_paused = False

        total_pts = sum(len(s.points) for s in self.strokes)
        curr_pt_idx = 0

        self.sig_status.emit(f"🚀 Bắt đầu thực thi {len(self.strokes)} nét vẽ ({total_pts} điểm)...", "ok")

        q_curr = list(self.bridge.latest_positions)
        if len(q_curr) < 6:
            q_curr = [0.0] * 6

        # BƯỚC 1: Tiếp cận an toàn tới điểm đầu tiên của nét vẽ đầu tiên
        first_pt = self.strokes[0].points[0]
        if first_pt.q_sol is not None:
            q_start = list(first_pt.q_sol)
        else:
            pos_m = [first_pt.x / 1000.0, first_pt.y / 1000.0, first_pt.z / 1000.0]
            q_sol, ok, _, _ = self.engine.inverse_kinematics(
                pos_m, q_seed=q_curr, use_flange=True, mode="hybrid", tool_pitch_rad=math.pi
            )
            q_start = list(q_sol) if (ok and q_sol is not None) else list(q_curr)

        max_dq = max(abs(q_start[i] - q_curr[i]) for i in range(min(len(q_start), len(q_curr))))
        if max_dq > 0.03:
            transit_dur = max(1.8, min(4.2, max_dq / 1.0))
            self.sig_status.emit(f"🤖 Đang di chuyển cánh tay tới điểm bắt đầu ({transit_dur:.1f}s)...", "info")
            self.bridge.send_trajectory(q_start, duration_sec=transit_dur, zero_velocity=True)
            
            t_start = time.time()
            max_wait = transit_dur * 3.0 + 1.0
            while (time.time() - t_start < max_wait) and self.is_running:
                pos_act = list(self.bridge.latest_positions)
                if len(pos_act) >= 6:
                    dq = max(abs(pos_act[i] - q_start[i]) for i in range(6))
                    if dq < 0.035:
                        break
                time.sleep(0.04)
                with self.pause_cond:
                    while self.is_paused and self.is_running:
                        self.pause_cond.wait()
            q_curr = list(q_start)

        # BƯỚC 2: Thực thi từng nét vẽ liên tục tức thời mượt mà (Instant Fluid Execution)
        for s_idx, stroke in enumerate(self.strokes):
            if not self.is_running:
                break

            with self.pause_cond:
                while self.is_paused and self.is_running:
                    self.pause_cond.wait()

            if not self.is_running:
                break

            is_drawing = stroke.is_draw
            stroke_pts = stroke.points
            if not stroke_pts:
                continue

            n_pts = len(stroke_pts)
            for p_idx, pt in enumerate(stroke_pts):
                if not self.is_running:
                    break

                with self.pause_cond:
                    while self.is_paused and self.is_running:
                        self.pause_cond.wait()

                if not self.is_running:
                    break

                pos_m = [pt.x / 1000.0, pt.y / 1000.0, pt.z / 1000.0]
                if pt.q_sol is not None:
                    q_target = pt.q_sol
                else:
                    q_sol, ok, _, _ = self.engine.inverse_kinematics(
                        pos_m, q_seed=q_curr, use_flange=True, mode="hybrid",
                        tool_pitch_rad=math.pi
                    )
                    if not ok or q_sol is None:
                        continue
                    q_target = q_sol

                is_last_pt = (p_idx == n_pts - 1)
                # Tốc độ di chuyển tức thời mượt mà (0.035s - 0.12s)
                dt_step = max(0.035, min(0.12, 0.055 / self.speed_scale))

                # Gửi lệnh Trajectory tới bộ điều khiển Gazebo:
                # zero_velocity=False giúp chuyển động liên tục lượn sóng không bị khựng phanh giật từng bước;
                # zero_velocity=True chỉ ở điểm cuối nét để dừng dứt khoát
                self.bridge.send_trajectory(q_target, duration_sec=dt_step, zero_velocity=is_last_pt)
                q_curr = list(q_target)

                # Xuất bản vệt vẽ tức thời thời gian thực tới RViz2 & Gazebo Sim
                if self.trail_pub:
                    self.trail_pub.add_point(pt.x, pt.y, pt.z, is_drawing)
                    self.trail_pub.publish_update(current_pos_m=pos_m, is_draw=is_drawing)

                # Cập nhật tiến trình GUI & con trỏ trên Canvas 2D tức thời
                curr_pt_idx += 1
                pct = (curr_pt_idx / total_pts * 100.0) if total_pts > 0 else 0.0
                q_list = [float(v) for v in q_target]
                self.sig_progress.emit(curr_pt_idx, total_pts, pct, pt.x, pt.y, pt.z, is_drawing, q_list)

                time.sleep(dt_step * 0.92)

            if is_drawing and self.trail_pub:
                self.trail_pub.end_stroke()

        if self.is_running:
            self.sig_status.emit("🎉 HOÀN THÀNH QUÁ TRÌNH VẼ HÌNH THÀNH CÔNG!", "ok")
        self.is_running = False
        self.sig_finished.emit()


# =============================================================================
# INTERACTIVE 2D CANVAS WIDGET (Hiển thị Toolpath & Vệt Vẽ Trực Tiếp)
# =============================================================================
class InteractiveDrawingCanvas(QWidget):
    """
    Khung vẽ tương tác trực quan 2D hiển thị hình ảnh, đường chạy dao G-code
    và vệt mực vẽ theo thời gian thực (Real-time Live Trail).
    Hỗ trợ phóng to, thu nhỏ, cuộn chuột và di chuột xem tọa độ.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 420)
        self.setStyleSheet("background-color: #050b14; border: 1px solid #1e293b; border-radius: 8px;")

        self.strokes: List[Stroke] = []
        self.executed_points: List[Tuple[float, float, bool]] = []
        self.current_cursor: Optional[Tuple[float, float, float, bool]] = None
        self.processed_pixmap: Optional[QPixmap] = None

        # Camera view (zoom & pan)
        self.scale_factor = 1.0
        self.pan_offset = QPointF(0.0, 0.0)
        self.is_dragging = False
        self.last_mouse_pos = QPointF()

        # Workspace limits (mm)
        self.ws_center_x = -380.0
        self.ws_center_y = 0.0
        self.ws_width = 160.0
        self.ws_height = 160.0

        self.setMouseTracking(True)

    def set_data(self, strokes: List[Stroke], center_x: float, center_y: float, width_mm: float, height_mm: float):
        self.strokes = strokes
        self.executed_points = []
        self.current_cursor = None
        self.ws_center_x = center_x
        self.ws_center_y = center_y
        self.ws_width = width_mm
        self.ws_height = height_mm
        self.reset_view()
        self.update()

    def update_cursor_progress(self, x: float, y: float, z: float, is_draw: bool):
        self.current_cursor = (x, y, z, is_draw)
        self.executed_points.append((x, y, is_draw))
        self.update()

    def clear_trail(self):
        self.executed_points = []
        self.current_cursor = None
        self.update()

    def reset_view(self):
        self.scale_factor = 1.0
        self.pan_offset = QPointF(0.0, 0.0)
        self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        zoom = 1.15 if delta > 0 else 0.85
        self.scale_factor = max(0.2, min(8.0, self.scale_factor * zoom))
        self.update()

    def mousePressEvent(self, event):
        if event.button() in [Qt.LeftButton, Qt.MiddleButton]:
            self.is_dragging = True
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.is_dragging:
            delta = event.pos() - self.last_mouse_pos
            self.pan_offset += QPointF(delta.x(), delta.y())
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() in [Qt.LeftButton, Qt.MiddleButton]:
            self.is_dragging = False

    def mouseDoubleClickEvent(self, event):
        self.reset_view()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        cx = w / 2.0 + self.pan_offset.x()
        cy = h / 2.0 + self.pan_offset.y()

        ppm = (min(w, h) / (self.ws_width * 1.6)) * self.scale_factor

        def to_canvas(rx: float, ry: float) -> QPointF:
            px = cx + (ry - self.ws_center_y) * ppm
            py = cy - (rx - self.ws_center_x) * ppm
            return QPointF(px, py)

        # 1. Vẽ lưới tọa độ Millimeter Grid
        painter.setPen(QPen(QColor("#0f172a"), 1, Qt.SolidLine))
        grid_step_mm = 20.0
        grid_step_px = grid_step_mm * ppm

        if grid_step_px > 10:
            for gx in np.arange(cx % grid_step_px, w, grid_step_px):
                painter.drawLine(int(gx), 0, int(gx), h)
            for gy in np.arange(cy % grid_step_px, h, grid_step_px):
                painter.drawLine(0, int(gy), w, int(gy))

        # 2. Khung giới hạn bàn vẽ (Workspace Bounding Box)
        hw = self.ws_width / 2.0
        hh = self.ws_height / 2.0
        p_tl = to_canvas(self.ws_center_x + hh, self.ws_center_y - hw)
        p_br = to_canvas(self.ws_center_x - hh, self.ws_center_y + hw)
        rect_ws = QRectF(p_tl, p_br).normalized()

        painter.setPen(QPen(QColor("#1e293b"), 1, Qt.DashLine))
        painter.setBrush(QBrush(QColor(15, 23, 42, 80)))
        painter.drawRoundedRect(rect_ws, 6, 6)

        painter.setPen(QPen(QColor("#334155"), 1))
        painter.drawLine(int(cx - 15), int(cy), int(cx + 15), int(cy))
        painter.drawLine(int(cx), int(cy - 15), int(cx), int(cy + 15))
        painter.setPen(QColor("#64748b"))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(int(cx + 6), int(cy - 6), f"Tâm ({self.ws_center_x:.0f}, {self.ws_center_y:.0f})")

        # 3. Vẽ đường chạy dao G-code dự kiến (Template Toolpath)
        for s in self.strokes:
            if not s.points:
                continue

            if not s.is_draw:
                painter.setPen(QPen(QColor(234, 179, 8, 120), 1, Qt.DashLine))
            else:
                painter.setPen(QPen(QColor(34, 197, 94, 90), 1.5, Qt.SolidLine))

            pts_canvas = [to_canvas(pt.x, pt.y) for pt in s.points]
            for i in range(len(pts_canvas) - 1):
                painter.drawLine(pts_canvas[i], pts_canvas[i + 1])

        # 4. Vẽ VỆT DI CHUYỂN THỰC TẾ (Executed Live Drawing Trail)
        pen_trail = QPen(QColor("#00f0ff"), 3.0, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(pen_trail)

        draw_path = QPainterPath()
        in_segment = False

        for pt_data in self.executed_points:
            x, y, is_draw = pt_data
            cp = to_canvas(x, y)
            if is_draw:
                if not in_segment:
                    draw_path.moveTo(cp)
                    in_segment = True
                else:
                    draw_path.lineTo(cp)
            else:
                in_segment = False

        painter.drawPath(draw_path)

        # 5. Vẽ đầu ngòi bút hiện tại (Active Pen Cursor)
        if self.current_cursor is not None:
            cur_x, cur_y, cur_z, cur_draw = self.current_cursor
            p_cursor = to_canvas(cur_x, cur_y)

            glow_col = QColor("#10b981") if cur_draw else QColor("#f59e0b")
            glow_col.setAlpha(80)
            painter.setBrush(QBrush(glow_col))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(p_cursor, 9, 9)

            tip_col = QColor("#34d399") if cur_draw else QColor("#fbbf24")
            painter.setBrush(QBrush(tip_col))
            painter.setPen(QPen(Qt.white, 1.5))
            painter.drawEllipse(p_cursor, 4, 4)

            state_str = "HẠ BÚT (VẼ)" if cur_draw else "NHẤC BÚT"
            painter.setFont(QFont("Monospace", 8, QFont.Bold))
            painter.setPen(tip_col)
            painter.drawText(
                int(p_cursor.x() + 12), int(p_cursor.y() - 6),
                f"[{cur_x:.1f}, {cur_y:.1f}, {cur_z:.1f}] - {state_str}"
            )


# =============================================================================
# MAIN UI TAB: IMAGE & G-CODE DRAWING INTERFACE
# =============================================================================
class ImageGCodeDrawingTab(QWidget):
    """
    Giao diện tích hợp hoàn chỉnh Tab 5:
    1. Chọn ảnh & Mẫu thử có sẵn
    2. Cài đặt lọc biên & Tự nhập độ dày/khoảng cách điểm (Point Spacing)
    3. Thiết lập không gian làm việc (Workspace Mapping X, Y, Z_draw, Z_lift)
    4. Sinh G-code, xem trước và chỉnh sửa
    5. Kiểm tra an toàn IK & Khả thi khớp
    6. Điều khiển thực thi mượt mà & Hiển thị vệt vẽ RViz2 + Canvas
    """
    def __init__(self, bridge, engine: DensoDynamicsEngine, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.engine = engine

        self.processor = ImageGCodeProcessor()
        self.validator = KinematicsValidator(engine)
        self.trail_pub = DrawingTrailPublisher(bridge.node) if (bridge and hasattr(bridge, "node") and bridge.node) else None
        self.worker: Optional[TrajectoryExecutionWorker] = None

        self.current_strokes: List[Stroke] = []
        self.joint_table_widget: Optional[GCodeJointAnglesTableWidget] = None
        self.sample_dir = "/home/dung/denso_ws/drawing_samples"

        self.init_ui()
        # Nạp mẫu thử lười (Lazy load) khi người dùng mở Tab 5, giúp khởi động ứng dụng tức thì trong 0.2s
        self._initial_loaded = False

    def showEvent(self, event):
        super().showEvent(event)
        if not getattr(self, '_initial_loaded', False):
            self._initial_loaded = True
            QTimer.singleShot(30, self._load_initial_sample)

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        splitter = QSplitter(Qt.Horizontal)

        # ---------------------------------------------------------------------
        # CỘT TRÁI: ĐIỀU KHIỂN & CÀI ĐẶT
        # ---------------------------------------------------------------------
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(8)

        # 1. Box Chọn ảnh
        box_input = QGroupBox("📁 1. CHỌN ẢNH ĐẦU VÀO & MẪU THỬ")
        box_input.setStyleSheet(self._group_style())
        in_lay = QVBoxLayout(box_input)
        in_lay.setSpacing(6)

        btn_row = QHBoxLayout()
        self.btn_open_img = QPushButton("📂 Mở tệp ảnh...")
        self.btn_open_img.setStyleSheet(self._btn_secondary())
        self.btn_open_img.clicked.connect(self._browse_image)
        btn_row.addWidget(self.btn_open_img)

        self.combo_samples = QComboBox()
        self.combo_samples.setStyleSheet(self._combo_style())
        self.combo_samples.addItem("⭐ Ngôi sao 5 cánh (Star)", "star_5point.png")
        self.combo_samples.addItem("🎓 Logo Bách Khoa (HCMUT - BK)", "logoBK.png")
        self.combo_samples.addItem("🤖 Logo DENSO Robot", "denso_robot_logo.png")
        self.combo_samples.addItem("⚙️ Bánh răng cơ khí (Gear)", "gear_pattern.png")
        self.combo_samples.addItem("🌀 Hoa xoắn ốc (Spiral)", "spiral_flower.png")
        self.combo_samples.addItem("❤️ Trái tim nghệ thuật (Heart)", "heart_shape.png")
        self.combo_samples.currentIndexChanged.connect(self._on_sample_changed)
        btn_row.addWidget(self.combo_samples)
        in_lay.addLayout(btn_row)

        self.lbl_img_path = QLabel("Tệp: star_5point.png")
        self.lbl_img_path.setStyleSheet("color: #94a3b8; font-size: 11px;")
        in_lay.addWidget(self.lbl_img_path)
        left_layout.addWidget(box_input)

        # 2. Box Thông số phân rã điểm (Point Spacing)
        box_pts = QGroupBox("⚙️ 2. PHÂN RÃ ĐIỂM & ĐỘ DÀY (POINT SPACING)")
        box_pts.setStyleSheet(self._group_style())
        pts_lay = QGridLayout(box_pts)
        pts_lay.setSpacing(6)

        # KHOẢNG CÁCH / ĐỘ DÀY CÁC ĐIỂM (NGƯỜI DÙNG TỰ NHẬP)
        pts_lay.addWidget(QLabel("📏 Khoảng cách điểm (mm):"), 0, 0)
        self.sp_point_spacing = QDoubleSpinBox()
        self.sp_point_spacing.setRange(0.2, 10.0)
        self.sp_point_spacing.setValue(1.5)
        self.sp_point_spacing.setSingleStep(0.2)
        self.sp_point_spacing.setDecimals(2)
        self.sp_point_spacing.setToolTip("Khoảng cách nội suy giữa các điểm kề nhau (càng nhỏ điểm càng dày, nét càng mịn)")
        self.sp_point_spacing.setStyleSheet(self._spin_style())
        pts_lay.addWidget(self.sp_point_spacing, 0, 1)

        # ĐỘ DÀY VỆT VẼ TRONG RVIZ2 (TRAIL THICKNESS)
        pts_lay.addWidget(QLabel("🖌️ Độ dày vệt vẽ RViz (mm):"), 1, 0)
        self.sp_trail_thick = QDoubleSpinBox()
        self.sp_trail_thick.setRange(1.0, 25.0)
        self.sp_trail_thick.setValue(8.0)
        self.sp_trail_thick.setSingleStep(0.5)
        self.sp_trail_thick.setStyleSheet(self._spin_style())
        self.sp_trail_thick.valueChanged.connect(self._on_trail_thick_changed)
        pts_lay.addWidget(self.sp_trail_thick, 1, 1)

        # Chế độ xử lý ảnh
        pts_lay.addWidget(QLabel("🔍 Chế độ lọc nét:"), 2, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.setStyleSheet(self._combo_style())
        self.combo_mode.addItem("Đường nét viền (Contours/Edges)", "contours")
        self.combo_mode.addItem("Tô bóng gạch chéo (Hatching)", "hatching")
        pts_lay.addWidget(self.combo_mode, 2, 1)

        # Ngưỡng Canny Edge
        pts_lay.addWidget(QLabel("⚡ Ngưỡng lọc Canny:"), 3, 0)
        canny_lay = QHBoxLayout()
        self.sp_canny_low = QSpinBox()
        self.sp_canny_low.setRange(10, 255)
        self.sp_canny_low.setValue(50)
        self.sp_canny_low.setStyleSheet(self._spin_style())
        self.sp_canny_high = QSpinBox()
        self.sp_canny_high.setRange(20, 255)
        self.sp_canny_high.setValue(150)
        self.sp_canny_high.setStyleSheet(self._spin_style())
        canny_lay.addWidget(self.sp_canny_low)
        canny_lay.addWidget(self.sp_canny_high)
        pts_lay.addLayout(canny_lay, 3, 1)

        self.chk_invert = QCheckBox("Đảo màu trắng/đen (Invert)")
        self.chk_invert.setChecked(True)
        self.chk_invert.setStyleSheet("color: #cbd5e1;")
        pts_lay.addWidget(self.chk_invert, 4, 0, 1, 2)

        self.btn_process = QPushButton("⚡ Phân Rã Điểm & Tạo Quỹ Đạo")
        self.btn_process.setStyleSheet(self._btn_accent())
        self.btn_process.clicked.connect(self.process_image_to_trajectory)
        pts_lay.addWidget(self.btn_process, 5, 0, 1, 2)

        left_layout.addWidget(box_pts)

        # 3. Box Không gian làm việc trên bàn (Workspace Mapping)
        box_ws = QGroupBox("📐 3. TỌA ĐỘ VÀ KÍCH THƯỚC TRÊN BÀN ROBOT")
        box_ws.setStyleSheet(self._group_style())
        ws_lay = QGridLayout(box_ws)
        ws_lay.setSpacing(6)

        ws_lay.addWidget(QLabel("Tâm X (mm):"), 0, 0)
        self.sp_center_x = QDoubleSpinBox()
        self.sp_center_x.setRange(-520.0, -280.0)
        self.sp_center_x.setValue(-380.0)
        self.sp_center_x.setStyleSheet(self._spin_style())
        ws_lay.addWidget(self.sp_center_x, 0, 1)

        ws_lay.addWidget(QLabel("Tâm Y (mm):"), 0, 2)
        self.sp_center_y = QDoubleSpinBox()
        self.sp_center_y.setRange(-200.0, 200.0)
        self.sp_center_y.setValue(0.0)
        self.sp_center_y.setStyleSheet(self._spin_style())
        ws_lay.addWidget(self.sp_center_y, 0, 3)

        ws_lay.addWidget(QLabel("Cao độ vẽ Z_draw:"), 1, 0)
        self.sp_z_draw = QDoubleSpinBox()
        self.sp_z_draw.setRange(50.0, 300.0)
        self.sp_z_draw.setValue(180.0)
        self.sp_z_draw.setStyleSheet(self._spin_style())
        ws_lay.addWidget(self.sp_z_draw, 1, 1)

        ws_lay.addWidget(QLabel("Nhấc bút Z_lift:"), 1, 2)
        self.sp_z_lift = QDoubleSpinBox()
        self.sp_z_lift.setRange(80.0, 350.0)
        self.sp_z_lift.setValue(205.0)
        self.sp_z_lift.setStyleSheet(self._spin_style())
        ws_lay.addWidget(self.sp_z_lift, 1, 3)

        ws_lay.addWidget(QLabel("Chiều rộng (mm):"), 2, 0)
        self.sp_width = QDoubleSpinBox()
        self.sp_width.setRange(20.0, 280.0)
        self.sp_width.setValue(200.0)
        self.sp_width.setStyleSheet(self._spin_style())
        ws_lay.addWidget(self.sp_width, 2, 1)

        ws_lay.addWidget(QLabel("Chiều cao (mm):"), 2, 2)
        self.sp_height = QDoubleSpinBox()
        self.sp_height.setRange(20.0, 280.0)
        self.sp_height.setValue(200.0)
        self.sp_height.setStyleSheet(self._spin_style())
        ws_lay.addWidget(self.sp_height, 2, 3)

        left_layout.addWidget(box_ws)

        # 4. Box Thực thi & Vệt vẽ
        box_exec = QGroupBox("🎮 4. ĐIỀU KHIỂN ROBOT VẼ & VỆT DI CHUYỂN")
        box_exec.setStyleSheet(self._group_style())
        ex_lay = QVBoxLayout(box_exec)
        ex_lay.setSpacing(6)

        ctrl_btns = QHBoxLayout()
        self.btn_start = QPushButton("🚀 BẮT ĐẦU VẼ")
        self.btn_start.setStyleSheet(self._btn_green())
        self.btn_start.clicked.connect(self._start_execution)
        ctrl_btns.addWidget(self.btn_start)

        self.btn_pause = QPushButton("⏸️ Tạm Dừng")
        self.btn_pause.setStyleSheet(self._btn_secondary())
        self.btn_pause.clicked.connect(self._toggle_pause)
        ctrl_btns.addWidget(self.btn_pause)

        self.btn_stop = QPushButton("⏹️ Dừng Khẩn")
        self.btn_stop.setStyleSheet(self._btn_red())
        self.btn_stop.clicked.connect(self._cancel_execution)
        ctrl_btns.addWidget(self.btn_stop)
        ex_lay.addLayout(ctrl_btns)

        util_btns = QHBoxLayout()
        self.btn_clear_trail = QPushButton("🧹 Xóa Vệt Vẽ (Clear Trail)")
        self.btn_clear_trail.setStyleSheet(self._btn_secondary())
        self.btn_clear_trail.clicked.connect(self._clear_all_trails)
        util_btns.addWidget(self.btn_clear_trail)

        self.btn_home = QPushButton("🏠 Về Vị Trí Sẵn Sàng (Home)")
        self.btn_home.setStyleSheet(self._btn_secondary())
        self.btn_home.clicked.connect(self._go_home)
        util_btns.addWidget(self.btn_home)
        ex_lay.addLayout(util_btns)

        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                background-color: #0b1120; border: 1px solid #1e293b;
                border-radius: 4px; height: 16px; text-align: center; color: #f8fafc; font-size: 10px;
            }
            QProgressBar::chunk { background-color: #06b6d4; border-radius: 3px; }
        """)
        self.progress_bar.setValue(0)
        ex_lay.addWidget(self.progress_bar)

        self.lbl_exec_status = QLabel("Trạng thái: Sẵn sàng thực thi")
        self.lbl_exec_status.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold;")
        ex_lay.addWidget(self.lbl_exec_status)

        # -------------------------------------------------------------
        # KHUNG THEO DÕI TỌA ĐỘ & 5 GÓC KHỚP TỨC THỜI (5-LINK G-CODE)
        # -------------------------------------------------------------
        box_telemetry = QFrame()
        box_telemetry.setStyleSheet("""
            QFrame {
                background-color: #030712;
                border: 1px solid #1e293b;
                border-radius: 6px;
                padding: 6px;
            }
        """)
        t_lay = QVBoxLayout(box_telemetry)
        t_lay.setContentsMargins(6, 6, 6, 6)
        t_lay.setSpacing(4)

        # Hàng 1: Điểm tức thời & Trạng thái nét vẽ
        h_row = QHBoxLayout()
        self.lbl_live_pt = QLabel("📍 ĐIỂM: #0 / 0 (0.0%)")
        self.lbl_live_pt.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold;")
        h_row.addWidget(self.lbl_live_pt)

        self.lbl_live_state = QLabel("⚪ CHỜ LỆNH")
        self.lbl_live_state.setStyleSheet("""
            background-color: #1e293b; color: #94a3b8; font-size: 10px;
            font-weight: bold; border-radius: 3px; padding: 2px 6px;
        """)
        h_row.addWidget(self.lbl_live_state)
        t_lay.addLayout(h_row)

        # Hàng 2: Tọa độ Đề-các tức thời (X, Y, Z mm)
        self.lbl_live_xyz = QLabel("Tọa độ XYZ: X=0.00 mm | Y=0.00 mm | Z=0.00 mm")
        self.lbl_live_xyz.setStyleSheet("color: #e2e8f0; font-family: monospace; font-size: 11px;")
        t_lay.addWidget(self.lbl_live_xyz)

        # Hàng 3: 6 Góc Khớp Tức Thời (A, B, C, U, V, W theo độ)
        self.lbl_live_angles = QLabel("Góc 6-Link: A=0.000° B=0.000° C=0.000° U=0.000° V=0.000° W=0.000°")
        self.lbl_live_angles.setStyleSheet("color: #fbbf24; font-family: monospace; font-size: 11px; font-weight: bold;")
        t_lay.addWidget(self.lbl_live_angles)

        # Hàng 4: Dòng lệnh G-code tức thời (terminal style)
        self.lbl_live_gcode = QLabel("G1 A0 B0 C0 U0 V0 W0 F1200")
        self.lbl_live_gcode.setStyleSheet("""
            background-color: #050b14; color: #4ade80; font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px; font-weight: bold; border: 1px solid #14532d; border-radius: 4px;
            padding: 4px 8px;
        """)
        t_lay.addWidget(self.lbl_live_gcode)

        ex_lay.addWidget(box_telemetry)

        # Nút Xuất G-Code 6 Link trực tiếp
        export_row = QHBoxLayout()
        self.btn_export_5link = QPushButton("💾 XUẤT 6-LINK G-CODE (A, B, C, U, V, W)...")
        self.btn_export_5link.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #0369a1);
                color: #ffffff; font-weight: bold; font-size: 11px; padding: 7px 12px;
                border: 1px solid #38bdf8; border-radius: 5px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0369a1, stop:1 #075985);
                border: 1px solid #7dd3fc;
            }
        """)
        self.btn_export_5link.clicked.connect(self._export_5link_gcode)
        export_row.addWidget(self.btn_export_5link)

        self.btn_open_tab6 = QPushButton("📋 XEM BẢNG 6-LINK (TAB 6) ➔")
        self.btn_open_tab6.setStyleSheet("""
            QPushButton {
                background-color: #1e293b; color: #38bdf8; font-weight: bold; font-size: 11px;
                padding: 7px 12px; border: 1px solid #0284c7; border-radius: 5px;
            }
            QPushButton:hover { background-color: #0c4a6e; border-color: #38bdf8; color: #7dd3fc; }
        """)
        self.btn_open_tab6.clicked.connect(self._switch_to_tab6)
        export_row.addWidget(self.btn_open_tab6)

        ex_lay.addLayout(export_row)

        left_layout.addWidget(box_exec)

        left_layout.addStretch(1)
        splitter.addWidget(left_widget)

        # ---------------------------------------------------------------------
        # CỘT PHẢI: CANVAS 2D & XEM MÃ G-CODE & THỐNG KÊ
        # ---------------------------------------------------------------------
        right_tabs = QTabWidget()
        right_tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #1e293b; background-color: #050b14; border-radius: 6px; }
            QTabBar::tab { background: #0f172a; color: #94a3b8; padding: 8px 14px; font-weight: bold; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px; }
            QTabBar::tab:selected { background: #1e293b; color: #38bdf8; border-bottom: 2px solid #38bdf8; }
        """)

        # Sub-tab 1: Interactive Canvas
        self.canvas = InteractiveDrawingCanvas(self)
        right_tabs.addTab(self.canvas, "🖼️ Bản Đồ Nét Vẽ & Vệt Di Chuyển 2D")

        # Sub-tab 2: G-Code Text Editor
        gcode_panel = QWidget()
        gp_lay = QVBoxLayout(gcode_panel)
        gp_lay.setContentsMargins(8, 8, 8, 8)
        gp_lay.setSpacing(6)

        gcode_top = QHBoxLayout()
        self.btn_export_gcode = QPushButton("💾 Lưu File G-Code (.nc)...")
        self.btn_export_gcode.setStyleSheet(self._btn_secondary())
        self.btn_export_gcode.clicked.connect(self._export_gcode_file)
        gcode_top.addWidget(self.btn_export_gcode)

        self.btn_import_gcode = QPushButton("📂 Mở File G-Code...")
        self.btn_import_gcode.setStyleSheet(self._btn_secondary())
        self.btn_import_gcode.clicked.connect(self._import_gcode_file)
        gcode_top.addWidget(self.btn_import_gcode)

        self.btn_reload_gcode = QPushButton("🔄 Cập Nhật Từ Trình Soạn Thảo")
        self.btn_reload_gcode.setStyleSheet(self._btn_accent())
        self.btn_reload_gcode.clicked.connect(self._reload_from_editor)
        gcode_top.addWidget(self.btn_reload_gcode)

        self.btn_export_5link_tab = QPushButton("💾 Xuất 5-Link G-Code (A, B, C, U, V)...")
        self.btn_export_5link_tab.setStyleSheet(self._btn_green())
        self.btn_export_5link_tab.clicked.connect(self._export_5link_gcode)
        gcode_top.addWidget(self.btn_export_5link_tab)
        gp_lay.addLayout(gcode_top)

        self.txt_gcode = QTextEdit()
        self.txt_gcode.setStyleSheet("""
            QTextEdit {
                background-color: #050b14; color: #4ade80; font-family: monospace;
                font-size: 11px; border: 1px solid #1e293b; border-radius: 4px;
            }
        """)
        gp_lay.addWidget(self.txt_gcode)
        right_tabs.addTab(gcode_panel, "📜 Trình Xem / Sửa Mã G-Code")

        # Sub-tab 3: Báo cáo Thống kê & Kiểm tra Động học
        report_panel = QWidget()
        rp_lay = QVBoxLayout(report_panel)
        rp_lay.setContentsMargins(12, 12, 12, 12)
        rp_lay.setSpacing(8)

        self.lbl_report_title = QLabel("📊 BÁO CÁO PHÂN TÍCH QUỸ ĐẠO & AN TOÀN ĐỘNG HỌC")
        self.lbl_report_title.setStyleSheet("color: #38bdf8; font-size: 13px; font-weight: bold;")
        rp_lay.addWidget(self.lbl_report_title)

        self.txt_report = QTextEdit()
        self.txt_report.setReadOnly(True)
        self.txt_report.setStyleSheet("""
            QTextEdit {
                background-color: #0b1120; color: #cbd5e1; font-family: monospace;
                font-size: 12px; border: 1px solid #1e293b; border-radius: 6px; padding: 8px;
            }
        """)
        rp_lay.addWidget(self.txt_report)
        right_tabs.addTab(report_panel, "📊 Báo Cáo An Toàn & Động Học")

        # Sub-tab 4: Thẻ dẫn hướng sang Tab 6 Riêng Biệt
        tab6_card = QWidget()
        t6_lay = QVBoxLayout(tab6_card)
        t6_lay.setContentsMargins(24, 30, 24, 30)
        t6_lay.setSpacing(14)
        t6_lay.setAlignment(Qt.AlignCenter)

        t6_title = QLabel("🤖 BẢNG TRA GÓC 5-LINK ĐÃ ĐƯỢC CHUYỂN THÀNH TAB 6 RIÊNG BIỆT")
        t6_title.setStyleSheet("color: #38bdf8; font-size: 15px; font-weight: bold;")
        t6_title.setAlignment(Qt.AlignCenter)
        t6_lay.addWidget(t6_title)

        t6_desc = QLabel(
            "Để hiển thị đầy đủ và rõ ràng toàn bộ số liệu 5 Link (Link 1..Link 5 / Trục A, B, C, U, V),\n"
            "tọa độ không gian X, Y, Z, trạng thái IK và sai số mà không bị che khuất,\n"
            "Bảng Tra Góc 5-Link hiện là Tab 6 độc lập với giao diện toàn màn hình."
        )
        t6_desc.setStyleSheet("color: #94a3b8; font-size: 12px; line-height: 1.6;")
        t6_desc.setAlignment(Qt.AlignCenter)
        t6_lay.addWidget(t6_desc)

        self.btn_switch_to_tab6 = QPushButton("🚀 BẤM VÀO ĐÂY ĐỂ MỞ TAB 6 (BẢNG TRA GÓC 5-LINK TOÀN MÀN HÌNH) ➔")
        self.btn_switch_to_tab6.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #0369a1);
                color: #ffffff; font-size: 13px; font-weight: bold; padding: 12px 24px;
                border: 2px solid #38bdf8; border-radius: 8px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0369a1, stop:1 #075985);
                border-color: #7dd3fc;
            }
        """)
        self.btn_switch_to_tab6.clicked.connect(self._switch_to_tab6)
        t6_lay.addWidget(self.btn_switch_to_tab6)

        right_tabs.addTab(tab6_card, "🤖 Bảng Tra Góc 5-Link (Tab 6 Toàn Màn Hình)")

        splitter.addWidget(right_tabs)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 6)

        main_layout.addWidget(splitter)

    def _group_style(self):
        return """
            QGroupBox {
                background-color: #0b1120; border: 1px solid #1e293b;
                border-radius: 8px; margin-top: 10px; padding-top: 14px;
                color: #38bdf8; font-size: 11px; font-weight: bold;
            }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 10px; padding: 0 4px; }
            QLabel { color: #cbd5e1; font-size: 11px; }
        """

    def _spin_style(self):
        return """
            QDoubleSpinBox, QSpinBox {
                background-color: #050b14; color: #38bdf8; border: 1px solid #334155;
                border-radius: 4px; padding: 3px 6px; font-weight: bold; font-size: 11px;
            }
            QDoubleSpinBox:focus, QSpinBox:focus { border: 1px solid #0284c7; }
        """

    def _combo_style(self):
        return """
            QComboBox {
                background-color: #050b14; color: #f8fafc; border: 1px solid #334155;
                border-radius: 4px; padding: 4px 8px; font-size: 11px;
            }
            QComboBox QAbstractItemView {
                background-color: #0f172a; color: #f8fafc; selection-background-color: #0284c7;
            }
        """

    def _btn_accent(self):
        return """
            QPushButton {
                background-color: #0284c7; color: white; font-weight: bold;
                border-radius: 5px; padding: 6px 12px; font-size: 11px;
            }
            QPushButton:hover { background-color: #0369a1; }
            QPushButton:pressed { background-color: #075985; }
        """

    def _btn_secondary(self):
        return """
            QPushButton {
                background-color: #1e293b; color: #e2e8f0; font-weight: bold;
                border-radius: 5px; padding: 6px 10px; font-size: 11px;
            }
            QPushButton:hover { background-color: #334155; }
        """

    def _btn_green(self):
        return """
            QPushButton {
                background-color: #059669; color: white; font-weight: bold;
                border-radius: 5px; padding: 8px 14px; font-size: 12px;
            }
            QPushButton:hover { background-color: #047857; }
        """

    def _btn_red(self):
        return """
            QPushButton {
                background-color: #dc2626; color: white; font-weight: bold;
                border-radius: 5px; padding: 8px 12px; font-size: 11px;
            }
            QPushButton:hover { background-color: #b91c1c; }
        """

    def _load_initial_sample(self):
        sample_path = os.path.join(self.sample_dir, "star_5point.png")
        if os.path.exists(sample_path):
            self.processor.load_image(sample_path)
            self.lbl_img_path.setText(f"Tệp: star_5point.png")
            self.process_image_to_trajectory()

    def _browse_image(self):
        fn, _ = QFileDialog.getOpenFileName(
            self, "Chọn hình ảnh", self.sample_dir,
            "Images (*.png *.jpg *.jpeg *.bmp *.svg)"
        )
        if fn:
            if self.processor.load_image(fn):
                self.lbl_img_path.setText(f"Tệp: {os.path.basename(fn)}")
                self.process_image_to_trajectory()
            else:
                QMessageBox.warning(self, "Lỗi", "Không thể đọc định dạng hình ảnh này!")

    def _on_sample_changed(self, idx):
        sample_name = self.combo_samples.currentData()
        if sample_name:
            sample_path = os.path.join(self.sample_dir, sample_name)
            if os.path.exists(sample_path):
                self.processor.load_image(sample_path)
                self.lbl_img_path.setText(f"Mẫu: {sample_name}")
                self.process_image_to_trajectory()

    def _on_trail_thick_changed(self, val):
        if self.trail_pub:
            self.trail_pub.set_thickness(val)

    def process_image_to_trajectory(self):
        spacing = self.sp_point_spacing.value()
        thick = self.sp_trail_thick.value()
        mode = self.combo_mode.currentData()
        c_low = self.sp_canny_low.value()
        c_high = self.sp_canny_high.value()
        invert = self.chk_invert.isChecked()

        cx = self.sp_center_x.value()
        cy = self.sp_center_y.value()
        zd = self.sp_z_draw.value()
        zl = self.sp_z_lift.value()
        tw = self.sp_width.value()
        th = self.sp_height.value()

        if self.trail_pub:
            self.trail_pub.set_thickness(thick)

        strokes = self.processor.process(
            point_spacing_mm=spacing,
            threshold_mode="canny",
            canny_low=c_low,
            canny_high=c_high,
            invert=invert,
            mode=mode,
            center_x=cx,
            center_y=cy,
            z_draw=zd,
            z_lift=zl,
            target_width_mm=tw,
            target_height_mm=th,
            keep_aspect=True
        )

        self.current_strokes = strokes
        self.canvas.set_data(strokes, cx, cy, tw, th)
        self.txt_gcode.setPlainText(self.processor.gcode_text)

        val_res = self.validator.validate(strokes)
        if hasattr(self, "joint_table_widget") and self.joint_table_widget is not None:
            self.joint_table_widget.set_strokes(strokes, item_name=self.lbl_img_path.text())
        self._init_monitor_from_strokes(strokes)
        sm = self.processor.summary

        rep = [
            "=" * 60,
            "           BÁO CÁO TỔNG QUAN XỬ LÝ ẢNH & ĐỘNG HỌC ROBOT",
            "=" * 60,
            f"• Số nét vẽ (Strokes):            {sm.get('total_strokes', 0)} nét",
            f"• Tổng số điểm nội suy:           {sm.get('total_points', 0)} điểm",
            f"• Khoảng cách giữa các điểm:      {spacing:.2f} mm (Người dùng thiết lập)",
            f"• Độ dày nét vệt vẽ RViz:         {thick:.2f} mm",
            f"• Tổng chiều dài đường vẽ:        {sm.get('draw_length_mm', 0):.1f} mm",
            f"• Chiều dài bay không tải:        {sm.get('rapid_length_mm', 0):.1f} mm",
            f"• Thời gian dự kiến hoàn thành:   ~{sm.get('estimated_time_sec', 0):.1f} giây",
            "-" * 60,
            "KẾT QUẢ KIỂM TRA ĐỘNG HỌC NGHỊCH (IK) & AN TOÀN VA CHẠM:",
            f"• Điểm hợp lệ trong tầm với:      {val_res['valid_points']}/{val_res['total_points']} ({val_res['feasibility_pct']}%)",
            f"• Điểm ngoài tầm / Vượt giới hạn: {val_res['invalid_points']} điểm",
            f"• Điểm có nguy cơ va chạm bàn:    {val_res['collision_points']} điểm",
            "-" * 60,
        ]

        if val_res["is_safe"]:
            rep.append("✅ KẾT LUẬN: 100% QUỸ ĐẠO KHẢ THI VÀ AN TOÀN! SẴN SÀNG THỰC THI.")
            self.lbl_exec_status.setText(f"✅ Quỹ đạo sẵn sàng: {sm.get('total_points', 0)} điểm | 100% Khả thi")
            self.lbl_exec_status.setStyleSheet("color: #4ade80; font-weight: bold;")
        else:
            rep.append("⚠️ CẢNH BÁO: Phát hiện điểm ngoài tầm với hoặc va chạm! Hãy thu nhỏ kích thước hoặc dịch tâm.")
            self.lbl_exec_status.setText(f"⚠️ Chú ý: {val_res['invalid_points']} điểm ngoài tầm với an toàn!")
            self.lbl_exec_status.setStyleSheet("color: #f87171; font-weight: bold;")

        self.txt_report.setPlainText("\n".join(rep))

    def _start_execution(self):
        if not self.current_strokes:
            QMessageBox.warning(self, "Chú ý", "Vui lòng phân rã điểm ảnh trước khi bắt đầu!")
            return

        if self.worker and self.worker.isRunning():
            QMessageBox.information(self, "Thông báo", "Robot đang trong quá trình thực thi!")
            return

        self.worker = TrajectoryExecutionWorker(self.bridge, self.engine, self.trail_pub)
        self.worker.load_strokes(self.current_strokes, speed_scale=1.0)
        self.worker.sig_progress.connect(self._on_worker_progress)
        self.worker.sig_status.connect(self._on_worker_status)
        self.worker.sig_finished.connect(self._on_worker_finished)

        self.progress_bar.setValue(0)
        self.btn_start.setEnabled(False)
        self.btn_pause.setText("⏸️ Tạm Dừng")
        self.worker.start()

    def _toggle_pause(self):
        if not self.worker or not self.worker.isRunning():
            return
        if self.worker.is_paused:
            self.worker.resume()
            self.btn_pause.setText("⏸️ Tạm Dừng")
        else:
            self.worker.pause()
            self.btn_pause.setText("▶️ Tiếp Tục")

    def _cancel_execution(self):
        if self.worker and self.worker.isRunning():
            self.worker.cancel()
            self.btn_start.setEnabled(True)
            self.btn_pause.setText("⏸️ Tạm Dừng")

    def _on_worker_progress(self, curr, total, pct, x, y, z, is_draw, q_target=None):
        self.progress_bar.setValue(int(pct))
        st = "ĐANG VẼ (Hạ bút)" if is_draw else "NHẤC BÚT (Di chuyển)"
        self.lbl_exec_status.setText(f"Điểm {curr}/{total} ({pct:.1f}%) | [{x:.1f}, {y:.1f}, {z:.1f}] - {st}")
        self.canvas.update_cursor_progress(x, y, z, is_draw)

        # Cập nhật khung hiển thị tọa độ & 5 góc tức thời
        self._update_live_monitor(curr, total, pct, x, y, z, is_draw, q_target)

        # Tự động highlight và cuộn theo thời gian thực trên Bảng Tra Góc 5-Link
        if hasattr(self, "joint_table_widget") and self.joint_table_widget is not None:
            self.joint_table_widget.highlight_point(curr)

    def _on_worker_status(self, msg, level):
        col = "#38bdf8"
        if level == "ok": col = "#4ade80"
        elif level == "warn": col = "#fbbf24"
        elif level == "error": col = "#f87171"
        self.lbl_exec_status.setStyleSheet(f"color: {col}; font-weight: bold;")
        self.lbl_exec_status.setText(msg)

    def _on_worker_finished(self):
        self.btn_start.setEnabled(True)
        self.btn_pause.setText("⏸️ Tạm Dừng")

    def _clear_all_trails(self):
        if self.trail_pub:
            self.trail_pub.clear()
        self.canvas.clear_trail()
        self.progress_bar.setValue(0)
        self.lbl_exec_status.setText("Đã xóa sạch vệt vẽ trên RViz2 và Canvas.")

    def _go_home(self):
        if self.bridge:
            # Vị trí sẵn sàng trên tâm bàn vẽ Z = 205mm (X = -380mm, Y = 0mm), đầu bút hướng thẳng đứng vuông góc xuống mặt giấy
            q_ready = [0.0, 0.5812, 2.3233, 0.0, 0.237, 0.0]
            self.bridge.send_trajectory(q_ready, duration_sec=2.5, zero_velocity=True)
            self.lbl_exec_status.setText("Robot đang di chuyển về vị trí sẵn sàng trên bàn vẽ (X=-380, Y=0, Z=205mm)...")

    def _export_gcode_file(self):
        fn, _ = QFileDialog.getSaveFileName(
            self, "Lưu File G-Code", "/home/dung/denso_ws/drawing_samples/trajectory.gcode",
            "G-Code Files (*.gcode *.nc *.txt)"
        )
        if fn:
            with open(fn, "w", encoding="utf-8") as f:
                f.write(self.txt_gcode.toPlainText())
            QMessageBox.information(self, "Thành công", f"Đã xuất tệp G-code tới:\n{fn}")

    def _import_gcode_file(self):
        fn, _ = QFileDialog.getOpenFileName(
            self, "Mở File G-Code", self.sample_dir,
            "G-Code Files (*.gcode *.nc *.txt)"
        )
        if fn:
            with open(fn, "r", encoding="utf-8") as f:
                content = f.read()
            self.txt_gcode.setPlainText(content)
            self._reload_from_editor()

    def _reload_from_editor(self):
        text = self.txt_gcode.toPlainText()
        spacing = self.sp_point_spacing.value()
        zd = self.sp_z_draw.value()
        strokes = self.processor.parse_gcode(text, point_spacing_mm=spacing, z_draw_thresh=zd + 2.0)
        self.current_strokes = strokes

        cx = self.sp_center_x.value()
        cy = self.sp_center_y.value()
        tw = self.sp_width.value()
        th = self.sp_height.value()

        self.canvas.set_data(strokes, cx, cy, tw, th)
        val_res = self.validator.validate(strokes)
        if hasattr(self, "joint_table_widget") and self.joint_table_widget is not None:
            self.joint_table_widget.set_strokes(strokes, item_name="G-Code Editor")
        self._init_monitor_from_strokes(strokes)
        self.lbl_exec_status.setText(f"Đã nạp {len(strokes)} nét từ G-code | {val_res['valid_points']} điểm hợp lệ.")
        QMessageBox.information(self, "Thông báo", f"Đã phân tích thành công {len(strokes)} nét vẽ từ mã G-code!")

    def _init_monitor_from_strokes(self, strokes: List[Stroke]):
        """Khởi tạo giá trị trên khung giám sát tức thời từ điểm đầu tiên của quỹ đạo."""
        if not strokes:
            self._update_live_monitor(0, 0, 0.0, 0.0, 0.0, 0.0, False, None)
            return
        total = sum(len(s.points) for s in strokes)
        first_pt = None
        for s in strokes:
            if s.points:
                first_pt = s.points[0]
                break
        if first_pt:
            self._update_live_monitor(1, total, (1.0 / total * 100.0) if total > 0 else 0.0,
                                      first_pt.x, first_pt.y, first_pt.z, first_pt.is_draw, first_pt.q_sol)

    def _update_live_monitor(self, curr: int, total: int, pct: float, x: float, y: float, z: float, is_draw: bool, q_target=None):
        """Cập nhật giao diện hiển thị tọa độ & góc khớp 5 Link tức thời."""
        if not hasattr(self, 'lbl_live_pt'):
            return

        self.lbl_live_pt.setText(f"📍 ĐIỂM: #{curr} / {total} ({pct:.1f}%)" if total > 0 else "📍 ĐIỂM: #0 / 0 (0.0%)")
        if total > 0:
            if is_draw:
                self.lbl_live_state.setText("✏️ ĐANG VẼ (HẠ BÚT)")
                self.lbl_live_state.setStyleSheet("""
                    background-color: #14532d; color: #4ade80; font-size: 10px;
                    font-weight: bold; border-radius: 3px; padding: 2px 6px;
                """)
            else:
                self.lbl_live_state.setText("✈️ NHẤC BÚT (BAY)")
                self.lbl_live_state.setStyleSheet("""
                    background-color: #854d0e; color: #fde047; font-size: 10px;
                    font-weight: bold; border-radius: 3px; padding: 2px 6px;
                """)
        else:
            self.lbl_live_state.setText("⚪ CHỜ LỆNH")
            self.lbl_live_state.setStyleSheet("""
                background-color: #1e293b; color: #94a3b8; font-size: 10px;
                font-weight: bold; border-radius: 3px; padding: 2px 6px;
            """)

        self.lbl_live_xyz.setText(f"Tọa độ XYZ: X={x:+.2f} mm | Y={y:+.2f} mm | Z={z:+.2f} mm")

        # Định dạng 6 góc khớp A, B, C, U, V, W theo độ
        if q_target is not None and len(q_target) >= 6:
            deg = [math.degrees(float(q)) for q in q_target[:6]]
            def fmt(v):
                rv = round(v, 3)
                if abs(rv) < 1e-6:
                    rv = 0.0
                return str(int(rv)) if rv == int(rv) else str(rv)

            A_s, B_s, C_s, U_s, V_s, W_s = [fmt(d) for d in deg]
            self.lbl_live_angles.setText(f"Góc 6-Link: A={A_s}° B={B_s}° C={C_s}° U={U_s}° V={V_s}° W={W_s}°")
            feed_s = " F1200" if curr <= 1 else ""
            self.lbl_live_gcode.setText(f"G1 A{A_s} B{B_s} C{C_s} U{U_s} V{V_s} W{W_s}{feed_s}")
        elif q_target is not None and len(q_target) >= 5:
            deg = [math.degrees(float(q)) for q in q_target[:5]]
            def fmt(v):
                rv = round(v, 3)
                if abs(rv) < 1e-6:
                    rv = 0.0
                return str(int(rv)) if rv == int(rv) else str(rv)

            A_s, B_s, C_s, U_s, V_s = [fmt(d) for d in deg]
            self.lbl_live_angles.setText(f"Góc 6-Link: A={A_s}° B={B_s}° C={C_s}° U={U_s}° V={V_s}° W=0°")
            feed_s = " F1200" if curr <= 1 else ""
            self.lbl_live_gcode.setText(f"G1 A{A_s} B{B_s} C{C_s} U{U_s} V{V_s} W0{feed_s}")
        else:
            self.lbl_live_angles.setText("Góc 6-Link: A=--- B=--- C=--- U=--- V=--- W=---")
            self.lbl_live_gcode.setText("G1 A--- B--- C--- U--- V--- W---")

    def _on_table_point_selected(self, index: int, x: float, y: float, z: float):
        """Khi người dùng nhấp chọn 1 hàng trong Bảng Tra Góc 6-Link, đồng bộ con trỏ và màn hình tức thời."""
        if hasattr(self, 'canvas') and self.canvas:
            self.canvas.update_cursor_progress(x, y, z, is_draw=True)
        if hasattr(self, 'joint_table_widget') and self.joint_table_widget:
            records = getattr(self.joint_table_widget, 'records', [])
            if 0 <= index - 1 < len(records):
                r = records[index - 1]
                total = len(records)
                pct = (r.index / total * 100.0) if total > 0 else 0.0
                self._update_live_monitor(r.index, total, pct, r.x, r.y, r.z, r.is_draw, r.q_rad)

    def _export_5link_gcode(self):
        """Xuất G-code 6 Link (A=J1, B=J2, C=J3, U=J4, V=J5, W=J6) theo định dạng chuẩn ra thư mục OneDrive Denso."""
        if not self.current_strokes:
            QMessageBox.warning(self, "Xuất G-Code 6 Link", "Chưa có dữ liệu quỹ đạo! Vui lòng nạp ảnh hoặc G-code trước.")
            return

        records = extract_records_from_strokes(self.current_strokes, self.engine)
        if not records:
            QMessageBox.warning(self, "Xuất G-Code 6 Link", "Không thể trích xuất các điểm quỹ đạo!")
            return

        # Thư mục đích: OneDrive > Máy tính > Denso (như trong ảnh của người dùng)
        onedrive_dir = "/mnt/c/Users/thanh/OneDrive/Máy tính/Denso"
        if os.path.isdir(onedrive_dir):
            gcode_dir = onedrive_dir
        else:
            gcode_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gcode"))
            os.makedirs(gcode_dir, exist_ok=True)

        ts = time.strftime("%Y%m%d_%H%M%S")
        default_filename = f"gcode_6link_{ts}.gcode"
        default_path = os.path.join(gcode_dir, default_filename)

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Xuất File G-Code 6 Link Robot Denso VS-6556",
            default_path,
            "G-Code Files (*.gcode *.nc);;All Files (*)"
        )
        if not file_path:
            return

        if export_records_to_gcode(records, file_path):
            valid_cnt = sum(1 for r in records if r.q_deg)
            skipped = len(records) - valid_cnt
            info = (
                f"✅ Xuất thành công tệp G-code 6 Link chuẩn Denso!\n\n"
                f"📁 Đường dẫn tệp:\n{file_path}\n\n"
                f"📊 Thống kê quỹ đạo:\n"
                f"   • Tổng số điểm: {len(records)} điểm\n"
                f"   • Số điểm IK khả thi: {valid_cnt}\n"
                f"   • Số điểm bỏ qua (IK lỗi): {skipped}\n"
                f"   • Cấu trúc 6 trục: A=Link1(J1), B=Link2(J2), C=Link3(J3), U=Link4(J4), V=Link5(J5), W=Link6(J6)\n"
                f"   • Trục W (Khớp xoay mặt bích Link 6): Đã tích hợp đầy đủ\n"
                f"   • Tốc độ chạy dao: F1200 ở dòng đầu tiên"
            )
            QMessageBox.information(self, "Xuất G-Code 6 Link Thành Công", info)
        else:
            QMessageBox.critical(self, "Lỗi", f"Không thể ghi tệp G-code ra:\n{file_path}")

    def _switch_to_tab6(self):
        """Chuyển ngay sang Tab 6 (Bảng Tra Góc 6-Link)."""
        p = self.parent()
        while p is not None:
            if hasattr(p, "tabs") and isinstance(p.tabs, QTabWidget):
                p.tabs.setCurrentIndex(5)
                return
            p = p.parent()
