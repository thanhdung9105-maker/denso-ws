#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 G-CODE 5-LINK ANGLES ENGINE & VIEWER
Trình phân tích, giải động học nghịch (IK) và xuất bảng tra góc của 5 Link
tương ứng với từng điểm G-code cho cánh tay robot Denso VS-6556 trong Gazebo & MoveIt 2.
"""

import os
import sys
import math
import time
import csv
import argparse
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple, Any

import numpy as np

# PyQt5 imports
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QTabWidget, QHeaderView, QMessageBox,
    QFileDialog, QComboBox, QLineEdit, QGroupBox, QFrame,
    QSplitter, QAbstractItemView, QApplication, QMainWindow,
    QDialog, QGridLayout, QMenu
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont, QIcon, QBrush

# Denso Dynamics Engine & Constants
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "denso_vs6556_moveit_config", "scripts"))
try:
    from denso_dynamics_engine import DensoDynamicsEngine, JOINT_NAMES, JOINT_LIMITS
except ImportError:
    try:
        sys.path.append("/home/dung/denso_ws/src/denso_vs6556_moveit_config/scripts")
        from denso_dynamics_engine import DensoDynamicsEngine, JOINT_NAMES, JOINT_LIMITS
    except ImportError:
        DensoDynamicsEngine = None
        JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        JOINT_LIMITS = [
            (-2.967, 2.967),
            (-2.094, 2.356),
            (-2.077, 2.897),
            (-3.316, 3.316),
            (-2.094, 2.094),
            (-6.283, 6.283)
        ]

LINK_NAMES = [
    "Link 1 (J1 Đế / Base)",
    "Link 2 (J2 Vai / Shoulder)",
    "Link 3 (J3 Khuỷu / Elbow)",
    "Link 4 (J4 Cẳng tay / Forearm)",
    "Link 5 (J5 Cổ tay / Wrist)",
    "Link 6 (J6 Mặt bích / Flange Roll)"
]


@dataclass
class GCodePointRecord:
    index: int             # 1-based point index
    stroke_idx: int        # Stroke index
    cmd: str               # G00, G01, G02, G03
    is_draw: bool          # True = Vẽ (Down), False = Bay (Up)
    x: float               # mm
    y: float               # mm
    z: float               # mm
    feedrate: float        # mm/min
    q_rad: Optional[List[float]] = None  # [q1, q2, q3, q4, q5] in rad
    q_deg: Optional[List[float]] = None  # [q1, q2, q3, q4, q5] in deg
    is_valid: bool = True
    status: str = "OK"     # "OK", "Va chạm", "Ngoài tầm với"
    err_mm: float = 0.0


# =============================================================================
# EXPORT FUNCTIONS (CSV, TXT, HTML, CLIPBOARD)
# =============================================================================

def export_records_to_csv(records: List[GCodePointRecord], file_path: str) -> bool:
    """Xuất danh sách điểm và góc 6 bậc ra tệp CSV (UTF-8 BOM tương thích Excel)."""
    try:
        with open(file_path, mode="w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow([
                "STT",
                "Lenh_GCode",
                "Che_Do",
                "X_mm",
                "Y_mm",
                "Z_mm",
                "Link1_J1_deg",
                "Link2_J2_deg",
                "Link3_J3_deg",
                "Link4_J4_deg",
                "Link5_J5_deg",
                "Link6_J6_deg",
                "Link1_J1_rad",
                "Link2_J2_rad",
                "Link3_J3_rad",
                "Link4_J4_rad",
                "Link5_J5_rad",
                "Link6_J6_rad",
                "Trang_Thai_IK",
                "Sai_So_DeltaE_mm",
                "Toc_Do_F_mm_min"
            ])
            for r in records:
                che_do = "Vẽ (Pen Down)" if r.is_draw else "Bay (Rapid Lift)"
                j_deg = [f"{d:.3f}" for d in r.q_deg] if r.q_deg else ["N/A"] * 6
                j_rad = [f"{rd:.4f}" for rd in r.q_rad] if r.q_rad else ["N/A"] * 6
                # Pad to 6 if needed
                while len(j_deg) < 6:
                    j_deg.append("0.000")
                while len(j_rad) < 6:
                    j_rad.append("0.0000")
                writer.writerow([
                    r.index,
                    r.cmd,
                    che_do,
                    f"{r.x:.3f}",
                    f"{r.y:.3f}",
                    f"{r.z:.3f}",
                    j_deg[0], j_deg[1], j_deg[2], j_deg[3], j_deg[4], j_deg[5],
                    j_rad[0], j_rad[1], j_rad[2], j_rad[3], j_rad[4], j_rad[5],
                    r.status,
                    f"{r.err_mm:.4f}",
                    f"{r.feedrate:.1f}"
                ])
        return True
    except Exception as e:
        print(f"[ERROR] Xuất CSV thất bại: {e}")
        return False


def export_records_to_txt(records: List[GCodePointRecord], file_path: str, title: str = "BẢNG TRA GÓC 6 BẬC (6-DOF) THEO G-CODE - DENSO VS-6556") -> bool:
    """Xuất báo cáo dạng bảng text monospaced chuyên nghiệp."""
    try:
        valid_cnt = sum(1 for r in records if r.is_valid)
        total_cnt = len(records)
        pct = (valid_cnt / total_cnt * 100.0) if total_cnt > 0 else 0.0

        lines = [
            "=" * 156,
            f" {title.center(154)}",
            f" Thời gian xuất: {time.strftime('%Y-%m-%d %H:%M:%S')} | Tổng số điểm: {total_cnt} | Tỷ lệ khả thi IK: {pct:.1f}%",
            "=" * 156,
            f"{'#':>5} | {'LỆNH':<5} | {'CHẾ ĐỘ':<8} | {'X (mm)':>9} | {'Y (mm)':>9} | {'Z (mm)':>9} | {'Link 1 (°)':>10} | {'Link 2 (°)':>10} | {'Link 3 (°)':>10} | {'Link 4 (°)':>10} | {'Link 5 (°)':>10} | {'Link 6 (°)':>10} | {'TRẠNG THÁI':<12} | {'ΔE (mm)':>8} | {'F (mm/p)':>8}",
            "-" * 156
        ]

        for r in records:
            che_do = "VẼ" if r.is_draw else "BAY"
            if r.q_deg:
                j_str = [f"{d:>10.2f}" for d in r.q_deg]
            else:
                j_str = [f"{'N/A':>10}"] * 6
            while len(j_str) < 6:
                j_str.append(f"{0.0:>10.2f}")

            line = f"{r.index:>5} | {r.cmd:<5} | {che_do:<8} | {r.x:>9.2f} | {r.y:>9.2f} | {r.z:>9.2f} | {j_str[0]} | {j_str[1]} | {j_str[2]} | {j_str[3]} | {j_str[4]} | {j_str[5]} | {r.status:<12} | {r.err_mm:>8.3f} | {r.feedrate:>8.1f}"
            lines.append(line)

        lines.extend([
            "-" * 156,
            "GIỚI HẠN GÓC VẬT LÝ DENSO VS-6556 (6-DOF):",
            "  • Link 1 (J1 Đế / Base):      [-170.0°, +170.0°] ([-2.967, +2.967] rad)",
            "  • Link 2 (J2 Vai / Shoulder): [-120.0°, +135.0°] ([-2.094, +2.356] rad)",
            "  • Link 3 (J3 Khuỷu / Elbow):  [-119.0°, +166.0°] ([-2.077, +2.897] rad)",
            "  • Link 4 (J4 Cẳng tay):       [-190.0°, +190.0°] ([-3.316, +3.316] rad)",
            "  • Link 5 (J5 Cổ tay / Wrist): [-120.0°, +120.0°] ([-2.094, +2.094] rad)",
            "  • Link 6 (J6 Mặt bích):       [-360.0°, +360.0°] ([-6.283, +6.283] rad)",
            "=" * 156
        ])

        with open(file_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        return True
    except Exception as e:
        print(f"[ERROR] Xuất TXT thất bại: {e}")
        return False


def export_records_to_html(records: List[GCodePointRecord], file_path: str, title: str = "BẢNG TRA CÁC GÓC 5 LINK THEO G-CODE - ROBOT DENSO VS-6556") -> bool:
    """Xuất báo cáo HTML hiện đại, chuẩn in ấn và báo cáo đồ án."""
    try:
        valid_cnt = sum(1 for r in records if r.is_valid)
        total_cnt = len(records)
        pct = (valid_cnt / total_cnt * 100.0) if total_cnt > 0 else 0.0
        draw_cnt = sum(1 for r in records if r.is_draw)
        rapid_cnt = total_cnt - draw_cnt

        html = f"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<title>{title}</title>
<style>
    body {{
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
        background-color: #0b1120;
        color: #e2e8f0;
        margin: 24px;
        padding: 0;
    }}
    .header {{
        background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
        border: 1px solid #334155;
        border-radius: 12px;
        padding: 24px;
        margin-bottom: 24px;
        box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3);
    }}
    h1 {{
        color: #38bdf8;
        margin: 0 0 10px 0;
        font-size: 22px;
    }}
    .meta-cards {{
        display: flex;
        gap: 16px;
        margin-top: 16px;
        flex-wrap: wrap;
    }}
    .card {{
        background: #0f172a;
        border: 1px solid #1e293b;
        border-radius: 8px;
        padding: 12px 18px;
        min-width: 140px;
    }}
    .card-title {{
        font-size: 11px;
        color: #94a3b8;
        text-transform: uppercase;
        font-weight: 600;
    }}
    .card-val {{
        font-size: 18px;
        font-weight: 700;
        color: #38bdf8;
        margin-top: 4px;
        font-family: monospace;
    }}
    table {{
        width: 100%;
        border-collapse: collapse;
        background: #0f172a;
        border-radius: 8px;
        overflow: hidden;
        border: 1px solid #1e293b;
        font-size: 12px;
    }}
    th {{
        background: #1e293b;
        color: #94a3b8;
        font-weight: 700;
        padding: 10px 12px;
        text-align: right;
        border-bottom: 1px solid #334155;
    }}
    th.left, td.left {{ text-align: left; }}
    th.center, td.center {{ text-align: center; }}
    td {{
        padding: 8px 12px;
        border-bottom: 1px solid #1e293b;
        font-family: monospace;
        text-align: right;
    }}
    tr:nth-child(even) {{ background: #0b1120; }}
    tr:hover {{ background: #1e293b; }}
    .tag-draw {{ color: #4ade80; font-weight: bold; }}
    .tag-rapid {{ color: #fbbf24; font-weight: bold; }}
    .tag-ok {{ color: #4ade80; }}
    .tag-err {{ color: #f87171; font-weight: bold; }}
    .footer {{
        margin-top: 24px;
        padding: 16px;
        color: #64748b;
        font-size: 11px;
        border-top: 1px solid #1e293b;
        text-align: center;
    }}
    @media print {{
        body {{ background: #ffffff; color: #000000; }}
        .header {{ background: #f8fafc; border-color: #cbd5e1; }}
        h1 {{ color: #0284c7; }}
        table {{ background: #ffffff; border-color: #cbd5e1; }}
        th {{ background: #f1f5f9; color: #334155; }}
        td {{ border-color: #e2e8f0; }}
    }}
</style>
</head>
<body>
<div class="header">
    <h1>🦾 {title}</h1>
    <div>Mô phỏng tích hợp Gazebo Sim & MoveIt 2 - Cánh tay Robot Denso VS-6556 (5-DOF)</div>
    <div class="meta-cards">
        <div class="card">
            <div class="card-title">Tổng số điểm</div>
            <div class="card-val">{total_cnt}</div>
        </div>
        <div class="card">
            <div class="card-title">Nét vẽ / Bay</div>
            <div class="card-val">{draw_cnt} / {rapid_cnt}</div>
        </div>
        <div class="card">
            <div class="card-title">Tỷ lệ khả thi IK</div>
            <div class="card-val" style="color: {'#4ade80' if pct == 100.0 else '#f87171'};">{pct:.1f}%</div>
        </div>
        <div class="card">
            <div class="card-title">Thời gian xuất</div>
            <div class="card-val" style="font-size: 14px;">{time.strftime("%H:%M:%S")}</div>
        </div>
    </div>
</div>

<table>
    <thead>
        <tr>
            <th class="center">#</th>
            <th class="center">Lệnh</th>
            <th class="center">Chế độ</th>
            <th>X (mm)</th>
            <th>Y (mm)</th>
            <th>Z (mm)</th>
            <th>Link 1 (J1°)</th>
            <th>Link 2 (J2°)</th>
            <th>Link 3 (J3°)</th>
            <th>Link 4 (J4°)</th>
            <th>Link 5 (J5°)</th>
            <th>Link 6 (J6°)</th>
            <th class="center">Trạng thái IK</th>
            <th>Sai số (mm)</th>
            <th>F (mm/p)</th>
        </tr>
    </thead>
    <tbody>
"""
        for r in records:
            c_tag = "tag-draw" if r.is_draw else "tag-rapid"
            c_name = "Vẽ (Pen Down)" if r.is_draw else "Bay (Rapid)"
            st_class = "tag-ok" if r.is_valid else "tag-err"
            if r.q_deg:
                j_vals = [f"{d:.2f}°" for d in r.q_deg]
            else:
                j_vals = ["N/A"] * 6
            while len(j_vals) < 6:
                j_vals.append("0.00°")
            j1, j2, j3, j4, j5, j6 = j_vals[:6]

            html += f"""        <tr>
            <td class="center">{r.index}</td>
            <td class="center"><b>{r.cmd}</b></td>
            <td class="center"><span class="{c_tag}">{c_name}</span></td>
            <td>{r.x:.2f}</td>
            <td>{r.y:.2f}</td>
            <td>{r.z:.2f}</td>
            <td><b>{j1}</b></td>
            <td><b>{j2}</b></td>
            <td><b>{j3}</b></td>
            <td><b>{j4}</b></td>
            <td><b>{j5}</b></td>
            <td><b>{j6}</b></td>
            <td class="center"><span class="{st_class}">{r.status}</span></td>
            <td>{r.err_mm:.3f}</td>
            <td>{r.feedrate:.0f}</td>
        </tr>\n"""

        html += """    </tbody>
</table>

<div class="footer">
    Bản quyền © 2026 Denso Robotics Lab | Trình trích xuất góc 5 Link từ G-Code cho Gazebo & MoveIt 2.
</div>
</body>
</html>
"""
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(html)
        return True
    except Exception as e:
        print(f"[ERROR] Xuất HTML thất bại: {e}")
        return False



def export_records_to_gcode(records, file_path):
    """Xuất G-code 6 link (A=J1, B=J2, C=J3, U=J4, V=J5, W=J6).
    Khớp xoay mới Link 6 được định nghĩa là trục W. Feed rate F chỉ ghi ở dòng đầu."""
    try:
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        with open(file_path, mode="w", encoding="utf-8") as f:
            f.write("; Robot Denso VS-6556 - 6 Link G-code\n")
            f.write("; Truc: A=Link1(J1), B=Link2(J2), C=Link3(J3), U=Link4(J4), V=Link5(J5), W=Link6(J6)\n")
            f.write(f"; Tong so diem: {len(records)}\n")
            first_f = True
            for r in records:
                if r.q_deg and len(r.q_deg) >= 5:
                    def fmt(v):
                        rv = round(v, 3)
                        if abs(rv) < 1e-6:
                            rv = 0.0
                        return str(int(rv)) if rv == int(rv) else str(rv)
                    w_deg = r.q_deg[5] if len(r.q_deg) >= 6 else 0.0
                    line = (f"G1 A{fmt(r.q_deg[0])} B{fmt(r.q_deg[1])}"
                            f" C{fmt(r.q_deg[2])} U{fmt(r.q_deg[3])} V{fmt(r.q_deg[4])} W{fmt(w_deg)}")
                    if first_f and getattr(r, "feedrate", 0) and r.feedrate > 0:
                        line += f" F{int(r.feedrate)}"
                        first_f = False
                    f.write(line + "\n")
                else:
                    f.write(f"; [SKIP] Diem #{r.index} - IK loi ({r.status})\n")
        return True
    except Exception as e:
        print(f"[ERROR] Xuat G-code that bai: {e}")
        return False

def records_to_tsv(records: List[GCodePointRecord], unit: str = "deg") -> str:
    """Tạo chuỗi dữ liệu phân cách bằng Tab để người dùng dán trực tiếp vào Excel (Ctrl+V)."""
    lines = [
        "STT\tLệnh\tChế độ\tX (mm)\tY (mm)\tZ (mm)\tLink 1 [J1]\tLink 2 [J2]\tLink 3 [J3]\tLink 4 [J4]\tLink 5 [J5]\tLink 6 [J6]\tTrạng thái IK\tSai số ΔE (mm)\tF (mm/phút)"
    ]
    for r in records:
        che_do = "Vẽ" if r.is_draw else "Bay"
        if unit == "rad" and r.q_rad:
            q_padded = list(r.q_rad)
            while len(q_padded) < 6:
                q_padded.append(0.0)
            j_vals = [f"{v:.4f}" for v in q_padded[:6]]
        elif r.q_deg:
            q_padded = list(r.q_deg)
            while len(q_padded) < 6:
                q_padded.append(0.0)
            j_vals = [f"{v:.2f}°" for v in q_padded[:6]]
        else:
            j_vals = ["N/A"] * 6
        line = f"{r.index}\t{r.cmd}\t{che_do}\t{r.x:.2f}\t{r.y:.2f}\t{r.z:.2f}\t" + "\t".join(j_vals) + f"\t{r.status}\t{r.err_mm:.3f}\t{r.feedrate:.0f}"
        lines.append(line)
    return "\n".join(lines)


# =============================================================================
# DATA PARSING & KINEMATICS EXTRACTION
# =============================================================================

def extract_records_from_strokes(strokes: Any, engine: Any = None) -> List[GCodePointRecord]:
    """
    Trích xuất toàn bộ các điểm từ danh sách Stroke (của denso_gcode_vision_engine),
    chuyển thành danh sách GCodePointRecord kèm các góc 5 Link.
    """
    records: List[GCodePointRecord] = []
    pt_counter = 0

    q_seed = np.array([0.0, 0.908, 1.035, 0.0, 1.199, 0.0])

    for s_idx, stroke in enumerate(strokes):
        is_stroke_draw = getattr(stroke, "is_draw", True)
        stroke_pts = getattr(stroke, "points", [])

        for pt in stroke_pts:
            pt_counter += 1
            x = float(getattr(pt, "x", 0.0))
            y = float(getattr(pt, "y", 0.0))
            z = float(getattr(pt, "z", 0.0))
            is_draw = bool(getattr(pt, "is_draw", is_stroke_draw))
            feed = float(getattr(pt, "feedrate", 1200.0))
            cmd = getattr(pt, "cmd", "G01" if is_draw else "G00")

            q_sol = getattr(pt, "q_sol", None)
            is_valid = getattr(pt, "is_valid", True)
            err_mm = float(getattr(pt, "err_mm", 0.0))
            status = "OK"

            # Nếu chưa có q_sol và có engine -> giải IK tại chỗ
            if q_sol is None and engine is not None:
                pos_m = [x / 1000.0, y / 1000.0, z / 1000.0]
                q_res, ok, iters, err_m = engine.inverse_kinematics(
                    pos_m, q_seed=q_seed, use_flange=True, mode="hybrid", tool_pitch_rad=math.pi
                )
                err_mm = err_m * 1000.0
                if ok and q_res is not None:
                    is_col, _ = engine.check_collision(q_res)
                    if not is_col:
                        q_sol = q_res
                        is_valid = True
                        status = "OK"
                        q_seed = q_res.copy()
                    else:
                        q_sol = q_res
                        is_valid = False
                        status = "Va chạm bàn"
                else:
                    is_valid = False
                    status = "Ngoài tầm với"
            elif q_sol is not None:
                if not is_valid:
                    status = "Lỗi góc/Va chạm"
                else:
                    status = "OK"

            q_rad = list(q_sol) if q_sol is not None else None
            q_deg = [math.degrees(v) for v in q_sol] if q_sol is not None else None

            rec = GCodePointRecord(
                index=pt_counter,
                stroke_idx=s_idx,
                cmd=cmd,
                is_draw=is_draw,
                x=x,
                y=y,
                z=z,
                feedrate=feed,
                q_rad=q_rad,
                q_deg=q_deg,
                is_valid=is_valid,
                status=status,
                err_mm=err_mm
            )
            records.append(rec)

    return records


# =============================================================================
# G-CODE JOINT ANGLES TABLE WIDGET (UI COMPONENT)
# =============================================================================

class GCodeJointAnglesTableWidget(QWidget):
    """
    Trang hiển thị chi tiết các góc của 5 link tương ứng với từng điểm G-code:
    - Bảng số liệu trực quan: Tọa độ X, Y, Z và 5 góc khớp J1..J5 (Độ / Radian)
    - Tùy chọn chuyển đổi đơn vị góc (Độ, Rad, hoặc Song song cả 2)
    - Bộ lọc (Tất cả, Vẽ, Bay, Hợp lệ, Lỗi) & Ô tìm kiếm nhanh
    - Nút Xuất file CSV (Excel-ready), Text report, HTML in ấn
    - Nút Sao chép vào Clipboard
    - Thao tác gửi 5 góc tới Gazebo / ROS 2 để kiểm tra chuyển động
    - Highlight và cuộn theo thời gian thực khi robot đang vẽ trong mô phỏng!
    """
    sig_point_selected = pyqtSignal(int, float, float, float)  # index, x, y, z

    def __init__(self, bridge=None, engine=None, is_embedded: bool = False, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.engine = engine
        self.is_embedded = is_embedded
        self.popout_window: Optional[QMainWindow] = None

        self.records: List[GCodePointRecord] = []
        self.filtered_records: List[GCodePointRecord] = []
        self.current_title = "Quỹ Đạo G-Code Robot Denso VS-6556"
        self.unit_mode = "deg"  # "deg", "rad", "both"
        self.active_highlight_row: int = -1

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # ---------------------------------------------------------------------
        # 1. HEADER & KPI METRIC CARDS
        # ---------------------------------------------------------------------
        header_card = QFrame()
        header_card.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0f172a, stop:1 #1e293b);
                border: 1px solid #334155;
                border-radius: 8px;
                padding: 6px;
            }
        """)
        h_layout = QVBoxLayout(header_card)
        h_layout.setContentsMargins(8, 6, 8, 6)
        h_layout.setSpacing(6)

        # Tiêu đề & Badge
        title_row = QHBoxLayout()
        self.lbl_main_title = QLabel("🦾 BẢNG TRA GÓC 6 LINK (A, B, C, U, V, W) TƯƠNG ỨNG TỪNG ĐIỂM G-CODE")
        self.lbl_main_title.setStyleSheet("color: #38bdf8; font-size: 14px; font-weight: bold;")
        title_row.addWidget(self.lbl_main_title)

        title_row.addStretch(1)

        self.lbl_status_badge = QLabel("Chưa nạp quỹ đạo")
        self.lbl_status_badge.setStyleSheet("""
            color: #94a3b8; background-color: #0b1120; border: 1px solid #334155;
            border-radius: 4px; padding: 3px 8px; font-size: 11px; font-weight: bold;
        """)
        title_row.addWidget(self.lbl_status_badge)

        h_layout.addLayout(title_row)

        # Hàng thẻ KPI Metrics
        kpi_row = QHBoxLayout()
        kpi_row.setSpacing(8)

        self.card_pts = self._make_kpi("Tổng Số Điểm", "0 điểm", "#38bdf8")
        self.card_types = self._make_kpi("Nét Vẽ / Bay", "0 / 0", "#a78bfa")
        self.card_feasibility = self._make_kpi("Tỷ Lệ Khả Thi IK", "0.0%", "#4ade80")
        self.card_j1 = self._make_kpi("Link 1 (J1 Đế)", "---", "#38bdf8")
        self.card_j2 = self._make_kpi("Link 2 (J2 Vai)", "---", "#38bdf8")
        self.card_j3 = self._make_kpi("Link 3 (J3 Khuỷu)", "---", "#38bdf8")
        self.card_j5 = self._make_kpi("Link 5 (J5 Cổ tay)", "---", "#38bdf8")

        kpi_row.addWidget(self.card_pts)
        kpi_row.addWidget(self.card_types)
        kpi_row.addWidget(self.card_feasibility)
        kpi_row.addWidget(self.card_j1)
        kpi_row.addWidget(self.card_j2)
        kpi_row.addWidget(self.card_j3)
        kpi_row.addWidget(self.card_j5)

        h_layout.addLayout(kpi_row)
        main_layout.addWidget(header_card)

        # ---------------------------------------------------------------------
        # 2. THANH CÔNG CỤ (TOOLBAR: BỘ LỌC, TÌM KIẾM, ĐƠN VỊ & CÁC NÚT XUẤT)
        # ---------------------------------------------------------------------
        toolbar = QFrame()
        toolbar.setStyleSheet("""
            QFrame {
                background-color: #0b1120; border: 1px solid #1e293b;
                border-radius: 6px; padding: 4px;
            }
            QLabel { color: #94a3b8; font-size: 11px; font-weight: bold; }
        """)
        tb_vbox = QVBoxLayout(toolbar)
        tb_vbox.setContentsMargins(8, 6, 8, 6)
        tb_vbox.setSpacing(6)

        # HÀNG 1: ĐƠN VỊ, BỘ LỌC, TÌM KIẾM, DI CHUYỂN, CỬA SỔ RIÊNG & QUAY LẠI TAB 5
        row1 = QHBoxLayout()
        row1.setSpacing(8)

        row1.addWidget(QLabel("📐 Đơn vị:"))
        self.combo_unit = QComboBox()
        self.combo_unit.setStyleSheet(self._combo_style())
        self.combo_unit.addItem("Độ (°)", "deg")
        self.combo_unit.addItem("Radian (rad)", "rad")
        self.combo_unit.addItem("Cả Độ & Radian", "both")
        self.combo_unit.currentIndexChanged.connect(self._on_unit_changed)
        row1.addWidget(self.combo_unit)

        row1.addWidget(QLabel("🔍 Lọc:"))
        self.combo_filter = QComboBox()
        self.combo_filter.setStyleSheet(self._combo_style())
        self.combo_filter.addItem("Tất cả các điểm", "all")
        self.combo_filter.addItem("🖊️ Nét vẽ (Pen Down)", "draw")
        self.combo_filter.addItem("✈️ Bay không tải (Rapid)", "rapid")
        self.combo_filter.addItem("✅ Điểm hợp lệ (OK)", "valid")
        self.combo_filter.addItem("⚠️ Cảnh báo / Lỗi IK", "invalid")
        self.combo_filter.currentIndexChanged.connect(self._apply_filters)
        row1.addWidget(self.combo_filter)

        self.txt_search = QLineEdit()
        self.txt_search.setPlaceholderText("Nhập STT hoặc tọa độ X/Y...")
        self.txt_search.setStyleSheet("""
            QLineEdit {
                background-color: #050b14; color: #f8fafc; border: 1px solid #334155;
                border-radius: 4px; padding: 4px 8px; font-size: 11px; min-width: 150px;
            }
            QLineEdit:focus { border: 1px solid #0284c7; }
        """)
        self.txt_search.textChanged.connect(self._apply_filters)
        row1.addWidget(self.txt_search)

        row1.addStretch(1)

        self.btn_move_to_point = QPushButton("🚀 Di Chuyển Robot Tới Điểm Này")
        self.btn_move_to_point.setToolTip("Gửi 5 góc khớp của điểm đang chọn tới Gazebo / ROS 2")
        self.btn_move_to_point.setStyleSheet(self._btn_accent())
        self.btn_move_to_point.clicked.connect(self._move_robot_to_selected_point)
        row1.addWidget(self.btn_move_to_point)

        self.btn_popout = QPushButton("🗖 Phóng To Cửa Sổ Riêng")
        self.btn_popout.setToolTip("Mở bảng dữ liệu ra cửa sổ độc lập để xem kích thước lớn")
        self.btn_popout.setStyleSheet(self._btn_secondary())
        self.btn_popout.clicked.connect(self.open_popout_window)
        row1.addWidget(self.btn_popout)

        self.btn_back_tab5 = QPushButton("🎨 Quay Lại Tab 5 (Vẽ Hình)")
        self.btn_back_tab5.setToolTip("Chuyển nhanh về Tab 5 để điều khiển khắc / vẽ hình ảnh")
        self.btn_back_tab5.setStyleSheet("""
            QPushButton {
                background-color: #1e293b; color: #38bdf8; border: 1px solid #0284c7;
                border-radius: 4px; padding: 5px 10px; font-size: 11px; font-weight: bold;
            }
            QPushButton:hover { background-color: #0c4a6e; color: #7dd3fc; border-color: #38bdf8; }
        """)
        self.btn_back_tab5.clicked.connect(self._back_to_tab5)
        row1.addWidget(self.btn_back_tab5)

        tb_vbox.addLayout(row1)

        # HÀNG 2: CÁC NÚT XUẤT FILE & SAO CHÉP
        row2 = QHBoxLayout()
        row2.setSpacing(8)

        self.btn_export_gcode = QPushButton("💾 XUẤT 6-LINK G-CODE (A, B, C, U, V, W)...")
        self.btn_export_gcode.setToolTip(
            "Xuất tệp G-code 6 link (A/B/C/U/V/W) thẳng vào thư mục OneDrive/Máy tính/Denso\n"
            "Trục W (Link 6 Khớp Xoay Mới) được xuất đầy đủ, F1200 ở dòng đầu tiên."
        )
        self.btn_export_gcode.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #059669, stop:1 #047857);
                color: #ffffff; border: 1px solid #10b981;
                border-radius: 4px; padding: 6px 14px; font-size: 11px; font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #047857, stop:1 #065f46);
                border-color: #34d399;
            }
        """)
        self.btn_export_gcode.clicked.connect(self._export_gcode)
        row2.addWidget(self.btn_export_gcode)

        self.btn_export_csv = QPushButton("📊 Xuất CSV (Excel)...")
        self.btn_export_csv.setToolTip("Xuất toàn bộ bảng góc 5 link ra tệp CSV (Excel-ready)")
        self.btn_export_csv.setStyleSheet(self._btn_secondary())
        self.btn_export_csv.clicked.connect(self._export_csv)
        row2.addWidget(self.btn_export_csv)

        self.btn_export_txt = QPushButton("📄 Xuất Báo Cáo Text...")
        self.btn_export_txt.setToolTip("Xuất báo cáo dạng bảng text monospaced căn lề đẹp")
        self.btn_export_txt.setStyleSheet(self._btn_secondary())
        self.btn_export_txt.clicked.connect(self._export_txt)
        row2.addWidget(self.btn_export_txt)

        self.btn_export_html = QPushButton("🌐 Xuất Báo Cáo HTML...")
        self.btn_export_html.setToolTip("Xuất báo cáo trang web đẹp mắt chuẩn in ấn / PDF")
        self.btn_export_html.setStyleSheet(self._btn_secondary())
        self.btn_export_html.clicked.connect(self._export_html)
        row2.addWidget(self.btn_export_html)

        self.btn_copy = QPushButton("📋 Sao Chép Bảng (Ctrl+V)")
        self.btn_copy.setToolTip("Sao chép dữ liệu dạng bảng để dán trực tiếp vào Excel")
        self.btn_copy.setStyleSheet(self._btn_secondary())
        self.btn_copy.clicked.connect(self._copy_to_clipboard)
        row2.addWidget(self.btn_copy)

        row2.addStretch(1)

        lbl_hint = QLabel("💡 Mẹo: Nhấp đúp vào 1 điểm trên bảng để robot trong Gazebo di chuyển tới điểm đó.")
        lbl_hint.setStyleSheet("color: #64748b; font-size: 11px; font-style: italic;")
        row2.addWidget(lbl_hint)

        tb_vbox.addLayout(row2)
        main_layout.addWidget(toolbar)

        # ---------------------------------------------------------------------
        # 3. BẢNG DỮ LIỆU TƯƠNG TÁC (QTABLEWIDGET)
        # ---------------------------------------------------------------------
        self.table = QTableWidget()
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setHighlightSections(False)
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self._show_context_menu)
        self.table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.table.cellDoubleClicked.connect(lambda r, c: self._move_robot_to_selected_point())

        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #050b14;
                alternate-background-color: #0b1120;
                color: #e2e8f0;
                gridline-color: #1e293b;
                border: 1px solid #1e293b;
                border-radius: 6px;
                font-family: monospace;
                font-size: 11px;
                selection-background-color: #1e3a8a;
                selection-color: #ffffff;
            }
            QHeaderView::section {
                background-color: #0f172a;
                color: #38bdf8;
                font-weight: bold;
                padding: 6px 8px;
                border: none;
                border-right: 1px solid #1e293b;
                border-bottom: 2px solid #0284c7;
                font-size: 11px;
            }
            QScrollBar:vertical {
                background: #0b1120; width: 12px; margin: 0px; border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background: #334155; min-height: 20px; border-radius: 6px;
            }
            QScrollBar::handle:vertical:hover { background: #0284c7; }
        """)

        main_layout.addWidget(self.table, 1)

        # ---------------------------------------------------------------------
        # 4. THANH TRẠNG THÁI DƯỚI ĐÁY (STATUS & SELECTED POINT DETAILS)
        # ---------------------------------------------------------------------
        bottom_bar = QFrame()
        bottom_bar.setStyleSheet("""
            QFrame {
                background-color: #0b1120; border: 1px solid #1e293b;
                border-radius: 6px; padding: 4px 8px;
            }
        """)
        bot_lay = QHBoxLayout(bottom_bar)
        bot_lay.setContentsMargins(6, 2, 6, 2)

        self.lbl_sel_info = QLabel("💡 Hướng dẫn: Chọn một điểm để xem chi tiết hoặc nhấp đúp để di chuyển robot trong Gazebo.")
        self.lbl_sel_info.setStyleSheet("color: #94a3b8; font-size: 11px; font-family: monospace;")
        bot_lay.addWidget(self.lbl_sel_info, 1)

        self.lbl_footer_count = QLabel("0 điểm hiển thị")
        self.lbl_footer_count.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold;")
        bot_lay.addWidget(self.lbl_footer_count)

        main_layout.addWidget(bottom_bar)

        self._setup_table_headers()

    def _setup_table_headers(self):
        """Cập nhật tiêu đề cột dựa trên đơn vị được chọn."""
        if self.unit_mode == "deg":
            headers = [
                "#", "Lệnh", "Chế Độ", "X (mm)", "Y (mm)", "Z (mm)",
                "Link 1 [J1 - Trục A] (°)", "Link 2 [J2 - Trục B] (°)", "Link 3 [J3 - Trục C] (°)",
                "Link 4 [J4 - Trục U] (°)", "Link 5 [J5 - Trục V] (°)", "Link 6 [J6 - Mặt Bích] (°)",
                "Trạng Thái IK", "Sai Số ΔE", "F (mm/p)"
            ]
        elif self.unit_mode == "rad":
            headers = [
                "#", "Lệnh", "Chế Độ", "X (mm)", "Y (mm)", "Z (mm)",
                "Link 1 [J1] (rad)", "Link 2 [J2] (rad)", "Link 3 [J3] (rad)",
                "Link 4 [J4] (rad)", "Link 5 [J5] (rad)", "Link 6 [J6] (rad)",
                "Trạng Thái IK", "Sai Số ΔE", "F (mm/p)"
            ]
        else:  # both
            headers = [
                "#", "Lệnh", "Chế Độ", "X (mm)", "Y (mm)", "Z (mm)",
                "Link 1 [J1] (° / rad)", "Link 2 [J2] (° / rad)", "Link 3 [J3] (° / rad)",
                "Link 4 [J4] (° / rad)", "Link 5 [J5] (° / rad)", "Link 6 [J6] (° / rad)",
                "Trạng Thái IK", "Sai Số ΔE", "F (mm/p)"
            ]

        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)

        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Interactive)
        header.setStretchLastSection(True)
        col_widths = [60, 65, 85, 80, 80, 80, 130, 130, 130, 130, 130, 130, 100, 85, 75]
        for c, w in enumerate(col_widths):
            if c < self.table.columnCount():
                self.table.setColumnWidth(c, w)

    def _make_kpi(self, title: str, initial_val: str, color: str = "#38bdf8") -> QFrame:
        box = QFrame()
        box.setStyleSheet("""
            QFrame {
                background-color: #0b1120; border: 1px solid #1e293b;
                border-radius: 6px; padding: 4px 8px;
            }
        """)
        v = QVBoxLayout(box)
        v.setContentsMargins(2, 2, 2, 2)
        v.setSpacing(2)
        lbl_t = QLabel(title)
        lbl_t.setStyleSheet("color: #94a3b8; font-size: 10px; font-weight: bold; text-transform: uppercase;")
        lbl_v = QLabel(initial_val)
        lbl_v.setStyleSheet(f"color: {color}; font-size: 13px; font-weight: bold; font-family: monospace;")
        v.addWidget(lbl_t)
        v.addWidget(lbl_v)
        box.val_lbl = lbl_v
        return box

    def _combo_style(self):
        return """
            QComboBox {
                background-color: #050b14; color: #f8fafc; border: 1px solid #334155;
                border-radius: 4px; padding: 3px 8px; font-size: 11px; font-weight: bold;
            }
            QComboBox QAbstractItemView {
                background-color: #0f172a; color: #f8fafc; selection-background-color: #0284c7;
            }
        """

    def _btn_accent(self):
        return """
            QPushButton {
                background-color: #0284c7; color: #ffffff; border: none;
                border-radius: 4px; padding: 5px 12px; font-weight: bold; font-size: 11px;
            }
            QPushButton:hover { background-color: #0369a1; }
            QPushButton:disabled { background-color: #334155; color: #64748b; }
        """

    def _btn_secondary(self):
        return """
            QPushButton {
                background-color: #1e293b; color: #f8fafc; border: 1px solid #334155;
                border-radius: 4px; padding: 5px 10px; font-size: 11px; font-weight: bold;
            }
            QPushButton:hover { background-color: #334155; border-color: #0284c7; }
        """

    # =========================================================================
    # PUBLIC API: NẠP DỮ LIỆU & ĐỒNG BỘ
    # =========================================================================

    def set_records(self, records: List[GCodePointRecord], item_name: str = ""):
        """Nạp trực tiếp danh sách GCodePointRecord và làm mới hiển thị."""
        self.records = records
        if item_name:
            self.current_title = f"Quỹ đạo G-Code: {item_name}"
            self.lbl_main_title.setText(f"🦾 BẢNG GÓC 5 LINK: {item_name}")
        self._update_kpi_cards()
        self._apply_filters()

    def set_strokes(self, strokes: Any, item_name: str = ""):
        """Nạp từ danh sách Stroke (denso_gcode_vision_engine)."""
        records = extract_records_from_strokes(strokes, self.engine)
        self.set_records(records, item_name)

    def load_gcode_file(self, file_path: str):
        """Nạp và phân tích từ tệp G-Code."""
        if not os.path.exists(file_path):
            QMessageBox.warning(self, "Lỗi", f"Không tìm thấy tệp: {file_path}")
            return
        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                text = f.read()
            self.load_gcode_text(text, os.path.basename(file_path))
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", f"Không thể đọc tệp G-Code: {e}")

    def load_gcode_text(self, gcode_text: str, item_name: str = "Tùy chỉnh"):
        """Phân tích chuỗi G-Code và giải động học nghịch."""
        from denso_gcode_vision_engine import ImageGCodeProcessor, KinematicsValidator
        proc = ImageGCodeProcessor()
        strokes = proc.parse_gcode(gcode_text)
        if self.engine:
            val = KinematicsValidator(self.engine)
            val.validate(strokes)
        self.set_strokes(strokes, item_name)

    # =========================================================================
    # CẬP NHẬT GIAO DIỆN & BẢNG
    # =========================================================================

    def _update_kpi_cards(self):
        total = len(self.records)
        if total == 0:
            self.card_pts.val_lbl.setText("0 điểm")
            self.card_types.val_lbl.setText("0 / 0")
            self.card_feasibility.val_lbl.setText("0.0%")
            self.lbl_status_badge.setText("Chưa nạp quỹ đạo")
            self.lbl_status_badge.setStyleSheet("color: #94a3b8; background-color: #0b1120; border: 1px solid #334155; border-radius: 4px; padding: 3px 8px; font-size: 11px;")
            return

        draw_cnt = sum(1 for r in self.records if r.is_draw)
        rapid_cnt = total - draw_cnt
        valid_cnt = sum(1 for r in self.records if r.is_valid)
        pct = (valid_cnt / total * 100.0) if total > 0 else 0.0

        self.card_pts.val_lbl.setText(f"{total:,} điểm")
        self.card_types.val_lbl.setText(f"{draw_cnt} vẽ / {rapid_cnt} bay")

        feas_color = "#4ade80" if pct >= 99.9 else ("#fbbf24" if pct > 80.0 else "#f87171")
        self.card_feasibility.val_lbl.setText(f"{pct:.1f}%")
        self.card_feasibility.val_lbl.setStyleSheet(f"color: {feas_color}; font-size: 13px; font-weight: bold; font-family: monospace;")

        # Min/Max các khớp
        j_degs = [r.q_deg for r in self.records if r.q_deg is not None]
        if j_degs:
            j_arr = np.array(j_degs)  # Shape (N, 5)
            min_vals = np.min(j_arr, axis=0)
            max_vals = np.max(j_arr, axis=0)
            self.card_j1.val_lbl.setText(f"{min_vals[0]:.0f}° ~ {max_vals[0]:.0f}°")
            self.card_j2.val_lbl.setText(f"{min_vals[1]:.0f}° ~ {max_vals[1]:.0f}°")
            self.card_j3.val_lbl.setText(f"{min_vals[2]:.0f}° ~ {max_vals[2]:.0f}°")
            self.card_j5.val_lbl.setText(f"{min_vals[4]:.0f}° ~ {max_vals[4]:.0f}°")
        else:
            self.card_j1.val_lbl.setText("N/A")
            self.card_j2.val_lbl.setText("N/A")
            self.card_j3.val_lbl.setText("N/A")
            self.card_j5.val_lbl.setText("N/A")

        if valid_cnt == total:
            self.lbl_status_badge.setText("✅ 100% Khả thi")
            self.lbl_status_badge.setStyleSheet("color: #4ade80; background-color: #064e3b; border: 1px solid #059669; border-radius: 4px; padding: 3px 8px; font-size: 11px; font-weight: bold;")
        else:
            self.lbl_status_badge.setText(f"⚠️ {total - valid_cnt} điểm lỗi")
            self.lbl_status_badge.setStyleSheet("color: #f87171; background-color: #450a0a; border: 1px solid #dc2626; border-radius: 4px; padding: 3px 8px; font-size: 11px; font-weight: bold;")

    def _on_unit_changed(self):
        self.unit_mode = self.combo_unit.currentData()
        self._setup_table_headers()
        self._populate_table()

    def _apply_filters(self):
        """Lọc danh sách theo Chế độ và Ô tìm kiếm."""
        f_type = self.combo_filter.currentData()
        search = self.txt_search.text().strip().lower()

        filtered = []
        for r in self.records:
            if f_type == "draw" and not r.is_draw:
                continue
            if f_type == "rapid" and r.is_draw:
                continue
            if f_type == "valid" and not r.is_valid:
                continue
            if f_type == "invalid" and r.is_valid:
                continue

            if search:
                # Tìm theo STT, X, Y, Z, lệnh
                match_idx = str(r.index) == search
                match_cmd = search in r.cmd.lower()
                match_x = f"{r.x:.1f}".startswith(search)
                match_y = f"{r.y:.1f}".startswith(search)
                if not (match_idx or match_cmd or match_x or match_y):
                    continue

            filtered.append(r)

        self.filtered_records = filtered
        if self.isVisible():
            self._table_dirty = False
            self._populate_table()
        else:
            self._table_dirty = True

    def showEvent(self, event):
        super().showEvent(event)
        if getattr(self, '_table_dirty', False):
            self._table_dirty = False
            self._populate_table()

    def _populate_table(self):
        """Đổ dữ liệu vào QTableWidget với hiệu năng cao."""
        self.table.setUpdatesEnabled(False)
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.filtered_records))
        self._seq_to_row = {}

        for row_idx, r in enumerate(self.filtered_records):
            self._seq_to_row[r.index] = row_idx
            # Cột 0: STT
            item_idx = QTableWidgetItem(str(r.index))
            item_idx.setTextAlignment(Qt.AlignCenter)
            item_idx.setData(Qt.UserRole, r)

            # Cột 1: Lệnh
            item_cmd = QTableWidgetItem(r.cmd)
            item_cmd.setTextAlignment(Qt.AlignCenter)

            # Cột 2: Chế độ
            if r.is_draw:
                item_mode = QTableWidgetItem("🖊️ Vẽ")
                item_mode.setForeground(QBrush(QColor("#4ade80")))
            else:
                item_mode = QTableWidgetItem("✈️ Bay")
                item_mode.setForeground(QBrush(QColor("#fbbf24")))
            item_mode.setTextAlignment(Qt.AlignCenter)

            # Cột 3, 4, 5: X, Y, Z
            item_x = QTableWidgetItem(f"{r.x:.2f}")
            item_x.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            item_y = QTableWidgetItem(f"{r.y:.2f}")
            item_y.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            item_z = QTableWidgetItem(f"{r.z:.2f}")
            item_z.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)

            # Cột 6..11: Link 1..6
            joint_items = []
            num_j = len(JOINT_NAMES)
            for j_i in range(num_j):
                if r.q_deg and j_i < len(r.q_deg):
                    deg_val = r.q_deg[j_i]
                    rad_val = r.q_rad[j_i] if r.q_rad and j_i < len(r.q_rad) else math.radians(deg_val)
                    if self.unit_mode == "deg":
                        val_str = f"{deg_val:.3f}°"
                    elif self.unit_mode == "rad":
                        val_str = f"{rad_val:.4f}"
                    else:  # both
                        val_str = f"{deg_val:.1f}° ({rad_val:.3f})"
                else:
                    val_str = "---"

                it = QTableWidgetItem(val_str)
                it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)

                # Cảnh báo màu nếu vượt ngưỡng
                if r.q_rad and j_i < len(r.q_rad) and j_i < len(JOINT_LIMITS):
                    q_val = r.q_rad[j_i]
                    q_min, q_max = JOINT_LIMITS[j_i]
                    if q_val < q_min or q_val > q_max:
                        it.setForeground(QBrush(QColor("#f87171")))
                    elif abs(q_val - q_min) < 0.05 or abs(q_val - q_max) < 0.05:
                        it.setForeground(QBrush(QColor("#fbbf24")))
                    else:
                        it.setForeground(QBrush(QColor("#38bdf8")))
                joint_items.append(it)

            # Cột trạng thái IK
            item_st = QTableWidgetItem(r.status)
            item_st.setTextAlignment(Qt.AlignCenter)
            if r.is_valid:
                item_st.setForeground(QBrush(QColor("#4ade80")))
            else:
                item_st.setForeground(QBrush(QColor("#f87171")))

            # Cột Sai số ΔE (mm)
            item_err = QTableWidgetItem(f"{r.err_mm:.3f}")
            item_err.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            if r.err_mm > 1.0:
                item_err.setForeground(QBrush(QColor("#f87171")))

            # Cột F (mm/phút)
            item_f = QTableWidgetItem(f"{r.feedrate:.0f}")
            item_f.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)

            self.table.setItem(row_idx, 0, item_idx)
            self.table.setItem(row_idx, 1, item_cmd)
            self.table.setItem(row_idx, 2, item_mode)
            self.table.setItem(row_idx, 3, item_x)
            self.table.setItem(row_idx, 4, item_y)
            self.table.setItem(row_idx, 5, item_z)
            col_ptr = 6
            for jit in joint_items:
                self.table.setItem(row_idx, col_ptr, jit)
                col_ptr += 1
            self.table.setItem(row_idx, col_ptr, item_st)
            self.table.setItem(row_idx, col_ptr + 1, item_err)
            self.table.setItem(row_idx, col_ptr + 2, item_f)

        self.table.blockSignals(False)
        self.table.setUpdatesEnabled(True)

        self.lbl_footer_count.setText(f"{len(self.filtered_records):,} / {len(self.records):,} điểm")

    def _on_table_selection_changed(self):
        items = self.table.selectedItems()
        if not items:
            self.lbl_sel_info.setText("💡 Hướng dẫn: Chọn một điểm để xem chi tiết hoặc nhấp đúp để di chuyển robot trong Gazebo.")
            return

        row = self.table.currentRow()
        r_item = self.table.item(row, 0)
        if not r_item:
            return
        record: GCodePointRecord = r_item.data(Qt.UserRole)
        if not record:
            return

        mode_str = "VẼ (Hạ bút)" if record.is_draw else "BAY (Nhấc bút)"
        j_str = " | ".join([f"Link {i+1}={d:.2f}°" for i, d in enumerate(record.q_deg)]) if record.q_deg else "Chưa có nghiệm IK"
        self.lbl_sel_info.setText(
            f"📍 Điểm #{record.index} [{record.cmd} - {mode_str}] | Tọa độ: X={record.x:.1f}, Y={record.y:.1f}, Z={record.z:.1f} mm | {j_str} | Sai số: {record.err_mm:.3f} mm"
        )
        self.sig_point_selected.emit(record.index, record.x, record.y, record.z)

    # =========================================================================
    # ĐỒNG BỘ THỜI GIAN THỰC (LIVE TRACKING KHI VẼ TRONG GAZEBO)
    # =========================================================================

    def highlight_point(self, pt_index: int):
        if not self.isVisible():
            return
        """
        Highlight hàng điểm đang thực thi và tự động cuộn bảng theo thời gian thực (O(1) hiệu năng cao).
        Được gọi trực tiếp từ tiến trình TrajectoryExecutionWorker.
        """
        if not self.filtered_records:
            return

        target_row = getattr(self, "_seq_to_row", {}).get(pt_index, -1)
        if target_row == -1:
            if 0 <= pt_index - 1 < self.table.rowCount():
                target_row = pt_index - 1
            else:
                return

        if target_row == self.active_highlight_row:
            return

        # Bỏ highlight dòng cũ
        if self.active_highlight_row != -1 and self.active_highlight_row < self.table.rowCount():
            for c in range(self.table.columnCount()):
                it = self.table.item(self.active_highlight_row, c)
                if it:
                    it.setBackground(QBrush(QColor(0, 0, 0, 0)))

        # Highlight dòng mới với màu Cyan nổi bật
        self.active_highlight_row = target_row
        for c in range(self.table.columnCount()):
            it = self.table.item(target_row, c)
            if it:
                it.setBackground(QBrush(QColor("#0284c7")))

        it_scroll = self.table.item(target_row, 0)
        if it_scroll:
            self.table.scrollToItem(it_scroll, QAbstractItemView.PositionAtCenter)

    # =========================================================================
    # HÀNH ĐỘNG: GỬI GÓC TỚI GAZEBO ROBOT
    # =========================================================================

    def _move_robot_to_selected_point(self):
        """Gửi 5 góc của điểm được chọn tới bộ điều khiển trong Gazebo / ROS 2."""
        row = self.table.currentRow()
        if row == -1:
            QMessageBox.information(self, "Chú ý", "Vui lòng chọn một điểm trên bảng để điều khiển robot di chuyển tới!")
            return

        r_item = self.table.item(row, 0)
        if not r_item:
            return
        record: GCodePointRecord = r_item.data(Qt.UserRole)
        if not record or not record.q_rad:
            QMessageBox.warning(self, "Không thể di chuyển", "Điểm này không có nghiệm động học nghịch khả thi.")
            return

        if not self.bridge:
            QMessageBox.warning(self, "Chưa kết nối", "Chưa kết nối với ROS 2 / Gazebo Bridge.")
            return

        try:
            self.bridge.send_trajectory(record.q_rad, duration_sec=1.5, zero_velocity=True)
            deg_str = ", ".join([f"{d:.1f}°" for d in record.q_deg])
            self.lbl_sel_info.setText(f"🚀 Đã gửi lệnh tới Robot: Di chuyển tới Điểm #{record.index} [Góc 5 Link: {deg_str}]")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", f"Không thể gửi lệnh quỹ đạo: {e}")

    # =========================================================================
    # XUẤT DỮ LIỆU (CSV, TXT, HTML, CLIPBOARD)
    # =========================================================================

    def _export_csv(self):
        if not self.records:
            QMessageBox.warning(self, "Chú ý", "Không có dữ liệu để xuất!")
            return
        fn, _ = QFileDialog.getSaveFileName(self, "Lưu file CSV", "gcode_5link_angles.csv", "CSV Files (*.csv)")
        if fn:
            if export_records_to_csv(self.records, fn):
                QMessageBox.information(self, "Thành công", f"Đã xuất thành công {len(self.records)} điểm ra file:\n{fn}")

    def _export_txt(self):
        if not self.records:
            QMessageBox.warning(self, "Chú ý", "Không có dữ liệu để xuất!")
            return
        fn, _ = QFileDialog.getSaveFileName(self, "Lưu file Text Báo Cáo", "gcode_5link_angles_report.txt", "Text Files (*.txt)")
        if fn:
            if export_records_to_txt(self.records, fn, self.current_title):
                QMessageBox.information(self, "Thành công", f"Đã xuất thành công báo cáo Text ra file:\n{fn}")

    def _export_html(self):
        if not self.records:
            QMessageBox.warning(self, "Chú ý", "Không có dữ liệu để xuất!")
            return
        fn, _ = QFileDialog.getSaveFileName(self, "Lưu Báo Cáo HTML", "gcode_5link_angles_report.html", "HTML Files (*.html)")
        if fn:
            if export_records_to_html(self.records, fn, self.current_title):
                QMessageBox.information(self, "Thành công", f"Đã xuất thành công báo cáo HTML ra file:\n{fn}")


    def _export_gcode(self):
        import time as _t
        if not self.records:
            QMessageBox.warning(self, 'Xuat G-Code', 'Khong co du lieu!')
            return
        # Thu muc xuat: OneDrive > May tinh > Denso
        _onedrive = '/mnt/c/Users/thanh/OneDrive/Máy tính/Denso'
        if os.path.isdir(_onedrive):
            gcode_dir = _onedrive
        else:
            # fallback: thu muc gcode trong package
            gcode_dir = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(__file__)), '..', 'gcode'))
        ts = _t.strftime('%Y%m%d_%H%M%S')
        default_filename = f'gcode_6link_{ts}.gcode'
        default_path = os.path.join(gcode_dir, default_filename)

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Xuất File G-Code 6 Link Robot Denso (A, B, C, U, V, W)",
            default_path,
            "G-Code Files (*.gcode *.nc);;All Files (*)"
        )
        if not file_path:
            return

        filename = os.path.basename(file_path)
        gcode_dir = os.path.dirname(file_path)
        if export_records_to_gcode(self.records, file_path):
            valid_cnt = sum(1 for r in self.records if r.q_deg)
            skipped = len(self.records) - valid_cnt
            info = ('\u2705 Xuat thanh cong G-code 6 link!\n\n'
                    '\U0001f4c1 Thu muc: ' + gcode_dir + '\n'
                    '\U0001f4c4 File: ' + filename + '\n\n'
                    '\U0001f4ca Tong: ' + str(len(self.records)) + ' diem\n'
                    '   Xuat IK kha thi: ' + str(valid_cnt) + '\n'
                    '   Bo qua IK loi: ' + str(skipped) + '\n'
                    '   Truc W (Link 6): Da tich hop day du')
            QMessageBox.information(self, 'Xuat G-Code thanh cong', info)
        else:
            QMessageBox.critical(self, 'Loi', 'Khong the xuat G-code ra: ' + file_path)

    def _copy_to_clipboard(self):
        if not self.records:
            return
        tsv = records_to_tsv(self.filtered_records, self.unit_mode)
        QApplication.clipboard().setText(tsv)
        QMessageBox.information(self, "Đã sao chép", f"Đã sao chép {len(self.filtered_records)} dòng dữ liệu vào Clipboard!\nBạn có thể dán trực tiếp (Ctrl+V) vào Microsoft Excel hoặc Google Sheets.")

    def _show_context_menu(self, pos):
        menu = QMenu(self)
        menu.setStyleSheet("""
            QMenu { background-color: #0f172a; color: #f8fafc; border: 1px solid #334155; }
            QMenu::item:selected { background-color: #0284c7; }
        """)
        act_move = menu.addAction("🚀 Di chuyển Robot tới điểm này")
        menu.addSeparator()
        act_copy = menu.addAction("📋 Sao chép dòng này")
        act_csv = menu.addAction("💾 Xuất toàn bộ ra CSV...")
        act_html = menu.addAction("🌐 Xuất toàn bộ ra HTML...")

        action = menu.exec_(self.table.mapToGlobal(pos))
        if action == act_move:
            self._move_robot_to_selected_point()
        elif action == act_copy:
            row = self.table.currentRow()
            if row != -1:
                r_item = self.table.item(row, 0)
                if r_item:
                    rec: GCodePointRecord = r_item.data(Qt.UserRole)
                    if rec:
                        tsv = records_to_tsv([rec], self.unit_mode)
                        QApplication.clipboard().setText(tsv)
        elif action == act_csv:
            self._export_csv()
        elif action == act_html:
            self._export_html()

    # =========================================================================
    # POP-OUT CỬA SỔ ĐỘC LẬP
    # =========================================================================

    def open_popout_window(self):
        """Mở bảng ra một cửa sổ độc lập (Standalone Pop-out Window)."""
        if self.popout_window is None:
            self.popout_window = GCodeJointAnglesWindow(bridge=self.bridge, engine=self.engine)
        self.popout_window.table_widget.set_records(self.records, self.current_title)
        self.popout_window.show()
        self.popout_window.raise_()
        self.popout_window.activateWindow()

    def _back_to_tab5(self):
        """Chuyển nhanh sang Tab 5 (Khắc / Vẽ Hình Ảnh)."""
        p = self.parent()
        while p is not None:
            if hasattr(p, "tabs") and isinstance(p.tabs, QTabWidget):
                p.tabs.setCurrentIndex(4)
                return
            p = p.parent()



# =============================================================================
# CỬA SỔ ĐỘC LẬP (STANDALONE WINDOW)
# =============================================================================

class GCodeJointAnglesWindow(QMainWindow):
    """Cửa sổ độc lập hiển thị toàn màn hình bảng tra góc 5 link theo G-code."""
    def __init__(self, bridge=None, engine=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("🦾 Bảng Tra Góc 6 Link (A, B, C, U, V, W) Robot Denso VS-6556 Theo G-Code [Gazebo & MoveIt 2]")
        self.resize(1200, 750)
        self.setStyleSheet("background-color: #050b14;")

        self.table_widget = GCodeJointAnglesTableWidget(bridge=bridge, engine=engine, is_embedded=False, parent=self)
        self.setCentralWidget(self.table_widget)


# =============================================================================
# CLI ENTRY POINT
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="DENSO VS-6556 G-Code 5-Link Angles Exporter & Viewer")
    parser.add_argument("--input", "-i", type=str, help="Đường dẫn file G-code đầu vào (.gcode, .nc, .txt)")
    parser.add_argument("--csv", type=str, help="Xuất ra tệp CSV")
    parser.add_argument("--txt", type=str, help="Xuất ra tệp Text report")
    parser.add_argument("--html", type=str, help="Xuất ra tệp HTML report")
    parser.add_argument("--gui", action="store_true", help="Khởi động giao diện GUI độc lập")
    args = parser.parse_args()

    if args.gui or not args.input:
        app = QApplication(sys.argv)
        eng = DensoDynamicsEngine() if DensoDynamicsEngine else None
        win = GCodeJointAnglesWindow(engine=eng)
        if args.input and os.path.exists(args.input):
            win.table_widget.load_gcode_file(args.input)
        win.show()
        sys.exit(app.exec_())

    if args.input:
        if not os.path.exists(args.input):
            print(f"[ERROR] Không tìm thấy file: {args.input}")
            sys.exit(1)

        eng = DensoDynamicsEngine()
        from denso_gcode_vision_engine import ImageGCodeProcessor, KinematicsValidator
        with open(args.input, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()

        proc = ImageGCodeProcessor()
        strokes = proc.parse_gcode(text)
        val = KinematicsValidator(eng)
        val.validate(strokes)
        records = extract_records_from_strokes(strokes, eng)

        base_name = os.path.splitext(args.input)[0]
        csv_file = args.csv or f"{base_name}_5link_angles.csv"
        txt_file = args.txt or f"{base_name}_5link_angles.txt"
        html_file = args.html or f"{base_name}_5link_angles.html"

        export_records_to_csv(records, csv_file)
        print(f"[INFO] Đã xuất CSV: {csv_file}")
        export_records_to_txt(records, txt_file)
        print(f"[INFO] Đã xuất TXT: {txt_file}")
        export_records_to_html(records, html_file)
        print(f"[INFO] Đã xuất HTML: {html_file}")
        print(f"[DONE] Hoàn thành trích xuất {len(records)} điểm G-Code.")

if __name__ == '__main__':
    main()


