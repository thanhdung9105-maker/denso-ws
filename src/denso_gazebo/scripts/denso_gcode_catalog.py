#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO G-CODE CATALOG & MANAGEMENT SYSTEM
Module quản lý danh mục mã G-Code tùy chỉnh và giao diện thêm/lưu G-Code
cho cánh tay robot Denso VS-6556 trong Gazebo Simulation & RViz2.
"""

import os
import sys
import re
import json
import math
import time
from datetime import datetime
from typing import List, Tuple, Dict, Any, Optional

from PyQt5.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QTextEdit, QComboBox, QCheckBox, QGroupBox, QFrame,
    QGridLayout, QFileDialog, QMessageBox, QApplication
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

# -----------------------------------------------------------------------------
# MẪU G-CODE CHUẨN CHO BÀN VẼ ROBOT DENSO (Tâm X=-380, Y=0, Z_draw=180, Z_lift=205)
# -----------------------------------------------------------------------------
GCODE_TEMPLATES = {
    'square': '''; --- Hinh Vuong Chuan 80x80mm ---
; Tam ve: X=-380.00, Y=0.00 | Cao do Z_draw=180.00, Z_lift=205.00
G21 ; mm
G90 ; Toa do tuyet doi
G00 Z205.00 F2500 ; Nhac but an toan
G00 X-420.00 Y-40.00 ; Di chuyen den goc bat dau
G01 Z180.00 F1000 ; Ha but tiep xuc ban ve
G01 X-340.00 Y-40.00 F1200 ; Canh 1
G01 X-340.00 Y40.00 F1200 ; Canh 2
G01 X-420.00 Y40.00 F1200 ; Canh 3
G01 X-420.00 Y-40.00 F1200 ; Canh 4 khep kin
G00 Z205.00 F2500 ; Nhac but
G00 X-380.00 Y0.00 ; Ve tam san sang
M30
''',
    'circle': '''; --- Vong Tron Ban Kinh R=40mm ---
; Tam ve: X=-380.00, Y=0.00 | Cao do Z_draw=180.00, Z_lift=205.00
G21 ; mm
G90 ; Toa do tuyet doi
G00 Z205.00 F2500
G00 X-340.00 Y0.00 ; Diem khoi dau tren duong tron
G01 Z180.00 F1000
G02 X-340.00 Y0.00 I-40.00 J0.00 F1200 ; Cung tron 360 do kim dong ho
G00 Z205.00 F2500
G00 X-380.00 Y0.00
M30
''',
    'triangle': '''; --- Hinh Tam Giac Deu Canh 80mm ---
; Tam ve: X=-380.00, Y=0.00
G21
G90
G00 Z205.00 F2500
G00 X-340.00 Y0.00
G01 Z180.00 F1000
G01 X-400.00 Y-40.00 F1200
G01 X-400.00 Y40.00 F1200
G01 X-340.00 Y0.00 F1200
G00 Z205.00 F2500
G00 X-380.00 Y0.00
M30
''',
    'denso': '''; --- Khac Chu DENSO Don Net ---
; Tam ve: X=-380.00, Y=0.00
G21
G90
; Chu D
G00 Z205.00 F2500
G00 X-365.00 Y-55.00
G01 Z180.00 F1000
G01 X-395.00 Y-55.00 F1200
G01 X-395.00 Y-45.00
G01 X-380.00 Y-38.00
G01 X-365.00 Y-45.00
G01 X-365.00 Y-55.00
G00 Z205.00 F2500

; Chu E
G00 X-365.00 Y-30.00
G01 Z180.00 F1000
G01 X-395.00 Y-30.00 F1200
G01 X-395.00 Y-18.00
G00 Z205.00 F2500
G00 X-380.00 Y-30.00
G01 Z180.00 F1000
G01 X-380.00 Y-20.00
G00 Z205.00 F2500
G00 X-365.00 Y-18.00
G01 Z180.00 F1000
G01 X-365.00 Y-30.00
G00 Z205.00 F2500

; Chu N
G00 X-395.00 Y-10.00
G01 Z180.00 F1000
G01 X-365.00 Y-10.00 F1200
G01 X-395.00 Y5.00
G01 X-365.00 Y5.00
G00 Z205.00 F2500

; Chu S
G00 X-365.00 Y20.00
G01 Z180.00 F1000
G01 X-365.00 Y32.00 F1200
G01 X-380.00 Y32.00
G01 X-380.00 Y20.00
G01 X-395.00 Y20.00
G01 X-395.00 Y32.00
G00 Z205.00 F2500

; Chu O
G00 X-365.00 Y42.00
G01 Z180.00 F1000
G01 X-395.00 Y42.00 F1200
G01 X-395.00 Y55.00
G01 X-365.00 Y55.00
G01 X-365.00 Y42.00
G00 Z205.00 F2500

G00 X-380.00 Y0.00
M30
'''
}

# Sinh đường xoắn ốc Archimedes
_sp_lines = [
    '; --- Hoa Van Xoan Oc Archimedes ---',
    '; Tam ve: X=-380.00, Y=0.00',
    'G21',
    'G90',
    'G00 Z205.00 F2500'
]
for _i in range(120):
    _theta = _i * (8.0 * math.pi / 120.0)
    _r = 5.0 + (35.0 * (_i / 120.0))
    _x = -380.0 + _r * math.cos(_theta)
    _y = 0.0 + _r * math.sin(_theta)
    if _i == 0:
        _sp_lines.append(f'G00 X{_x:.2f} Y{_y:.2f}')
        _sp_lines.append('G01 Z180.00 F1000')
    else:
        _sp_lines.append(f'G01 X{_x:.2f} Y{_y:.2f} F1200')
_sp_lines.append('G00 Z205.00 F2500')
_sp_lines.append('G00 X-380.00 Y0.00')
_sp_lines.append('M30')
GCODE_TEMPLATES['spiral'] = '\n'.join(_sp_lines)


def normalize_gcode_for_denso(
    gcode_str: str,
    auto_center: bool = True,
    auto_z: bool = True,
    target_center_x: float = -380.0,
    target_center_y: float = 0.0,
    target_z_draw: float = 180.0,
    target_z_lift: float = 205.0
) -> Tuple[str, Dict[str, Any]]:
    """
    Chuẩn hóa tọa độ G-Code phù hợp với không gian làm việc của bàn vẽ robot Denso VS-6556.
    - Nếu auto_center=True và tâm bản vẽ ở gốc (0,0) hoặc ngoài vùng bàn vẽ, tự động bù offset
      về tâm bàn vẽ robot (X=-380mm, Y=0mm).
    - Nếu auto_z=True và cao độ Z nằm gần 0 (chuẩn CNC phay thông thường Z<=25),
      tự động chuẩn hóa cao độ hạ nét vẽ về target_z_draw (180mm) và nhấc bút về target_z_lift (205mm).
    """
    lines = gcode_str.splitlines()
    xs, ys, zs = [], [], []

    for line in lines:
        line_clean = line.split(';')[0].split('(')[0].strip().upper()
        if not line_clean:
            continue
        tokens = line_clean.split()
        for tok in tokens:
            if len(tok) > 1 and tok[0] in 'XYZ':
                try:
                    val = float(tok[1:])
                    if tok[0] == 'X': xs.append(val)
                    elif tok[0] == 'Y': ys.append(val)
                    elif tok[0] == 'Z': zs.append(val)
                except ValueError:
                    pass

    if not xs or not ys:
        return gcode_str, {'normalized': False, 'reason': 'Không tìm thấy tọa độ XY'}

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    orig_cx = (min_x + max_x) / 2.0
    orig_cy = (min_y + max_y) / 2.0
    width = max_x - min_x
    height = max_y - min_y

    dx = 0.0
    dy = 0.0
    if auto_center:
        if abs(orig_cx - target_center_x) > 30.0 or abs(orig_cy - target_center_y) > 30.0:
            dx = target_center_x - orig_cx
            dy = target_center_y - orig_cy

    apply_z = False
    z_thresh = 1.0
    if auto_z and zs:
        if min(zs) < 160.0 or max(zs) < 160.0:
            apply_z = True
            z_thresh = (min(zs) + max(zs)) / 2.0

    new_lines = []
    for line in lines:
        comment_part = ''
        code_part = line
        if ';' in line:
            code_part, comment_part = line.split(';', 1)
            comment_part = ';' + comment_part

        code_part = code_part.strip()
        if not code_part:
            new_lines.append(line)
            continue

        tokens = code_part.split()
        new_tokens = []
        for tok in tokens:
            tok_u = tok.upper()
            if tok_u.startswith('X') and len(tok_u) > 1:
                try:
                    val = float(tok_u[1:])
                    new_tokens.append(f'X{val + dx:.2f}')
                except ValueError:
                    new_tokens.append(tok)
            elif tok_u.startswith('Y') and len(tok_u) > 1:
                try:
                    val = float(tok_u[1:])
                    new_tokens.append(f'Y{val + dy:.2f}')
                except ValueError:
                    new_tokens.append(tok)
            elif tok_u.startswith('Z') and len(tok_u) > 1 and apply_z:
                try:
                    val = float(tok_u[1:])
                    new_z = target_z_draw if val <= z_thresh else target_z_lift
                    new_tokens.append(f'Z{new_z:.2f}')
                except ValueError:
                    new_tokens.append(tok)
            else:
                new_tokens.append(tok)

        reconstructed = ' '.join(new_tokens)
        if comment_part:
            reconstructed += ('  ' if reconstructed else '') + comment_part
        new_lines.append(reconstructed)

    meta = {
        'normalized': (dx != 0.0 or dy != 0.0 or apply_z),
        'orig_center': (orig_cx, orig_cy),
        'new_center': (orig_cx + dx, orig_cy + dy),
        'width': width,
        'height': height,
        'dx': dx,
        'dy': dy,
        'z_adapted': apply_z
    }
    return '\n'.join(new_lines), meta


class GCodeCatalogManager:
    """
    Quản lý danh mục các mục mã G-Code tùy chỉnh lưu trong drawing_samples/custom_gcode.
    Tự động khởi tạo các mục mẫu chuẩn nếu chưa có catalog.json.
    """
    def __init__(self, base_samples_dir: str):
        self.base_dir = base_samples_dir
        self.gcode_dir = os.path.join(base_samples_dir, 'custom_gcode')
        self.catalog_file = os.path.join(self.gcode_dir, 'catalog.json')
        os.makedirs(self.gcode_dir, exist_ok=True)
        self._ensure_starter_items()

    def _ensure_starter_items(self):
        if not os.path.exists(self.catalog_file):
            starter_items = [
                {
                    'id': 'starter_square_80',
                    'name': 'Hình vuông chuẩn (Square 80x80)',
                    'icon': '📐',
                    'filename': 'starter_square_80x80.gcode',
                    'created_at': 'Mặc định hệ thống',
                    'description': 'Quỹ đạo hình vuông 80x80mm tại tâm bàn vẽ X=-380, Y=0'
                },
                {
                    'id': 'starter_circle_r40',
                    'name': 'Vòng tròn kỹ thuật (Circle R=40)',
                    'icon': '⭕',
                    'filename': 'starter_circle_r40.gcode',
                    'created_at': 'Mặc định hệ thống',
                    'description': 'Quỹ đạo vòng tròn bán kính 40mm nội suy cung G02'
                },
                {
                    'id': 'starter_triangle_80',
                    'name': 'Hình tam giác đều (Triangle 80mm)',
                    'icon': '🔺',
                    'filename': 'starter_triangle_80.gcode',
                    'created_at': 'Mặc định hệ thống',
                    'description': 'Tam giác đều cạnh 80mm cân đối tại tâm bàn vẽ'
                },
                {
                    'id': 'starter_spiral',
                    'name': 'Hoa xoắn ốc Archimedes (Spiral)',
                    'icon': '🌀',
                    'filename': 'starter_spiral_archimedes.gcode',
                    'created_at': 'Mặc định hệ thống',
                    'description': 'Đường xoắn ốc nghệ thuật bán kính từ 5mm đến 40mm'
                },
                {
                    'id': 'starter_denso_text',
                    'name': 'Khắc chữ DENSO (Lettering)',
                    'icon': '⚡',
                    'filename': 'starter_denso_lettering.gcode',
                    'created_at': 'Mặc định hệ thống',
                    'description': 'Khắc chữ DENSO nét đơn giản trên bề mặt gia công'
                }
            ]
            self._write_starter_files()
            try:
                with open(self.catalog_file, 'w', encoding='utf-8') as f:
                    json.dump(starter_items, f, ensure_ascii=False, indent=2)
            except Exception as e:
                print(f'[WARN] Lỗi ghi catalog.json: {e}')

    def _write_starter_files(self):
        mapping = {
            'starter_square_80x80.gcode': GCODE_TEMPLATES['square'],
            'starter_circle_r40.gcode': GCODE_TEMPLATES['circle'],
            'starter_triangle_80.gcode': GCODE_TEMPLATES['triangle'],
            'starter_spiral_archimedes.gcode': GCODE_TEMPLATES['spiral'],
            'starter_denso_lettering.gcode': GCODE_TEMPLATES['denso']
        }
        for fn, text in mapping.items():
            fpath = os.path.join(self.gcode_dir, fn)
            try:
                with open(fpath, 'w', encoding='utf-8') as f:
                    f.write(text)
            except Exception as e:
                print(f'[WARN] Không thể tạo file mẫu {fn}: {e}')

    def load_items(self) -> List[Dict[str, Any]]:
        if not os.path.exists(self.catalog_file):
            self._ensure_starter_items()
        try:
            with open(self.catalog_file, 'r', encoding='utf-8') as f:
                items = json.load(f)
                return items if isinstance(items, list) else []
        except Exception as e:
            print(f'[WARN] Lỗi đọc catalog.json: {e}')
            return []

    def save_item(self, name: str, icon: str, gcode_text: str, description: str = '') -> Dict[str, Any]:
        items = self.load_items()
        slug = re.sub(r'[^a-zA-Z0-9_\-]', '_', name.strip()).strip('_').lower()
        if not slug:
            slug = 'custom_gcode'
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{slug}_{timestamp_str}.gcode'
        file_path = os.path.join(self.gcode_dir, filename)

        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(gcode_text)

        item_id = f'item_{int(time.time() * 1000)}'
        new_item = {
            'id': item_id,
            'name': name.strip(),
            'icon': icon.strip(),
            'filename': filename,
            'created_at': datetime.now().strftime('%Y-%m-%d %H:%M'),
            'description': description.strip()
        }
        items.append(new_item)

        try:
            with open(self.catalog_file, 'w', encoding='utf-8') as f:
                json.dump(items, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f'[WARN] Lỗi ghi catalog.json: {e}')

        return new_item

    def delete_item(self, item_id: str) -> bool:
        items = self.load_items()
        target = None
        for it in items:
            if it.get('id') == item_id:
                target = it
                break
        if not target:
            return False

        filename = target.get('filename')
        if filename:
            file_path = os.path.join(self.gcode_dir, filename)
            if os.path.exists(file_path):
                try:
                    os.remove(file_path)
                except Exception as e:
                    print(f'[WARN] Không thể xóa file {file_path}: {e}')

        items = [it for it in items if it.get('id') != item_id]
        try:
            with open(self.catalog_file, 'w', encoding='utf-8') as f:
                json.dump(items, f, ensure_ascii=False, indent=2)
            return True
        except Exception as e:
            print(f'[WARN] Lỗi cập nhật catalog.json sau khi xóa: {e}')
            return False

    def get_gcode_content(self, filename: str) -> Optional[str]:
        file_path = os.path.join(self.gcode_dir, filename)
        if os.path.exists(file_path):
            with open(file_path, 'r', encoding='utf-8') as f:
                return f.read()
        return None


class NewGCodeDialog(QDialog):
    """
    Giao diện thêm mã G-Code mới và lưu thành 1 mục mẫu trong denso_gazebo_control.
    """
    def __init__(self, parent=None, catalog_mgr=None, initial_gcode='', suggested_name=''):
        super().__init__(parent)
        self.catalog_mgr = catalog_mgr
        self.saved_item: Optional[Dict[str, Any]] = None
        self.setWindowTitle('✨ Thêm Mục Mã G-Code Mới - Denso Robot Control')
        self.resize(880, 720)
        self.setStyleSheet('''
            QDialog {
                background-color: #0b1120;
                color: #f8fafc;
                font-family: 'Segoe UI', system-ui, sans-serif;
            }
            QLabel {
                color: #cbd5e1;
                font-size: 12px;
            }
            QLineEdit {
                background-color: #050b14;
                color: #f8fafc;
                border: 1px solid #334155;
                border-radius: 5px;
                padding: 6px 10px;
                font-size: 12px;
            }
            QLineEdit:focus {
                border-color: #38bdf8;
            }
            QComboBox {
                background-color: #0f172a;
                color: #f8fafc;
                border: 1px solid #334155;
                border-radius: 5px;
                padding: 5px 10px;
                font-size: 12px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox QAbstractItemView {
                background-color: #0f172a;
                color: #f8fafc;
                selection-background-color: #0284c7;
                border: 1px solid #334155;
            }
            QTextEdit {
                background-color: #050b14;
                color: #4ade80;
                font-family: 'Consolas', 'DejaVu Sans Mono', monospace;
                font-size: 11px;
                border: 1px solid #1e293b;
                border-radius: 5px;
                padding: 6px;
            }
            QGroupBox {
                border: 1px solid #1e293b;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                font-weight: bold;
                color: #38bdf8;
                font-size: 11px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 4px;
            }
            QCheckBox {
                color: #cbd5e1;
                font-size: 11px;
                spacing: 6px;
            }
        ''')

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)

        # 1. Header Banner
        header = QFrame()
        header.setStyleSheet('''
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0f172a, stop:1 #1e293b);
                border: 1px solid #334155;
                border-radius: 8px;
                padding: 8px;
            }
        ''')
        h_lay = QVBoxLayout(header)
        h_lay.setContentsMargins(8, 4, 8, 4)
        h_lay.setSpacing(2)

        title_lbl = QLabel('✨ THÊM MỤC MÃ G-CODE MỚI VÀO BẢNG ĐIỀU KHIỂN DENSO')
        title_lbl.setStyleSheet('color: #38bdf8; font-size: 14px; font-weight: bold;')
        sub_lbl = QLabel('Nhập hoặc nạp mã G-Code, chuẩn hóa tọa độ và lưu thành một mục mẫu có sẵn trong denso_gazebo_control.')
        sub_lbl.setStyleSheet('color: #94a3b8; font-size: 11px;')
        h_lay.addWidget(title_lbl)
        h_lay.addWidget(sub_lbl)
        layout.addWidget(header)

        # 2. Form Tên mục & Icon
        form_box = QGroupBox('📝 THÔNG TIN MỤC MẪU (ITEM INFO)')
        form_lay = QGridLayout(form_box)
        form_lay.setContentsMargins(10, 10, 10, 10)
        form_lay.setSpacing(8)

        form_lay.addWidget(QLabel('🏷️ Tên mục hiển thị:'), 0, 0)
        self.txt_name = QLineEdit()
        self.txt_name.setPlaceholderText('VD: Logo Khắc Kim Loại, Bàn Cờ 8x8, Quỹ Đạo Thử Nghiệm...')
        if suggested_name:
            self.txt_name.setText(suggested_name)
        form_lay.addWidget(self.txt_name, 0, 1)

        form_lay.addWidget(QLabel('Biểu tượng (Icon):'), 0, 2)
        self.combo_icon = QComboBox()
        self.combo_icon.addItem('📜 Mã G-Code Tùy Biến', '📜')
        self.combo_icon.addItem('📐 Hình Học Kỹ Thuật (Geometry)', '📐')
        self.combo_icon.addItem('⚡ Biểu Tượng / Logo', '⚡')
        self.combo_icon.addItem('🖋️ Khắc Chữ / Nghệ Thuật', '🖋️')
        self.combo_icon.addItem('⚙️ Cơ Khí & Toolpath', '⚙️')
        self.combo_icon.addItem('🌟 Hoa Văn / Họa Tiết', '🌟')
        self.combo_icon.addItem('🎯 Quỹ Đạo Kiểm Tra', '🎯')
        self.combo_icon.addItem('⭕ Vòng Tròn / Đường Cong', '⭕')
        self.combo_icon.addItem('🔺 Hình Tam Giác', '🔺')
        self.combo_icon.addItem('🌀 Hoa Xoắn Ốc', '🌀')
        form_lay.addWidget(self.combo_icon, 0, 3)

        layout.addWidget(form_box)

        # 3. Khu vực soạn thảo / nạp G-code
        editor_box = QGroupBox('💻 NỘI DUNG MÃ G-CODE (G-CODE CONTENT)')
        ed_lay = QVBoxLayout(editor_box)
        ed_lay.setContentsMargins(10, 10, 10, 10)
        ed_lay.setSpacing(6)

        tb_lay = QHBoxLayout()
        tb_lay.setSpacing(6)

        btn_load = QPushButton('📂 Nạp Từ File (.gcode, .nc, .txt)...')
        btn_load.setStyleSheet('''
            QPushButton {
                background-color: #1e293b; color: #38bdf8; border: 1px solid #0284c7;
                border-radius: 4px; padding: 5px 10px; font-weight: bold; font-size: 11px;
            }
            QPushButton:hover { background-color: #0369a1; color: white; }
        ''')
        btn_load.clicked.connect(self._load_from_file)
        tb_lay.addWidget(btn_load)

        btn_paste = QPushButton('📋 Dán Từ Clipboard')
        btn_paste.setStyleSheet('''
            QPushButton {
                background-color: #1e293b; color: #cbd5e1; border: 1px solid #334155;
                border-radius: 4px; padding: 5px 10px; font-size: 11px;
            }
            QPushButton:hover { background-color: #334155; color: white; }
        ''')
        btn_paste.clicked.connect(self._paste_clipboard)
        tb_lay.addWidget(btn_paste)

        tb_lay.addSpacing(10)
        tb_lay.addWidget(QLabel('Chèn mẫu nhanh:'))
        self.combo_templates = QComboBox()
        self.combo_templates.addItem('-- Chọn mẫu nhanh --', '')
        self.combo_templates.addItem('⏹️ Hình vuông 80x80 mm', 'square')
        self.combo_templates.addItem('⭕ Vòng tròn R=40 mm', 'circle')
        self.combo_templates.addItem('🔺 Tam giác đều cạnh 80mm', 'triangle')
        self.combo_templates.addItem('🌀 Hoa xoắn ốc Archimedes', 'spiral')
        self.combo_templates.addItem('⚡ Chữ DENSO đơn nét', 'denso')
        self.combo_templates.currentIndexChanged.connect(self._apply_template)
        tb_lay.addWidget(self.combo_templates)

        btn_clear = QPushButton('🧹 Xóa Trống')
        btn_clear.setStyleSheet('''
            QPushButton {
                background-color: #1e293b; color: #94a3b8; border: 1px solid #334155;
                border-radius: 4px; padding: 5px 8px; font-size: 11px;
            }
            QPushButton:hover { background-color: #7f1d1d; color: #f87171; border-color: #ef4444; }
        ''')
        btn_clear.clicked.connect(self._clear_editor)
        tb_lay.addWidget(btn_clear)

        tb_lay.addStretch(1)
        ed_lay.addLayout(tb_lay)

        self.txt_editor = QTextEdit()
        if initial_gcode:
            self.txt_editor.setPlainText(initial_gcode)
        else:
            self.txt_editor.setPlainText(GCODE_TEMPLATES['square'])
        ed_lay.addWidget(self.txt_editor)
        layout.addWidget(editor_box, 1)

        # 4. Box Tùy chọn chuẩn hóa tọa độ bàn vẽ Denso
        opt_box = QGroupBox('🛠️ TÙY CHỌN CHUẨN HÓA TỌA ĐỘ BÀN VẼ ROBOT DENSO')
        opt_lay = QHBoxLayout(opt_box)
        opt_lay.setContentsMargins(10, 8, 10, 8)
        opt_lay.setSpacing(12)

        self.chk_auto_center = QCheckBox('🎯 Tự động căn tâm về bàn vẽ (X=-380, Y=0) nếu vẽ tại gốc (0,0)')
        self.chk_auto_center.setChecked(True)
        self.chk_auto_center.setToolTip('Nếu G-code vẽ tại gốc (0,0), tự động bù offset để tâm hình trùng tâm bàn vẽ Denso X=-380mm, Y=0mm')
        opt_lay.addWidget(self.chk_auto_center)

        self.chk_auto_z = QCheckBox('📏 Tự động chuẩn hóa cao độ Z về bàn vẽ (Z_draw=180, Z_lift=205)')
        self.chk_auto_z.setChecked(True)
        self.chk_auto_z.setToolTip('Chuẩn hóa cao độ hạ bút và nhấc bút phù hợp với mặt bàn Gazebo (180mm)')
        opt_lay.addWidget(self.chk_auto_z)

        btn_analyze = QPushButton('🔍 Phân Tích & Xem Trước')
        btn_analyze.setStyleSheet('''
            QPushButton {
                background-color: #0284c7; color: white; border: none;
                border-radius: 4px; padding: 6px 14px; font-weight: bold; font-size: 11px;
            }
            QPushButton:hover { background-color: #0369a1; }
        ''')
        btn_analyze.clicked.connect(self._analyze_gcode)
        opt_lay.addWidget(btn_analyze)

        layout.addWidget(opt_box)

        # 5. Khung hiển thị kết quả phân tích
        self.preview_frame = QFrame()
        self.preview_frame.setStyleSheet('''
            QFrame {
                background-color: #050b14;
                border: 1px solid #1e293b;
                border-radius: 6px;
                padding: 6px;
            }
        ''')
        pf_lay = QHBoxLayout(self.preview_frame)
        pf_lay.setContentsMargins(8, 4, 8, 4)
        pf_lay.setSpacing(10)

        self.lbl_stats_summary = QLabel('📊 Nhấn "Phân Tích & Xem Trước" để kiểm tra cú pháp và kích thước quỹ đạo.')
        self.lbl_stats_summary.setStyleSheet('color: #94a3b8; font-size: 11px;')
        pf_lay.addWidget(self.lbl_stats_summary, 1)

        self.lbl_safety_badge = QLabel('⚡ Sẵn sàng phân tích')
        self.lbl_safety_badge.setStyleSheet('color: #38bdf8; font-weight: bold; font-size: 11px;')
        pf_lay.addWidget(self.lbl_safety_badge)

        layout.addWidget(self.preview_frame)

        # 6. Hàng nút điều khiển dưới cùng
        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(10)

        self.chk_select_now = QCheckBox('✅ Tự động chọn và nạp mục này vào bàn vẽ ngay sau khi lưu')
        self.chk_select_now.setChecked(True)
        bottom_row.addWidget(self.chk_select_now)

        bottom_row.addStretch(1)

        btn_cancel = QPushButton('❌ Hủy Bỏ')
        btn_cancel.setStyleSheet('''
            QPushButton {
                background-color: #1e293b; color: #cbd5e1; border: 1px solid #334155;
                border-radius: 5px; padding: 8px 16px; font-size: 12px; font-weight: bold;
            }
            QPushButton:hover { background-color: #334155; color: white; }
        ''')
        btn_cancel.clicked.connect(self.reject)
        bottom_row.addWidget(btn_cancel)

        self.btn_save = QPushButton('💾 Lưu Thành Mục Mới')
        self.btn_save.setStyleSheet('''
            QPushButton {
                background-color: #059669; color: white; border: none;
                border-radius: 5px; padding: 8px 20px; font-size: 12px; font-weight: bold;
            }
            QPushButton:hover { background-color: #047857; }
        ''')
        self.btn_save.clicked.connect(self._save_item)
        bottom_row.addWidget(self.btn_save)

        layout.addLayout(bottom_row)

        # Phân tích ban đầu
        self._analyze_gcode()

    def _load_from_file(self):
        start_dir = self.catalog_mgr.base_dir if self.catalog_mgr else '/home/dung/denso_ws/drawing_samples'
        fn, _ = QFileDialog.getOpenFileName(
            self, 'Chọn Tệp Mã G-Code', start_dir,
            'G-Code Files (*.gcode *.nc *.ngc *.tap *.txt);;All Files (*)'
        )
        if fn:
            try:
                with open(fn, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                self.txt_editor.setPlainText(content)
                if not self.txt_name.text().strip():
                    base = os.path.splitext(os.path.basename(fn))[0]
                    self.txt_name.setText(base.replace('_', ' ').replace('-', ' ').title())
                self._analyze_gcode()
            except Exception as e:
                QMessageBox.warning(self, 'Lỗi', f'Không thể đọc file: {e}')

    def _paste_clipboard(self):
        cb = QApplication.clipboard()
        text = cb.text()
        if text.strip():
            self.txt_editor.setPlainText(text)
            self._analyze_gcode()
        else:
            QMessageBox.information(self, 'Thông báo', 'Bộ nhớ tạm (Clipboard) không có văn bản!')

    def _clear_editor(self):
        self.txt_editor.clear()
        self.lbl_stats_summary.setText('Đã xóa sạch trình soạn thảo.')
        self.lbl_safety_badge.setText('Trống')
        self.lbl_safety_badge.setStyleSheet('color: #94a3b8;')

    def _apply_template(self, idx):
        key = self.combo_templates.currentData()
        if key and key in GCODE_TEMPLATES:
            self.txt_editor.setPlainText(GCODE_TEMPLATES[key])
            if not self.txt_name.text().strip():
                display_name = self.combo_templates.currentText().split('(')[0].strip()
                self.txt_name.setText(display_name)
            self._analyze_gcode()
        self.combo_templates.setCurrentIndex(0)

    def _analyze_gcode(self):
        text = self.txt_editor.toPlainText().strip()
        if not text:
            self.lbl_stats_summary.setText('Trình soạn thảo chưa có nội dung.')
            self.lbl_safety_badge.setText('Trống')
            self.lbl_safety_badge.setStyleSheet('color: #94a3b8;')
            return

        lines = text.splitlines()
        xs, ys, zs = [], [], []
        g_cmd_count = 0

        for line in lines:
            line_c = line.split(';')[0].split('(')[0].strip().upper()
            if not line_c: continue
            toks = line_c.split()
            has_cmd = False
            for tok in toks:
                if tok.startswith('G') or tok.startswith('M'):
                    has_cmd = True
                elif len(tok) > 1:
                    try:
                        v = float(tok[1:])
                        if tok[0] == 'X': xs.append(v)
                        elif tok[0] == 'Y': ys.append(v)
                        elif tok[0] == 'Z': zs.append(v)
                    except ValueError:
                        pass
            if has_cmd:
                g_cmd_count += 1

        if not xs or not ys:
            self.lbl_stats_summary.setText(f'Tìm thấy {len(lines)} dòng ({g_cmd_count} lệnh), nhưng không phát hiện tọa độ X/Y.')
            self.lbl_safety_badge.setText('⚠️ Thiếu tọa độ X/Y')
            self.lbl_safety_badge.setStyleSheet('color: #fbbf24; font-weight: bold;')
            return

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0
        w = max_x - min_x
        h = max_y - min_y
        z_min = min(zs) if zs else 180.0
        z_max = max(zs) if zs else 205.0

        if self.chk_auto_center.isChecked() and (abs(cx - -380.0) > 30.0 or abs(cy) > 30.0):
            target_cx, target_cy = -380.0, 0.0
        else:
            target_cx, target_cy = cx, cy

        box_min_x = target_cx - w / 2.0
        box_max_x = target_cx + w / 2.0
        box_min_y = target_cy - h / 2.0
        box_max_y = target_cy + h / 2.0

        is_safe = (-530.0 <= box_min_x <= -230.0) and (-200.0 <= box_min_y) and (box_max_y <= 200.0)

        stats_str = (
            f"📊 {len(lines)} dòng ({g_cmd_count} lệnh G/M) | Kích thước: "
            f"{w:.1f} x {h:.1f} mm | Tâm: ("
            f"{target_cx:.1f}, {target_cy:.1f}) mm | Z: ["
            f"{z_min:.1f} .. {z_max:.1f}] mm"
        )
        self.lbl_stats_summary.setText(stats_str)

        if is_safe:
            self.lbl_safety_badge.setText("✅ Trong vùng an toàn bàn vẽ Denso")
            self.lbl_safety_badge.setStyleSheet("color: #4ade80; font-weight: bold;")
        else:
            self.lbl_safety_badge.setText("⚠️ Vượt ngoài vùng an toàn bàn vẽ")
            self.lbl_safety_badge.setStyleSheet("color: #f87171; font-weight: bold;")

    def _on_save_clicked(self):
        name = self.txt_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Thiếu thông tin", "Vui lòng nhập tên mục hiển thị!")
            self.txt_name.setFocus()
            return

        gcode_text = self.txt_editor.toPlainText().strip()
        if not gcode_text:
            QMessageBox.warning(self, "Thiếu thông tin", "Nội dung mã G-Code không được để trống!")
            return

        icon = self.combo_icon.currentData() or "📜"

        norm_code = gcode_text
        if self.chk_auto_center.isChecked() or self.chk_auto_z.isChecked():
            norm_code, meta = normalize_gcode_for_denso(
                gcode_text,
                auto_center=self.chk_auto_center.isChecked(),
                auto_z=self.chk_auto_z.isChecked(),
                target_center_x=-380.0,
                target_center_y=0.0,
                target_z_draw=180.0,
                target_z_lift=205.0
            )

        if not self.catalog_mgr:
            self.catalog_mgr = GCodeCatalogManager("/home/dung/denso_ws/drawing_samples")

        new_item = self.catalog_mgr.save_item(name, icon, norm_code)
        self.saved_item = new_item
        self.accept()
