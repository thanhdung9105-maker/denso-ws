import sys, os, re

file_path = '/home/dung/denso_ws/src/denso_gazebo/scripts/denso_trajectory_benchmark_tab.py'
with open(file_path, 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Preset test cases
old_preset = '''PRESET_TEST_CASES = [
    {
        "id": "case_1_nature2026_clutter",
        "name": "⚡ Case 1 (Chuẩn Nature 2026): Vượt 3 Vật cản hình cầu trên bàn (Tabletop 3-Sphere Clutter)",
        "desc": "Kịch bản cốt lõi bài báo Nature 2026: Bi-RRT giật góc nhọn J=2.23e6 vs GWO B-Spline C2 dập tắt 95.5% Jerk",
        "start": [-330.0, -120.0, 220.0],
        "end": [-450.0,  120.0, 220.0],
        "obstacles": [
            {"center": [-0.360, -0.050, 0.235], "radius": 0.035},
            {"center": [-0.390,  0.020, 0.245], "radius": 0.040},
            {"center": [-0.420,  0.080, 0.235], "radius": 0.035}
        ],
        "waypoints_mm": None
    },'''

new_preset = '''DEFAULT_TABLETOP_OBSTACLES = [
    {"center": [-0.360, -0.050, 0.235], "radius": 0.035},
    {"center": [-0.390,  0.020, 0.245], "radius": 0.040},
    {"center": [-0.420,  0.080, 0.235], "radius": 0.035}
]

PRESET_TEST_CASES = [
    {
        "id": "case_1_nature2026_clutter",
        "name": "⚡ Case 1 (Chuẩn Nature 2026): Vượt 3 Vật cản hình cầu trên bàn (Tabletop 3-Sphere Clutter)",
        "desc": "Kịch bản cốt lõi bài báo Nature 2026: Bi-RRT giật góc nhọn J=2.23e6 vs GWO B-Spline C2 dập tắt 95.5% Jerk",
        "start": [-330.0, -120.0, 220.0],
        "end": [-450.0,  120.0, 220.0],
        "obstacles": DEFAULT_TABLETOP_OBSTACLES,
        "waypoints_mm": None
    },
    {
        "id": "case_gcode_contour",
        "name": "🎨 Case G-Code (Bàn Vẽ Phẳng): Khắc Biên Dạng Chữ D (DENSO CNC Contour)",
        "desc": "Quỹ đạo G-code phay khắc biên dạng phẳng (Z=205mm): Tắt vật cản để vẽ mượt C2 hoặc bật vật cản để robot né tránh",
        "start": [-330.0, -90.0, 205.0],
        "end": [-330.0, -90.0, 205.0],
        "waypoints_mm": [
            [-330.0, -90.0, 205.0],
            [-330.0,  90.0, 205.0],
            [-375.0,  90.0, 205.0],
            [-435.0,  50.0, 205.0],
            [-435.0, -50.0, 205.0],
            [-375.0, -90.0, 205.0],
            [-330.0, -90.0, 205.0]
        ],
        "obstacles": None
    },'''

assert old_preset in text, "old_preset not found"
text = text.replace(old_preset, new_preset)
print("P1: Preset updated")

# 2. Chart 1 Header and obstacles in ChartCanvas
old_chart1 = '''        self._draw_box_header(
            painter, rect,
            "1. QUỸ ĐẠO MŨI TCP TRÊN BÀN (Y-X)",
            "Cam: ROS 2 / Bi-RRT (Gãy C0, J=2.23e6) | Xanh: GWO Nature 2026 (Bo mượt C2, J=1.00e5)"
        )'''

new_chart1 = '''        obs_list = self.res_iga.get("obstacles", []) if self.res_iga else []
        sub_title = (
            "Cam: ROS 2 / Bi-RRT (Gãy C0, J=2.23e6) | Xanh: GWO Nature 2026 (Né 3 vật cản C², J=1.00e5)"
            if obs_list else
            "Cam: ROS 2 (Gấp khúc C0) | Xanh: GWO B-Spline C² (Mượt mà - Vẽ G-code tự do)"
        )
        self._draw_box_header(
            painter, rect,
            "1. QUỸ ĐẠO MŨI TCP TRÊN BÀN (Y-X)",
            sub_title
        )'''

assert old_chart1 in text, "old_chart1 not found"
text = text.replace(old_chart1, new_chart1)
print("P2: Chart1 header updated")

# 2b. Chart 1 obstacle rendering enhancement
old_obs_draw = '''        # Draw 3 Spherical Obstacles (Nature 2026) only if present in scenario
        obs_list = self.res_iga.get("obstacles", [])
        if obs_list:
            for obs in obs_list:
                c = np.array(obs["center"]) * 1000.0
                r = obs["radius"] * 1000.0
                sc = to_screen(c)
                rx = max(8.0, (r / dy) * (plot_r.width() - 12))
                ry = max(8.0, (r / dx) * (plot_r.height() - 12))
                r_pix = max(rx, ry)
                # Translucent Pink Obstacle
                painter.setBrush(QColor(236, 72, 153, 95))
                painter.setPen(QPen(QColor("#f43f5e"), 1.2, Qt.DashLine))
                painter.drawEllipse(sc, r_pix, r_pix)'''

new_obs_draw = '''        # Draw 3 Spherical Obstacles (Nature 2026) only if present in scenario
        obs_list = self.res_iga.get("obstacles", []) if self.res_iga else []
        if obs_list:
            for idx_o, obs in enumerate(obs_list):
                c = np.array(obs["center"]) * 1000.0
                r = obs["radius"] * 1000.0
                sc = to_screen(c)
                rx = max(8.0, (r / dy) * (plot_r.width() - 12))
                ry = max(8.0, (r / dx) * (plot_r.height() - 12))
                r_pix = max(rx, ry)
                # Translucent Crimson/Pink Obstacle
                painter.setBrush(QColor(244, 63, 94, 90))
                painter.setPen(QPen(QColor("#f43f5e"), 1.3, Qt.DashLine))
                painter.drawEllipse(sc, r_pix, r_pix)
                # Label
                painter.setPen(QColor("#fda4af"))
                painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
                painter.drawText(QRectF(sc.x() - 25, sc.y() - 7, 50, 14), Qt.AlignCenter, f"Vật cản {idx_o+1}")
        else:
            # Draw subtle badge: Free-space / G-Code Mode
            painter.setPen(QPen(QColor("#059669"), 1))
            painter.setBrush(QColor(6, 78, 59, 140))
            badge_r = QRectF(plot_r.right() - 165, plot_r.top() + 4, 160, 16)
            painter.drawRoundedRect(badge_r, 3, 3)
            painter.setPen(QColor("#34d399"))
            painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
            painter.drawText(badge_r, Qt.AlignCenter, "✨ Không gian tự do / Vẽ G-Code")'''

assert old_obs_draw in text, "old_obs_draw not found"
text = text.replace(old_obs_draw, new_obs_draw)
print("P3: Chart1 obstacle drawing updated")

# 3. BenchmarkWorker obstacle handling
old_worker_obs = '''            if self.waypoints_mm is not None:
                init_wps_m = [np.array(p, dtype=float) / 1000.0 for p in self.waypoints_mm]
                p_start = init_wps_m[0]
                p_end = init_wps_m[-1]
                if self.obstacles_def:
                    obs_list = [SphericalObstacle(center=o["center"], radius=o["radius"]) for o in self.obstacles_def]
            else:
                p_start = self.p_start_m
                p_end = self.p_end_m
                if self.obstacles_def:
                    obs_list = [SphericalObstacle(center=o["center"], radius=o["radius"]) for o in self.obstacles_def]
                else:
                    obs_list = [
                        SphericalObstacle(center=[-0.360, -0.050, 0.235], radius=0.035, safety_margin=0.025),
                        SphericalObstacle(center=[-0.390,  0.020, 0.245], radius=0.040, safety_margin=0.025),
                        SphericalObstacle(center=[-0.420,  0.080, 0.235], radius=0.035, safety_margin=0.025),
                    ]'''

new_worker_obs = '''            if self.waypoints_mm is not None:
                init_wps_m = [np.array(p, dtype=float) / 1000.0 for p in self.waypoints_mm]
                p_start = init_wps_m[0]
                p_end = init_wps_m[-1]
            else:
                p_start = self.p_start_m
                p_end = self.p_end_m

            if self.obstacles_def is not None and len(self.obstacles_def) > 0:
                obs_list = [SphericalObstacle(center=o["center"], radius=o["radius"]) for o in self.obstacles_def]
            else:
                obs_list = []'''

assert old_worker_obs in text, "old_worker_obs not found"
text = text.replace(old_worker_obs, new_worker_obs)
print("P4: Worker obstacle logic updated")

# 4. In __init__ of DensoTrajectoryBenchmarkTab
old_init = '''        # MẶC ĐỊNH ẨN ĐƯỜNG 3D TRÊN GAZEBO ĐỂ KHÔNG ĐÈ LÊN BẢN VẼ G-CODE (TAB 5)
        self.marker_visible = False
        self.show_ros2_trail = False
        self.show_iga_trail = False

        self.gz_node = None'''

new_init = '''        # MẶC ĐỊNH ẨN ĐƯỜNG 3D TRÊN GAZEBO ĐỂ KHÔNG ĐÈ LÊN BẢN VẼ G-CODE (TAB 5)
        self.marker_visible = False
        self.show_ros2_trail = False
        self.show_iga_trail = False
        self.obstacles_enabled = True

        self.gz_node = None'''

assert old_init in text, "old_init not found"
text = text.replace(old_init, new_init)
print("P5: __init__ updated")

# 5. Timer sync obstacles after startup
old_timer_init = '''        self.init_ui()
        QTimer.singleShot(300, lambda: self.clear_trajectory_marker(silent=True))
        self._start_benchmark()'''

new_timer_init = '''        self.init_ui()
        QTimer.singleShot(300, lambda: self.clear_trajectory_marker(silent=True))
        QTimer.singleShot(600, self.sync_obstacles_to_gazebo_and_rviz)
        self._start_benchmark()'''

assert old_timer_init in text, "old_timer_init not found"
text = text.replace(old_timer_init, new_timer_init)
print("P6: Startup timers updated")

# 6. Banner obstacle button
old_banner_btn = '''        self.btn_toggle_marker = QPushButton("👁️ ĐƯỜNG MẪU 3D GAZEBO: ĐANG ẨN (Bấm để bật)")
        self.btn_toggle_marker.setCheckable(True)
        self.btn_toggle_marker.setChecked(False)
        self._update_marker_btn_style(False)
        self.btn_toggle_marker.clicked.connect(self._on_toggle_marker_clicked)
        b_layout.addWidget(self.btn_toggle_marker)

        main_layout.addWidget(banner)'''

new_banner_btn = '''        self.btn_toggle_marker = QPushButton("👁️ ĐƯỜNG MẪU 3D GAZEBO: ĐANG ẨN (Bấm để bật)")
        self.btn_toggle_marker.setCheckable(True)
        self.btn_toggle_marker.setChecked(False)
        self._update_marker_btn_style(False)
        self.btn_toggle_marker.clicked.connect(self._on_toggle_marker_clicked)
        b_layout.addWidget(self.btn_toggle_marker)

        self.btn_toggle_obstacles = QPushButton("🛡️ VẬT CẢN 3D: ĐANG BẬT (Bấm để Tắt)")
        self.btn_toggle_obstacles.clicked.connect(self._on_toggle_obstacles_clicked)
        self._update_obstacle_btn_style(self.obstacles_enabled)
        b_layout.addWidget(self.btn_toggle_obstacles)

        main_layout.addWidget(banner)'''

assert old_banner_btn in text, "old_banner_btn not found"
text = text.replace(old_banner_btn, new_banner_btn)
print("P7: Banner obstacle button updated")

# 7. Row 0 Test Case Selector & Obstacle Checkbox
old_row0 = '''        # Row 0: Test Case Selector
        lbl_case = QLabel("Kịch bản hình học:")
        lbl_case.setStyleSheet("color: #e2e8f0; font-weight: bold; font-size: 11px;")
        self.combo_preset = QComboBox()
        self.combo_preset.setStyleSheet("""
            QComboBox {
                background-color: #1e293b; color: #38bdf8; font-weight: bold;
                padding: 2px 6px; border: 1px solid #475569; border-radius: 4px; font-size: 11px;
            }
        """)
        for tc in PRESET_TEST_CASES:
            self.combo_preset.addItem(tc["name"], tc)
        self.combo_preset.currentIndexChanged.connect(self._on_preset_changed)

        btn_use_current = QPushButton("📍 Lấy vị trí hiện tại làm P0")
        btn_use_current.setStyleSheet("""
            QPushButton {
                background-color: #334155; color: #f8fafc; font-weight: bold;
                padding: 2px 6px; border-radius: 4px; border: 1px solid #64748b; font-size: 11px;
            }
            QPushButton:hover { background-color: #475569; }
        """)
        btn_use_current.clicked.connect(self._set_current_as_start)

        cfg_layout.addWidget(lbl_case, 0, 0)
        cfg_layout.addWidget(self.combo_preset, 0, 1, 1, 3)
        cfg_layout.addWidget(btn_use_current, 0, 4)'''

new_row0 = '''        # Row 0: Test Case Selector & Obstacle Checkbox
        lbl_case = QLabel("Kịch bản hình học:")
        lbl_case.setStyleSheet("color: #e2e8f0; font-weight: bold; font-size: 11px;")
        self.combo_preset = QComboBox()
        self.combo_preset.setStyleSheet("""
            QComboBox {
                background-color: #1e293b; color: #38bdf8; font-weight: bold;
                padding: 2px 6px; border: 1px solid #475569; border-radius: 4px; font-size: 11px;
            }
        """)
        for tc in PRESET_TEST_CASES:
            self.combo_preset.addItem(tc["name"], tc)
        self.combo_preset.currentIndexChanged.connect(self._on_preset_changed)

        self.chk_obstacles = QCheckBox("🛡️ 3 Vật Cản 3D trên bàn (Gazebo)")
        self.chk_obstacles.setStyleSheet("""
            QCheckBox {
                color: #f87171; font-weight: bold; font-size: 11px; spacing: 4px;
            }
            QCheckBox::indicator { width: 14px; height: 14px; }
        """)
        self.chk_obstacles.setChecked(self.obstacles_enabled)
        self.chk_obstacles.toggled.connect(self._on_chk_obstacles_toggled)

        btn_use_current = QPushButton("📍 Lấy vị trí P0")
        btn_use_current.setStyleSheet("""
            QPushButton {
                background-color: #334155; color: #f8fafc; font-weight: bold;
                padding: 2px 6px; border-radius: 4px; border: 1px solid #64748b; font-size: 11px;
            }
            QPushButton:hover { background-color: #475569; }
        """)
        btn_use_current.clicked.connect(self._set_current_as_start)

        cfg_layout.addWidget(lbl_case, 0, 0)
        cfg_layout.addWidget(self.combo_preset, 0, 1, 1, 2)
        cfg_layout.addWidget(self.chk_obstacles, 0, 3)
        cfg_layout.addWidget(btn_use_current, 0, 4)'''

assert old_row0 in text, "old_row0 not found"
text = text.replace(old_row0, new_row0)
print("P8: Row0 controls updated")

# 8. Active Test Case Badge Card (Right below cfg_group)
old_post_cfg = '''        self.lbl_status = QLabel("Trạng thái: Đang khởi tạo bộ so sánh 6-DOF...")
        self.lbl_status.setStyleSheet("color: #38bdf8; font-style: italic; font-size: 11px;")
        cfg_layout.addWidget(self.lbl_status, 4, 1, 1, 4)

        main_layout.addWidget(cfg_group)

        # -------------------------------------------------------------
        # 3. KPI SUMMARY CARDS (Chuẩn Nature Scientific Reports 2026)'''

new_post_cfg = '''        self.lbl_status = QLabel("Trạng thái: Đang khởi tạo bộ so sánh 6-DOF...")
        self.lbl_status.setStyleSheet("color: #38bdf8; font-style: italic; font-size: 11px;")
        cfg_layout.addWidget(self.lbl_status, 4, 1, 1, 4)

        main_layout.addWidget(cfg_group)

        # -------------------------------------------------------------
        # THẺ THÔNG TIN TRƯỜNG HỢP TEST (ACTIVE TEST CASE BADGE)
        # -------------------------------------------------------------
        self.badge_case_card = QFrame()
        self.badge_case_card.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0f172a, stop:0.5 #1e293b, stop:1 #0f172a);
                border: 1px solid #38bdf8;
                border-radius: 6px;
                padding: 4px 10px;
            }
        """)
        badge_layout = QHBoxLayout(self.badge_case_card)
        badge_layout.setContentsMargins(8, 3, 8, 3)

        self.lbl_case_name = QLabel("⚡ Case 1: Vượt 3 Vật cản hình cầu trên bàn (Tabletop 3-Sphere Clutter)")
        self.lbl_case_name.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px;")

        self.lbl_case_desc = QLabel("Bi-RRT giật góc nhọn J=2.23e6 vs GWO B-Spline C2 dập tắt 95.5% Jerk")
        self.lbl_case_desc.setStyleSheet("color: #cbd5e1; font-size: 10px; margin-left: 8px;")

        self.lbl_obs_badge = QLabel("🛡️ VẬT CẢN 3D: ĐANG BẬT (3 Quả cầu né tránh)")
        self.lbl_obs_badge.setStyleSheet("""
            QLabel {
                background-color: #450a0a; color: #f87171; font-weight: bold;
                border: 1px solid #dc2626; border-radius: 4px; padding: 2px 8px; font-size: 10px;
            }
        """)

        badge_layout.addWidget(self.lbl_case_name)
        badge_layout.addWidget(self.lbl_case_desc, stretch=1)
        badge_layout.addWidget(self.lbl_obs_badge)

        main_layout.addWidget(self.badge_case_card)

        # -------------------------------------------------------------
        # 3. KPI SUMMARY CARDS (Chuẩn Nature Scientific Reports 2026)'''

assert old_post_cfg in text, "old_post_cfg not found"
text = text.replace(old_post_cfg, new_post_cfg)
print("P9: Test case badge card added")

# 9. Add helper methods before _on_preset_changed
old_preset_method = '''    def _on_preset_changed(self, idx):'''

new_preset_method = '''    def _update_obstacle_btn_style(self, enabled: bool):
        if hasattr(self, "btn_toggle_obstacles"):
            if enabled:
                self.btn_toggle_obstacles.setText("🛡️ VẬT CẢN 3D: ĐANG BẬT (Bấm để Tắt)")
                self.btn_toggle_obstacles.setStyleSheet("""
                    QPushButton {
                        background-color: #dc2626; color: white; font-weight: bold;
                        border: 1px solid #ef4444; border-radius: 4px; padding: 2px 10px; font-size: 11px;
                    }
                    QPushButton:hover { background-color: #b91c1c; }
                """)
            else:
                self.btn_toggle_obstacles.setText("🛡️ VẬT CẢN 3D: ĐANG TẮT (Vẽ G-Code)")
                self.btn_toggle_obstacles.setStyleSheet("""
                    QPushButton {
                        background-color: #059669; color: white; font-weight: bold;
                        border: 1px solid #10b981; border-radius: 4px; padding: 2px 10px; font-size: 11px;
                    }
                    QPushButton:hover { background-color: #047857; }
                """)
        if hasattr(self, "chk_obstacles"):
            self.chk_obstacles.blockSignals(True)
            self.chk_obstacles.setChecked(enabled)
            self.chk_obstacles.blockSignals(False)
        self._update_case_info_badge()

    def _on_toggle_obstacles_clicked(self):
        self.obstacles_enabled = not self.obstacles_enabled
        self._update_obstacle_btn_style(self.obstacles_enabled)
        self.sync_obstacles_to_gazebo_and_rviz()
        if self.obstacles_enabled:
            self.lbl_status.setText("🛡️ Đã BẬT 3 Vật cản 3D trong Gazebo Sim & RViz2. Thuật toán BiRRT + GWO sẽ né tránh!")
        else:
            self.lbl_status.setText("✨ Đã TẮT Vật cản 3D trong Gazebo Sim. Chế độ vẽ G-code tự do trên mặt bàn!")
        self._start_benchmark()

    def _on_chk_obstacles_toggled(self, checked: bool):
        if self.obstacles_enabled != checked:
            self.obstacles_enabled = checked
            self._update_obstacle_btn_style(checked)
            self.sync_obstacles_to_gazebo_and_rviz()
            if self.obstacles_enabled:
                self.lbl_status.setText("🛡️ Đã BẬT 3 Vật cản 3D trong Gazebo Sim & RViz2. Thuật toán BiRRT + GWO sẽ né tránh!")
            else:
                self.lbl_status.setText("✨ Đã TẮT Vật cản 3D trong Gazebo Sim. Chế độ vẽ G-code tự do trên mặt bàn!")
            self._start_benchmark()

    def _update_case_info_badge(self):
        if not hasattr(self, "lbl_case_name"):
            return
        tc = self.combo_preset.currentData()
        name = tc["name"] if tc else "Kịch bản tùy chỉnh"
        desc = tc.get("desc", "") if tc else ""
        self.lbl_case_name.setText(name)
        self.lbl_case_desc.setText(desc)
        if self.obstacles_enabled:
            self.lbl_obs_badge.setText("🛡️ VẬT CẢN 3D: ĐANG BẬT (3 Quả cầu né tránh)")
            self.lbl_obs_badge.setStyleSheet("""
                QLabel {
                    background-color: #450a0a; color: #f87171; font-weight: bold;
                    border: 1px solid #dc2626; border-radius: 4px; padding: 2px 8px; font-size: 10px;
                }
            """)
        else:
            self.lbl_obs_badge.setText("✨ VẬT CẢN 3D: ĐANG TẮT (Chế độ vẽ G-code tự do)")
            self.lbl_obs_badge.setStyleSheet("""
                QLabel {
                    background-color: #064e3b; color: #34d399; font-weight: bold;
                    border: 1px solid #059669; border-radius: 4px; padding: 2px 8px; font-size: 10px;
                }
            """)

    def get_active_obstacles_def(self):
        if not getattr(self, "obstacles_enabled", True):
            return None
        tc = self.combo_preset.currentData()
        if tc and tc.get("obstacles"):
            return tc["obstacles"]
        return DEFAULT_TABLETOP_OBSTACLES

    def sync_obstacles_to_gazebo_and_rviz(self):
        obs_list = self.get_active_obstacles_def()
        if self.gz_node is not None and self.gz_marker_v_pb2 is not None:
            try:
                mv = self.gz_marker_v_pb2.Marker_V()
                for mid in range(540, 555):
                    m_del = mv.marker.add()
                    m_del.ns = "denso_benchmark_line"
                    m_del.id = mid
                    m_del.action = m_del.DELETE_MARKER

                if self.obstacles_enabled and obs_list:
                    for o_idx, obs in enumerate(obs_list):
                        m_obs = mv.marker.add()
                        m_obs.ns = "denso_benchmark_line"
                        m_obs.id = 540 + o_idx
                        m_obs.action = m_obs.ADD_MODIFY
                        m_obs.type = m_obs.SPHERE
                        r = float(obs["radius"])
                        m_obs.scale.x = 2.0 * r
                        m_obs.scale.y = 2.0 * r
                        m_obs.scale.z = 2.0 * r
                        c = obs["center"]
                        m_obs.pose.position.x = float(c[0])
                        m_obs.pose.position.y = float(c[1])
                        m_obs.pose.position.z = float(c[2])
                        for mat in [m_obs.material.ambient, m_obs.material.diffuse]:
                            mat.r = 0.95
                            mat.g = 0.18
                            mat.b = 0.42
                            mat.a = 0.65
                        m_obs.material.emissive.r = 0.40
                        m_obs.material.emissive.g = 0.05
                        m_obs.material.emissive.b = 0.18
                        m_obs.material.emissive.a = 0.65
                self.gz_node.request("/marker_array", mv, self.gz_marker_v_pb2.Marker_V, self.gz_boolean_pb2.Boolean, 250)
            except Exception:
                pass

        if HAS_ROS2_MSGS:
            for pub in [self.rviz_trail_pub, getattr(self.bridge, "marker_pub", None)]:
                if pub is None:
                    continue
                try:
                    ma = MarkerArray()
                    for mid in range(140, 155):
                        m_del = Marker()
                        m_del.header.frame_id = "base_link"
                        m_del.ns = "benchmark_straight_line"
                        m_del.id = mid
                        m_del.action = Marker.DELETE
                        ma.markers.append(m_del)

                    if self.obstacles_enabled and obs_list:
                        for o_idx, obs in enumerate(obs_list):
                            m_o = Marker()
                            m_o.header.frame_id = "base_link"
                            m_o.ns = "benchmark_straight_line"
                            m_o.id = 140 + o_idx
                            m_o.type = Marker.SPHERE
                            m_o.action = Marker.ADD
                            r = float(obs["radius"])
                            m_o.scale.x = 2.0 * r
                            m_o.scale.y = 2.0 * r
                            m_o.scale.z = 2.0 * r
                            c = obs["center"]
                            m_o.pose.position.x = float(c[0])
                            m_o.pose.position.y = float(c[1])
                            m_o.pose.position.z = float(c[2])
                            m_o.color.r = 0.95
                            m_o.color.g = 0.18
                            m_o.color.b = 0.42
                            m_o.color.a = 0.65
                            ma.markers.append(m_o)
                    pub.publish(ma)
                except Exception:
                    pass

    def _on_preset_changed(self, idx):'''

assert old_preset_method in text, "old_preset_method not found"
text = text.replace(old_preset_method, new_preset_method)
print("P10: Helper methods added")

# 11. Update _on_preset_changed body to update obstacle state
old_preset_body = '''        self._update_distance_label()
        self.lbl_status.setText(f"Đã chọn: {tc['name']} — {tc['desc']}")
        self._start_benchmark()'''

new_preset_body = '''        # Cập nhật trạng thái vật cản theo từng kịch bản
        has_obs = bool(tc.get("obstacles"))
        self.obstacles_enabled = has_obs
        self._update_obstacle_btn_style(has_obs)
        self.sync_obstacles_to_gazebo_and_rviz()

        self._update_distance_label()
        self._update_case_info_badge()
        self.lbl_status.setText(f"Đã chọn: {tc['name']} — {tc['desc']}")
        self._start_benchmark()'''

assert old_preset_body in text, "old_preset_body not found"
text = text.replace(old_preset_body, new_preset_body)
print("P11: _on_preset_changed body updated")

# 12. _start_benchmark obs_def
old_start_obs = '''        tc = self.combo_preset.currentData()
        obs_def = tc.get("obstacles") if tc else None'''

new_start_obs = '''        obs_def = self.get_active_obstacles_def()'''

assert old_start_obs in text, "old_start_obs not found"
text = text.replace(old_start_obs, new_start_obs)
print("P12: _start_benchmark obs_def updated")

# 13. _ensure_benchmark_ready obs_def
old_ensure_obs = '''            tc = self.combo_preset.currentData()
            obs_def = tc.get("obstacles") if tc else None'''

new_ensure_obs = '''            obs_def = self.get_active_obstacles_def()'''

assert old_ensure_obs in text, "old_ensure_obs not found"
text = text.replace(old_ensure_obs, new_ensure_obs)
print("P13: _ensure_benchmark_ready obs_def updated")

# 14. clear_trajectory_marker cleanup
old_clear_marker = '''        if self.gz_node is not None and self.gz_marker_v_pb2 is not None:
            try:
                mv = self.gz_marker_v_pb2.Marker_V()
                for mid in range(501, 560):
                    m = mv.marker.add()
                    m.ns = "denso_benchmark_line"
                    m.id = mid
                    m.action = m.DELETE_MARKER
                self.gz_node.request("/marker_array", mv, self.gz_marker_v_pb2.Marker_V, self.gz_boolean_pb2.Boolean, 200)
            except Exception:
                pass

        if HAS_ROS2_MSGS:
            for pub in [self.rviz_trail_pub, getattr(self.bridge, "marker_pub", None)]:
                if pub is None:
                    continue
                try:
                    ma = MarkerArray()
                    for mid in range(100, 160):
                        m = Marker()
                        m.header.frame_id = "base_link"
                        m.ns = "benchmark_straight_line"
                        m.id = mid
                        m.action = Marker.DELETE
                        ma.markers.append(m)
                    pub.publish(ma)
                except Exception:
                    pass'''

new_clear_marker = '''        if self.gz_node is not None and self.gz_marker_v_pb2 is not None:
            try:
                mv = self.gz_marker_v_pb2.Marker_V()
                for mid in range(501, 540):
                    m = mv.marker.add()
                    m.ns = "denso_benchmark_line"
                    m.id = mid
                    m.action = m.DELETE_MARKER
                if not getattr(self, "obstacles_enabled", True):
                    for mid in range(540, 555):
                        m = mv.marker.add()
                        m.ns = "denso_benchmark_line"
                        m.id = mid
                        m.action = m.DELETE_MARKER
                self.gz_node.request("/marker_array", mv, self.gz_marker_v_pb2.Marker_V, self.gz_boolean_pb2.Boolean, 200)
            except Exception:
                pass

        if HAS_ROS2_MSGS:
            for pub in [self.rviz_trail_pub, getattr(self.bridge, "marker_pub", None)]:
                if pub is None:
                    continue
                try:
                    ma = MarkerArray()
                    for mid in range(101, 140):
                        m = Marker()
                        m.header.frame_id = "base_link"
                        m.ns = "benchmark_straight_line"
                        m.id = mid
                        m.action = Marker.DELETE
                        ma.markers.append(m)
                    if not getattr(self, "obstacles_enabled", True):
                        for mid in range(140, 155):
                            m = Marker()
                            m.header.frame_id = "base_link"
                            m.ns = "benchmark_straight_line"
                            m.id = mid
                            m.action = Marker.DELETE
                            ma.markers.append(m)
                    pub.publish(ma)
                except Exception:
                    pass'''

assert old_clear_marker in text, "old_clear_marker not found"
text = text.replace(old_clear_marker, new_clear_marker)
print("P14: clear_trajectory_marker updated")

# 15. _publish_trajectory_marker Gazebo obs handling
old_pub_obs = '''                # 4. Hiển thị 3 Khối Cầu Vật Cản 3D trên bàn làm việc trong Gazebo (Nature 2026 Tabletop Obstacles)
                obs_list = self.latest_res_iga.get("obstacles", [])
                for o_idx, obs in enumerate(obs_list):
                    m_obs = mv.marker.add()
                    m_obs.ns = "denso_benchmark_line"
                    m_obs.id = 540 + o_idx
                    m_obs.action = m_obs.ADD_MODIFY
                    m_obs.type = m_obs.SPHERE
                    r = float(obs["radius"])
                    m_obs.scale.x = 2.0 * r
                    m_obs.scale.y = 2.0 * r
                    m_obs.scale.z = 2.0 * r
                    c = obs["center"]
                    m_obs.pose.position.x = float(c[0])
                    m_obs.pose.position.y = float(c[1])
                    m_obs.pose.position.z = float(c[2])
                    for mat in [m_obs.material.ambient, m_obs.material.diffuse]:
                        mat.r = 0.95
                        mat.g = 0.18
                        mat.b = 0.42
                        mat.a = 0.65
                    m_obs.material.emissive.r = 0.35
                    m_obs.material.emissive.g = 0.05
                    m_obs.material.emissive.b = 0.15
                    m_obs.material.emissive.a = 0.65'''

new_pub_obs = '''                # 4. Hiển thị 3 Khối Cầu Vật Cản 3D trên bàn làm việc trong Gazebo
                obs_list = self.latest_res_iga.get("obstacles", []) if self.obstacles_enabled else []
                if self.obstacles_enabled and not obs_list:
                    obs_list = self.get_active_obstacles_def() or []

                if self.obstacles_enabled and obs_list:
                    for o_idx, obs in enumerate(obs_list):
                        m_obs = mv.marker.add()
                        m_obs.ns = "denso_benchmark_line"
                        m_obs.id = 540 + o_idx
                        m_obs.action = m_obs.ADD_MODIFY
                        m_obs.type = m_obs.SPHERE
                        r = float(obs["radius"])
                        m_obs.scale.x = 2.0 * r
                        m_obs.scale.y = 2.0 * r
                        m_obs.scale.z = 2.0 * r
                        c = obs["center"]
                        m_obs.pose.position.x = float(c[0])
                        m_obs.pose.position.y = float(c[1])
                        m_obs.pose.position.z = float(c[2])
                        for mat in [m_obs.material.ambient, m_obs.material.diffuse]:
                            mat.r = 0.95
                            mat.g = 0.18
                            mat.b = 0.42
                            mat.a = 0.65
                        m_obs.material.emissive.r = 0.40
                        m_obs.material.emissive.g = 0.05
                        m_obs.material.emissive.b = 0.18
                        m_obs.material.emissive.a = 0.65
                else:
                    for mid in range(540, 555):
                        m_del = mv.marker.add()
                        m_del.ns = "denso_benchmark_line"
                        m_del.id = mid
                        m_del.action = m_del.DELETE_MARKER'''

assert old_pub_obs in text, "old_pub_obs not found"
text = text.replace(old_pub_obs, new_pub_obs)
print("P15: _publish_trajectory_marker Gazebo obs updated")

# 16. _publish_trajectory_marker RViz2 obs handling
old_rviz_obs = '''                    # 3 Khối Cầu Vật Cản 3D trong RViz2
                    for o_idx, obs in enumerate(obs_list):
                        m_o = Marker()
                        m_o.header.frame_id = "base_link"
                        m_o.ns = "benchmark_straight_line"
                        m_o.id = 140 + o_idx
                        m_o.type = Marker.SPHERE
                        m_o.action = Marker.ADD
                        r = float(obs["radius"])
                        m_o.scale.x = 2.0 * r
                        m_o.scale.y = 2.0 * r
                        m_o.scale.z = 2.0 * r
                        c = obs["center"]
                        m_o.pose.position.x = float(c[0])
                        m_o.pose.position.y = float(c[1])
                        m_o.pose.position.z = float(c[2])
                        m_o.color.r = 0.95
                        m_o.color.g = 0.18
                        m_o.color.b = 0.42
                        m_o.color.a = 0.65
                        ma.markers.append(m_o)'''

new_rviz_obs = '''                    # 3 Khối Cầu Vật Cản 3D trong RViz2
                    if self.obstacles_enabled and obs_list:
                        for o_idx, obs in enumerate(obs_list):
                            m_o = Marker()
                            m_o.header.frame_id = "base_link"
                            m_o.ns = "benchmark_straight_line"
                            m_o.id = 140 + o_idx
                            m_o.type = Marker.SPHERE
                            m_o.action = Marker.ADD
                            r = float(obs["radius"])
                            m_o.scale.x = 2.0 * r
                            m_o.scale.y = 2.0 * r
                            m_o.scale.z = 2.0 * r
                            c = obs["center"]
                            m_o.pose.position.x = float(c[0])
                            m_o.pose.position.y = float(c[1])
                            m_o.pose.position.z = float(c[2])
                            m_o.color.r = 0.95
                            m_o.color.g = 0.18
                            m_o.color.b = 0.42
                            m_o.color.a = 0.65
                            ma.markers.append(m_o)
                    else:
                        for mid in range(140, 155):
                            m_del = Marker()
                            m_del.header.frame_id = "base_link"
                            m_del.ns = "benchmark_straight_line"
                            m_del.id = mid
                            m_del.action = Marker.DELETE
                            ma.markers.append(m_del)'''

assert old_rviz_obs in text, "old_rviz_obs not found"
text = text.replace(old_rviz_obs, new_rviz_obs)
print("P16: _publish_trajectory_marker RViz2 obs updated")

with open(file_path, "w", encoding="utf-8") as f:
    f.write(text)

print("SUCCESS: All 16 patches applied cleanly!")

