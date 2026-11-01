# Lịch Sử Phiên Làm Việc & Tổng Hợp Cải Tiến Hệ Thống Robot Denso VS-6556
**Đồ án Ngành Cơ Kỹ Thuật — Trường Đại học Bách Khoa TP.HCM (HCMUT)**  
**Mã phiên hội thoại (Conversation ID)**: `7aae2463-9858-494b-9166-1c384a40d1d3`  
**Thời gian lưu**: `2026-10-29 08:09 (+07:00)`

---

## 1. Vị trí lưu trữ tự động của Antigravity
Toàn bộ lịch sử hội thoại gốc (bao gồm câu hỏi, hình ảnh đính kèm, chuỗi suy luận và mọi lệnh đã thực thi) được Antigravity tự động lưu trữ bền vững tại:
- **Bản ghi rút gọn (Compact Transcript - Windows)**:  
  `C:\Users\thanh\.gemini\antigravity\brain\7aae2463-9858-494b-9166-1c384a40d1d3\.system_generated\logs\transcript.jsonl`
- **Bản ghi đầy đủ (Full Transcript - Windows)**:  
  `C:\Users\thanh\.gemini\antigravity\brain\7aae2463-9858-494b-9166-1c384a40d1d3\.system_generated\logs\transcript_full.jsonl`
- **Truy cập từ trong WSL2 (`Ubuntu-24.04`)**:  
  `/mnt/c/Users/thanh/.gemini/antigravity/brain/7aae2463-9858-494b-9166-1c384a40d1d3/.system_generated/logs/transcript_full.jsonl`

---

## 2. Tổng hợp các lỗi đã khắc phục & cải tiến kỹ thuật

### 2.1. Động học ngược giải tích (Analytical IK) & Động lực học — `denso_dynamics_engine.py`
- **File**: `src/denso_vs6556_moveit_config/scripts/denso_dynamics_engine.py`
- **Khắc phục lỗi chiếu hình học `solve_ik_analytical_flange`**:
  - Sửa lỗi nhánh truyền `tool_pitch_rad` trước đây chiếu `d_reach` theo góc phương vị `atan2(yf, xf)` thay vì xét cả 2 nghiệm phẳng `(q1_back, -r_xy)` và `(q1_fwd, +r_xy)`.
  - Sai số vị trí Flange IK giảm từ `160.00 mm` xuống **`6.18e-14 mm`** (chuẩn máy).
- **Bổ sung Ma trận Jacobian hình học $J(q) \in \mathbb{R}^{6 \times 5}$ và Chỉ số Khả thao tác Yoshikawa $w(q)$**:
  - Thêm phương thức `compute_geometric_jacobian(q)` và `compute_manipulability(q)` tính chỉ số Yoshikawa $w(q) = \sqrt{\det(J_v J_v^T)}$ và số điều kiện $\kappa(J)$ phục vụ phân tích kỳ dị động học trong thuyết minh đồ án.
- **Đồng bộ thông số quán tính & khối tâm CAD**:
  - Đồng bộ `COMS` và `INERTIAS` giữa `denso_dynamics_engine.py`, `denso_vs6556.urdf` và `denso_vs6556_gazebo.urdf.xacro` (`link_4`: 2.0 kg, `link_5`: 1.2 kg).

### 2.2. Kiểm nghiệm Ma trận truyền thuần nhất DH — `verify_denso_dh.py`
- **File**: `src/denso_vs6556_moveit_config/scripts/verify_denso_dh.py`
- Sửa công thức giải tích 3D trong `fk_planar_sincos` khi khớp `q4 != 0`, đạt sai số **`0.000000 mm`** trên toàn bộ các tư thế kiểm thử.

### 2.3. Giao diện Điều khiển Gazebo & Module Khắc/Vẽ G-Code (Tab 5 & Tab 6)
- **Các file**:
  - `src/denso_gazebo/scripts/denso_gazebo_control_panel.py`
  - `src/denso_gazebo/scripts/denso_gcode_vision_engine.py`
  - `src/denso_gazebo/scripts/denso_gcode_catalog.py`
- **Khắc phục**:
  - Bổ sung tham số `zero_velocity=True` vào `GazeboRosBridge.send_trajectory`.
  - Xóa bỏ màn hình chờ `Đang chuẩn bị module Khắc / Vẽ...`, khởi tạo trực tiếp `ImageGCodeDrawingTab` ngay khi mở GUI.
  - Sửa lỗi `AttributeError: 'ImageGCodeDrawingTab' object has no attribute 'current_points'` bằng cách đồng bộ sang `current_strokes` và `set_strokes()`.
  - Nâng cấp bộ biên dịch G-Code (`parse_gcode`) hỗ trợ lệnh modal (`G00`, `G01`, `G02`, `G03`), nội suy đoạn thẳng theo `point_spacing_mm` và nội suy cung tròn `G02`/`G03` (`I, J` và `R`). Toàn bộ 5 mẫu chuẩn đạt **100.0% khả thi động học**.

### 2.4. Hiển thị Trọng tâm từng Part (Link CoM & Masses) trên RViz2
- **Các file**:
  - `src/denso_vs6556_moveit_config/scripts/denso_com_visualizer.py`
  - `src/denso_vs6556_moveit_config/config/moveit.rviz`
  - `src/denso_vs6556/rviz/display.rviz` & `src/denso_vs6556/rviz/display.rviz.golden`
- **Khắc phục**:
  - Bổ sung 6 mục bật/tắt trọng tâm riêng biệt cho từng khâu (`Part 0 - Base (Đế) (8.0 kg)` đến `Part 5 - Link 5 (Cổ tay/Bích) (1.2 kg)`) cùng 5 lớp hiển thị trực quan (Điểm trọng tâm Xanh Neon, Khung ngắm Hồng tâm, Hệ trục quán tính 3D X-Y-Z, Đường chỉ dẫn, Nhãn khối lượng).
  - Sử dụng dấu `-` thay cho `:` trong tên namespace của từng Part để tương thích hoàn toàn với bộ đọc YAML `yaml-cpp` của RViz2.

### 2.5. Đồng bộ Clock, Khởi tạo Gazebo & Lưu Góc nhìn Camera
- **Các file**:
  - `src/denso_gazebo/config/gz_bridge.yaml`
  - `src/denso_gazebo/launch/denso_gazebo.launch.py`
  - `src/denso_gazebo/scripts/denso_log_cleaner.py`
  - `src/denso_gazebo/config/gui.config`
- **Khắc phục**:
  - Bật `lazy: false` trong `gz_bridge.yaml` và trì hoãn `spawn_robot` 1.5s để luồng `/clock` hoạt động ổn định trước khi khởi tạo `controller_manager`.
  - Lưu cố định góc nhìn camera cận cảnh bàn vẽ `<camera_pose>-0.786214 0.478947 0.647084 0.0 0.442514 -0.655606</camera_pose>` vào `gui.config` và tự động đồng bộ sang `~/.gz/sim/8/gui.config` khi khởi chạy.

---

## 3. Các lệnh khởi chạy nhanh trong Workspace
```bash
# 1. Khởi chạy toàn bộ hệ thống Gazebo Harmonic + MoveIt 2 + RViz2 (kèm Trọng tâm 6 Part)
cd ~/denso_ws
./launch_denso_gazebo_moveit.sh

# 2. Mở Bảng điều khiển trực tiếp & Tab 5 Khắc/Vẽ Hình ảnh - G-Code
cd ~/denso_ws
./denso_drawing_gcode.sh

# 3. Kiểm nghiệm độc lập Động học Thuận/Ngược, Jacobian & Khả thao tác Yoshikawa
cd ~/denso_ws
python3 src/denso_vs6556_moveit_config/scripts/verify_denso_dh.py
```

