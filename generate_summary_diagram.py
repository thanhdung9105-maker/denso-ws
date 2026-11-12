#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script vẽ sơ đồ tóm tắt tổng thể phương pháp nghiên cứu cho đề tài:
'TỐI ƯU CHUYỂN ĐỘNG TAY MÁY 6 BẬC TỰ DO SỬ DỤNG THUẬT TOÁN TỐI ƯU BẦY ĐÀN NHẰM TIẾT KIỆM NĂNG LƯỢNG'
"""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, ArrowStyle

desktop_img_path = '/mnt/c/Users/thanh/Desktop/So_Do_Tong_The_Toi_Uu_Nang_Luong_Tay_May_6DOF.png'

fig = plt.figure(figsize=(15, 8.5), dpi=300)
ax = fig.add_subplot(111)
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.axis('off')

# Set background to clean white (perfect for thesis document / reports)
fig.patch.set_facecolor('#ffffff')
ax.set_facecolor('#ffffff')

# Register Arial / DejaVu fonts
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']

# Title Box
title_box = FancyBboxPatch((2, 91), 96, 7.5, boxstyle="round,pad=0.5,rounding_size=1.5",
                           ec="#0284c7", fc="#f0f9ff", lw=2)
ax.add_patch(title_box)
ax.text(50, 95.8, "SƠ ĐỒ TỔNG THỂ PHƯƠNG PHÁP NGHIÊN CỨU & QUY TRÌNH TỐI ƯU HÓA",
        ha='center', va='center', fontsize=14, fontweight='bold', color='#0369a1')
ax.text(50, 92.6, "ĐỀ TÀI: TỐI ƯU CHUYỂN ĐỘNG TAY MÁY 6 BẬC TỰ DO SỬ DỤNG THUẬT TOÁN TỐI ƯU BẦY ĐÀN NHẰM TIẾT KIỆM NĂNG LƯỢNG",
        ha='center', va='center', fontsize=10.5, fontweight='bold', color='#0f172a')

# Colors
c_input = "#f8fafc"
b_input = "#64748b"
c_param = "#eff6ff"
b_param = "#3b82f6"
c_dyn = "#fefce8"
b_dyn = "#eab308"
c_opt = "#fdf4ff"
b_opt = "#a855f7"
c_out = "#f0fdf4"
b_out = "#22c55e"

def draw_block(x, y, w, h, bg_col, border_col, title, items, badge=""):
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.5,rounding_size=1.2",
                         ec=border_col, fc=bg_col, lw=1.8)
    ax.add_patch(box)
    
    # Title bar
    header_box = FancyBboxPatch((x, y + h - 5.5), w, 5.5, boxstyle="round,pad=0.2,rounding_size=0.8",
                                ec=border_col, fc=border_col, lw=1)
    ax.add_patch(header_box)
    ax.text(x + w/2, y + h - 2.8, title, ha='center', va='center',
            fontsize=10.5, fontweight='bold', color='#ffffff')
    
    if badge:
        ax.text(x + 2.5, y + h + 1.6, badge, ha='left', va='center',
                fontsize=8.5, fontweight='bold', color=border_col)
        
    # Text lines
    start_y = y + h - 9.0
    for idx, itm in enumerate(items):
        ax.text(x + 2.0, start_y - idx * 4.6, itm, ha='left', va='center',
                fontsize=8.6, color='#1e293b', wrap=True)

# 1. Block: DỮ LIỆU ĐẦU VÀO
draw_block(2, 47, 26, 38, c_input, b_input,
           "1. DỮ LIỆU ĐẦU VÀO & RÀNG BUỘC",
           [
               "• Robot 6-DOF: Denso VS-6556",
               "  - Bảng D-H, khối lượng link, ma sát",
               "  - Quán tính rotor phản chiếu J_eff",
               "  - Tỉ số truyền Harmonic N_i, R_m, K_t",
               "• Điểm mốc tác vụ (Task Waypoints):",
               "  - Điểm xuất phát P_start, điểm đích P_goal",
               "  - Đường dẫn biên dạng / vượt vật cản",
               "• Giới hạn cơ khí & an toàn:",
               "  - Góc quay [q_min, q_max]",
               "  - Vận tốc max [qd_max], gia tốc [qdd_max]"
           ],
           badge="[ GIAI ĐOẠN 1: THIẾT LẬP BÀI TOÁN ]")

# 2. Block: THAM SỐ HÓA QUỸ ĐẠO
draw_block(36, 47, 28, 38, c_param, b_param,
           "2. THAM SỐ HÓA QUỸ ĐẠO (C2)",
           [
               "• Động học nghịch 6-DOF (Analytical IK):",
               "  - Giải tích hình học Link 5 (Point P)",
               "  - Phân tích nhánh nghiệm uốn khuỷu",
               "• Xây dựng đường cong nội suy:",
               "  - Nội suy đường cong trơn B-Spline C2",
               "  - Triệt tiêu góc gấp nhọn gián đoạn",
               "• Tham số hóa biến tối ưu (Variables):",
               "  - Vector khoảng thời gian: Δt = [Δt_1, ..., Δt_m]",
               "  - Điều chỉnh cấu hình góc khớp trung gian"
           ],
           badge="[ GIAI ĐOẠN 2: THIẾT KẾ ĐƯỜNG ĐI ]")

# 3. Block: MÔ HÌNH ĐỘNG LỰC HỌC & ĐIỆN NĂNG
draw_block(72, 47, 26, 38, c_dyn, b_dyn,
           "3. MÔ HÌNH NĂNG LƯỢNG ĐỘNG LỰC",
           [
               "• Động lực học RNEA Euler-Lagrange:",
               "  τ(t) = M(q)qdd + C(q,qd)qd + g(q) + F_fric",
               "• Tổn hao điện năng động cơ Servo:",
               "  P_elec(t) = P_mech(t) + P_joule(t)",
               "  P_joule(t) = (R_m / K_t^2) · τ^2(t)",
               "• Hàm chi phí đa mục tiêu (Fitness Fun):",
               "  Min F = w1·E_total + w2·T_total + w3·J_jerk",
               "  với E_total = ∫ P_elec(t) dt",
               "  và phạt vi phạm biên ràng buộc (Penalty)"
           ],
           badge="[ GIAI ĐOẠN 3: HÀM MỤC TIÊU NĂNG LƯỢNG ]")

# 4. Block: THUẬT TOÁN TỐI ƯU BẦY ĐÀN
draw_block(18, 5, 34, 34, c_opt, b_opt,
           "4. THUẬT TOÁN TỐI ƯU BẦY ĐÀN (SWARM)",
           [
               "• Khởi tạo quần thể bầy đàn (Swarm Population):",
               "  - Mỗi cá thể đại diện cho 1 cấu hình quỹ đạo & Δt",
               "• Cơ chế tìm kiếm & chia sẻ thông tin:",
               "  - Thuật toán Tối ưu Bầy Sói (Grey Wolf Optimizer - GWO)",
               "    hoặc Bầy đàn hạt (Particle Swarm Optimization - PSO)",
               "  - Cập nhật vị trí săn mồi theo 3 sói đầu đàn (α, β, δ)",
               "• Đánh giá thích nghi (Fitness Evaluation):",
               "  - Tính toán E_total và độ giật Jerk qua từng thế hệ",
               "• Tiêu chuẩn dừng: Hội tụ sai số hoặc max số thế hệ"
           ],
           badge="[ GIAI ĐOẠN 4: TỐI ƯU HÓA THÔNG MINH ]")

# 5. Block: XUẤT KẾT QUẢ & KIỂM CHỨNG
draw_block(60, 5, 38, 34, c_out, b_out,
           "5. KẾT QUẢ TỐI ƯU & KIỂM CHỨNG GAZEBO",
           [
               "• Quỹ đạo tối ưu xuất ra (Optimal Output):",
               "  - Chuỗi vị trí, vận tốc, gia tốc: q*(t), qd*(t), qdd*(t)",
               "  - Phân bố thời gian thích nghi: Δt* (đầm ở góc cua)",
               "• Mô phỏng số hóa & Kiểm chứng thực nghiệm:",
               "  - Môi trường mô phỏng ROS 2 Jazzy & Gazebo Sim 3D",
               "  - Bộ điều khiển Joint Trajectory Controller vòng kín",
               "• Đánh giá hiệu quả định lượng (KPI):",
               "  - Tiết kiệm điện năng: Giảm 20% - 42% (tùy kịch bản)",
               "  - Triệt tiêu độ giật Jerk: Giảm >90% (bảo vệ hộp số)"
           ],
           badge="[ GIAI ĐOẠN 5: XÁC THỰC THỰC THI ]")

# Arrows between blocks
def draw_arrow(p1, p2, text="", rad=0.0, color="#0284c7"):
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="-|>,head_width=0.4,head_length=0.7",
                                color=color, lw=2.2,
                                connectionstyle=f"arc3,rad={rad}"))
    if text:
        mx = (p1[0] + p2[0]) / 2.0
        my = (p1[1] + p2[1]) / 2.0
        ax.text(mx, my + 1.8, text, ha='center', va='center',
                fontsize=8.5, fontweight='bold', color=color,
                bbox=dict(boxstyle="round,pad=0.2", fc="#ffffff", ec=color, lw=0.8))

# Arrow 1 -> 2
draw_arrow((28, 66), (36, 66), "Dữ liệu góc")
# Arrow 2 -> 3
draw_arrow((64, 66), (72, 66), "Quỹ đạo q(t)")
# Arrow 3 -> 4 (downwards to Swarm Optimizer)
draw_arrow((85, 47), (52, 22), "Hàm mục tiêu F", rad=0.25, color="#a855f7")
# Feedback loop from 4 to 2 (Swarm updating parameters)
draw_arrow((35, 39), (45, 47), "Cập nhật Δt*", rad=0.2, color="#3b82f6")
# Arrow 4 -> 5
draw_arrow((52, 22), (60, 22), "Quỹ đạo q*(t)", color="#22c55e")

plt.tight_layout()
plt.savefig(desktop_img_path, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
plt.close()
print(f"Image successfully saved to: {desktop_img_path}")

