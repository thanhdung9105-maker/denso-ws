#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script vẽ sơ đồ tóm tắt ĐƠN GIẢN, TINH GỌN, KHÔNG BỊ TRÀN CHỮ
'TỐI ƯU CHUYỂN ĐỘNG TAY MÁY 6 BẬC TỰ DO SỬ DỤNG THUẬT TOÁN TỐI ƯU BẦY ĐÀN NHẰM TIẾT KIỆM NĂNG LƯỢNG'
"""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

import shutil

temp_img_path = '/tmp/So_Do_Don_Gian_Toi_Uu_Nang_Luong.png'
desktop_img_path = '/mnt/c/Users/thanh/Desktop/So_Do_Don_Gian_Toi_Uu_Nang_Luong.png'
desktop_img_path_v2 = '/mnt/c/Users/thanh/Desktop/So_Do_Don_Gian_Toi_Uu_Nang_Luong_Chuan.png'
artifact_img_path = '/mnt/c/Users/thanh/.gemini/antigravity/brain/86903b5e-ed26-42f2-8659-95a5acf08d5c/So_Do_Don_Gian_Toi_Uu_Nang_Luong.png'

# Tăng chiều cao khung vẽ để có nhiều không gian đệm ở trên và dưới
fig = plt.figure(figsize=(15, 6.2), dpi=300)
ax = fig.add_subplot(111)
ax.set_xlim(0, 100)
ax.set_ylim(0, 50)
ax.axis('off')

# Nền trắng chuẩn xuất bản
fig.patch.set_facecolor('#ffffff')
ax.set_facecolor('#ffffff')

plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']

# Tiêu đề trên cùng
ax.text(50, 46.5, "QUY TRÌNH TỐI ƯU NĂNG LƯỢNG TAY MÁY 6 BẬC TỰ DO",
        ha='center', va='center', fontsize=14, fontweight='bold', color="#0f172a")

# Tọa độ thẻ:
# Y từ 10 đến 42 (chiều cao h = 32, cực kỳ rộng rãi)
# Đáy thẻ ở Y = 10, cách mép dưới 10 đơn vị (đủ chỗ cho mũi tên vòng phản hồi)
card_y = 10.0
card_h = 32.0

def draw_card(x, y, w, h, bg_col, border_col, step_num, title, lines):
    # Đổ bóng nhẹ
    shadow = FancyBboxPatch((x + 0.4, y - 0.4), w, h, boxstyle="round,pad=0.3,rounding_size=1.2",
                            ec="none", fc="#e2e8f0", lw=0)
    ax.add_patch(shadow)
    
    # Khung thẻ chính
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3,rounding_size=1.2",
                         ec=border_col, fc=bg_col, lw=2.0)
    ax.add_patch(box)
    
    # Vòng tròn số thứ tự bước
    badge = plt.Circle((x + 3.4, y + h - 4.2), 2.2, fc=border_col, ec="none")
    ax.add_patch(badge)
    ax.text(x + 3.4, y + h - 4.2, str(step_num), ha='center', va='center',
            fontsize=12, fontweight='bold', color="#ffffff")
    
    # Tiêu đề thẻ
    ax.text(x + 7.2, y + h - 4.2, title, ha='left', va='center',
            fontsize=11.5, fontweight='bold', color='#0f172a')
    
    # Đường kẻ ngang phân cách
    ax.plot([x + 2, x + w - 2], [y + h - 8.0, y + h - 8.0], color=border_col, lw=1.0, alpha=0.4)
    
    # Các dòng nội dung - căn khoảng cách dòng thoáng (step = 3.6), không bao giờ chạm đáy
    start_y = y + h - 12.0
    for idx, line in enumerate(lines):
        # In đậm nếu là dòng nhấn mạnh
        is_bold = any(k in line for k in ["Start", "B-Spline", "Min", "Tiết kiệm", "ROS 2"])
        f_weight = 'bold' if is_bold else 'normal'
        f_color = '#0f172a' if is_bold else '#334155'
        ax.text(x + 2.4, start_y - idx * 3.8, line, ha='left', va='center',
                fontsize=9.5, color=f_color, fontweight=f_weight)

# Thẻ 1: ĐẦU VÀO
draw_card(2, card_y, 20.5, card_h, "#f8fafc", "#64748b", 1, "ĐẦU VÀO", [
    "• Điểm đầu & Điểm cuối",
    "  (Start & Goal)",
    "• Các điểm mốc tác vụ",
    "• Thông số Denso VS-6556",
    "• Giới hạn góc, vận tốc"
])

# Thẻ 2: QUY HOẠCH ĐƯỜNG
draw_card(26.5, card_y, 21.5, card_h, "#eff6ff", "#0284c7", 2, "QUY HOẠCH ĐƯỜNG", [
    "• Động học nghịch (IK 6-DOF)",
    "• Nội suy B-Spline C²",
    "  (Bo cong trơn mượt)",
    "• Biến tối ưu hóa:",
    "  Bước thời gian Δt"
])

# Thẻ 3: TỐI ƯU BẦY ĐÀN
draw_card(52.0, card_y, 23.5, card_h, "#faf5ff", "#9333ea", 3, "TỐI ƯU BẦY ĐÀN", [
    "• Thuật toán bầy đàn (GWO/PSO)",
    "• Mô hình Động lực học RNEA",
    "• Mô hình điện năng tiêu thụ",
    "• Tối thiểu hóa hàm chi phí:",
    "  Min F = Năng lượng + Jerk"
])

# Thẻ 4: KẾT QUẢ ĐẦU RA
draw_card(79.5, card_y, 18.5, card_h, "#f0fdf4", "#16a34a", 4, "KẾT QUẢ ĐẦU RA", [
    "• Quỹ đạo tối ưu q*(t), Δt*",
    "• Tiết kiệm điện 20% - 42%",
    "• Triệt tiêu >90% rung giật",
    "• Mô phỏng ROS 2 & Gazebo"
])

# Mũi tên liên kết ngang to rõ ràng
def draw_arrow(x1, x2, y=26.0):
    ax.annotate("", xy=(x2, y), xytext=(x1, y),
                arrowprops=dict(arrowstyle="-|>,head_width=0.45,head_length=0.75",
                                color="#0284c7", lw=2.6))

draw_arrow(23.0, 26.0)
draw_arrow(48.5, 51.5)
draw_arrow(76.0, 79.0)

# Mũi tên phản hồi (Feedback) ở phía dưới thẻ, uốn cong mượt mà, cách xa mép dưới
ax.annotate("", xy=(37.25, card_y - 0.6), xytext=(63.75, card_y - 0.6),
            arrowprops=dict(arrowstyle="-|>,head_width=0.4,head_length=0.65",
                            color="#9333ea", lw=1.8,
                            connectionstyle="arc3,rad=-0.28", linestyle="--"))

# Nhãn cho vòng lặp phản hồi
ax.text(50.5, 4.0, "Vòng lặp cập nhật thông số (Feedback Evaluation)",
        ha='center', va='center', fontsize=9.2, fontweight='bold', color="#9333ea",
        bbox=dict(boxstyle="round,pad=0.3", fc="#ffffff", ec="#d8b4fe", lw=1.0))

plt.tight_layout()
plt.savefig(temp_img_path, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
plt.close()

# Copy to destinations safely
for dst in [desktop_img_path, desktop_img_path_v2, artifact_img_path]:
    try:
        shutil.copyfile(temp_img_path, dst)
        print(f"Copied to: {dst}")
    except Exception as e:
        print(f"Could not copy to {dst}: {e}")
