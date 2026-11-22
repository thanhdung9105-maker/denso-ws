import os
import matplotlib.pyplot as plt
import numpy as np

os.makedirs(r'c:\Users\thanh\denso_ws\heat_treatment_assets', exist_ok=True)
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Segoe UI']
plt.rcParams['axes.unicode_minus'] = False

# ----------------------------------------------------------------------------------
# FIGURE 1: GIẢN ĐỒ PHA SẮT - CACBON VÀ VÙNG NHIỆT ĐỘ NHIỆT LUYỆN (Ủ, THƯỜNG HÓA, TÔI, RAM)
# ----------------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 7), dpi=200)

# Trục C (%) từ 0 đến 2.14% (Khu vực thép)
c_hypo = np.linspace(0.02, 0.77, 100)
# Đường A3 (G-S): từ 912 độ C (C=0) xuống 727 độ C (C=0.77)
a3_curve = 912 - (912 - 727) * (c_hypo / 0.77)**0.8

c_hyper = np.linspace(0.77, 2.14, 100)
# Đường Acm (S-E): từ 727 độ C (C=0.77) lên 1147 độ C (C=2.14)
acm_curve = 727 + (1147 - 727) * ((c_hyper - 0.77) / (2.14 - 0.77))**0.9

# Đường A1 (P-S-K) = 727 C
c_all = np.linspace(0.02, 2.14, 200)

# Vẽ các đường pha chính
ax.plot(c_hypo, a3_curve, 'k-', lw=2.2, label=r'Đường $A_3$ ($G-S$)')
ax.plot(c_hyper, acm_curve, 'k-', lw=2.2, label=r'Đường $A_{cm}$ ($S-E$)')
ax.axhline(y=727, color='black', linestyle='--', lw=2, label=r'Đường $A_1$ ($727^\circ\mathrm{C}$, $P-S-K$)')
ax.axvline(x=0.77, color='gray', linestyle=':', lw=1.5, label='Điểm cùng tích ($0.77\% \mathrm{C}$)')

# TÔ MÀU CÁC VÙNG NHIỆT ĐỘ CỦA 4 PHƯƠNG PHÁP NHIỆT LUYỆN
# 1. Thường hóa (Normalizing): A3 hoặc Acm + (30-50 C)
norm_hypo_bot = a3_curve + 30
norm_hypo_top = a3_curve + 60
norm_hyper_bot = acm_curve + 30
norm_hyper_top = acm_curve + 60
ax.fill_between(c_hypo, norm_hypo_bot, norm_hypo_top, color='#3b82f6', alpha=0.35, label='VÙNG THƯỜNG HÓA ($A_3/A_{cm} + 30\div50^\circ\mathrm{C}$)')
ax.fill_between(c_hyper, norm_hyper_bot, norm_hyper_top, color='#3b82f6', alpha=0.35)

# 2. Tôi thép (Quenching): Thép trước cùng tích: A3 + 30-50 C; Thép sau cùng tích: A1 + 30-50 C
quench_hypo_bot = a3_curve + 30
quench_hypo_top = a3_curve + 55
quench_hyper_bot = np.full_like(c_hyper, 727 + 30)
quench_hyper_top = np.full_like(c_hyper, 727 + 55)
ax.fill_between(c_hypo, quench_hypo_bot, quench_hypo_top, color='#ef4444', alpha=0.4, label='VÙNG TÔI ($A_3 + 30\div50^\circ\mathrm{C}$ & $A_1 + 30\div50^\circ\mathrm{C}$)')
ax.fill_between(c_hyper, quench_hyper_bot, quench_hyper_top, color='#ef4444', alpha=0.4)

# 3. Ủ hoàn toàn (Full Annealing): Thép trước cùng tích: A3 + 30-50 C; Ủ không hoàn toàn: A1 + 20-40 C
anneal_hypo_bot = a3_curve + 20
anneal_hypo_top = a3_curve + 40
ax.fill_between(c_hypo, anneal_hypo_bot, anneal_hypo_top, color='#10b981', alpha=0.35, label='VÙNG Ủ HOÀN TOÀN ($A_3 + 20\div40^\circ\mathrm{C}$)')
ax.fill_between(c_hyper, 727 + 15, 727 + 40, color='#10b981', alpha=0.25, label='VÙNG Ủ CẦU HÓA ($A_1 + 20\div40^\circ\mathrm{C}$)')

# 4. VÙNG RAM (Tempering): Dưới A1 (< 727 C)
# Ram thấp: 150 - 250 C
ax.axhspan(150, 250, color='#f59e0b', alpha=0.2, label='VÙNG RAM THẤP ($150 \div 250^\circ\mathrm{C}$): Martensite Ram')
# Ram trung bình: 350 - 450 C
ax.axhspan(350, 450, color='#f97316', alpha=0.2, label='VÙNG RAM TRUNG BÌNH ($350 \div 450^\circ\mathrm{C}$): Troostite Ram')
# Ram cao: 500 - 650 C
ax.axhspan(500, 650, color='#d97706', alpha=0.2, label='VÙNG RAM CAO ($500 \div 650^\circ\mathrm{C}$): Sorbite Ram (Cải Thiện)')

# Chú thích các pha
ax.text(0.4, 980, 'AUSTENITE (γ)\n[Pha dẻo, mạng FCC]', fontsize=11, fontweight='bold', ha='center', color='#1e3a8a')
ax.text(0.2, 770, 'α + γ', fontsize=10, fontweight='bold', ha='center', color='#374151')
ax.text(1.3, 850, 'γ + Fe₃C_II', fontsize=10, fontweight='bold', ha='center', color='#374151')
ax.text(0.35, 670, 'FERRITE + PERLITE (P)\n[Thép trước cùng tích]', fontsize=10, ha='center', color='#111827', fontweight='bold')
ax.text(1.4, 670, 'PERLITE + Fe₃C_II\n[Thép sau cùng tích]', fontsize=10, ha='center', color='#111827', fontweight='bold')

# Điểm tới hạn
ax.scatter([0, 0.77, 2.14], [912, 727, 1147], color='black', s=40, zorder=5)
ax.text(0.01, 915, 'G (912°C)', fontsize=9, fontweight='bold', va='bottom')
ax.text(0.77, 735, 'S (0.77%, 727°C)', fontsize=9, fontweight='bold', ha='center', va='bottom')
ax.text(2.05, 1150, 'E (2.14%, 1147°C)', fontsize=9, fontweight='bold', ha='right', va='bottom')

ax.set_title('GIẢN ĐỒ TRẠNG THÁI SẮT - CACBON (Fe - Fe₃C) & VÙNG NHIỆT ĐỘ 4 PHƯƠNG PHÁP NHIỆT LUYỆN', fontsize=12, fontweight='bold', pad=15, color='#0f172a')
ax.set_xlabel('Hàm lượng Cacbon (% wt C) [Phạm vi Thép: 0.02% ÷ 2.14%]', fontsize=11, fontweight='bold')
ax.set_ylabel('Nhiệt độ (°C)', fontsize=11, fontweight='bold')
ax.set_xlim(0, 2.14)
ax.set_ylim(50, 1180)
ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='upper right', fontsize=8.5, framealpha=0.95)

plt.tight_layout()
fig.savefig(r'c:\Users\thanh\denso_ws\heat_treatment_assets\fe_c_heat_treatment.png')
plt.close(fig)
print('Generated fe_c_heat_treatment.png')

# ----------------------------------------------------------------------------------
# FIGURE 2: ĐỒ THỊ CHU TRÌNH NHIỆT (TỐC ĐỘ NGUỘI CỦA Ủ, THƯỜNG HÓA, TÔI, RAM)
# ----------------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5.5), dpi=200)

t = np.linspace(0, 10, 500)

# Nung lên 860 độ C
ax.plot([0, 2, 4], [25, 860, 860], 'k-', lw=2.5, label='Nung nóng & Giữ nhiệt tạo Austenite (860°C)')

# Đường 1: Tôi (Nguội nhanh trong Nước / Dầu)
t_quench = np.linspace(4, 5.2, 100)
T_quench = 25 + (860 - 25) * np.exp(-4.5 * (t_quench - 4))
ax.plot(t_quench, T_quench, color='#ef4444', lw=3, label='1. TÔI: Nguội nhanh (Nước/Dầu) -> MARTENSITE (Độ cứng HRC 60-65)')

# Đường 2: Thường hóa (Nguội trong không khí tĩnh)
t_norm = np.linspace(4, 7.5, 150)
T_norm = 25 + (860 - 25) * np.exp(-1.1 * (t_norm - 4))
ax.plot(t_norm, T_norm, color='#3b82f6', lw=3, label='2. THƯỜNG HÓA: Nguội không khí tĩnh -> PERLITE MỊN / SORBIT')

# Đường 3: Ủ (Nguội chậm cùng lò)
t_anneal = np.linspace(4, 10, 200)
T_anneal = 25 + (860 - 25) * np.exp(-0.45 * (t_anneal - 4))
ax.plot(t_anneal, T_anneal, color='#10b981', lw=3, label='3. Ủ: Nguội rất chậm cùng lò -> PERLITE THÔ (Mềm dẻo nhất)')

# Đường 4: Ram (Nung lại sau Tôi ở 150-650 C rồi làm nguội)
t_temper = [5.5, 6.5, 7.5, 8.5]
T_temper = [25, 450, 450, 25]
ax.plot(t_temper, T_temper, color='#f59e0b', lw=3, linestyle='--', label='4. RAM: Nung lại sau Tôi (<727°C) -> Điều chỉnh độ cứng & độ dai')

# Đường A1
ax.axhline(y=727, color='gray', linestyle=':', lw=1.5, label='Đường tới hạn A1 (727°C)')
ax.axhline(y=200, color='purple', linestyle=':', lw=1.2, label='Nhiệt độ bắt đầu chuyển biến Martensite (Ms ~ 200°C)')

ax.set_title('SO SÁNH TỐC ĐỘ LÀM NGUỘI VÀ CHUYỂN BIẾN PHA CỦA 4 PHƯƠNG PHÁP NHIỆT LUYỆN', fontsize=11, fontweight='bold', pad=12, color='#0f172a')
ax.set_xlabel('Thời gian t (Thời gian nguội tăng dần: Tôi >> Thường Hóa >> Ủ)', fontsize=10, fontweight='bold')
ax.set_ylabel('Nhiệt độ T (°C)', fontsize=10, fontweight='bold')
ax.set_ylim(0, 950)
ax.set_xlim(0, 10)
ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='upper right', fontsize=8.5, framealpha=0.95)

plt.tight_layout()
fig.savefig(r'c:\Users\thanh\denso_ws\heat_treatment_assets\heat_treatment_cooling_curves.png')
plt.close(fig)
print('Generated heat_treatment_cooling_curves.png')

# ----------------------------------------------------------------------------------
# FIGURE 3: QUY TRÌNH KẾT HỢP VÀ ỨNG DỤNG SẢN PHẨM CỤ THỂ TRONG CHẾ TẠO CƠ KHÍ
# ----------------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5), dpi=200)
ax.axis('off')

# Vẽ flow chart quy trình chế tạo Bánh Răng / Trục Cơ Khí
steps = [
    ("BƯỚC 1: PHÔI THÔ\n(Rèn dập / Đúc)", "#64748b", "Hạt thô, ứng suất dư lớn"),
    ("BƯỚC 2: Ủ / THƯỜNG HÓA\n(Nhiệt luyện sơ bộ)", "#10b981", "Làm mịn hạt, mềm thép\nDễ gia công cắt gọt"),
    ("BƯỚC 3: GIA CÔNG CƠ\n(Phay, tiện, xọc răng)", "#0284c7", "Cắt gọt tạo hình chuẩn\nKích thước chính xác"),
    ("BƯỚC 4: TÔI THÉP\n(Tôi thể tích / Bề mặt)", "#ef4444", "Tạo pha Martensite\nĐộ cứng cao HRC 55-65"),
    ("BƯỚC 5: RAM THÉP\n(Nhiệt luyện kết thúc)", "#f59e0b", "Khử ứng suất, chống nứt\nĐạt độ dai va đập"),
    ("BƯỚC 6: MÀI TINH\n(Sản phẩm hoàn thiện)", "#059669", "Bánh răng hộp số\nTrục khuỷu động cơ")
]

x_start = 0.05
box_w = 0.125
spacing = 0.035
y_box = 0.45
h_box = 0.35

for i, (title, color, desc) in enumerate(steps):
    x = x_start + i * (box_w + spacing)
    rect = plt.Rectangle((x, y_box), box_w, h_box, facecolor=color, edgecolor='#0f172a', lw=1.5, zorder=2)
    ax.add_patch(rect)
    ax.text(x + box_w/2, y_box + h_box*0.68, title, fontsize=7.5, fontweight='bold', ha='center', va='center', color='white', zorder=3)
    ax.text(x + box_w/2, y_box + h_box*0.25, desc, fontsize=6.5, ha='center', va='center', color='#f8fafc', zorder=3)
    
    if i < len(steps) - 1:
        x_arrow = x + box_w
        ax.annotate('', xy=(x_arrow + spacing, y_box + h_box/2), xytext=(x_arrow, y_box + h_box/2),
                    arrowprops=dict(arrowstyle="->", color='#0f172a', lw=2.5))

ax.set_title('SƠ ĐỒ ỨNG DỤNG THỰC TẾ: QUY TRÌNH NHIỆT LUYỆN TRONG CHUỖI GIA CÔNG CƠ KHÍ', fontsize=11, fontweight='bold', pad=10, color='#0f172a')
plt.tight_layout()
fig.savefig(r'c:\Users\thanh\denso_ws\heat_treatment_assets\case_study_process.png')
plt.close(fig)
print('Generated case_study_process.png')
