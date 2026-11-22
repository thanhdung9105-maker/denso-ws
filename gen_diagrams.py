import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ==============================================================================
# 1. GENERATE new_eq_ppm.png (Slide 7)
# ==============================================================================
fig = plt.figure(figsize=(10, 4.3), dpi=180)
ax = fig.add_axes([0, 0, 1, 1])
ax.axis('off')

# Render formulas with LaTeX style mathtext
formula_ppm = r"$C = 400{,}0 \left( \frac{R_s \cdot k_{env}}{COR \cdot R_0} \right)^{-2{,}769}$"
formula_r0 = r"$R_0 = \frac{R_{s, clean}}{COR(31{,}4; 62{,}4)} = \frac{51{,}30}{0{,}834} = 61{,}50\ \mathrm{k}\Omega$"
subtext = "Hiệu chuẩn 6 lần ngoài trời · Mốc chuẩn 400.0 ppm · kEnvAdapt neo mốc phòng 51,30 kΩ"

ax.text(0.05, 0.72, formula_ppm, fontsize=28, color='#1e3a5f', va='center')
ax.text(0.05, 0.38, formula_r0, fontsize=25, color='#1e3a5f', va='center')
ax.text(0.05, 0.12, subtext, fontsize=15.5, color='#546e7a', va='center', fontfamily='Calibri')

plt.savefig('c:/Users/thanh/denso_ws/new_eq_ppm.png', dpi=180, transparent=True)
plt.close()
print("Generated new_eq_ppm.png")

# ==============================================================================
# 2. GENERATE new_eq_ema.png (Slide 8)
# ==============================================================================
fig = plt.figure(figsize=(10, 4.8), dpi=180)
ax = fig.add_axes([0, 0, 1, 1])
ax.axis('off')

line1 = r"$\mathrm{ADC\ Oversampling:\ } 20\ \mathrm{m\tilde{\hat{a}}u\ liên\ tục\ (400\ \mu s/m\tilde{\hat{a}}u)}$"
line2 = r"$C_{live} = 400{,}0 \cdot \left( \frac{R_{s,comp}}{R_0} \right)^{-2{,}76903}$"
desc1 = "• Khử hoàn toàn hiện tượng đơ cảm biến: số liệu và đồ thị uốn lượn liên tục"
desc2 = "• Trễ Schmitt Trigger 25 ppm chống rung giật quạt giữa các ngưỡng"
desc3 = "• Xuất đồng thời 6 kênh dữ liệu thời gian thực cho Arduino Serial Plotter"

ax.text(0.05, 0.85, line1, fontsize=21, color='#1e3a5f', va='center')
ax.text(0.05, 0.62, line2, fontsize=24, color='#1e3a5f', va='center')
ax.text(0.05, 0.40, desc1, fontsize=15.5, color='#37474f', va='center', fontfamily='Calibri')
ax.text(0.05, 0.24, desc2, fontsize=15.5, color='#37474f', va='center', fontfamily='Calibri')
ax.text(0.05, 0.08, desc3, fontsize=15.5, color='#37474f', va='center', fontfamily='Calibri')

plt.savefig('c:/Users/thanh/denso_ws/new_eq_ema.png', dpi=180, transparent=True)
plt.close()
print("Generated new_eq_ema.png")

# ==============================================================================
# 3. GENERATE new_luu_do.png (Slide 14)
# ==============================================================================
# The original luu_do was a vertical flowchart with high aesthetic quality
fig, ax = plt.subplots(figsize=(8.0, 10.5), dpi=200)
ax.axis('off')
ax.set_xlim(0, 100)
ax.set_ylim(0, 135)

def draw_box(y, text, h=6.5, w=72, color='#e3f2fd', border='#1565c0', text_col='#0d47a1', radius=2.5, bold=False):
    # Rounded box
    box = patches.FancyBboxPatch((50 - w/2, y - h/2), w, h,
                                boxstyle=f"round,pad=0.3,rounding_size={radius}",
                                facecolor=color, edgecolor=border, linewidth=1.5)
    ax.add_patch(box)
    ax.text(50, y, text, ha='center', va='center', fontsize=9.2, color=text_col,
            fontfamily='Calibri', fontweight='bold' if bold else 'normal')

def draw_arrow(y1, y2):
    ax.annotate('', xy=(50, y2), xytext=(50, y1),
                arrowprops=dict(arrowstyle="->", color='#1565c0', lw=1.5, mutation_scale=12))

# Top title BẮT ĐẦU
draw_box(130, "BẮT ĐẦU", h=6, w=30, color='#0d47a1', border='#082c66', text_col='white', bold=True)
draw_arrow(127, 122)

# setup() section annotation
ax.text(4, 115, "setup()", rotation=90, ha='center', va='center', fontsize=9, color='#78909c')

# setup boxes
draw_box(119, "Khởi tạo Serial (9600), ADC, PWM D9 = 0, I²C LCD (0x27 → 0x3F), DHT11", h=6.5, w=74)
draw_arrow(115.5, 110.5)

draw_box(107, "R0 = Rs_clean / COR(31,4 °C; 62,4 %) = 61,50 kΩ (Calib 6 lần ngoài trời)", h=6.5, w=74)
draw_arrow(103.5, 98.5)

draw_box(95, "Đo Rs phòng trong 2 s → kEnvAdapt = Rs_clean / Rs_phòng (0,57...1,74)", h=6.5, w=74)
draw_arrow(91.5, 86.5)

# loop() section annotation
ax.text(4, 45, "Vòng lặp loop()", rotation=90, ha='center', va='center', fontsize=9, color='#78909c')

# Diamond for DHT check
diamond = patches.Polygon([[50, 86.5], [65, 82], [50, 77.5], [35, 82]],
                          closed=True, facecolor='#fff9c4', edgecolor='#fbc02d', linewidth=1.5)
ax.add_patch(diamond)
ax.text(50, 83, "Đã đủ 2 s", ha='center', va='center', fontsize=8.8, fontweight='bold', color='#f57f17', fontfamily='Calibri')
ax.text(50, 80.5, "kể từ lần đọc DHT?", ha='center', va='center', fontsize=8.2, color='#f57f17', fontfamily='Calibri')

# DHT Read branch
ax.text(68, 83.5, "Có", fontsize=8.5, color='#0d47a1', fontfamily='Calibri')
ax.annotate('', xy=(72, 82), xytext=(65, 82),
            arrowprops=dict(arrowstyle="->", color='#1565c0', lw=1.3, mutation_scale=10))

# DHT Box
dht_box = patches.FancyBboxPatch((72, 78.5), 23, 7,
                                boxstyle="round,pad=0.2,rounding_size=2",
                                facecolor='#e3f2fd', edgecolor='#1565c0', linewidth=1.3)
ax.add_patch(dht_box)
ax.text(83.5, 83, "Đọc T, RH (kiểm tra", ha='center', va='center', fontsize=8.2, color='#0d47a1', fontfamily='Calibri')
ax.text(83.5, 80.2, "hợp lệ, bù −2 °C)", ha='center', va='center', fontsize=8.2, color='#0d47a1', fontfamily='Calibri')

# Reconnect DHT branch to main trunk
ax.plot([83.5, 83.5, 50], [78.5, 75.5, 75.5], color='#1565c0', lw=1.3)

# Diamond downwards "Không"
ax.text(44, 76, "Không", fontsize=8.5, color='#0d47a1', fontfamily='Calibri')
draw_arrow(77.5, 73.5)

# Loop boxes
draw_box(70, "ADC A0: Oversampling 20 mẫu (chu kỳ 400 µs) → chống đơ, độ mịn cao", h=6.5, w=74)
draw_arrow(66.5, 62.5)

draw_box(59, "Vout → Rs = RL(5 − Vout)/Vout → ×kEnvAdapt → ÷COR(T, RH)", h=6.5, w=74)
draw_arrow(55.5, 51.5)

draw_box(48, "ppm = 400,0 · (Rs / R0)^−2,76903 → phản chiếu dao động tức thì", h=6.5, w=74)
draw_arrow(44.5, 40.5)

draw_box(37, "Vi dao động tự nhiên cho 6 kênh: CO₂, PWM, ADC, T, RH, DHT Live", h=6.5, w=74)
draw_arrow(33.5, 29.5)

draw_box(26, "Phân cấp TỐT / TB / CAO / NGUY với trễ Schmitt 25 ppm", h=6.5, w=74)
draw_arrow(22.5, 18.5)

draw_box(15, "PWM mục tiêu: 0 · 90–145 · 150–210 · 255 | Soft-ramp: +14 / −6", h=6.5, w=74)
draw_arrow(11.5, 7.5)

draw_box(4, "In Serial 6 kênh chuẩn Plotter · LCD 1602 · delay(100 ms)", h=6.5, w=74, color='#bbdefb', border='#0d47a1', bold=True)

# Loopback line from bottom to loop start
ax.plot([50, 9, 9, 50], [0.5, 0.5, 86.5, 86.5], color='#0d47a1', lw=1.6)
ax.annotate('', xy=(50, 86.5), xytext=(40, 86.5),
            arrowprops=dict(arrowstyle="->", color='#0d47a1', lw=1.6, mutation_scale=12))

plt.tight_layout()
plt.savefig('c:/Users/thanh/denso_ws/new_luu_do.png', dpi=200, bbox_inches='tight')
plt.close()
print("Generated new_luu_do.png")

