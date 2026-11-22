import matplotlib.pyplot as plt
import numpy as np

# Set matplotlib parameters for clean scientific look
plt.rcParams['font.sans-serif'] = ['Calibri', 'Arial', 'DejaVu Sans']
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 0.8

# ==============================================================================
# 1. GENERATE new_dieu_khien_quat.png (Slide 10)
# ==============================================================================
fig, ax1 = plt.subplots(figsize=(10.5, 5.25), dpi=160)

# Background zones
ax1.axvspan(400, 650, color='#e8f5e9', alpha=0.7)  # TOT (Xanh nhạt)
ax1.axvspan(650, 950, color='#fffde7', alpha=0.8)  # TB (Vàng nhạt)
ax1.axvspan(950, 1300, color='#ffe0b2', alpha=0.6) # CAO (Cam nhạt)
ax1.axvspan(1300, 1800, color='#ffebee', alpha=0.8) # NGUY (Hồng nhạt)

# Zone labels
ax1.text(525, 270, 'TỐT', fontsize=12, fontweight='bold', color='#1b5e20', ha='center')
ax1.text(800, 270, 'TB', fontsize=12, fontweight='bold', color='#b78103', ha='center')
ax1.text(1125, 270, 'CAO', fontsize=12, fontweight='bold', color='#e65100', ha='center')
ax1.text(1550, 270, 'NGUY', fontsize=12, fontweight='bold', color='#b71c1c', ha='center')

# Grid
ax1.grid(True, linestyle='-', color='#e0e0e0', alpha=0.6)

# CO2 Tang curve
co2_tang = np.linspace(400, 1800, 1401)
pwm_tang = []
for c in co2_tang:
    if c < 650:
        pwm_tang.append(0)
    elif c < 950:
        pwm_tang.append(90 + (c - 650) * (145 - 90) / 300.0)
    elif c < 1300:
        pwm_tang.append(150 + (c - 950) * (210 - 150) / 350.0)
    else:
        pwm_tang.append(255)

# CO2 Giam curve (hysteresis 25 ppm)
co2_giam = np.linspace(1800, 400, 1401)
pwm_giam = []
for c in co2_giam:
    if c >= 1275:
        pwm_giam.append(255)
    elif c >= 925:
        clamped = max(950, min(1300, c))
        # in state 2:
        pwm_giam.append(150 + (c - 950) * (210 - 150) / 350.0)
    elif c >= 625:
        pwm_giam.append(90 + (c - 650) * (145 - 90) / 300.0)
    else:
        pwm_giam.append(0)

# Plot lines
ax1.plot(co2_tang, pwm_tang, color='#1565c0', linewidth=2.8, label=r'CO$_2$ tăng')
ax1.plot(co2_giam, pwm_giam, color='#e64a19', linestyle='--', linewidth=2.2, label=r'CO$_2$ giảm (trễ 25 ppm)')

ax1.set_xlim(400, 1800)
ax1.set_ylim(-5, 290)
ax1.set_xlabel(r'Nồng độ CO$_2$ tức thì sau bù nhiệt ẩm (ppm)', fontsize=11, fontweight='bold')
ax1.set_ylabel('PWM mục tiêu (0–255)', fontsize=11, fontweight='bold')

# Secondary y-axis for fan percent
ax2 = ax1.twinx()
ax2.set_ylim(-5 * 100 / 255.0, 290 * 100 / 255.0)
ax2.set_ylabel('Công suất quạt (%)', fontsize=11, fontweight='bold')

ax1.legend(loc='center left', fontsize=10.5, framealpha=0.9, edgecolor='#cccccc')
plt.tight_layout()
plt.savefig('c:/Users/thanh/denso_ws/new_dieu_khien_quat.png', dpi=160, bbox_inches='tight')
plt.close()
print("Generated new_dieu_khien_quat.png")

# ==============================================================================
# 2. GENERATE new_dac_tuyen_b.png (Slide 9)
# ==============================================================================
fig, ax = plt.subplots(figsize=(8.0, 5.25), dpi=160)

vout_arr = np.linspace(1.15, 2.40, 500)
R0 = 61.50
cor = 0.83407
kEnv = 1.74
RL = 10.0

co2_arr = []
for v in vout_arr:
    rs = RL * (5.0 - v) / v
    ppm = 400.0 * ((rs * kEnv) / (cor * R0)) ** (-2.76903)
    if ppm > 4000.0:
        ppm = 4000.0
    co2_arr.append(ppm)

ax.plot(vout_arr, co2_arr, color='#1565c0', linewidth=3.0)
ax.grid(True, linestyle='-', color='#e0e0e0', alpha=0.6)

# Threshold points: 650 ppm, 950 ppm, 1300 ppm
pts = [
    (650, 1.439, '#2e7d32', '650 ppm · 1,44 V (Bật quạt Mức 1 - TB)'),
    (950, 1.584, '#ef6c00', '950 ppm · 1,58 V (Quạt Mức 2 - CAO)'),
    (1300, 1.709, '#c62828', '1300 ppm · 1,71 V (Quạt Mức 3 - NGUY 100%)')
]

for ppm_val, v_val, col, lbl in pts:
    ax.axhline(ppm_val, color=col, linestyle='--', linewidth=1.2, alpha=0.85)
    ax.scatter([v_val], [ppm_val], color=col, s=60, zorder=5)
    ax.text(1.92, ppm_val + 45, lbl, color=col, fontsize=9.5, fontweight='bold', va='bottom')

ax.set_xlim(1.10, 2.45)
ax.set_ylim(200, 4200)
ax.set_xlabel('Điện áp ngõ ra AO, Vout (V)', fontsize=11, fontweight='bold')
ax.set_ylabel(r'CO$_2$ (ppm)', fontsize=11, fontweight='bold')
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

plt.tight_layout()
plt.savefig('c:/Users/thanh/denso_ws/new_dac_tuyen_b.png', dpi=160, bbox_inches='tight')
plt.close()
print("Generated new_dac_tuyen_b.png")

