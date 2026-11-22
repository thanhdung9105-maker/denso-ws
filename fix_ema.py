import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10, 4.8), dpi=180)
ax = fig.add_axes([0, 0, 1, 1])
ax.axis('off')

line1_math = r"$\mathbf{ADC\ Oversampling:\ 20\ m\check{a}u\ (400\ \mu s/m\check{a}u)}$"
# Or even better, use regular text for the title so Vietnamese is 100% perfect:
ax.text(0.05, 0.86, "ADC Oversampling: 20 mẫu liên tục (400 µs/mẫu)", fontsize=21, color='#1e3a5f', va='center', fontfamily='Calibri', fontweight='bold')

line2 = r"$C_{live} = 400{,}0 \cdot \left( \frac{R_{s,comp}}{R_0} \right)^{-2{,}76903}$"
ax.text(0.05, 0.62, line2, fontsize=25, color='#1e3a5f', va='center')

desc1 = "• Khử hoàn toàn hiện tượng đơ cảm biến: số liệu và đồ thị uốn lượn liên tục"
desc2 = "• Trễ Schmitt Trigger 25 ppm chống rung giật quạt giữa các ngưỡng"
desc3 = "• Xuất đồng thời 6 kênh dữ liệu thời gian thực cho Arduino Serial Plotter"

ax.text(0.05, 0.40, desc1, fontsize=15.5, color='#37474f', va='center', fontfamily='Calibri')
ax.text(0.05, 0.24, desc2, fontsize=15.5, color='#37474f', va='center', fontfamily='Calibri')
ax.text(0.05, 0.08, desc3, fontsize=15.5, color='#37474f', va='center', fontfamily='Calibri')

plt.savefig('c:/Users/thanh/denso_ws/new_eq_ema.png', dpi=180, transparent=True)
plt.close()
print("Regenerated new_eq_ema.png")

