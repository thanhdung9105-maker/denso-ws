#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TỔNG HỢP & XUẤT ĐỒ THỊ KHOA HỌC: TỐI ƯU HÓA NĂNG LƯỢNG VẼ G-CODE
CÁNH TAY ROBOT 6-DOF (DENSO VS-6556) TRONG ROS 2 GAZEBO
Công suất định mức 6 động cơ: [750W, 400W, 200W, 80W, 50W, 50W]
Khối lượng các khâu: [7.5, 6.3, 4.8, 2.6, 1.0, 0.3] kg (Tổng: 22.5 kg)
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(r'c:\Users\thanh\denso_ws\src\denso_gazebo\scripts')
sys.path.append(r'c:\Users\thanh\denso_ws\src\denso_vs6556_moveit_config\scripts')

from denso_gcode_energy_optimizer import (
    DensoGCodeEnergyOptimizer, BUILTIN_GCODE_PATTERNS,
    MOTOR_RATED_POWERS, LINK_MASSES, TORQUE_LIMITS
)

def generate_scientific_benchmark():
    optimizer = DensoGCodeEnergyOptimizer()
    out_dir = r'c:\Users\thanh\denso_ws\report_figures'
    os.makedirs(out_dir, exist_ok=True)
    
    # 1. Chạy phân tích mẫu Hình Vuông 80x80mm
    print("=" * 80)
    print("CHẠY PHÂN TÍCH TỐI ƯU NĂNG LƯỢNG VẼ G-CODE 6-DOF (HÌNH VUÔNG 80x80MM)")
    print("=" * 80)
    
    gcode_square = BUILTIN_GCODE_PATTERNS["square"]["gcode"]
    comp = optimizer.compare_all_configurations(gcode_square, step_mm=2.5, dt=0.02)
    
    results = comp["results"]
    best_k = comp["best_mode"]
    worst_k = comp["worst_mode"]
    
    best_res = results[best_k]
    worst_res = results[worst_k]
    
    print(f"\n🏆 DẠNG TỐI ƯU NHẤT: {best_res['title']}")
    print(f"🔋 Năng lượng dạng tối ưu: {comp['best_energy_j']:.1f} J")
    print(f"⚠️ Năng lượng dạng hao phí nhất: {comp['worst_energy_j']:.1f} J")
    print(f"⚡ Mức tiết kiệm: -{comp['energy_saved_pct']:.1f} %")
    
    # 2. Xuất đồ thị khoa học 4 khung hình (Multi-Panel Figure)
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), dpi=300)
    
    t_vec = best_res["time_vec"]
    
    # Subplot 1: Mô-men quay RNEA tau(t) của các khớp chính (J1, J2, J3)
    ax1 = axes[0, 0]
    ax1.plot(t_vec, best_res["tau_traj"][:, 0], 'b-', linewidth=1.8, label="J1 (Đế 750W) - Tối Ưu")
    ax1.plot(t_vec, best_res["tau_traj"][:, 1], 'r-', linewidth=2.0, label="J2 (Vai 400W) - Tối Ưu")
    ax1.plot(t_vec, best_res["tau_traj"][:, 2], 'g-', linewidth=1.8, label="J3 (Khuỷu 200W) - Tối Ưu")
    ax1.plot(worst_res["time_vec"], worst_res["tau_traj"][:, 1], 'r--', alpha=0.5, label="J2 (Vai 400W) - Hao Phí")
    ax1.plot(worst_res["time_vec"], worst_res["tau_traj"][:, 2], 'g--', alpha=0.5, label="J3 (Khuỷu 200W) - Hao Phí")
    ax1.set_title("Mô-Men Quay RNEA τ_i(t) Các Khớp Chịu Tải Chính", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Thời gian quỹ đạo (s)", fontsize=10)
    ax1.set_ylabel("Mô-men quay (N·m)", fontsize=10)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, linestyle=':', alpha=0.6)
    
    # Subplot 2: Công suất tức thời P(t) so với công suất định mức
    ax2 = axes[0, 1]
    p_tot_best = np.sum(best_res["p_total_traj"], axis=1)
    p_tot_worst = np.sum(worst_res["p_total_traj"], axis=1)
    ax2.plot(t_vec, p_tot_best, 'g-', linewidth=2.2, label=f"Tổng Công Suất: Tối Ưu ({np.max(p_tot_best):.1f}W)")
    ax2.plot(worst_res["time_vec"], p_tot_worst, 'r--', linewidth=1.8, label=f"Tổng Công Suất: Hao Phí ({np.max(p_tot_worst):.1f}W)")
    ax2.axhline(y=400.0, color='orange', linestyle=':', label="Định mức J2 (400W)")
    ax2.axhline(y=750.0, color='purple', linestyle=':', label="Định mức J1 (750W)")
    ax2.set_title("Công Suất Tiêu Thụ Tức Thời P_total(t) Toàn Cánh Tay", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Thời gian quỹ đạo (s)", fontsize=10)
    ax2.set_ylabel("Công suất điện (Watts)", fontsize=10)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, linestyle=':', alpha=0.6)
    
    # Subplot 3: Năng lượng tích lũy E(t) = integral(P dt)
    ax3 = axes[1, 0]
    dt = float(t_vec[1] - t_vec[0])
    e_cum_best = np.cumsum(p_tot_best) * dt
    e_cum_worst = np.cumsum(p_tot_worst) * dt
    ax3.plot(t_vec, e_cum_best, 'g-', linewidth=2.5, label=f"Dạng Tối Ưu ({comp['best_energy_j']:.1f} J)")
    ax3.plot(worst_res["time_vec"], e_cum_worst, 'r--', linewidth=2.0, label=f"Dạng Hao Phí ({comp['worst_energy_j']:.1f} J)")
    ax3.fill_between(t_vec, e_cum_best, e_cum_worst, color='green', alpha=0.15, label=f"Tiết Kiệm -{comp['energy_saved_pct']:.1f}%")
    ax3.set_title("Đường Cong Năng Lượng Tích Lũy E(t) = ∫ Σ P_i dt", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Thời gian quỹ đạo (s)", fontsize=10)
    ax3.set_ylabel("Năng lượng tiêu hao (Joules)", fontsize=10)
    ax3.legend(loc='upper left', fontsize=9)
    ax3.grid(True, linestyle=':', alpha=0.6)
    
    # Subplot 4: Biểu đồ cột so sánh tất cả các dạng cấu hình
    ax4 = axes[1, 1]
    modes_labels = [v["title"].split(":")[1].split("(")[0].strip() for v in results.values()]
    energies = [v["e_total_j"] for v in results.values()]
    colors_bar = ['#22c55e' if v.get("is_best") else ('#ef4444' if k == worst_k else '#38bdf8') for k, v in results.items()]
    
    bars = ax4.bar(modes_labels, energies, color=colors_bar, width=0.55, edgecolor='black', linewidth=0.8)
    for b, e in zip(bars, energies):
        ax4.text(b.get_x() + b.get_width()/2.0, e + 5, f"{e:.1f} J", ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax4.set_title("So Sánh Tổng Năng Lượng Giữa Các Dạng Cấu Hình", fontsize=11, fontweight='bold')
    ax4.set_ylabel("Tổng Năng Lượng (Joules)", fontsize=10)
    ax4.set_ylim(0, max(energies) * 1.18)
    plt.setp(ax4.get_xticklabels(), rotation=15, ha='right', fontsize=9)
    ax4.grid(True, linestyle=':', alpha=0.6, axis='y')
    
    fig.suptitle("NGHIÊN CỨU TỐI ƯU HÓA NĂNG LƯỢNG VẼ G-CODE THEO CẤU HÌNH ĐỘNG HỌC & ĐỘNG LỰC HỌC\nRobot Denso VS-6556 6-DOF | Động cơ: [750W, 400W, 200W, 80W, 50W, 50W]",
                 fontsize=12, fontweight='bold', y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    
    chart_path = os.path.join(out_dir, "denso_gcode_energy_optimization_analysis.png")
    fig.savefig(chart_path, dpi=300)
    plt.close(fig)
    print(f"\n✅ Đã xuất đồ thị khoa học 300 DPI tại: {chart_path}")

if __name__ == '__main__':
    generate_scientific_benchmark()
