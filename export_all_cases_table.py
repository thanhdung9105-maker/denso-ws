#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XUẤT BẢNG SỐ LIỆU ĐA TRƯỜNG HỢP: KHỚP UP, KHỚP DOWN, LẬT CỔ TAY, NGHIÊNG BÚT, CỰ LY BÀN
Dành cho 4 mẫu G-code: Hình Vuông 80x80, Vòng Tròn R=40, Ngôi Sao 5 Cánh, Tam Giác Đều
Động cơ: [750W, 400W, 200W, 80W, 50W, 50W] | Khối lượng: 22.5 kg
"""

import os
import sys
import csv
import numpy as np

sys.path.append(r'c:\Users\thanh\denso_ws\src\denso_gazebo\scripts')
from denso_gcode_energy_optimizer import (
    DensoGCodeEnergyOptimizer, BUILTIN_GCODE_PATTERNS,
    CASE_DEFINITIONS, MOTOR_RATED_POWERS, TORQUE_LIMITS
)

def export_all():
    opt = DensoGCodeEnergyOptimizer()
    out_dir = r'c:\Users\thanh\denso_ws\report_figures'
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "Bang_So_Sanh_Cac_Truong_Hop_Nang_Luong.csv")

    all_rows = []

    print("=" * 105)
    print("BẢNG TỔNG HỢP SO SÁNH NĂNG LƯỢNG & MÔ-MEN QUAY THEO CÁC TRƯỜNG HỢP CẤU HÌNH (DENSO VS-6556)")
    print("=" * 105)

    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(["BẢNG TỔNG HỢP SO SÁNH NĂNG LƯỢNG ROBOT VẼ G-CODE THEO CÁC TRƯỜNG HỢP"])
        writer.writerow(["Động cơ định mức (W)", "J1: 750W", "J2: 400W", "J3: 200W", "J4: 80W", "J5: 50W", "J6: 50W"])
        writer.writerow([])
        writer.writerow([
            "Mẫu G-Code", "Trường Hợp", "Cấu Hình Khớp", "Tổng NL (J)",
            "NL Cơ Học (J)", "Tổn Hao Joule (J)", "Tiết Kiệm (%)",
            "Max Tau J1 (Nm)", "Max Tau J2 (Nm)", "Max Tau J3 (Nm)",
            "Max Power (W)", "Tải Đỉnh (%)", "Đánh Giá An Toàn"
        ])

        for pat_k, pat_v in BUILTIN_GCODE_PATTERNS.items():
            print(f"\n--- MẪU G-CODE: {pat_v['name'].upper()} ---")
            comp = opt.compare_all_cases(pat_v['gcode'], step_mm=2.5, dt=0.02)
            results = comp["results"]

            print(f"{'Trường Hợp Khảo Sát':<38} | {'Tổng NL':<10} | {'Tiết Kiệm':<10} | {'Max Tau J2':<12} | {'Max Tau J1':<12} | {'Tải Đỉnh':<10}")
            print("-" * 105)

            for case_k, case_res in results.items():
                p_tau = case_res["peak_torques_nm"]
                e_tot = case_res["e_total_j"]
                e_mech = case_res["e_mech_total_j"]
                e_joule = case_res["e_joule_total_j"]
                saved = case_res.get("saved_vs_worst_pct", 0.0)
                max_p = float(np.max(case_res["peak_powers_w"]))
                max_load = float(np.max(case_res["power_utilization_pct"]))
                safety = "An Toàn" if max_load < 60 else ("Trung Bình" if max_load <= 100 else "Quá Tải")

                print(f"{case_res['title']:<38} | {e_tot:7.1f} J  | {saved:6.1f} %   | {p_tau[1]:7.2f} Nm   | {p_tau[0]:7.2f} Nm   | {max_load:5.1f} %")

                writer.writerow([
                    pat_v["name"], case_res["title"], case_res["badge"],
                    f"{e_tot:.2f}", f"{e_mech:.2f}", f"{e_joule:.2f}", f"{saved:.1f}%",
                    f"{p_tau[0]:.2f}", f"{p_tau[1]:.2f}", f"{p_tau[2]:.2f}",
                    f"{max_p:.2f}", f"{max_load:.1f}%", safety
                ])

    print("\n" + "=" * 105)
    print(f"✅ Đã xuất đầy đủ bảng số liệu ra file CSV tại: {csv_path}")

if __name__ == '__main__':
    export_all()
