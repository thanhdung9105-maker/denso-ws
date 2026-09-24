#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CHƯƠNG TRÌNH TÍNH ĐỘNG HỌC THUẬN TỪ BẢNG THAM SỐ D-H (STANDARD DENAVIT-HARTENBERG)
- Nhập bảng tham số D-H (theta, d, a, alpha)
- Tính ma trận biến đổi thuần nhất A_i từng bước và nhân tích lũy T_0_i = T_{i-1} * A_i
- Xuất tọa độ Cartesian [X, Y, Z] (mm & m), bán kính tầm vươn R, góc Euler Roll-Pitch-Yaw
"""

import math
import numpy as np


def calculate_fk_dh(dh_table, angle_unit="deg", length_unit="m"):
    """
    Tính động học thuận từ bảng Denavit-Hartenberg (Standard DH).

    Cấu trúc mỗi dòng trong dh_table:
      [theta, d, a, alpha]
      - theta: Góc quay quanh trục Z_{i-1}
      - d:     Độ dịch chuyển dọc trục Z_{i-1}
      - a:     Độ dài khâu (khoảng cách dọc trục X_i)
      - alpha: Góc xoắn quanh trục X_i
    """
    T = np.eye(4)
    scale_to_mm = 1000.0 if length_unit == "m" else 1.0
    scale_to_m = 1.0 if length_unit == "m" else 0.001

    print("=" * 84)
    print(f"{'Khâu':<6} | {'theta (' + angle_unit + ')':<12} | {'d (' + length_unit + ')':<10} | {'a (' + length_unit + ')':<10} | {'alpha (' + angle_unit + ')':<12} | Tọa độ sau bước [X, Y, Z] (mm)")
    print("-" * 84)
    print(f"{'Base':<6} | {'-':<12} | {'-':<10} | {'-':<10} | {'-':<12} | [   +0.00,    +0.00,    +0.00] mm")

    for i, row in enumerate(dh_table, 1):
        th_val, d_val, a_val, al_val = row

        # Chuyển sang radian nếu góc nhập là độ (deg)
        th = math.radians(th_val) if angle_unit == "deg" else th_val
        al = math.radians(al_val) if angle_unit == "deg" else al_val

        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)

        # Ma trận Standard DH: Rot(Z, th) * Trans(Z, d) * Trans(X, a) * Rot(X, al)
        A_i = np.array([
            [ct, -st * ca,  st * sa, a_val * ct],
            [st,  ct * ca, -ct * sa, a_val * st],
            [0.0,     sa,      ca,        d_val],
            [0.0,    0.0,     0.0,          1.0]
        ])

        # Nhân ma trận tích lũy: T_i = T_{i-1} * A_i
        T = T @ A_i

        pos_mm = T[:3, 3] * scale_to_mm
        print(f"Link {i:<1} | {th_val:<12.1f} | {d_val:<10.3f} | {a_val:<10.3f} | {al_val:<12.1f} | [{pos_mm[0]:+8.2f}, {pos_mm[1]:+8.2f}, {pos_mm[2]:+8.2f}] mm")

    print("=" * 84)

    # Tọa độ khâu công tác cuối cùng (End-Effector)
    pos_end_mm = T[:3, 3] * scale_to_mm
    pos_end_m = T[:3, 3] * scale_to_m

    # Tính góc Euler Roll-Pitch-Yaw (ZYX) từ ma trận quay 3x3
    R = T[:3, :3]
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.atan2(-R[2, 0], sy))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    else:
        roll = math.degrees(math.atan2(-R[1, 2], R[1, 1]))
        pitch = math.degrees(math.atan2(-R[2, 0], sy))
        yaw = 0.0

    print("\n🎯 TỌA ĐỘ KHÂU CUỐI (END-EFFECTOR CARTESIAN):")
    print(f"  ▸ X = {pos_end_mm[0]:+9.2f} mm  ({pos_end_m[0]:+8.4f} m)")
    print(f"  ▸ Y = {pos_end_mm[1]:+9.2f} mm  ({pos_end_m[1]:+8.4f} m)")
    print(f"  ▸ Z = {pos_end_mm[2]:+9.2f} mm  ({pos_end_m[2]:+8.4f} m)")
    print(f"  ▸ Bán kính tầm vươn R (Gốc đến TCP): {np.linalg.norm(pos_end_mm):.2f} mm")
    print(f"  ▸ Hướng Euler (Roll, Pitch, Yaw)  : [{roll:+.1f}°, {pitch:+.1f}°, {yaw:+.1f}°]")

    print("\n📐 MA TRẬN BIẾN ĐỔI ĐỒNG NHẤT T_0_n (4x4):")
    np.set_printoptions(precision=4, suppress=True)
    print(T)
    return pos_end_mm, T



def input_interactive():
    print('--- CHẾ ĐỘ NHẬP TỪNG KHÂU TỪ BÀN PHÍM ---')
    try:
        n = int(input('Nhập số lượng khâu (khớp) của robot: '))
    except ValueError:
        print('Giá trị không hợp lệ!')
        return None
    
    table = []
    print('Nhập theo thứ tự: theta(độ) d(m) a(m) alpha(độ) cách nhau bởi dấu cách:')
    for i in range(1, n + 1):
        while True:
            try:
                line = input(f'Khâu {i}: ').strip()
                parts = [float(x) for x in line.split()]
                if len(parts) != 4:
                    print('Vui lòng nhập đúng 4 giá trị: theta d a alpha')
                    continue
                table.append(parts)
                break
            except Exception as e:
                print('Lỗi nhập liệu, vui lòng nhập lại!')
    return table

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] in ['-i', '--interactive', 'nhap']:
        dh_table = input_interactive()
        if dh_table:
            calculate_fk_dh(dh_table, angle_unit='deg', length_unit='m')
    else:
        # BẢNG THAM SỐ D-H MẪU (DENSO VS-6556):
        # Mỗi dòng: [theta (độ), d (m), a (m), alpha (độ)]
        dh_table = [
            [   0.0,  0.335, -0.075, -90.0],  # Khâu 1: Đế -> Vai J2
            [ -90.0,  0.000,  0.270,   0.0],  # Khâu 2: Cánh tay trên (J2 -> J3)
            [  90.0,  0.000,  0.090,  90.0],  # Khâu 3: Khuỷu lệch (J3 -> Cẳng tay)
            [   0.0,  0.295,  0.000, -90.0],  # Khâu 4: Cẳng tay ( -> Point P)
            [ -90.0,  0.000,  0.080,   0.0],  # Khâu 5: Khâu cuối (Point P -> Flange)
        ]
        calculate_fk_dh(dh_table, angle_unit='deg', length_unit='m')
