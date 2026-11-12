#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tạo hình minh họa trực quan chi tiết cho tất cả các điểm trong Phương trình (1), (2), (3)
của bài báo s41598-026-37676-8.pdf:
  - X_s, X_g, vector d = X_g - X_s, đoạn thẳng X(t) = X_s + t*d
  - Tâm chướng ngại vật C_k, bán kính r_k = d_k/2, biên an toàn r_k + delta, dự trữ epsilon
  - Điểm chiếu vuông góc P_k^* = X_s + t_k^* * d và khoảng cách d_k^* = ||P_k^* - C_k||
  - Điểm nút tránh né w_k (detour waypoint) đẩy dọc theo phương bán kính từ C_k qua P_k^*
  - Dãy điểm nút W = {X_s, w_1, w_2, w_3, X_g}, điểm điều khiển P_i và đường cong B-spline bậc ba C(u)
"""

import os
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from scipy.interpolate import CubicSpline


def generate_points_diagram(out_png_paths):
    plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False

    fig = plt.figure(figsize=(16.2, 12.4), dpi=240)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.08, 1.05], width_ratios=[1.02, 1.08], hspace=0.26, wspace=0.18)

    # =========================================================================
    # PANEL (A): SƠ ĐỒ NGUYÊN LÝ HÌNH HỌC PHÓNG TO TẠI 1 CHƯỚNG NGẠI VẬT THỨ k
    # (Giải thích từng ký hiệu trong Phương trình 1, Phương trình 2, Phương trình 3)
    # =========================================================================
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_facecolor('#f8fafc')

    Xs_schem = np.array([1.0, 1.3])
    Xg_schem = np.array([9.2, 6.4])
    d_vec = Xg_schem - Xs_schem
    d_norm2 = np.dot(d_vec, d_vec)

    Ck_schem = np.array([5.5, 2.9])
    rk = 1.30
    delta = 0.65
    eps = 0.45

    # Chiếu vuông góc C_k lên đoạn XsXg (Phương trình 1)
    tk_star = np.clip(np.dot(Ck_schem - Xs_schem, d_vec) / d_norm2, 0.0, 1.0)
    Pk_star = Xs_schem + tk_star * d_vec

    # Vector từ C_k qua P_k^*
    vec_CP = Pk_star - Ck_schem
    dk_star = np.linalg.norm(vec_CP)
    u_radial = vec_CP / dk_star

    # Điểm nút tránh né w_k (Phương trình 2): cách C_k một khoảng (r_k + delta + eps)
    push_dist = (rk + delta) - dk_star + eps
    wk_schem = Pk_star + push_dist * u_radial

    # Vẽ 3 vòng tròn đồng tâm tại C_k:
    c_eps = Circle(Ck_schem, rk + delta + eps, facecolor='#ecfdf5', edgecolor='#10b981',
                   linestyle=':', linewidth=1.8, alpha=0.75, zorder=1,
                   label='Mặt cầu cộng dự trữ ε: bán kính (r_k + δ + ε)')
    c_safe = Circle(Ck_schem, rk + delta, facecolor='#fef3c7', edgecolor='#f59e0b',
                    linestyle='--', linewidth=2.0, alpha=0.80, zorder=2,
                    label='Vùng an toàn mở rộng: bán kính (r_k + δ)')
    c_obs = Circle(Ck_schem, rk, facecolor='#ef4444', edgecolor='#991b1b',
                   linewidth=2.0, alpha=0.82, zorder=3,
                   label='Chướng ngại vật hình cầu k: tâm C_k, bán kính r_k = d_k/2')
    ax1.add_patch(c_eps)
    ax1.add_patch(c_safe)
    ax1.add_patch(c_obs)

    # Vẽ đoạn thẳng nối trực tiếp X(t) = X_s + t*d
    ax1.plot([Xs_schem[0], Xg_schem[0]], [Xs_schem[1], Xg_schem[1]],
             color='#475569', linestyle='--', linewidth=2.2, zorder=4,
             label='Đoạn thẳng trực tiếp X(t) = X_s + t·d (Xâm phạm vì d_k* < r_k + δ)')

    # Vẽ đường cong B-spline C(u) uốn qua điểm nút tránh né w_k
    ctrl_x = [Xs_schem[0], 2.5, wk_schem[0], 7.4, Xg_schem[0]]
    ctrl_y = [Xs_schem[1], 3.1, wk_schem[1], 5.9, Xg_schem[1]]
    t_s = np.linspace(0, 1, 200)
    cs_x = CubicSpline([0, 0.25, 0.5, 0.75, 1.0], ctrl_x)
    cs_y = CubicSpline([0, 0.25, 0.5, 0.75, 1.0], ctrl_y)
    ax1.plot(cs_x(t_s), cs_y(t_s), color='#059669', linewidth=3.0, zorder=6,
             label='Đường cong B-spline bậc ba C(u) (Phương trình 3) đi qua w_k')

    # Vẽ ký hiệu góc vuông tại P_k^*
    u_line = d_vec / np.linalg.norm(d_vec)
    sq_size = 0.27
    p_corner1 = Pk_star - sq_size * u_line
    p_corner2 = Pk_star - sq_size * u_line - sq_size * u_radial
    p_corner3 = Pk_star - sq_size * u_radial
    ax1.plot([p_corner1[0], p_corner2[0], p_corner3[0]],
             [p_corner1[1], p_corner2[1], p_corner3[1]],
             color='#1e293b', linewidth=1.5, zorder=7)

    # Vẽ đoạn d_k^* từ C_k đến P_k^*
    ax1.plot([Ck_schem[0], Pk_star[0]], [Ck_schem[1], Pk_star[1]],
             color='#1e3a8a', linewidth=2.4, zorder=7)
    mid_CP = 0.5 * (Ck_schem + Pk_star)
    ax1.annotate(
        'Khoảng cách gần nhất:\nd_k* = ||P_k* − C_k|| < r_k + δ',
        xy=(mid_CP[0], mid_CP[1]), xytext=(6.65, 3.15),
        fontsize=8.1, fontweight='bold', color='#1e3a8a',
        bbox=dict(boxstyle='round,pad=0.22', facecolor='white', edgecolor='#93c5fd', alpha=0.95),
        arrowprops=dict(arrowstyle='->', color='#1e3a8a', lw=1.4), zorder=11
    )

    # Vẽ mũi tên đẩy hướng kính từ P_k^* ra w_k (Phương trình 2)
    arrow_push = FancyArrowPatch(
        posA=tuple(Pk_star), posB=tuple(wk_schem),
        arrowstyle='-|>', mutation_scale=18, linewidth=2.6,
        color='#dc2626', zorder=9
    )
    ax1.add_patch(arrow_push)
    mid_Pw = 0.5 * (Pk_star + wk_schem)
    ax1.annotate(
        'Vector đẩy hướng kính (PT 2):\n[(r_k + δ) − d_k* + ε] · (P_k*−C_k)/||P_k*−C_k||',
        xy=(mid_Pw[0], mid_Pw[1]), xytext=(0.35, 4.55),
        fontsize=8.0, fontweight='bold', color='#991b1b',
        bbox=dict(boxstyle='round,pad=0.24', facecolor='#fef2f2', edgecolor='#f87171', alpha=0.95),
        arrowprops=dict(arrowstyle='->', color='#dc2626', lw=1.5), zorder=11
    )

    # Vẽ các bán kính minh họa r_k và r_k + delta
    ang_r = -np.pi / 3.2
    p_rk = Ck_schem + rk * np.array([np.cos(ang_r), np.sin(ang_r)])
    ax1.annotate('', xy=p_rk, xytext=Ck_schem,
                 arrowprops=dict(arrowstyle='->', color='white', lw=1.8), zorder=8)
    ax1.text(Ck_schem[0] + 0.15, Ck_schem[1] - 0.65, 'r_k = d_k/2', color='white',
             fontsize=8.4, fontweight='bold', zorder=9)

    ang_del = -np.pi / 6.5
    p_rdel = Ck_schem + (rk + delta) * np.array([np.cos(ang_del), np.sin(ang_del)])
    ax1.annotate('', xy=p_rdel, xytext=Ck_schem,
                 arrowprops=dict(arrowstyle='->', color='#b45309', lw=1.6, ls='--'), zorder=8)
    ax1.text(Ck_schem[0] + 1.25, Ck_schem[1] - 1.15, 'Biên an toàn: r_k + δ', color='#92400e',
             fontsize=8.2, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.18', facecolor='#fef3c7', edgecolor='#f59e0b', alpha=0.92), zorder=9)

    # Vẽ các điểm chính: X_s, X_g, C_k, P_k^*, w_k
    ax1.scatter(*Xs_schem, color='#16a34a', s=115, zorder=10, edgecolors='black', linewidth=1.2)
    ax1.text(Xs_schem[0] + 0.22, Xs_schem[1] - 0.52, 'X_s = (X_i, Y_i, Z_i)\n(Điểm xuất phát, t = 0)',
             fontsize=8.5, fontweight='bold', color='#15803d',
             bbox=dict(boxstyle='round,pad=0.18', facecolor='#f0fdf4', edgecolor='#86efac', alpha=0.92), zorder=10)

    ax1.scatter(*Xg_schem, color='#7c3aed', s=135, marker='*', zorder=10, edgecolors='black', linewidth=1.0)
    ax1.text(Xg_schem[0] - 2.65, Xg_schem[1] + 0.25, 'X_g = (X_f, Y_f, Z_f)\n(Điểm đích, t = 1)',
             fontsize=8.5, fontweight='bold', color='#6d28d9',
             bbox=dict(boxstyle='round,pad=0.18', facecolor='#f5f3ff', edgecolor='#c4b5fd', alpha=0.92), zorder=10)

    ax1.scatter(*Ck_schem, color='#1e293b', s=75, zorder=10, edgecolors='white', linewidth=1.2)
    ax1.text(Ck_schem[0] - 0.52, Ck_schem[1] - 0.15, 'C_k', fontsize=9.5, fontweight='bold', color='white', zorder=11)

    ax1.scatter(*Pk_star, color='#2563eb', s=95, zorder=10, edgecolors='white', linewidth=1.2)
    ax1.annotate(
        'P_k* = X_s + t_k*·d\n(Hình chiếu vuông góc - PT 1)',
        xy=(Pk_star[0], Pk_star[1]), xytext=(6.25, 4.15),
        fontsize=8.2, fontweight='bold', color='#1d4ed8',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#eff6ff', edgecolor='#60a5fa', alpha=0.95),
        arrowprops=dict(arrowstyle='->', color='#2563eb', lw=1.4), zorder=11
    )

    ax1.scatter(*wk_schem, color='#dc2626', s=115, marker='D', zorder=11, edgecolors='white', linewidth=1.2)
    ax1.annotate(
        'w_k (Detour Waypoint - PT 2)\nĐiểm nút tránh né ứng viên',
        xy=(wk_schem[0], wk_schem[1]), xytext=(4.65, 5.45),
        fontsize=8.4, fontweight='bold', color='#b91c1c',
        bbox=dict(boxstyle='round,pad=0.22', facecolor='#fef2f2', edgecolor='#ef4444', alpha=0.95),
        arrowprops=dict(arrowstyle='->', color='#dc2626', lw=1.5), zorder=11
    )

    # Mũi tên chỉ vector hướng d = X_g - X_s
    p_d_start = Xs_schem + 0.15 * d_vec
    p_d_end = Xs_schem + 0.33 * d_vec
    ax1.annotate('', xy=p_d_end, xytext=p_d_start,
                 arrowprops=dict(arrowstyle='->', color='#334155', lw=2.3), zorder=8)
    ax1.text(p_d_start[0] + 0.55, p_d_start[1] - 0.15, 'Vector d = X_g − X_s',
             fontsize=8.3, fontweight='bold', color='#334155',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='#cbd5e1', alpha=0.9), zorder=9)

    ax1.set_xlim(0.1, 10.4)
    ax1.set_ylim(0.3, 8.2)
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_title('a) Sơ đồ Nguyên lý Hình học Phương trình (1), (2), (3):\nChiếu vuông góc C_k → P_k* và Đẩy hướng kính P_k* → Điểm tránh né w_k',
                  fontsize=10.2, fontweight='bold', color='#0f172a', pad=8)
    ax1.set_xlabel('Không gian Tác vụ X [Đơn vị quy chuẩn]', fontsize=9)
    ax1.set_ylabel('Không gian Tác vụ Y [Đơn vị quy chuẩn]', fontsize=9)
    ax1.grid(True, linestyle='--', alpha=0.35)
    ax1.legend(loc='upper left', fontsize=7.3, framealpha=0.95)

    # =========================================================================
    # PANEL (B): TỌA ĐỘ THỰC TẾ TRONG ROS 2 TRÊN MẶT PHẲNG XY CHO CẢ 3 VẬT CẢN
    # (Hiển thị đầy đủ X_s, X_g, C_1..3, P_1^*..P_3^*, w_1..3, W và đường B-spline C(u))
    # =========================================================================
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_facecolor('#f8fafc')

    Xs = np.array([-310.0, -170.0, 220.0])
    Xg = np.array([-460.0,  170.0, 220.0])
    d_real = Xg - Xs
    d_real_norm2 = np.dot(d_real, d_real)

    obs_list = [
        {"name": "C₁", "c": np.array([-360.0, -50.0, 235.0]), "r": 35.0, "w": np.array([-463.0, -50.0, 220.0])},
        {"name": "C₂", "c": np.array([-390.0,  20.0, 245.0]), "r": 40.0, "w": np.array([-498.0,  20.0, 220.0])},
        {"name": "C₃", "c": np.array([-420.0,  80.0, 235.0]), "r": 35.0, "w": np.array([-523.0,  80.0, 220.0])},
    ]
    delta_mm = 50.0

    # Vẽ đoạn thẳng nối trực tiếp X_s -> X_g
    ax2.plot([Xs[0], Xg[0]], [Xs[1], Xg[1]], color='#64748b', linestyle='--', linewidth=2.0, zorder=3,
             label='Đoạn thẳng X(t) = X_s + t·d (Xuyên qua 3 vật cản)')

    Pk_list = []
    tk_list = []
    c_label_offsets = [(-225.0, -70.0), (-225.0, 15.0), (-225.0, 95.0)]
    w_label_offsets = [(-575.0, -68.0), (-600.0, 18.0), (-605.0, 92.0)]

    for idx, ob in enumerate(obs_list):
        Ck = ob["c"]
        rk_mm = ob["r"]
        wk = ob["w"]
        tk = np.clip(np.dot(Ck - Xs, d_real) / d_real_norm2, 0.0, 1.0)
        Pk = Xs + tk * d_real
        Pk_list.append(Pk)
        tk_list.append(tk)

        # Vòng tròn vật cản r_k và biên an toàn r_k + delta
        c_s = Circle((Ck[0], Ck[1]), rk_mm + delta_mm, facecolor='#fef3c7', edgecolor='#f59e0b',
                     linestyle='--', linewidth=1.4, alpha=0.52, zorder=2,
                     label='Vùng an toàn (r_k + δ)' if idx == 0 else "")
        c_o = Circle((Ck[0], Ck[1]), rk_mm, facecolor='#ef4444', edgecolor='#991b1b',
                     linewidth=1.6, alpha=0.78, zorder=4,
                     label='Vật cản hình cầu C_k (bán kính r_k)' if idx == 0 else "")
        ax2.add_patch(c_s)
        ax2.add_patch(c_o)

        # Tâm C_k & Điểm chiếu P_k^*
        ax2.scatter(Ck[0], Ck[1], color='#1e293b', s=50, zorder=8, edgecolors='white')
        ax2.scatter(Pk[0], Pk[1], color='#2563eb', s=68, zorder=9, edgecolors='white',
                    label='Điểm chiếu vuông góc P_k* (PT 1)' if idx == 0 else "")

        # Chú thích C_k và P_k^* bên phải không bị chồng chữ
        tx_c, ty_c = c_label_offsets[idx]
        ax2.annotate(
            f"C_{idx+1}({Ck[0]:.0f}, {Ck[1]:.0f}), r_{idx+1}={rk_mm:.0f}mm\n"
            f"P_{idx+1}*({Pk[0]:.1f}, {Pk[1]:.1f}), t_{idx+1}*={tk:.3f}",
            xy=(Pk[0], Pk[1]), xytext=(tx_c, ty_c),
            fontsize=7.3, fontweight='bold', color='#1e3a8a', ha='center',
            bbox=dict(boxstyle='round,pad=0.18', facecolor='white', edgecolor='#93c5fd', alpha=0.94),
            arrowprops=dict(arrowstyle='->', color='#2563eb', lw=1.2), zorder=10
        )

        # Mũi tên đẩy từ P_k^* ra điểm tránh né w_k (Phương trình 2)
        arr = FancyArrowPatch(
            posA=(Pk[0], Pk[1]), posB=(wk[0], wk[1]),
            arrowstyle='-|>', mutation_scale=14, linewidth=2.0,
            color='#dc2626', linestyle='-', zorder=8
        )
        ax2.add_patch(arr)

        # Điểm nút tránh né w_k
        ax2.scatter(wk[0], wk[1], color='#dc2626', s=85, marker='D', zorder=10, edgecolors='white',
                    label='Điểm nút tránh né w_k (PT 2)' if idx == 0 else "")
        tx_w, ty_w = w_label_offsets[idx]
        ax2.annotate(
            f"w_{idx+1}({wk[0]:.0f}, {wk[1]:.0f})",
            xy=(wk[0], wk[1]), xytext=(tx_w, ty_w),
            fontsize=7.8, fontweight='bold', color='#b91c1c', ha='center',
            bbox=dict(boxstyle='round,pad=0.18', facecolor='#fef2f2', edgecolor='#f87171', alpha=0.94),
            arrowprops=dict(arrowstyle='->', color='#dc2626', lw=1.2), zorder=10
        )

    # Tập điểm nút có thứ tự W = {X_s, w_in, w_1, w_2, w_3, w_out, X_g} và Đường cong B-spline C(u) (Phương trình 3)
    w_in = np.array([-386.5, -120.0, 220.0])
    w_out = np.array([-491.5,  150.0, 220.0])
    W_seq = np.array([Xs, w_in, obs_list[0]["w"], obs_list[1]["w"], obs_list[2]["w"], w_out, Xg])

    ax2.plot(W_seq[:, 0], W_seq[:, 1], 'o:', color='#d97706', linewidth=1.5, markersize=5, zorder=6,
             label='Đa giác điều khiển P_i / Dãy điểm nút W')

    chord = np.zeros(len(W_seq))
    for i in range(1, len(W_seq)):
        chord[i] = chord[i-1] + np.linalg.norm(W_seq[i] - W_seq[i-1])
    u_knots = chord / chord[-1]
    u_fine = np.linspace(0, 1, 250)
    cs_bx = CubicSpline(u_knots, W_seq[:, 0], bc_type='natural')
    cs_by = CubicSpline(u_knots, W_seq[:, 1], bc_type='natural')
    ax2.plot(cs_bx(u_fine), cs_by(u_fine), color='#059669', linewidth=2.8, zorder=7,
             label='Đường cong B-spline bậc ba C(u) (PT 3 - Liên tục C²)')

    # Vẽ X_s và X_g
    ax2.scatter(Xs[0], Xs[1], color='#16a34a', s=105, zorder=10, edgecolors='black', label='Điểm xuất phát X_s (-310, -170, 220) mm')
    ax2.text(Xs[0] + 10, Xs[1] - 6, 'X_s (-310, -170)', fontsize=8.2, fontweight='bold', color='#15803d',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='#f0fdf4', edgecolor='#86efac', alpha=0.92))

    ax2.scatter(Xg[0], Xg[1], color='#7c3aed', s=125, marker='*', zorder=10, edgecolors='black', label='Điểm đích X_g (-460, +170, 220) mm')
    ax2.text(Xg[0] + 10, Xg[1] - 4, 'X_g (-460, +170)', fontsize=8.2, fontweight='bold', color='#6d28d9',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='#f5f3ff', edgecolor='#c4b5fd', alpha=0.92))

    ax2.set_xlim(-665, -135)
    ax2.set_ylim(-330, 215)
    ax2.set_aspect('equal', adjustable='box')
    ax2.set_title('b) Minh họa Tọa độ Thực tế trong ROS 2 (Mặt phẳng XY, Z = 220 mm):\nDãy điểm chiếu P₁*, P₂*, P₃* → Điểm tránh né w₁, w₂, w₃ và Đường cong B-spline C(u)',
                  fontsize=10.2, fontweight='bold', color='#0f172a', pad=8)
    ax2.set_xlabel('Tọa độ X [mm] (Hệ tọa độ gốc Robot DENSO VS-6556)', fontsize=9)
    ax2.set_ylabel('Tọa độ Y [mm]', fontsize=9)
    ax2.grid(True, linestyle='--', alpha=0.35)
    ax2.legend(loc='lower left', ncol=2, fontsize=6.9, framealpha=0.95)

    # =========================================================================
    # PANEL (C): PHỐI CẢNH KHÔNG GIAN 3D (X, Y, Z) CỦA CÁC ĐIỂM VÀ 3 MẶT CẦU
    # =========================================================================
    ax3 = fig.add_subplot(gs[1, 0], projection='3d')

    u_sph = np.linspace(0, 2 * np.pi, 24)
    v_sph = np.linspace(0, np.pi, 16)
    for idx, ob in enumerate(obs_list):
        Ck = ob["c"]
        rk_mm = ob["r"]
        xs = Ck[0] + rk_mm * np.outer(np.cos(u_sph), np.sin(v_sph))
        ys = Ck[1] + rk_mm * np.outer(np.sin(u_sph), np.sin(v_sph))
        zs = Ck[2] + rk_mm * np.outer(np.ones_like(u_sph), np.cos(v_sph))
        ax3.plot_surface(xs, ys, zs, color='#ef4444', alpha=0.45, edgecolor='#b91c1c', linewidth=0.2)

        r_safe = rk_mm + delta_mm
        xs_s = Ck[0] + r_safe * np.outer(np.cos(u_sph), np.sin(v_sph))
        ys_s = Ck[1] + r_safe * np.outer(np.sin(u_sph), np.sin(v_sph))
        zs_s = Ck[2] + r_safe * np.outer(np.ones_like(u_sph), np.cos(v_sph))
        ax3.plot_wireframe(xs_s, ys_s, zs_s, color='#f59e0b', alpha=0.20, linewidth=0.5, rstride=3, cstride=3)

        Pk = Pk_list[idx]
        wk = ob["w"]
        ax3.scatter([Ck[0]], [Ck[1]], [Ck[2]], color='#1e293b', s=35)
        ax3.scatter([Pk[0]], [Pk[1]], [Pk[2]], color='#2563eb', s=48, label='Điểm chiếu P_k*' if idx == 0 else "")
        ax3.scatter([wk[0]], [wk[1]], [wk[2]], color='#dc2626', s=68, marker='D', label='Điểm tránh né w_k' if idx == 0 else "")

        ax3.plot([Ck[0], Pk[0]], [Ck[1], Pk[1]], [Ck[2], Pk[2]], color='#1d4ed8', linewidth=1.6)
        ax3.plot([Pk[0], wk[0]], [Pk[1], wk[1]], [Pk[2], wk[2]], color='#dc2626', linewidth=2.2)

        ax3.text(Ck[0] + 8, Ck[1], Ck[2] + rk_mm + 10, f"C_{idx+1}", color='#991b1b', fontsize=8.5, fontweight='bold')
        ax3.text(wk[0] - 28, wk[1], wk[2] + 8, f"w_{idx+1}", color='#b91c1c', fontsize=8.5, fontweight='bold')

    ax3.plot([Xs[0], Xg[0]], [Xs[1], Xg[1]], [Xs[2], Xg[2]], '--', color='#475569', linewidth=2.0, label='Đoạn thẳng X_s → X_g')
    cs_bz = CubicSpline(u_knots, W_seq[:, 2], bc_type='natural')
    ax3.plot(cs_bx(u_fine), cs_by(u_fine), cs_bz(u_fine), '-', color='#059669', linewidth=3.0, label='Đường cong B-spline 3D C(u)')

    ax3.scatter([Xs[0]], [Xs[1]], [Xs[2]], color='#16a34a', s=80, label='Điểm đầu X_s')
    ax3.scatter([Xg[0]], [Xg[1]], [Xg[2]], color='#7c3aed', s=95, marker='*', label='Điểm đích X_g')
    ax3.text(Xs[0] + 10, Xs[1], Xs[2] + 10, 'X_s', color='#15803d', fontsize=9, fontweight='bold')
    ax3.text(Xg[0] + 10, Xg[1], Xg[2] + 10, 'X_g', color='#6d28d9', fontsize=9, fontweight='bold')

    ax3.set_title('c) Phối cảnh Không gian 3 Chiều (X, Y, Z) [mm]:\nBa chướng ngại vật hình cầu {(C_k, d_k)} và Đường cong B-spline C(u)',
                  fontsize=10.2, fontweight='bold', color='#0f172a', pad=6)
    ax3.set_xlabel('X [mm]', fontsize=8.5)
    ax3.set_ylabel('Y [mm]', fontsize=8.5)
    ax3.set_zlabel('Z [mm]', fontsize=8.5)
    ax3.set_zlim(150, 325)
    ax3.view_init(elev=25, azim=-40)
    ax3.legend(loc='upper left', ncol=2, fontsize=7.2)

    # =========================================================================
    # PANEL (D): BẢNG TỔNG HỢP TỌA ĐỘ SỐ HỌC & Ý NGHĨA VẬT LÝ CỦA TẤT CẢ CÁC ĐIỂM
    # =========================================================================
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.axis('off')

    ax4.set_title('d) Bảng Giải thích Ký hiệu & Tọa độ Số học Thực tế của Các Điểm (Phương trình 1, 2, 3)',
                  fontsize=10.2, fontweight='bold', color='#0f172a', pad=6)

    table_rows = [
        ["Ký hiệu", "Tên gọi & Ý nghĩa hình học", "Công thức tính", "Giá trị thực tế trong ROS 2"],
        ["X_s", "Điểm xuất phát của đầu công tác\n(EE) tại thời điểm t = 0", "X_s = (X_i, Y_i, Z_i)", "[-310.0, -170.0, 220.0] mm"],
        ["X_g", "Điểm đích của đầu công tác\n(EE) tại thời điểm t = 1", "X_g = (X_f, Y_f, Z_f)", "[-460.0, +170.0, 220.0] mm"],
        ["d", "Vector hướng nối X_s → X_g\n(độ dài ||d|| = 371.6 mm)", "d = X_g − X_s", "[-150.0, +340.0,   0.0] mm"],
        ["C_1, r_1\nC_2, r_2\nC_3, r_3", "Tâm và bán kính 3 vật cản cầu\n(r_k = d_k/2, biên δ = 50 mm)", "{(C_k, d_k)}, k = 1,2,3",
         "C₁=(-360,-50,235), r₁=35mm\nC₂=(-390,+20,245), r₂=40mm\nC₃=(-420,+80,235), r₃=35mm"],
        ["t_k*", "Tham số hình chiếu vuông góc\ncủa tâm C_k trên đoạn X_s X_g", "clip_[0,1]([(C_k−X_s)·d]/||d||²)\n(Phương trình 1)",
         "t₁* = 0.3497 < t₂* = 0.5547\n< t₃* = 0.7350 ∈ [0, 1]"],
        ["P_1*\nP_2*\nP_3*", "Điểm trên đoạn X_s X_g nằm\ngần tâm vật cản C_k nhất", "P_k* = X_s + t_k* · d\n(Phương trình 1)",
         "P₁* = [-362.5, -51.1, 220] mm\nP₂* = [-393.2, +18.6, 220] mm\nP₃* = [-420.2, +79.9, 220] mm"],
        ["d_k*", "Khoảng cách từ C_k đến X_s X_g\n(Va chạm khi d_k* < r_k + δ)", "d_k* = ||P_k* − C_k||\n(Kiểm tra xâm phạm)",
         "d₁*=15.2 < 85 mm (Va chạm)\nd₂*=25.2 < 90 mm (Va chạm)\nd₃*=15.0 < 85 mm (Va chạm)"],
        ["w_1\nw_2\nw_3", "Điểm nút tránh né (Detour)\nđẩy P_k* ra ngoài mặt cầu", "P_k* + [(r_k+δ)−d_k*+ε]·\n(P_k*−C_k)/||P_k*−C_k|| (PT 2)",
         "w₁ = [-463.0, -50.0, 220] mm\nw₂ = [-498.0, +20.0, 220] mm\nw₃ = [-523.0, +80.0, 220] mm"],
        ["W & C(u)", "Dãy điểm nút W và Đường cong\nB-spline bậc ba (p=3) liên tục C²", "W = {X_s, w_1..m, X_g}\nC(u) = Σ N_{i,p}(u)·P_i (PT 3)",
         "Nội suy trơn qua tập điểm W\nĐộ dài cung S_tot = 491.3 mm"],
    ]

    tbl = ax4.table(
        cellText=table_rows[1:],
        colLabels=table_rows[0],
        colWidths=[0.11, 0.30, 0.28, 0.31],
        loc='center',
        cellLoc='left',
        bbox=[0.0, 0.0, 1.0, 0.96]
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.2)

    for (row, col), cell in tbl.get_celld().items():
        cell.set_edgecolor('#cbd5e1')
        cell.set_linewidth(0.6)
        cell.PAD = 0.03
        if row == 0:
            cell.set_facecolor('#1e3a8a')
            cell.set_text_props(color='white', fontweight='bold', ha='center')
            cell.set_height(0.075)
        else:
            if row % 2 == 1:
                cell.set_facecolor('#ffffff')
            else:
                cell.set_facecolor('#f1f5f9')
            if col == 0:
                cell.set_text_props(fontweight='bold', color='#0f172a', ha='center')
            elif col == 2:
                cell.set_text_props(color='#1e3a8a', fontweight='bold')
            elif col == 3:
                cell.set_text_props(color='#065f46', fontweight='bold')

            if row in [4, 6, 7, 8]:
                cell.set_height(0.125)
            else:
                cell.set_height(0.088)

    fig.suptitle(
        "MINH HỌA TRỰC QUAN CÁC ĐIỂM TRONG QUY HOẠCH ĐƯỜNG ĐI B-SPLINE (PHƯƠNG TRÌNH 1, 2, 3 - BÀI BÁO s41598-026-37676-8)",
        fontsize=12.2, fontweight='bold', color='#0f172a', y=0.985
    )

    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.04, top=0.93, wspace=0.18, hspace=0.25)
    primary = out_png_paths[0]
    os.makedirs(os.path.dirname(primary), exist_ok=True)
    plt.savefig(primary, dpi=240)
    plt.close(fig)
    print(f"[OK] Saved illustration to: {primary}")

    for extra_path in out_png_paths[1:]:
        try:
            os.makedirs(os.path.dirname(extra_path), exist_ok=True)
            shutil.copy2(primary, extra_path)
            print(f"[OK] Copied illustration to: {extra_path}")
        except Exception as exc:
            print(f"[WARN] Could not copy to {extra_path}: {exc}")


if __name__ == "__main__":
    outputs = [
        "/tmp/paper_vn_figs/minh_hoa_cac_diem_eq1_eq2_eq3.png",
        "/mnt/c/Users/thanh/.gemini/antigravity/brain/f0a8135a-32cd-435f-aabd-865d87596aef/minh_hoa_cac_diem_eq1_eq2_eq3.png",
        "/mnt/c/Users/thanh/OneDrive/Máy tính/Hinh_Minh_Hoa_Cac_Diem_Eq1_Eq2_Eq3.png",
        "/mnt/c/Users/thanh/Desktop/Hinh_Minh_Hoa_Cac_Diem_Eq1_Eq2_Eq3.png",
    ]
    generate_points_diagram(outputs)
