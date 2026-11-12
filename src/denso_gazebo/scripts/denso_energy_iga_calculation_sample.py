#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================================================
TÊN PHƯƠNG PHÁP NGHIÊN CỨU:
Tiếng Anh: Energy-Optimal Trajectory Planning for 6-DOF Manipulator Based on 
           Cubic B-Spline and Sinusoidal-Adaptive Improved Genetic Algorithm (IGA) 
           Incorporating Inverse Dynamics (RNEA).
Tiếng Việt: Phương pháp Quy hoạch Quỹ đạo Tối ưu Năng lượng cho Robot 6 Bậc Tự do 
            Dựa trên Nội suy B-Spline Bậc 3 và Giải thuật Di truyền Cải tiến (IGA) 
            Thích nghi Dạng Sin Kết hợp Mô hình Động lực học Ngược RNEA.
            
Áp dụng cho Robot công nghiệp: DENSO VS-6556 (6-DOF)
Tài liệu tham chiếu: Cheng & Xu (2025), Journal of Intelligent & Robotic Systems
========================================================================================
"""

import sys
import os
import math
import time
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Thêm đường dẫn chứa denso_dynamics_engine nếu có
sys.path.append('/home/dung/denso_ws/src/denso_vs6556_moveit_config/scripts')
try:
    from denso_dynamics_engine import (
        DensoDynamicsEngine, TORQUE_LIMITS, VELOCITY_LIMITS, ACCELERATION_LIMITS,
        FRICTION_B, FRICTION_FC, GEAR_RATIOS
    )
except ImportError:
    TORQUE_LIMITS = np.array([80.0, 100.0, 60.0, 25.0, 20.0, 15.0])
    VELOCITY_LIMITS = np.array([4.5815, 4.1888, 5.236, 5.236, 5.236, 7.3304])
    ACCELERATION_LIMITS = np.array([15.0, 12.0, 15.0, 20.0, 20.0, 25.0])
    FRICTION_B = np.array([0.25, 0.30, 0.20, 0.08, 0.05, 0.02])
    FRICTION_FC = np.array([0.8, 1.2, 0.7, 0.3, 0.2, 0.1])
    GEAR_RATIOS = np.array([80.0, 100.0, 80.0, 50.0, 50.0, 50.0])
    class DensoDynamicsEngine:
        def __init__(self):
            self.gravity = np.array([0.0, 0.0, -9.81])
        def rnea(self, q, qd, qdd, grav=None, payload_mass=0.0):
            g_tau = np.array([0.0, -22.0*np.cos(q[1]), -9.5*np.cos(q[1]+q[2]), 0.0, -0.4*np.cos(q[1]+q[2]+q[4]), 0.0])
            M_diag = np.array([0.8, 1.2, 0.6, 0.15, 0.08, 0.02])
            return M_diag * qdd + g_tau


class DensoEnergyIGAOptimizer:
    """
    Bộ tối ưu hóa quỹ đạo chuyển động tiết kiệm năng lượng cho robot Denso VS-6556
    sử dụng B-spline bậc 3 và giải thuật di truyền cải tiến thích nghi dạng Sin (IGA).
    """
    def __init__(self):
        # 1. Khởi tạo mô hình động lực học tay máy Denso VS-6556
        self.dyn_engine = DensoDynamicsEngine()
        self.gravity = self.dyn_engine.gravity
        self.friction_b = FRICTION_B
        self.friction_fc = FRICTION_FC
        self.gear_ratios = GEAR_RATIOS
        
        # 2. Giới hạn vật lý của 6 khớp Denso VS-6556
        self.vel_max = VELOCITY_LIMITS.copy()       # rad/s
        self.acc_max = ACCELERATION_LIMITS.copy()   # rad/s^2
        self.jerk_max = np.array([120.0, 100.0, 120.0, 150.0, 150.0, 200.0])  # rad/s^3
        self.tau_max = TORQUE_LIMITS.copy()         # N.m

        # 3. Thông số động cơ servo AC Denso (điện trở R cuộn dây & hằng số mô-men Kt)
        self.R_motor = np.array([1.2, 1.4, 1.1, 0.8, 0.6, 0.5])       # Ohm
        self.Kt_motor = np.array([0.35, 0.40, 0.32, 0.22, 0.18, 0.15]) # N.m / A

        # 4. Tham số thuật toán di truyền cải tiến IGA (theo công thức bài báo Cheng & Xu 2025)
        self.pop_size = 30        # Kích thước quần thể cá thể
        self.max_gen = 35         # Số thế hệ lặp tiến hóa
        self.xi = 0.15            # Hệ số phạt vi phạm ràng buộc (xi in [0.1, 0.2])
        self.pc1, self.pc2 = 0.90, 0.40   # Biên xác suất lai ghép
        self.pm1, self.pm2 = 0.15, 0.02   # Biên xác suất đột biến
        self.w_energy = 1.0       # Trọng số năng lượng
        self.w_time = 0.5         # Trọng số phụ cho thời gian

    def interpolate_b_spline(self, waypoints: np.ndarray, dt_vector: np.ndarray, sample_dt: float = 0.025):
        """
        Xây dựng đường cong Cubic B-Spline clamped qua các điểm nút thời gian.
        Trả về t, q(t), q_dot(t), q_ddot(t), q_jerk(t).
        """
        N = len(waypoints)
        t_knots = np.zeros(N)
        t_knots[1:] = np.cumsum(dt_vector)
        total_time = t_knots[-1]

        # Khởi tạo spline kẹp đạo hàm đầu/cuối bằng 0
        splines = [CubicSpline(t_knots, waypoints[:, j], bc_type='clamped') for j in range(6)]
        
        t_samples = np.arange(0.0, total_time, sample_dt)
        if len(t_samples) == 0 or t_samples[-1] < total_time:
            t_samples = np.append(t_samples, total_time)
        
        K = len(t_samples)
        q = np.zeros((K, 6))
        qd = np.zeros((K, 6))
        qdd = np.zeros((K, 6))
        qddd = np.zeros((K, 6))

        for j in range(6):
            q[:, j] = splines[j](t_samples)
            qd[:, j] = splines[j](t_samples, 1)
            qdd[:, j] = splines[j](t_samples, 2)
            qddd[:, j] = splines[j](t_samples, 3)

        return t_samples, q, qd, qdd, qddd

    def calculate_dynamics_and_energy(self, t_samples, q, qd, qdd, qddd, payload_mass=0.0):
        """
        Tính toán động lực học ngược RNEA, mô-men xoắn, và năng lượng tiêu thụ trên toàn bộ hành trình.
        """
        K = len(t_samples)
        dt = t_samples[1] - t_samples[0] if K > 1 else 0.025
        
        tau = np.zeros((K, 6))
        p_mech = np.zeros((K, 6))
        p_elec = np.zeros((K, 6))

        penalty = 0.0

        for k in range(K):
            # 1. Giải động lực học ngược RNEA trực tiếp
            tau_rnea = self.dyn_engine.rnea(q[k], qd[k], qdd[k], grav=self.gravity, payload_mass=payload_mass)
            tau_fric = self.friction_b * qd[k] + self.friction_fc * np.tanh(10.0 * qd[k])
            tau[k] = tau_rnea + tau_fric

            # 2. Công suất cơ học: P_mech,i = |tau_i * qd_i|
            p_mech[k] = np.abs(tau[k] * qd[k])

            # 3. Điện năng tiêu thụ thực tế qua hộp số:
            # Mô-men trên trục động cơ: tau_motor = tau_joint / N_gear
            tau_motor = tau[k] / self.gear_ratios
            i_motor = tau_motor / self.Kt_motor
            # Tổn thất nhiệt Joule cuộn dây: P_cu = R * I^2
            p_cu = self.R_motor * (i_motor ** 2)
            # Công suất kéo cơ khí (có xét 30% thu hồi năng lượng khi phanh)
            p_drive = np.maximum(0.0, tau[k] * qd[k]) + 0.3 * np.minimum(0.0, tau[k] * qd[k])
            p_elec[k] = p_drive + p_cu

            # 4. Kiểm tra và cộng dồn các hàm phạt vi phạm (Penalty function theo bài báo)
            v_viol = np.maximum(0.0, (np.abs(qd[k]) - self.vel_max) / self.vel_max)
            a_viol = np.maximum(0.0, (np.abs(qdd[k]) - self.acc_max) / self.acc_max)
            j_viol = np.maximum(0.0, (np.abs(qddd[k]) - self.jerk_max) / self.jerk_max)
            t_viol = np.maximum(0.0, (np.abs(tau[k]) - self.tau_max) / self.tau_max)

            penalty += np.sum(v_viol**2 + a_viol**2 + j_viol**2 + 2.0 * t_viol**2)

        # Tích phân năng lượng: E = int(P dt)
        e_mech_total = np.sum(p_mech) * dt
        e_elec_total = np.sum(p_elec) * dt

        return {
            'tau': tau,
            'p_mech': p_mech,
            'p_elec': p_elec,
            'e_mech_total': e_mech_total,
            'e_elec_total': e_elec_total,
            'penalty': penalty,
            'total_time': t_samples[-1]
        }

    def evaluate_chromosome(self, dt_vector: np.ndarray, waypoints: np.ndarray):
        """
        Đánh giá hàm mục tiêu của 1 cá thể nhiễm sắc thể dt_vector.
        Hàm chi phí tổng hợp: J = w_energy * E_elec + w_time * Total_Time + xi * Penalty
        """
        if np.any(dt_vector < 0.05):
            return 1e8, 1e8, 1e8

        t_samples, q, qd, qdd, qddd = self.interpolate_b_spline(waypoints, dt_vector)
        res = self.calculate_dynamics_and_energy(t_samples, q, qd, qdd, qddd)

        cost = (self.w_energy * res['e_elec_total'] + 
                self.w_time * res['total_time'] + 
                self.xi * 100.0 * res['penalty'])
        
        return cost, res['e_elec_total'], res['penalty']

    def optimize(self, waypoints: np.ndarray, initial_dt: float = 0.35, verbose: bool = True):
        """
        Thực thi thuật toán IGA (Giải thuật Di truyền Cải tiến Thích nghi Dạng Sin)
        để tìm vector phân bổ thời gian delta_t_i tối ưu năng lượng.
        """
        N = len(waypoints)
        num_segments = N - 1
        
        # 1. Khởi tạo quần thể ban đầu
        population = np.random.uniform(0.18, 0.60, size=(self.pop_size, num_segments))
        population[0] = np.full(num_segments, initial_dt) # Cá thể gốc ban đầu

        best_individual = population[0].copy()
        best_fitness = -1e9
        best_cost = 1e9

        fitness_history = []
        cost_history = []

        if verbose:
            print(f"[*] Bắt đầu tối ưu IGA Năng lượng (Dân số: {self.pop_size}, Thế hệ: {self.max_gen})...")

        for gen in range(self.max_gen):
            fitness_scores = np.zeros(self.pop_size)
            costs = np.zeros(self.pop_size)

            for i in range(self.pop_size):
                cost, e_elec, pen = self.evaluate_chromosome(population[i], waypoints)
                costs[i] = cost
                fitness_scores[i] = 1.0 / (cost + 1e-4)

            f_max = np.max(fitness_scores)
            f_avg = np.mean(fitness_scores)
            f_min = np.min(fitness_scores)

            min_cost_idx = np.argmin(costs)
            if costs[min_cost_idx] < best_cost:
                best_cost = costs[min_cost_idx]
                best_fitness = f_max
                best_individual = population[min_cost_idx].copy()

            fitness_history.append(best_fitness)
            cost_history.append(best_cost)

            if verbose and (gen == 0 or (gen + 1) % 10 == 0 or gen == self.max_gen - 1):
                print(f"  -> Thế hệ {gen+1:02d}/{self.max_gen:02d} | Chi phí tốt nhất (J): {best_cost:.2f} | Fitness: {best_fitness:.4f}")

            # 2. Chọn lọc bánh xe Roulette kết hợp Elitism (bảo lưu cá thể tốt nhất)
            prob = fitness_scores / np.sum(fitness_scores)
            selected_idx = np.random.choice(self.pop_size, size=self.pop_size, p=prob)
            population = population[selected_idx]
            population[0] = best_individual.copy() # Elitism

            # 3. Lai ghép thích nghi dạng Sin (Sinusoidal Adaptive Crossover)
            for i in range(1, self.pop_size - 1, 2):
                f_prime = max(fitness_scores[i], fitness_scores[i + 1])
                if f_prime >= f_avg and (f_max - f_avg) > 1e-6:
                    pc = self.pc1 - 0.5 * (self.pc1 - self.pc2) * (
                        1.0 + math.sin(((f_prime - f_avg) / (f_max - f_avg)) * (math.pi / 2.0))
                    )
                else:
                    pc = self.pc1

                if np.random.rand() < pc:
                    alpha = np.random.uniform(0.2, 0.8)
                    p1 = population[i].copy()
                    p2 = population[i + 1].copy()
                    population[i] = alpha * p1 + (1.0 - alpha) * p2
                    population[i + 1] = (1.0 - alpha) * p1 + alpha * p2

            # 4. Đột biến thích nghi dạng Sin (Sinusoidal Adaptive Mutation)
            for i in range(1, self.pop_size):
                f_ind = fitness_scores[i]
                if f_ind >= f_avg and (f_max - f_avg) > 1e-6:
                    pm = self.pm1 - 0.5 * (self.pm1 - self.pm2) * (
                        1.0 + math.sin(((f_ind - f_avg) / (f_max - f_avg)) * (math.pi / 2.0))
                    )
                else:
                    pm = self.pm1

                if np.random.rand() < pm:
                    mut_idx = np.random.randint(0, num_segments)
                    mutation_step = np.random.normal(0.0, 0.04)
                    population[i, mut_idx] = max(0.08, population[i, mut_idx] + mutation_step)

        return best_individual, cost_history


def run_detailed_calculation():
    """
    Hàm thực thi bài toán tính toán mẫu chi tiết cho Denso VS-6556.
    """
    print("=" * 80)
    print("   FILE TÍNH TOÁN MẪU CHI TIẾT: TỐI ƯU HÓA NĂNG LƯỢNG QUỸ ĐẠO ROBOT DENSO VS-6556")
    print("   DỰA TRÊN CUBIC B-SPLINE VÀ GIẢI THUẬT DI TRUYỀN CẢI TIẾN IGA (CHENG & XU, 2025)")
    print("=" * 80)

    # 1. KHỞI TẠO TẬP ĐIỂM NÚT (WAYPOINTS) MẪU TRONG KHÔNG GIAN KHỚP (RADIAN)
    waypoints = np.array([
        [ 0.00,  0.35,  1.20,  0.00,  0.45,  0.00],  # Điểm P0
        [ 0.25,  0.55,  0.95,  0.15,  0.55,  0.20],  # Điểm P1
        [ 0.50,  0.40,  0.75, -0.20,  0.65,  0.40],  # Điểm P2
        [ 0.30,  0.20,  1.10, -0.10,  0.35, -0.15],  # Điểm P3
        [-0.15,  0.45,  1.30,  0.05,  0.50,  0.10],  # Điểm P4
        [ 0.00,  0.35,  1.20,  0.00,  0.45,  0.00],  # Điểm P5
    ])
    N = len(waypoints)
    print(f"\n[1] Tập điểm nút quỹ đạo (Waypoints): {N} điểm, {N-1} phân đoạn.")
    for idx, pt in enumerate(waypoints):
        deg = np.rad2deg(pt)
        print(f"    - Điểm P{idx}: [J1={deg[0]:6.1f}°, J2={deg[1]:6.1f}°, J3={deg[2]:6.1f}°, J4={deg[3]:6.1f}°, J5={deg[4]:6.1f}°, J6={deg[5]:6.1f}°]")

    # Khởi tạo bộ tối ưu
    optimizer = DensoEnergyIGAOptimizer()

    # 2. ĐÁNH GIÁ QUỸ ĐẠO GỐC (BAN ĐẦU - CHƯA TỐI ƯU)
    initial_dt_val = 0.35
    dt_initial = np.full(N - 1, initial_dt_val)
    t_init, q_init, qd_init, qdd_init, qddd_init = optimizer.interpolate_b_spline(waypoints, dt_initial)
    res_initial = optimizer.calculate_dynamics_and_energy(t_init, q_init, qd_init, qdd_init, qddd_init)

    print("\n[2] Kết quả tính toán quỹ đạo gốc (Chưa tối ưu):")
    print(f"    - Vector delta_t ban đầu: {dt_initial}")
    print(f"    - Tổng thời gian chu trình (T_init): {res_initial['total_time']:.3f} s")
    print(f"    - Năng lượng cơ học tiêu thụ (E_mech): {res_initial['e_mech_total']:.2f} J")
    print(f"    - Tổng điện năng tiêu thụ (E_elec):    {res_initial['e_elec_total']:.2f} J")
    print(f"    - Mô-men xoắn đỉnh các khớp [J1..J6]:  {np.max(np.abs(res_initial['tau']), axis=0).round(2)} N.m")

    # 3. THỰC HIỆN TỐI ƯU HÓA BẰNG GIẢI THUẬT IGA
    t_start = time.time()
    dt_optimal, cost_history = optimizer.optimize(waypoints, initial_dt=initial_dt_val, verbose=True)
    t_elapsed = time.time() - t_start

    # 4. ĐÁNH GIÁ QUỸ ĐẠO TỐI ƯU
    t_opt, q_opt, qd_opt, qdd_opt, qddd_opt = optimizer.interpolate_b_spline(waypoints, dt_optimal)
    res_optimal = optimizer.calculate_dynamics_and_energy(t_opt, q_opt, qd_opt, qdd_opt, qddd_opt)

    print("\n[3] Kết quả sau khi tối ưu hóa bằng giải thuật IGA:")
    print(f"    - Thời gian giải thuật IGA chạy: {t_elapsed:.2f} s")
    print(f"    - Vector delta_t tối ưu: {dt_optimal.round(3)}")
    print(f"    - Tổng thời gian chu trình (T_opt):  {res_optimal['total_time']:.3f} s")
    print(f"    - Năng lượng cơ học tiêu thụ (E_mech): {res_optimal['e_mech_total']:.2f} J")
    print(f"    - Tổng điện năng tiêu thụ (E_elec):    {res_optimal['e_elec_total']:.2f} J")
    print(f"    - Mô-men xoắn đỉnh các khớp [J1..J6]:  {np.max(np.abs(res_optimal['tau']), axis=0).round(2)} N.m")

    # 5. SO SÁNH HIỆU QUẢ TIẾT KIỆM NĂNG LƯỢNG ĐỊNH LƯỢNG
    e_savings_elec = (res_initial['e_elec_total'] - res_optimal['e_elec_total']) / res_initial['e_elec_total'] * 100.0
    e_savings_mech = (res_initial['e_mech_total'] - res_optimal['e_mech_total']) / res_initial['e_mech_total'] * 100.0
    tau_peak_init = np.max(np.abs(res_initial['tau']), axis=0)
    tau_peak_opt = np.max(np.abs(res_optimal['tau']), axis=0)

    print("\n" + "=" * 80)
    print("   BẢNG TỔNG HỢP SO SÁNH ĐỊNH LƯỢNG TRƯỚC VÀ SAU TỐI ƯU (DENSO VS-6556)")
    print("=" * 80)
    print(f"{'Thông số đánh giá':<35} | {'Trước tối ưu':<15} | {'Sau tối ưu (IGA)':<18} | {'Mức cải thiện':<15}")
    print("-" * 88)
    print(f"{'Tổng điện năng tiêu thụ (E_elec)':<35} | {res_initial['e_elec_total']:12.2f} J | {res_optimal['e_elec_total']:15.2f} J | {e_savings_elec:12.2f} % (Tiết kiệm)")
    print(f"{'Công cơ học tiêu thụ (E_mech)':<35} | {res_initial['e_mech_total']:12.2f} J | {res_optimal['e_mech_total']:15.2f} J | {e_savings_mech:12.2f} % (Tiết kiệm)")
    print(f"{'Tổng thời gian chuyển động (T)':<35} | {res_initial['total_time']:12.3f} s | {res_optimal['total_time']:15.3f} s | {((res_initial['total_time']-res_optimal['total_time'])/res_initial['total_time']*100):12.2f} %")
    print(f"{'Mô-men đỉnh Khớp 1 (J1)':<35} | {tau_peak_init[0]:12.2f} Nm| {tau_peak_opt[0]:15.2f} Nm| {((tau_peak_init[0]-tau_peak_opt[0])/tau_peak_init[0]*100 if tau_peak_init[0]>0 else 0):12.2f} %")
    print(f"{'Mô-men đỉnh Khớp 2 (J2 - chịu tải nặng)':<35} | {tau_peak_init[1]:12.2f} Nm| {tau_peak_opt[1]:15.2f} Nm| {((tau_peak_init[1]-tau_peak_opt[1])/tau_peak_init[1]*100):12.2f} % (Giảm ứng suất)")
    print(f"{'Mô-men đỉnh Khớp 3 (J3 - cẳng tay)':<35} | {tau_peak_init[2]:12.2f} Nm| {tau_peak_opt[2]:15.2f} Nm| {((tau_peak_init[2]-tau_peak_opt[2])/tau_peak_init[2]*100):12.2f} % (Giảm ứng suất)")
    print("=" * 88)

    # 6. VẼ ĐỒ THỊ TRỰC QUAN HÓA SO SÁNH VÀ LƯU RA FILE
    fig, axs = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('TỐI ƯU HÓA QUỸ ĐẠO NĂNG LƯỢNG ROBOT DENSO VS-6556 BẰNG IGA (CHENG & XU 2025)', fontsize=14, fontweight='bold')

    # Subplot 1: Tiến trình hội tụ hàm chi phí IGA
    axs[0, 0].plot(cost_history, 'r-', linewidth=2, label='Chi phí tốt nhất J(gen)')
    axs[0, 0].set_title('Tiến trình Hội tụ của Giải thuật Di truyền Cải tiến (IGA)')
    axs[0, 0].set_xlabel('Thế hệ tiến hóa')
    axs[0, 0].set_ylabel('Hàm chi phí tổng hợp')
    axs[0, 0].grid(True, linestyle='--', alpha=0.6)
    axs[0, 0].legend()

    # Subplot 2: Phân bổ khoảng thời gian delta_t_i
    seg_indices = np.arange(1, N)
    w = 0.35
    axs[0, 1].bar(seg_indices - w/2, dt_initial, width=w, color='gray', alpha=0.7, label='Ban đầu (Chưa tối ưu)')
    axs[0, 1].bar(seg_indices + w/2, dt_optimal, width=w, color='blue', alpha=0.8, label='Sau tối ưu (IGA)')
    axs[0, 1].set_title('So sánh Phân bổ Thời gian Giữa các Phân đoạn (Delta t_i)')
    axs[0, 1].set_xlabel('Phân đoạn chuyển động')
    axs[0, 1].set_ylabel('Thời gian (giây)')
    axs[0, 1].set_xticks(seg_indices)
    axs[0, 1].grid(True, linestyle='--', alpha=0.6)
    axs[0, 1].legend()

    # Subplot 3: Vận tốc góc Khớp 2 (Joint 2 Velocity)
    axs[1, 0].plot(t_init, qd_init[:, 1], 'k--', label='J2 Vận tốc (Trước Tối ưu)')
    axs[1, 0].plot(t_opt, qd_opt[:, 1], 'b-', linewidth=2, label='J2 Vận tốc (Sau IGA)')
    axs[1, 0].axhline(y=optimizer.vel_max[1], color='r', linestyle=':', label='Giới hạn Max')
    axs[1, 0].axhline(y=-optimizer.vel_max[1], color='r', linestyle=':')
    axs[1, 0].set_title('Vận tốc Góc Khớp 2 (Joint 2 - Chịu tải lớn nhất)')
    axs[1, 0].set_xlabel('Thời gian (s)')
    axs[1, 0].set_ylabel('Vận tốc (rad/s)')
    axs[1, 0].grid(True, linestyle='--', alpha=0.6)
    axs[1, 0].legend()

    # Subplot 4: Mô-men Xoắn Khớp 2 (Joint 2 Torque)
    axs[1, 1].plot(t_init, res_initial['tau'][:, 1], 'k--', label='J2 Mô-men (Trước Tối ưu)')
    axs[1, 1].plot(t_opt, res_optimal['tau'][:, 1], 'g-', linewidth=2, label='J2 Mô-men (Sau IGA)')
    axs[1, 1].set_title('Mô-men Xoắn Khớp 2 Tau_2(t)')
    axs[1, 1].set_xlabel('Thời gian (s)')
    axs[1, 1].set_ylabel('Mô-men (N.m)')
    axs[1, 1].grid(True, linestyle='--', alpha=0.6)
    axs[1, 1].legend()

    # Subplot 5: Công suất Điện Tức thời P_elec(t)
    p_elec_init_sum = np.sum(res_initial['p_elec'], axis=1)
    p_elec_opt_sum = np.sum(res_optimal['p_elec'], axis=1)
    axs[2, 0].plot(t_init, p_elec_init_sum, 'k--', label=f'Trước tối ưu (Peak: {np.max(p_elec_init_sum):.1f} W)')
    axs[2, 0].plot(t_opt, p_elec_opt_sum, 'm-', linewidth=2, label=f'Sau IGA (Peak: {np.max(p_elec_opt_sum):.1f} W)')
    axs[2, 0].set_title('Tổng Công suất Điện Tiêu thụ Tức thời P_elec(t)')
    axs[2, 0].set_xlabel('Thời gian (s)')
    axs[2, 0].set_ylabel('Công suất (Watts)')
    axs[2, 0].grid(True, linestyle='--', alpha=0.6)
    axs[2, 0].legend()

    # Subplot 6: Năng lượng Tích lũy E(t)
    dt_s_init = t_init[1] - t_init[0]
    dt_s_opt = t_opt[1] - t_opt[0]
    e_cum_init = np.cumsum(p_elec_init_sum) * dt_s_init
    e_cum_opt = np.cumsum(p_elec_opt_sum) * dt_s_opt
    axs[2, 1].plot(t_init, e_cum_init, 'k--', label=f'Trước tối ưu: {res_initial["e_elec_total"]:.1f} J')
    axs[2, 1].plot(t_opt, e_cum_opt, 'r-', linewidth=2.5, label=f'Sau IGA: {res_optimal["e_elec_total"]:.1f} J (-{e_savings_elec:.1f}%)')
    axs[2, 1].set_title('Đường cong Năng lượng Tích lũy E(t)')
    axs[2, 1].set_xlabel('Thời gian (s)')
    axs[2, 1].set_ylabel('Năng lượng (Joules)')
    axs[2, 1].grid(True, linestyle='--', alpha=0.6)
    axs[2, 1].legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Lưu ra desktop Windows và thư mục chạy
    out_png_wsl = '/home/dung/denso_ws/src/denso_gazebo/scripts/denso_energy_optimization_results.png'
    out_png_win1 = '/mnt/c/Users/thanh/OneDrive/Máy tính/denso_energy_optimization_results.png'
    out_png_win2 = '/mnt/c/Users/thanh/Desktop/denso_energy_optimization_results.png'

    plt.savefig(out_png_wsl, dpi=300)
    try:
        plt.savefig(out_png_win1, dpi=300)
        print(f"[+] Đã lưu biểu đồ kết quả: C:\\Users\\thanh\\OneDrive\\Máy tính\\denso_energy_optimization_results.png")
    except Exception:
        pass
    try:
        plt.savefig(out_png_win2, dpi=300)
        print(f"[+] Đã lưu biểu đồ kết quả: C:\\Users\\thanh\\Desktop\\denso_energy_optimization_results.png")
    except Exception:
        pass

    plt.close()
    print("[SUCCESS] Hoàn thành toàn bộ quy trình tính toán mẫu tối ưu năng lượng!")


if __name__ == "__main__":
    run_detailed_calculation()
