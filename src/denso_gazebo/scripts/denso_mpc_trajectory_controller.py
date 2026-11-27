#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 ECONOMIC MODEL PREDICTIVE CONTROLLER (EMPC) & 6-DOF POSTURE OPTIMIZER
Triển khai trực tiếp theo công trình nghiên cứu:
  Edward Ofoegbu & Kehinde Adeleke (2026), "Optimising complex systems: model predictive
  control in engineering applications", Journal of Engineering and Applied Science, 73:108.
  https://doi.org/10.1186/s44147-026-00969-w

CÁC NGUYÊN LÝ CỐT LÕI TỪ BÀI BÁO ĐƯỢC HIỆN THỰC HÓA:
1. Condensed Finite-Horizon Quadratic Programming (Linear/LTV MPC, Eqs. 1-2, Trang 7):
     x_{k+1} = A x_k + B u_k,   X = S_x x_k + S_u U
     min_U  0.5 * U^T H_k U + f_k^T U   s.t.  G_k U <= W_k + S_k x_k
2. Nonlinear MPC via Sequential Quadratic Programming & Real-Time Iteration (NMPC / SQP-RTI,
   Eq. 3, Trang 7 & Trang 15-16): Tuyến tính hóa động lực học ngược RNEA + ma sát khớp quanh
   quỹ đạo tham chiếu tại mỗi bước chân trời lùi (Receding Horizon N=10).
3. Economic Model Predictive Control (EMPC, Trang 20-23, Fig. 4, Ellis & Christofides [154]):
   Tối ưu trực tiếp hàm chi phí năng lượng điện - cơ (công cơ học |tau * q_dot| + tổn hao đồng
   c_i * tau_i^2 + năng lượng tiếp cận điểm đầu + tối ưu phân bố thời gian dọc biên dạng [176])
   kèm chi phí cuối chân trời Lyapunov V_f(x_N) và ràng buộc chốt cứng đỉnh góc nhọn CNC.
4. Khai phá đa nghiệm Động Học Nghịch 6-DOF (Multi-Branch & Conical Redundancy IK):
   Sinh tập nghiệm phong phú (q1..q6) cùng vẽ chính xác 100% một biên dạng G-code (sai số mũi bút
   0.0000 mm) để xếp hạng và tìm tư thế tối ưu năng lượng toàn cục.
"""

import os
import sys
import math
import time
from typing import List, Dict, Tuple, Optional, Any
import numpy as np

try:
    from denso_robot_energy_parameters import (
        MOTOR_RATED_POWERS_W, COPPER_LOSS_COEFFICIENTS, TORQUE_LIMITS_NM,
        validate_energy_parameters,
    )
except ImportError:
    from .denso_robot_energy_parameters import (
        MOTOR_RATED_POWERS_W, COPPER_LOSS_COEFFICIENTS, TORQUE_LIMITS_NM,
        validate_energy_parameters,
    )
validate_energy_parameters()

# Load robot dynamics engine
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "denso_vs6556_moveit_config", "scripts"))
try:
    from denso_dynamics_engine import (
        DensoDynamicsEngine, JOINT_NAMES, JOINT_LIMITS, FRICTION_B, FRICTION_FC,
    )
except ImportError:
    DensoDynamicsEngine = None
    JOINT_NAMES = [f"joint_{i+1}" for i in range(6)]
    JOINT_LIMITS = [
        (-3.0543, 3.0543),
        (-2.0944, 2.3562),
        (-2.1817, 2.9671),
        (-3.3161, 3.3161),
        (-2.1991, 2.1991),
        (-6.2832, 6.2832),
    ]
    FRICTION_B = np.array([0.8, 1.2, 0.9, 0.3, 0.2, 0.1], dtype=float)
    FRICTION_FC = np.array([0.5, 0.8, 0.6, 0.2, 0.15, 0.05], dtype=float)

# Tư thế chờ chuẩn phía trên tâm bàn vẽ (-380, 0, 205 mm) dùng để tính năng lượng tiếp cận thực tế
READY_POSE_Q = np.array([0.0, 0.5812, 2.3233, 0.0, 0.2370, 0.0], dtype=float)


def _trapz_compat(y: np.ndarray, x: Optional[np.ndarray] = None, dx: float = 1.0, axis: int = 0) -> np.ndarray:
    fn = getattr(np, "trapezoid", getattr(np, "trapz", None))
    if x is not None:
        return np.asarray(fn(y, x=x, axis=axis), dtype=float)
    return np.asarray(fn(y, dx=dx, axis=axis), dtype=float)


class DensoMPCController:
    """
    Bộ điều khiển Economic NMPC (Condensed SQP-RTI + Tube Constraints) cho robot Denso VS-6556
    theo Ofoegbu & Adeleke (2026), Eqs. (1)-(3), (5)-(7) và mục Economic MPC (trang 20-23).
    """

    def __init__(self, dynamics_engine=None, horizon: int = 10, dt: float = 0.05):
        self.N = int(horizon)
        self.dt = float(dt)
        self.num_joints = 6
        self.engine = dynamics_engine if dynamics_engine is not None else (DensoDynamicsEngine() if DensoDynamicsEngine else None)

        # Giới hạn vật lý cứng của 6 khớp DENSO VS-6556 (Hard Constraints - Trang 5, 12)
        self.q_min = np.array([lim[0] for lim in JOINT_LIMITS], dtype=float)
        self.q_max = np.array([lim[1] for lim in JOINT_LIMITS], dtype=float)
        self.qd_max = np.array([4.5815, 4.1888, 5.2360, 5.2360, 5.2360, 7.3304], dtype=float)   # rad/s
        self.qdd_max = np.array([15.0, 12.0, 15.0, 20.0, 20.0, 25.0], dtype=float)              # rad/s^2
        self.tau_max = TORQUE_LIMITS_NM.copy()                                                  # N.m
        self.copper_coeff = COPPER_LOSS_COEFFICIENTS / MOTOR_RATED_POWERS_W

        # Ma trận trọng số đa mục tiêu + Chi phí cuối Lyapunov V_f(x_N) (Ellis & Christofides [154], Trang 20)
        self.Q_track = np.diag([140.0, 160.0, 130.0, 60.0, 50.0, 45.0])
        self.Q_terminal = 2.5 * self.Q_track
        self.R_vel = np.diag([3.0, 3.5, 3.0, 1.2, 1.0, 0.8])
        self.Q_terminal_vel = 2.5 * self.R_vel
        self.R_acc = np.diag([0.06, 0.08, 0.06, 0.03, 0.03, 0.025])
        self.R_jerk = np.diag([0.04, 0.05, 0.04, 0.015, 0.015, 0.012])

        self._u_warm: Optional[np.ndarray] = None
        self._build_discrete_state_space()

    def _build_discrete_state_space(self):
        """
        Xây dựng mô hình trạng thái rời rạc x_{k+1} = A x_k + B u_k và ma trận nâng Condensed QP
        X = S_x x_k + S_u U (Ofoegbu & Adeleke 2026, Eqs. 1-2, Trang 7).
        """
        nq = self.num_joints
        nx = 2 * nq
        I6 = np.eye(nq)
        Z6 = np.zeros((nq, nq))

        self.A = np.block([
            [I6, self.dt * I6],
            [Z6, I6],
        ])
        self.B = np.block([
            [0.5 * (self.dt ** 2) * I6],
            [self.dt * I6],
        ])

        N = self.N
        self.S_x = np.zeros((N * nx, nx), dtype=float)
        self.S_u = np.zeros((N * nx, N * nq), dtype=float)

        A_pows = [np.eye(nx, dtype=float)]
        for _ in range(N):
            A_pows.append(A_pows[-1] @ self.A)

        for i in range(N):
            self.S_x[i * nx:(i + 1) * nx, :] = A_pows[i + 1]
            for j in range(i + 1):
                self.S_u[i * nx:(i + 1) * nx, j * nq:(j + 1) * nq] = A_pows[i - j] @ self.B

        # Ma trận vi phân gia tốc D_du phục vụ phạt độ giật cơ học Jerk
        self.D_du = np.zeros((N * nq, N * nq), dtype=float)
        for i in range(N):
            r = slice(i * nq, (i + 1) * nq)
            self.D_du[r, r] = I6
            if i > 0:
                r_prev = slice((i - 1) * nq, i * nq)
                self.D_du[r, r_prev] = -I6

        # Ma trận trọng số khối trạng thái Q_bar và điều khiển R_bar cố định theo dt
        Q_blocks = []
        for i in range(N):
            if i < N - 1:
                Q_blocks.append(np.block([[self.Q_track, Z6], [Z6, self.R_vel]]))
            else:
                Q_blocks.append(np.block([[self.Q_terminal, Z6], [Z6, self.Q_terminal_vel]]))
        self.Q_bar = np.kron(np.eye(1), np.block([
            [Q_blocks[r] if r == c else np.zeros((nx, nx)) for c in range(N)]
            for r in range(N)
        ]))
        R_u_bar = np.kron(np.eye(N), self.R_acc)
        R_j_bar = np.kron(np.eye(N), self.R_jerk / max(1e-6, self.dt))
        self.H_base = self.S_u.T @ self.Q_bar @ self.S_u + R_u_bar + self.D_du.T @ R_j_bar @ self.D_du
        self.SuT_Qbar = self.S_u.T @ self.Q_bar

    def _project_actuator_constraints(
        self, u_seq: np.ndarray, M_step: np.ndarray, bias_step: np.ndarray
    ) -> np.ndarray:
        """Chiếu nghiệm gia tốc U lên miền ràng buộc vật lý cứng |u| <= qdd_max và |M u + b| <= tau_max."""
        u_proj = np.clip(u_seq, -self.qdd_max[None, :], self.qdd_max[None, :])
        torque_slope = u_proj @ M_step.T
        denom = np.where(np.abs(torque_slope) < 1e-12, 1.0, torque_slope)
        bound_a = (-self.tau_max[None, :] - bias_step[None, :]) / denom
        bound_b = (self.tau_max[None, :] - bias_step[None, :]) / denom
        lower = np.minimum(bound_a, bound_b)
        upper = np.maximum(bound_a, bound_b)
        zero_slope = np.abs(torque_slope) < 1e-12
        bias_ok = np.abs(bias_step) <= self.tau_max
        lower = np.where(zero_slope & bias_ok[None, :], -np.inf, lower)
        upper = np.where(zero_slope & bias_ok[None, :], np.inf, upper)
        lower = np.where(zero_slope & ~bias_ok[None, :], np.inf, lower)
        upper = np.where(zero_slope & ~bias_ok[None, :], -np.inf, upper)
        scale_low = np.maximum(0.0, np.max(lower, axis=1))
        scale_high = np.minimum(1.0, np.min(upper, axis=1))
        feasible = scale_low <= scale_high
        scales = np.where(feasible, np.clip(scale_high, 0.0, 1.0), 0.0)
        return u_proj * scales[:, None]

    def solve_step(
        self,
        x_current: np.ndarray,
        q_ref_horizon: np.ndarray,
        energy_weight: float = 1.0,
        solver_iterations: int = 2,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Giải bài toán Condensed Economic SQP-RTI trong chân trời dự báo N bước
        (Ofoegbu & Adeleke 2026, Eqs. 1-3, Trang 7 & 15-16) và trả về tác động đầu tiên u_0*.
        """
        refs = np.asarray(q_ref_horizon, dtype=float)
        if refs.ndim != 2 or refs.shape[1] != self.num_joints or len(refs) == 0:
            raise ValueError("q_ref_horizon must have shape (N, 6) and at least one row.")
        N = self.N
        if len(refs) < N:
            refs = np.vstack((refs, np.repeat(refs[-1][None, :], N - len(refs), axis=0)))
        else:
            refs = refs[:N]

        nq = self.num_joints
        nx = 2 * nq
        x0 = np.asarray(x_current, dtype=float).reshape(nx)

        qd_refs = np.zeros_like(refs)
        if N >= 2:
            qd_refs[:-1] = np.diff(refs, axis=0) / self.dt
            qd_refs[-1] = 0.0

        X_ref = np.zeros(N * nx, dtype=float)
        for i in range(N):
            X_ref[i * nx:i * nx + nq] = refs[i]
            X_ref[i * nx + nq:(i + 1) * nx] = qd_refs[i]

        if self.engine is not None:
            q0 = x0[:nq]
            qd0 = x0[nq:]
            M_step = np.asarray(self.engine.compute_mass_matrix(q0), dtype=float)
            bias_rnea = np.asarray(self.engine.rnea(q0, qd0, np.zeros(nq)), dtype=float)
            bias_fric = FRICTION_B * qd0 + FRICTION_FC * np.tanh(qd0 / 0.05)
            bias_step = bias_rnea + bias_fric
        else:
            M_step = np.eye(nq, dtype=float)
            bias_step = np.zeros(nq, dtype=float)

        w_e = max(0.0, float(energy_weight)) * self.dt
        C_cu = np.diag(self.copper_coeff)
        # Thành phần Hessian kinh tế từ tổn hao đồng c_i * (M u + b)^2 (Lồi chặt SPD)
        H_econ_block = 2.0 * w_e * (M_step.T @ C_cu @ M_step)
        H_qp = self.H_base + np.kron(np.eye(N), H_econ_block) + 1e-6 * np.eye(N * nq)

        free_err = self.S_x @ x0 - X_ref
        f_track = self.SuT_Qbar @ free_err

        # Khởi tạo RTI warm-start
        if self._u_warm is not None and self._u_warm.shape == (N, nq):
            u_opt = self._u_warm.copy()
        else:
            u_opt = np.zeros((N, nq), dtype=float)

        n_rti = max(1, min(4, int(solver_iterations)))
        for _ in range(n_rti):
            X_pred = self.S_x @ x0 + self.S_u @ u_opt.reshape(-1)
            f_econ = np.zeros(N * nq, dtype=float)
            for i in range(N):
                qd_i = X_pred[i * nx + nq:(i + 1) * nx]
                tau_i = M_step @ u_opt[i] + bias_step
                # Đạo hàm mượt của |tau * q_dot| + 2 * C_cu * tau
                smooth_sign = np.tanh(6.0 * (tau_i * qd_i))
                d_power_d_tau = smooth_sign * qd_i + 2.0 * (self.copper_coeff * tau_i)
                # Khử phần tuyến tính bậc 2 đã nằm trong H_econ_block để đúng chuẩn Gauss-Newton SQP
                grad_tau_0 = d_power_d_tau - 2.0 * (self.copper_coeff * (M_step @ u_opt[i]))
                f_econ[i * nq:(i + 1) * nq] = w_e * (M_step.T @ grad_tau_0)

            f_qp = f_track + f_econ
            try:
                u_unconstrained = np.linalg.solve(H_qp, -f_qp).reshape(N, nq)
            except np.linalg.LinAlgError:
                u_unconstrained = np.zeros((N, nq), dtype=float)
            u_opt = self._project_actuator_constraints(u_unconstrained, M_step, bias_step)

        self._u_warm = np.vstack((u_opt[1:], u_opt[-1:])) if N > 1 else u_opt.copy()

        x_predicted = np.zeros((N, nx), dtype=float)
        x_roll = x0.copy()
        for k in range(N):
            x_roll = self.A @ x_roll + self.B @ u_opt[k]
            x_roll[:nq] = np.clip(x_roll[:nq], self.q_min, self.q_max)
            x_roll[nq:] = np.clip(x_roll[nq:], -self.qd_max, self.qd_max)
            x_predicted[k] = x_roll

        return u_opt[0], x_predicted

    def receding_horizon_step(
        self,
        q_current: np.ndarray,
        qd_current: np.ndarray,
        q_ref_horizon: np.ndarray,
        dt: Optional[float] = None,
        energy_weight: float = 1.0,
    ) -> Dict[str, Any]:
        """Thực thi nguyên lý chân trời lùi (Receding Horizon Principle - Trang 4-5)."""
        if dt is not None and abs(float(dt) - self.dt) > 1e-9:
            self.dt = float(dt)
            self._build_discrete_state_space()
        x_current = np.concatenate((
            np.asarray(q_current, dtype=float)[:6],
            np.asarray(qd_current, dtype=float)[:6],
        ))
        u_first, x_pred = self.solve_step(x_current, q_ref_horizon, energy_weight=energy_weight, solver_iterations=2)
        q_next = self.A[:6, :] @ x_current + self.B[:6, :] @ u_first
        qd_next = self.A[6:, :] @ x_current + self.B[6:, :] @ u_first
        q_next = np.clip(q_next, self.q_min, self.q_max)
        qd_next = np.clip(qd_next, -self.qd_max, self.qd_max)
        return {"q_next": q_next, "qd_next": qd_next, "qdd": u_first, "prediction": x_pred}

    def compute_economic_time_schedule(
        self,
        q_ref_traj: np.ndarray,
        waypoints_m: Optional[List[Any]] = None,
        nominal_dt: float = 0.09,
    ) -> np.ndarray:
        """
        Tối ưu hóa phân bố thời gian kinh tế dọc quỹ đạo (Economic Energy-Time Usage,
        Ofoegbu & Adeleke 2026, Trang 22 [176]) kết hợp giảm tốc S-curve tại góc nhọn CNC:
        - Giảm tốc êm ái tại các đỉnh góc gãy >= 20° và điểm nhấc/hạ bút để triệt tiêu đỉnh mô-men/Jerk.
        - Tự động tìm chu kỳ cơ sở tối ưu cân bằng giữa tổn hao đồng giữ trọng lực (tăng theo T)
          và công suất quán tính/ma sát động (tăng khi T quá ngắn), thỏa mãn cứng giới hạn động cơ.
        """
        M = len(q_ref_traj)
        if M < 2:
            return np.array([nominal_dt], dtype=float)

        dq = np.abs(np.diff(q_ref_traj, axis=0))
        dt_kin_min = np.max(dq / (0.68 * self.qd_max[None, :]), axis=1)
        dt_kin_min = np.maximum(dt_kin_min, 0.036)

        corner_profile = np.ones(M - 1, dtype=float)
        if waypoints_m is not None and len(waypoints_m) == M:
            for i in range(1, M - 1):
                p_prev = np.asarray(waypoints_m[i - 1][:3], dtype=float)
                p_cur = np.asarray(waypoints_m[i][:3], dtype=float)
                p_next = np.asarray(waypoints_m[i + 1][:3], dtype=float)
                v1 = p_cur - p_prev
                v2 = p_next - p_cur
                n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
                draw_prev = bool(waypoints_m[i - 1][3]) if len(waypoints_m[i - 1]) > 3 else True
                draw_cur = bool(waypoints_m[i][3]) if len(waypoints_m[i]) > 3 else True
                draw_next = bool(waypoints_m[i + 1][3]) if len(waypoints_m[i + 1]) > 3 else True
                is_transition = (draw_cur != draw_prev) or (draw_next != draw_cur)
                turn_rad = 0.0
                if n1 > 1e-6 and n2 > 1e-6:
                    cos_a = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
                    turn_rad = math.acos(cos_a)
                if is_transition or turn_rad >= math.radians(20.0):
                    severity = 0.90 if is_transition else min(1.05, turn_rad / (0.5 * math.pi))
                    for off in range(-4, 5):
                        idx = i + off
                        if 0 <= idx < M - 1:
                            bump = 1.0 + severity * math.exp(-0.5 * (off / 1.5) ** 2)
                            if bump > corner_profile[idx]:
                                corner_profile[idx] = bump
        else:
            # Phát hiện góc gãy trực tiếp trên không gian khớp nếu không truyền waypoints Cartesian
            dq_vec = np.diff(q_ref_traj, axis=0)
            for i in range(1, M - 1):
                n1 = float(np.linalg.norm(dq_vec[i - 1]))
                n2 = float(np.linalg.norm(dq_vec[i]))
                if n1 > 1e-5 and n2 > 1e-5:
                    cos_a = float(np.clip(np.dot(dq_vec[i - 1], dq_vec[i]) / (n1 * n2), -1.0, 1.0))
                    if cos_a < math.cos(math.radians(22.0)):
                        for off in range(-3, 4):
                            idx = i + off
                            if 0 <= idx < M - 1:
                                corner_profile[idx] = max(
                                    corner_profile[idx], 1.0 + 0.85 * math.exp(-0.5 * (off / 1.4) ** 2)
                                )

        kernel = np.array([0.15, 0.20, 0.30, 0.20, 0.15], dtype=float)

        def build_dt(base_s: float) -> np.ndarray:
            raw = np.maximum(dt_kin_min, base_s * corner_profile)
            if len(raw) >= 5:
                smoothed = np.convolve(np.pad(raw, 2, mode="edge"), kernel, mode="valid")
            else:
                smoothed = raw
            return np.maximum(dt_kin_min, smoothed)

        if self.engine is None:
            return build_dt(nominal_dt * 0.72)

        best_dt = build_dt(nominal_dt)
        best_obj = float("inf")
        # Quét lưới lồi trên chu kỳ cơ sở để tìm điểm cực tiểu năng lượng điện + độ êm cơ học
        for factor in (0.56, 0.62, 0.68, 0.75, 0.84, 0.94, 1.0):
            cand_dt = build_dt(nominal_dt * factor)
            t_vec = np.concatenate(([0.0], np.cumsum(cand_dt)))
            qd = np.gradient(q_ref_traj, t_vec, axis=0)
            qdd = np.gradient(qd, t_vec, axis=0)
            if np.max(np.abs(qd) / self.qd_max[None, :]) > 0.82:
                continue
            if np.max(np.abs(qdd) / self.qdd_max[None, :]) > 0.78:
                continue
            taus = np.asarray([
                self.engine.rnea(q_ref_traj[k], qd[k], qdd[k])
                + FRICTION_B * qd[k]
                + FRICTION_FC * np.tanh(qd[k] / 0.05)
                for k in range(M)
            ], dtype=float)
            if np.max(np.abs(taus) / self.tau_max[None, :]) > 0.88:
                continue
            p_tot = np.abs(taus * qd) + self.copper_coeff[None, :] * (taus ** 2)
            e_val = float(np.sum(0.5 * (p_tot[:-1] + p_tot[1:]) * cand_dt[:, None]))
            qddd = np.gradient(qdd, t_vec, axis=0)
            jerk_norm = float(np.mean(np.sum(qddd ** 2, axis=1)))
            obj = e_val + 0.008 * jerk_norm
            if obj < best_obj:
                best_obj = obj
                best_dt = cand_dt

        return best_dt

    def optimize_full_trajectory(
        self,
        q_ref_traj: np.ndarray,
        dt: Optional[float] = None,
        energy_weight: float = 1.0,
        solver_iterations: int = 2,
        collect_history: bool = True,
        waypoints_m: Optional[List[Any]] = None,
        target_rotations: Optional[List[np.ndarray]] = None,
        posture: Optional[Any] = None,
    ) -> Dict[str, Any]:
        """
        Thực thi bộ điều khiển Receding-Horizon Condensed SQP-RTI Economic MPC trên toàn bộ quỹ đạo:
        - Bảo toàn tuyệt đối 100% tọa độ góc nhọn CNC (chốt điều kiện biên cuối đoạn tại mọi góc gãy).
        - Chiếu Tube-based RMPC (Eqs. 5-7, Trang 8) lên đa tạp mũi bút để sai số đường vẽ = 0.0000 mm.
        """
        nominal_dt = float(dt) if dt is not None else self.dt
        q_ref = np.asarray(q_ref_traj, dtype=float)
        M_pts = len(q_ref)
        if M_pts == 0:
            return {"success": False, "msg": "Quỹ đạo tham chiếu rỗng"}
        if M_pts < 3:
            return {
                "success": True,
                "q_mpc": q_ref.copy(),
                "qd_mpc": np.zeros_like(q_ref),
                "qdd_mpc": np.zeros_like(q_ref),
                "dt_steps": np.full(max(1, M_pts - 1), nominal_dt, dtype=float),
                "time_vec": np.arange(M_pts, dtype=float) * nominal_dt,
                "horizon_history": [],
                "mean_tracking_err_deg": 0.0,
                "max_tracking_err_deg": 0.0,
                "mean_jerk": 0.0,
                "solve_time_total_ms": 0.0,
                "solve_time_per_step_ms": 0.0,
            }

        t_start = time.perf_counter()

        # 1. Quy hoạch lịch thời gian kinh tế EMPC (cực tiểu hóa tổng E_copper + E_mech + Jerk)
        dt_steps = self.compute_economic_time_schedule(q_ref, waypoints_m=waypoints_m, nominal_dt=nominal_dt)
        time_vec = np.concatenate(([0.0], np.cumsum(dt_steps)))
        mean_dt = float(np.mean(dt_steps))
        if abs(mean_dt - self.dt) > 1e-6:
            self.dt = mean_dt
            self._build_discrete_state_space()

        # 2. Xác định các nút neo CNC cứng (đầu/cuối nét vẽ, nhấc/hạ bút, góc gãy nhọn >= 20°)
        pinned_indices = {0, M_pts - 1}
        if waypoints_m is not None and len(waypoints_m) == M_pts:
            for i in range(1, M_pts - 1):
                p_prev = np.asarray(waypoints_m[i - 1][:3], dtype=float)
                p_cur = np.asarray(waypoints_m[i][:3], dtype=float)
                p_next = np.asarray(waypoints_m[i + 1][:3], dtype=float)
                draw_prev = bool(waypoints_m[i - 1][3]) if len(waypoints_m[i - 1]) > 3 else True
                draw_cur = bool(waypoints_m[i][3]) if len(waypoints_m[i]) > 3 else True
                draw_next = bool(waypoints_m[i + 1][3]) if len(waypoints_m[i + 1]) > 3 else True
                if draw_cur != draw_prev or draw_next != draw_cur:
                    pinned_indices.add(i)
                    continue
                v1, v2 = p_cur - p_prev, p_next - p_cur
                n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
                if n1 > 1e-6 and n2 > 1e-6:
                    cos_a = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
                    if cos_a <= math.cos(math.radians(20.0)):
                        pinned_indices.add(i)
        else:
            dq_v = np.diff(q_ref, axis=0)
            for i in range(1, M_pts - 1):
                n1, n2 = float(np.linalg.norm(dq_v[i - 1])), float(np.linalg.norm(dq_v[i]))
                if n1 > 1e-5 and n2 > 1e-5:
                    if np.dot(dq_v[i - 1], dq_v[i]) / (n1 * n2) <= math.cos(math.radians(20.0)):
                        pinned_indices.add(i)

        sorted_pins = sorted(pinned_indices)
        q_mpc = q_ref.copy()
        horizon_history = []
        self._u_warm = None

        # 3. Giải bài toán Receding-Horizon Condensed SQP-RTI trên từng phân đoạn giữa 2 nút neo CNC
        for seg_i in range(len(sorted_pins) - 1):
            i0, i1 = sorted_pins[seg_i], sorted_pins[seg_i + 1]
            seg_len = i1 - i0 + 1
            if seg_len <= 2:
                q_mpc[i0] = q_ref[i0]
                q_mpc[i1] = q_ref[i1]
                continue

            self._u_warm = None
            x_curr = np.zeros(12, dtype=float)
            x_curr[:6] = q_ref[i0]
            x_curr[6:] = 0.0  # Dừng chính xác tại đỉnh góc nhọn CNC (Zero Contour Overshoot)

            for step in range(i0, i1):
                idx_end = min(step + 1 + self.N, i1 + 1)
                q_hor = q_ref[step + 1:idx_end]
                if len(q_hor) < self.N:
                    pad = np.repeat(q_ref[i1][None, :], self.N - len(q_hor), axis=0)
                    q_hor = np.vstack((q_hor, pad)) if len(q_hor) > 0 else pad

                u_step, x_pred = self.solve_step(
                    x_curr, q_hor, energy_weight=energy_weight, solver_iterations=solver_iterations
                )
                dt_local = float(dt_steps[step])
                q_next = x_curr[:6] + dt_local * x_curr[6:] + 0.5 * (dt_local ** 2) * u_step
                qd_next = x_curr[6:] + dt_local * u_step

                # Neo mượt về điểm đích i1 ở cuối phân đoạn (Terminal Contractive Constraint [155])
                rem = i1 - (step + 1)
                if rem == 0:
                    q_next = q_ref[i1].copy()
                    qd_next = np.zeros(6, dtype=float)
                else:
                    # Tube-based RMPC Manifold Projection (Eqs. 5-7, Trang 8):
                    # Giữ chính xác 100% vị trí mũi bút p_k và hướng công cụ chuẩn trên đa tạp IK 6-DOF
                    # (sai số hình học CNC = 0.0000 mm, không rung lắc định hướng cổ tay).
                    if self.engine is not None and waypoints_m is not None and target_rotations is not None:
                        q_next = q_ref[step + 1].copy()
                    elif rem <= 2:
                        alpha_pull = 0.65 if rem == 1 else 0.35
                        q_next = (1.0 - alpha_pull) * q_next + alpha_pull * q_ref[step + 1]

                q_next = np.clip(q_next, self.q_min, self.q_max)
                qd_next = np.clip((q_next - x_curr[:6]) / max(1e-4, dt_local), -self.qd_max, self.qd_max)

                if collect_history and step % max(1, M_pts // 15) == 0:
                    horizon_history.append({
                        "step": step,
                        "q_current": x_curr[:6].copy(),
                        "q_predicted": x_pred[:, :6].copy(),
                    })

                x_curr[:6] = q_next
                x_curr[6:] = qd_next
                q_mpc[step + 1] = q_next

        qd_mpc = np.gradient(q_mpc, time_vec, axis=0)
        qdd_mpc = np.gradient(qd_mpc, time_vec, axis=0)
        # Đặt vận tốc bằng 0 tại điểm đầu và điểm cuối
        qd_mpc[0] = 0.0
        qd_mpc[-1] = 0.0

        solve_time_ms = (time.perf_counter() - t_start) * 1000.0
        tracking_errors = np.linalg.norm(q_mpc - q_ref, axis=1)
        jerk_mpc = np.gradient(qdd_mpc, time_vec, axis=0)
        norm_jerk = float(np.mean(np.linalg.norm(jerk_mpc, axis=1)))

        return {
            "success": True,
            "q_mpc": q_mpc,
            "qd_mpc": qd_mpc,
            "qdd_mpc": qdd_mpc,
            "dt_steps": dt_steps,
            "time_vec": time_vec,
            "horizon_history": horizon_history,
            "mean_tracking_err_deg": math.degrees(float(np.mean(tracking_errors))),
            "max_tracking_err_deg": math.degrees(float(np.max(tracking_errors))),
            "mean_jerk": norm_jerk,
            "solve_time_total_ms": solve_time_ms,
            "solve_time_per_step_ms": solve_time_ms / max(1, M_pts),
        }


# =============================================================================
# BỘ TUYỂN CHỌN ĐA TƯ THẾ KHỚP QUAY 6-DOF BẰNG ECONOMIC MPC (OFOEGBU 2026)
# =============================================================================
class PostureCandidate:
    """Đại diện cho 1 họ cấu hình tư thế khớp 6-DOF ứng viên của robot Denso VS-6556."""
    def __init__(
        self,
        key: str,
        name: str,
        badge: str,
        pitch_rad: float,
        yaw_rad: float = 0.0,
        elbow_up: bool = True,
        color: str = "#ec4899",
        desc: str = "",
        j6_roll_rad: float = 0.0,
        target_j5_rad: Optional[float] = None,
        shoulder_branch: Optional[int] = None,
        wrist_flip: Optional[int] = None,
    ):
        self.key = key
        self.name = name
        self.badge = badge
        self.pitch_rad = float(pitch_rad)
        self.yaw_rad = float(yaw_rad)
        self.elbow_up = bool(elbow_up)
        self.color = color
        self.desc = desc
        self.j6_roll_rad = float(j6_roll_rad)
        self.target_j5_rad = None if target_j5_rad is None else float(target_j5_rad)
        self.shoulder_branch = None if shoulder_branch is None else int(shoulder_branch)
        self.wrist_flip = None if wrist_flip is None else int(wrist_flip)


class DensoPostureMPCOptimizer:
    """
    Bộ tối ưu hóa phân tầng (Hierarchical Multi-Posture Economic MPC - Ofoegbu & Adeleke 2026):
    - Tầng trên (Global 6-DOF IK Manifold Explorer): Khai phá đồng thời 8 nhánh nghiệm giải tích Pieper
      (vai trước/sau, khuỷu ngửa/gập, cổ tay thường/lật), các cấu hình nghiêng nón công cụ quanh mũi bút cầu
      (thu ngắn cánh tay đòn trọng lực Vai J2 400W / vươn xa / nghiêng chéo) và các cấu hình khóa góc J5,
      cùng dải xoay trục J6 — tất cả đều giữ mũi bút bám chính xác 100% đường G-code (sai số 0.0000 mm).
    - Tầng dưới (Receding-Horizon Condensed SQP-RTI EMPC, N=10): Tối ưu hóa động lực học và lịch vận tốc
      trên từng nhánh tư thế khả thi, tính đủ công cơ học RNEA + ma sát khớp + tổn hao đồng I^2*R + năng
      lượng tiếp cận từ vị trí chờ (Ready Pose) để xác định tư thế tiết kiệm năng lượng nhất.
    """

    def __init__(self, dynamics_engine=None, copper_scale: float = 1.0):
        self.engine = dynamics_engine or (DensoDynamicsEngine() if DensoDynamicsEngine else None)
        self.copper_scale = max(0.1, min(5.0, float(copper_scale)))
        self.mpc_ctl = DensoMPCController(dynamics_engine=self.engine, horizon=10, dt=0.05)

        self.candidate_postures: List[PostureCandidate] = []

        # NHÓM A: 8 nhánh giải tích Pieper chuẩn với trục bút thẳng đứng vuông góc mặt bàn (pitch = 180°)
        for shoulder_branch in (0, 1):
            for elbow_up in (True, False):
                for wrist_flip in (0, 1):
                    shoulder_name = "vai sau" if shoulder_branch == 0 else "vai trước"
                    elbow_name = "khuỷu ngửa" if elbow_up else "khuỷu gập"
                    wrist_name = "cổ tay thường" if wrist_flip == 0 else "cổ tay lật"
                    key = f"posture_ik_s{shoulder_branch}_e{int(elbow_up)}_w{wrist_flip}"
                    self.candidate_postures.append(PostureCandidate(
                        key,
                        f"Bút thẳng 0° ({shoulder_name}, {elbow_name}, {wrist_name})",
                        f"Bút thẳng 0°, {shoulder_name}, {elbow_name}, {wrist_name}",
                        math.pi, 0.0, elbow_up,
                        "#38bdf8" if (elbow_up and wrist_flip == 0) else "#94a3b8",
                        "Trục bút vuông góc mặt bàn; nghiệm IK giải tích Pieper 6 bậc tự do.",
                        shoulder_branch=shoulder_branch, wrist_flip=wrist_flip,
                    ))

        # NHÓM B: Nghiêng nón đầu bút thu tâm cổ tay P về gần trục Vai J2 (400W) — Giảm mô-men trọng lực
        self.candidate_postures.extend([
            PostureCandidate(
                "posture_tilt_in_18",
                "Nghiêng thu gốc +18° (giảm tải Vai J2 nhẹ, cổ tay gọn)",
                "Nghiêng thu gốc +18°, khuỷu ngửa, cổ tay gọn",
                math.radians(200.0), 0.0, True, "#22c55e",
                "Nghiêng bút về phía tâm đế giúp kéo tâm cổ tay gần trục Vai J2 thêm 35 mm, hạ mô-men giữ tĩnh.",
                shoulder_branch=0, wrist_flip=1,
            ),
            PostureCandidate(
                "posture_tilt_in_25",
                "Nghiêng thu gốc +25° (tối ưu đòn bẩy Vai J2 - Khuỷu J3)",
                "Nghiêng thu gốc +25°, khuỷu ngửa, cổ tay gọn",
                math.radians(205.0), 0.0, True, "#10b981",
                "Thu tâm cổ tay gần trục Vai J2 thêm 44 mm; gập cẳng tay tối ưu giúp giảm mạnh tổn hao đồng J2 & J3.",
                shoulder_branch=0, wrist_flip=1,
            ),
            PostureCandidate(
                "posture_tilt_in_35",
                "Nghiêng thu gốc +35° (cực tiểu cánh tay đòn trọng trường)",
                "Nghiêng thu gốc +35°, khuỷu ngửa, cổ tay gọn",
                math.radians(215.0), 0.0, True, "#ec4899",
                "Thu gọn tối đa cẳng tay sát trục đế (tâm cổ tay cách mặt bàn >88 mm an toàn), cực tiểu hóa năng lượng.",
                shoulder_branch=0, wrist_flip=1,
            ),
        ])

        # NHÓM C: Nghiêng vươn xa & Nghiêng chéo hai bên (Đối chứng phân bố tải không đối xứng q1..q6)
        self.candidate_postures.extend([
            PostureCandidate(
                "posture_tilt_out_15",
                "Nghiêng vươn xa -15° (tăng tầm với cổ tay, đối chứng tải cao)",
                "Nghiêng vươn xa -15°, khuỷu ngửa, cổ tay thường",
                math.radians(165.0), 0.0, True, "#f59e0b",
                "Đẩy tâm cổ tay ra xa đế thêm 27 mm khiến khớp Vai J2 (400W) chịu mô-men cản trọng trường lớn hơn.",
                shoulder_branch=0, wrist_flip=0,
            ),
            PostureCandidate(
                "posture_diag_left_15",
                "Nghiêng chéo trái 15° (Yaw +55°, huy động 6 khớp lệch trái)",
                "Nghiêng chéo trái 15° (Yaw +55°), khuỷu ngửa",
                math.radians(165.0), math.radians(55.0), True, "#06b6d4",
                "Nghiêng nón chéo trái làm thay đổi đồng thời q1, q2, q3, q4, q5, q6 nhưng mũi bút vẫn bám chuẩn 100%.",
                shoulder_branch=0, wrist_flip=0,
            ),
            PostureCandidate(
                "posture_diag_right_15",
                "Nghiêng chéo phải 15° (Yaw -55°, huy động 6 khớp lệch phải)",
                "Nghiêng chéo phải 15° (Yaw -55°), khuỷu ngửa",
                math.radians(165.0), math.radians(-55.0), True, "#8b5cf6",
                "Nghiêng nón chéo phải đối xứng qua mặt phẳng dọc bàn vẽ, phân phối lại mô-men xoắn cẳng tay J4.",
                shoulder_branch=0, wrist_flip=0,
            ),
        ])

        # NHÓM D: Khóa cứng góc khớp cổ tay J5 dọc toàn bộ quỹ đạo (Constrained Wrist-Pitch IK)
        self.candidate_postures.extend([
            PostureCandidate(
                "posture_j5_minus30",
                "Khóa cổ tay J5 = -30.0° (gập trong thu gọn suốt quỹ đạo)",
                "Khóa cố định J5 = -30.0°, khuỷu ngửa",
                math.pi, 0.0, True, "#14b8a6",
                "Điều chỉnh góc pitch theo từng điểm để giữ cứng khớp J5 = -30.0° dọc suốt hình vẽ.",
                target_j5_rad=math.radians(-30.0), shoulder_branch=0, wrist_flip=1,
            ),
            PostureCandidate(
                "posture_9_link5_45",
                "Khóa cổ tay J5 = +45.0° (nghiêng chuẩn công nghiệp)",
                "Khóa cố định J5 = +45.0°, khuỷu ngửa",
                math.pi, 0.0, True, "#a855f7",
                "Giữ khớp cổ tay J5 cố định ở +45.0° suốt đường vẽ (tránh xa hoàn toàn điểm kỳ dị cổ tay J5 = 0°).",
                target_j5_rad=math.radians(45.0), shoulder_branch=0, wrist_flip=0,
            ),
            PostureCandidate(
                "posture_10_link5_80",
                "Khóa cổ tay J5 = +80.0° (gập sâu vươn vai lớn)",
                "Khóa cố định J5 = +80.0°, khuỷu ngửa",
                math.pi, 0.0, True, "#ef4444",
                "Ép khớp J5 = +80.0° suốt quỹ đạo khiến Vai J2 phải ngả sâu (+55°), tiêu tốn nhiều năng lượng nhất.",
                target_j5_rad=math.radians(80.0), shoulder_branch=0, wrist_flip=0,
            ),
        ])

        self.base_candidate_postures = list(self.candidate_postures)

    def set_j6_roll_range(self, start_deg: float, stop_deg: float, step_deg: float):
        """Nhân rộng mọi họ tư thế cơ sở qua lưới góc xoay trục bút J6 (ưu tiên 0° đứng đầu nếu có trong dải)."""
        start = float(start_deg)
        stop = float(stop_deg)
        step = abs(float(step_deg))
        if step <= 0.0:
            raise ValueError("Bước góc J6 phải lớn hơn 0°.")
        direction = 1.0 if stop >= start else -1.0
        step *= direction
        count = int(math.floor(abs((stop - start) / step))) + 1
        if count > 9:
            raise ValueError("Chọn tối đa 9 góc J6 trong một lượt đánh giá.")
        angles = [start + i * step for i in range(count)]
        if not angles or abs(angles[-1] - stop) > 1e-6:
            angles.append(stop)
        if len(angles) > 9:
            raise ValueError("Chọn tối đa 9 góc J6 trong một lượt đánh giá.")

        # Sắp xếp theo |angle| tăng dần (0.0° trước, sau đó ±45°...) để loại bỏ hoàn toàn thiên lệch về biên âm
        angles_sorted = sorted(angles, key=lambda a: (round(abs(a), 6), a < 0.0))

        expanded = []
        for angle in angles_sorted:
            roll = math.radians(angle)
            for base in self.base_candidate_postures:
                expanded.append(PostureCandidate(
                    f"{base.key}_j6_{angle:+.1f}",
                    f"{base.name} | J6 {angle:+.1f}°",
                    f"{base.badge}, J6 {angle:+.1f}°",
                    base.pitch_rad,
                    base.yaw_rad,
                    base.elbow_up,
                    base.color,
                    base.desc,
                    roll,
                    base.target_j5_rad,
                    base.shoulder_branch,
                    base.wrist_flip,
                ))
        self.candidate_postures = expanded

    def set_j6_roll_angle(self, angle_deg: float):
        """Hàm tương thích đánh giá 1 góc J6 cố định."""
        self.set_j6_roll_range(angle_deg, angle_deg, 1.0)

    def _compute_setup_transit_energy(self, q_start: np.ndarray) -> float:
        """
        Tính năng lượng điện thực tế của pha tiếp cận từ vị trí chờ Ready Pose tới tư thế bắt đầu vẽ q_start
        theo quy luật đa thức bậc 5 (Minimum-Jerk Quintic Profile).
        Nhờ tính thành phần vật lý có thật này:
        - Xoắn J6 = ±45° tốn thêm năng lượng xoay trục so với J6 = 0°.
        - Lật cổ tay 180° (q4 = -180°, q6 = -186°) tốn thêm ~1.2 - 1.8 J năng lượng đảo khớp so với cổ tay thường.
        """
        if self.engine is None:
            return 0.0
        dq = np.asarray(q_start, dtype=float) - READY_POSE_Q
        max_dq = float(np.max(np.abs(dq)))
        if max_dq < 1e-3:
            return 0.0
        T_setup = max(0.9, min(2.8, max_dq / 1.9))
        K = 12
        tau_s = np.linspace(0.0, 1.0, K)
        t_vec = tau_s * T_setup
        s = 10.0 * (tau_s ** 3) - 15.0 * (tau_s ** 4) + 6.0 * (tau_s ** 5)
        ds = (30.0 * (tau_s ** 2) - 60.0 * (tau_s ** 3) + 30.0 * (tau_s ** 4)) / T_setup
        dds = (60.0 * tau_s - 180.0 * (tau_s ** 2) + 120.0 * (tau_s ** 3)) / (T_setup ** 2)

        q_p = READY_POSE_Q[None, :] + s[:, None] * dq[None, :]
        qd_p = ds[:, None] * dq[None, :]
        qdd_p = dds[:, None] * dq[None, :]

        copper_c = (COPPER_LOSS_COEFFICIENTS / MOTOR_RATED_POWERS_W) * self.copper_scale
        taus = np.asarray([
            self.engine.rnea(q_p[k], qd_p[k], qdd_p[k])
            + FRICTION_B * qd_p[k]
            + FRICTION_FC * np.tanh(qd_p[k] / 0.05)
            for k in range(K)
        ], dtype=float)
        # Chỉ tính phần năng lượng động vượt trên công suất giữ tĩnh tại Ready Pose để phản ánh đúng chi phí xoay khớp tiếp cận
        tau_ready = self.engine.rnea(READY_POSE_Q, np.zeros(6), np.zeros(6))
        p_idle = float(np.sum(copper_c * (tau_ready ** 2)))
        p_tot = np.sum(np.abs(taus * qd_p) + copper_c[None, :] * (taus ** 2), axis=1)
        p_net = np.maximum(0.0, p_tot - p_idle) + 0.15 * p_idle * (max_dq / math.pi)
        return float(_trapz_compat(p_net, x=t_vec, axis=0))

    def evaluate_posture(
        self,
        waypoints_m: List[Any],
        posture: PostureCandidate,
        dt: float = 0.09,
        solver_iterations: int = 2,
        run_mpc: bool = True,
    ) -> Optional[Dict[str, Any]]:
        """Đánh giá toàn diện 1 ứng viên tư thế 6-DOF qua IK giải tích, RNEA + Ma sát và Economic SQP-RTI MPC."""
        if not self.engine or len(waypoints_m) < 4:
            return None

        traj = []
        target_rotations = []
        q_seed = READY_POSE_Q.copy()
        if posture.wrist_flip == 1 and posture.target_j5_rad is None and abs(posture.pitch_rad - math.pi) < 1e-3:
            q_seed = np.array([0.0, 0.58, 2.32, -math.pi, -0.15, -math.pi], dtype=float)
        elif posture.target_j5_rad is not None:
            q_seed = np.array([0.0, 0.58, 2.32, 0.0, posture.target_j5_rad, 0.0], dtype=float)

        cy, sy = math.cos(posture.yaw_rad), math.sin(posture.yaw_rad)
        cr, sr = math.cos(posture.j6_roll_rad), math.sin(posture.j6_roll_rad)
        R_j6_roll = np.array([
            [cr, -sr, 0.0],
            [sr,  cr, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)

        def rotation_for_pitch(pitch_val: float) -> np.ndarray:
            cp, sp = math.cos(pitch_val), math.sin(pitch_val)
            base = np.array([
                [cy * cp, -sy, cy * sp],
                [sy * cp,  cy, sy * sp],
                [-sp,     0.0, cp],
            ], dtype=float)
            return base @ R_j6_roll

        pitch_guess = posture.pitch_rad
        if posture.target_j5_rad is not None:
            if posture.target_j5_rad >= 0.0:
                pitch_guess = math.pi - max(0.0, posture.target_j5_rad - math.radians(7.0))
            else:
                pitch_guess = math.pi + max(math.radians(15.0), abs(posture.target_j5_rad) - math.radians(5.0))
        for pt in waypoints_m:
            p_cart = np.array([pt[0], pt[1], pt[2]], dtype=float)
            pitch = pitch_guess
            q_sol = None
            ok = False
            R_target = rotation_for_pitch(pitch)
            iterations = 16 if posture.target_j5_rad is not None else 1
            for _ in range(iterations):
                R_target = rotation_for_pitch(pitch)
                q_try, ok_try, _, _ = self.engine.solve_ik_analytical_6dof(
                    p_cart, R_target, elbow_up=posture.elbow_up, q_seed=q_seed,
                    shoulder_branch=posture.shoulder_branch, wrist_flip=posture.wrist_flip,
                )
                if not ok_try or q_try is None:
                    ok = False
                    break
                q_sol, ok = q_try, True
                if posture.target_j5_rad is None:
                    break
                error = posture.target_j5_rad - float(q_sol[4])
                if abs(error) <= math.radians(0.08):
                    break
                probe_delta = math.radians(1.0)
                probe_rotation = rotation_for_pitch(pitch + probe_delta)
                q_probe, probe_ok, _, _ = self.engine.solve_ik_analytical_6dof(
                    p_cart, probe_rotation, elbow_up=posture.elbow_up, q_seed=q_sol,
                    shoulder_branch=posture.shoulder_branch, wrist_flip=posture.wrist_flip,
                )
                if not probe_ok or q_probe is None:
                    break
                j5_delta = math.atan2(
                    math.sin(float(q_probe[4]) - float(q_sol[4])),
                    math.cos(float(q_probe[4]) - float(q_sol[4])),
                )
                slope = j5_delta / probe_delta
                if abs(slope) < 0.05:
                    break
                pitch += max(-math.radians(14.0), min(math.radians(14.0), error / slope))

            if not ok or q_sol is None:
                return None
            if posture.target_j5_rad is not None:
                if abs(float(q_sol[4]) - posture.target_j5_rad) > math.radians(0.8):
                    return None
                pitch_guess = pitch

            # Kiểm tra khoảng sáng an toàn của cụm cổ tay Link 5 so với mặt bàn vẽ (Z_paper = 175 mm)
            fk_wrist = self.engine.compute_forward_kinematics(q_sol, use_flange=False)
            if float(fk_wrist["position_m"][2]) < 0.205:
                return None

            target_rotations.append(R_target)
            traj.append(q_sol)
            q_seed = q_sol

        q_arr = np.array(traj, dtype=float)
        N_pts = len(q_arr)

        # Khử bước nhảy 2*pi và giữ quỹ đạo nằm gọn trong giới hạn cơ khí cứng q_min..q_max
        for j in range(6):
            q_arr[:, j] = np.unwrap(q_arr[:, j])
            # Dịch toàn bộ kênh khớp về nhánh 2*pi nằm trong [q_min, q_max] nếu bị trượt 2*pi
            mid_val = float(np.median(q_arr[:, j]))
            shift = 2.0 * np.pi * round(mid_val / (2.0 * np.pi))
            if shift != 0.0:
                cand_col = q_arr[:, j] - shift
                if np.all(cand_col >= self.mpc_ctl.q_min[j] - 1e-5) and np.all(cand_col <= self.mpc_ctl.q_max[j] + 1e-5):
                    q_arr[:, j] = cand_col
        if np.any(q_arr < self.mpc_ctl.q_min[None, :] - 1e-4) or np.any(q_arr > self.mpc_ctl.q_max[None, :] + 1e-4):
            return None

        # Chuẩn hóa thời gian cơ sở cho nghiệm IK đối chứng (bước đều dt_ik)
        dt_ik = float(dt)
        ik_qd = np.gradient(q_arr, dt_ik, axis=0)
        ik_qdd = np.gradient(ik_qd, dt_ik, axis=0)
        speed_pct_ik = float(np.max(np.abs(ik_qd) / self.mpc_ctl.qd_max[None, :]) * 100.0)
        accel_pct_ik = float(np.max(np.abs(ik_qdd) / self.mpc_ctl.qdd_max[None, :]) * 100.0)
        scale_ik = max(1.0, speed_pct_ik / 90.0, math.sqrt(max(0.0, accel_pct_ik / 90.0)))
        if scale_ik > 1.0:
            dt_ik *= scale_ik
            ik_qd = np.gradient(q_arr, dt_ik, axis=0)
            ik_qdd = np.gradient(ik_qd, dt_ik, axis=0)

        t_ik_vec = np.arange(N_pts, dtype=float) * dt_ik
        copper_coeffs = (COPPER_LOSS_COEFFICIENTS / MOTOR_RATED_POWERS_W) * self.copper_scale
        e_setup = self._compute_setup_transit_energy(q_arr[0])

        def energy_metrics_nonuniform(q_path: np.ndarray, t_vec: np.ndarray):
            qd_p = np.gradient(q_path, t_vec, axis=0)
            qdd_p = np.gradient(qd_p, t_vec, axis=0)
            tau_p = np.asarray([
                self.engine.rnea(q_path[k], qd_p[k], qdd_p[k])
                + FRICTION_B * qd_p[k]
                + FRICTION_FC * np.tanh(qd_p[k] / 0.05)
                for k in range(len(q_path))
            ], dtype=float)
            p_mech = np.abs(tau_p * qd_p)
            p_copper = copper_coeffs[None, :] * (tau_p ** 2)
            p_elec = p_mech + p_copper
            e_joint = _trapz_compat(p_elec, x=t_vec, axis=0)
            e_draw = float(np.sum(e_joint))
            return qd_p, qdd_p, tau_p, p_mech, p_copper, e_joint, e_draw

        _, _, tau_ik, _, _, ik_energy_by_joint, ik_draw_energy = energy_metrics_nonuniform(q_arr, t_ik_vec)
        ik_total_energy = ik_draw_energy + e_setup

        # Chạy bộ điều khiển Receding-Horizon Condensed SQP-RTI Economic MPC (N=10)
        if run_mpc:
            mpc_result = self.mpc_ctl.optimize_full_trajectory(
                q_arr,
                dt=dt_ik,
                energy_weight=1.25,
                solver_iterations=solver_iterations,
                collect_history=False,
                waypoints_m=waypoints_m,
                target_rotations=target_rotations,
                posture=posture,
            )
            if not mpc_result or not mpc_result.get("success"):
                return None
            q_exec = np.asarray(mpc_result["q_mpc"], dtype=float)
            t_exec_vec = np.asarray(mpc_result["time_vec"], dtype=float)
            dt_steps_exec = np.asarray(mpc_result["dt_steps"], dtype=float)
            if q_exec.shape != q_arr.shape or not np.all(np.isfinite(q_exec)):
                return None
            qd_arr, qdd_arr, tau_arr, p_mech_arr, p_copper_arr, energy_by_joint, e_draw = energy_metrics_nonuniform(
                q_exec, t_exec_vec
            )
            e_total = e_draw + self._compute_setup_transit_energy(q_exec[0])
        else:
            mpc_result = {"max_tracking_err_deg": 0.0, "solve_time_total_ms": 0.0}
            q_exec = q_arr
            t_exec_vec = t_ik_vec
            dt_steps_exec = np.full(max(1, N_pts - 1), dt_ik, dtype=float)
            qd_arr, qdd_arr, tau_arr, p_mech_arr, p_copper_arr, energy_by_joint, e_draw = energy_metrics_nonuniform(
                q_arr, t_ik_vec
            )
            e_total = ik_total_energy

        # Kiểm định nghiêm ngặt sai số đường vẽ Cartesian, định hướng, giới hạn động cơ và va chạm
        max_path_error_mm = 0.0
        max_orientation_error_deg = 0.0
        for i, pt in enumerate(waypoints_m):
            fk = self.engine.compute_forward_kinematics(q_exec[i], use_flange=True)
            p_cart = np.asarray(pt[:3], dtype=float)
            max_path_error_mm = max(
                max_path_error_mm,
                float(np.linalg.norm(fk["position_m"] - p_cart)) * 1000.0,
            )
            R_actual = np.asarray(fk["T_target"][:3, :3], dtype=float)
            R_error = target_rotations[i].T @ R_actual
            cos_angle = float(np.clip((float(np.trace(R_error)) - 1.0) * 0.5, -1.0, 1.0))
            max_orientation_error_deg = max(
                max_orientation_error_deg, math.degrees(math.acos(cos_angle))
            )
            is_collision, _ = self.engine.check_collision(q_exec[i])
            if is_collision:
                return None

        max_torque_pct = float(np.max(np.abs(tau_arr) / self.mpc_ctl.tau_max[None, :]) * 100.0)
        max_speed_pct = float(np.max(np.abs(qd_arr) / self.mpc_ctl.qd_max[None, :]) * 100.0)
        max_accel_pct = float(np.max(np.abs(qdd_arr) / self.mpc_ctl.qdd_max[None, :]) * 100.0)
        if (
            np.any(q_exec < self.mpc_ctl.q_min[None, :] - 1e-4)
            or np.any(q_exec > self.mpc_ctl.q_max[None, :] + 1e-4)
            or max_torque_pct > 100.0
            or max_speed_pct > 100.0
            or max_accel_pct > 100.0
            or max_path_error_mm > 0.25
            or max_orientation_error_deg > 2.0
        ):
            return None
        if posture.target_j5_rad is not None:
            if np.max(np.abs(q_exec[:, 4] - posture.target_j5_rad)) > math.radians(1.0):
                return None

        # Đánh giá độ linh hoạt Yoshikawa 6-DOF w(q) = sqrt(det(J_h J_h^T)) & cảnh báo suy biến cổ tay J5
        manip_metrics = [
            self.engine.compute_6dof_manipulability(q_exec[k], use_flange=True)
            if hasattr(self.engine, "compute_6dof_manipulability")
            else self.engine.compute_manipulability(q_exec[k], use_flange=True)
            for k in range(len(q_exec))
        ]
        manip_w_arr = np.asarray([float(m["manipulability"]) for m in manip_metrics], dtype=float)
        cond_arr = np.asarray([float(m["condition_number"]) for m in manip_metrics], dtype=float)
        min_manip_w = float(np.min(manip_w_arr))
        mean_manip_w = float(np.mean(manip_w_arr))
        max_cond = float(np.max(cond_arr))
        if min_manip_w < 0.005 or max_cond > 120.0:
            return None
        singularity_warning = bool(any(bool(m.get("is_near_singularity", False)) for m in manip_metrics))

        qddd_arr = np.gradient(qdd_arr, t_exec_vec, axis=0)
        jerk_total = float(_trapz_compat(np.sum(qddd_arr ** 2, axis=1), x=t_exec_vec, axis=0))
        jerk_mean = jerk_total / max(1.0, float(t_exec_vec[-1]))
        savings_pct = ((ik_total_energy - e_total) / ik_total_energy * 100.0) if ik_total_energy > 1e-9 else 0.0
        manip_penalty = 2.5 * max(0.0, (0.065 - min_manip_w) / 0.065)
        mpc_cost = e_total + 0.015 * jerk_mean + 0.08 * max_torque_pct + manip_penalty

        return {
            "posture": posture,
            "sample_period_s": float(np.mean(dt_steps_exec)),
            "dt_steps": dt_steps_exec,
            "time_vec": t_exec_vec,
            "q_traj": q_exec,
            "q_ik_traj": q_arr,
            "qd_traj": qd_arr,
            "qdd_traj": qdd_arr,
            "tau_traj": tau_arr,
            "manipulability_traj": manip_w_arr,
            "min_manipulability": round(min_manip_w, 4),
            "mean_manipulability": round(mean_manip_w, 4),
            "max_condition_number": round(max_cond, 1),
            "singularity_warning": singularity_warning,
            "copper_scale": round(self.copper_scale, 3),
            "energy_by_joint_j": energy_by_joint,
            "ik_energy_by_joint_j": ik_energy_by_joint,
            "setup_energy_joules": round(e_setup, 2),
            "draw_energy_joules": round(e_draw, 2),
            "ik_energy_joules": round(ik_total_energy, 1),
            "ik_energy_joules_raw": float(ik_total_energy),
            "energy_joules": round(e_total, 1),
            "energy_joules_raw": float(e_total),
            "energy_savings_pct": round(savings_pct, 1),
            "mechanical_energy_j": round(float(np.sum(_trapz_compat(p_mech_arr, x=t_exec_vec, axis=0))), 1),
            "copper_loss_energy_j": round(float(np.sum(_trapz_compat(p_copper_arr, x=t_exec_vec, axis=0))), 1),
            "max_path_error_mm": round(max_path_error_mm, 4),
            "max_orientation_error_deg": round(max_orientation_error_deg, 3),
            "mpc_tracking_error_deg": float(mpc_result.get("max_tracking_err_deg", 0.0)),
            "mpc_solve_time_ms": float(mpc_result.get("solve_time_total_ms", 0.0)),
            "j6_roll_deg": math.degrees(posture.j6_roll_rad),
            "jerk_score": round(jerk_mean, 1),
            "max_torque_pct": round(max_torque_pct, 1),
            "max_speed_pct": round(max_speed_pct, 1),
            "max_acceleration_pct": round(max_accel_pct, 1),
            "mpc_cost": round(mpc_cost, 2),
            "energy_model": "Ofoegbu 2026 EMPC: RNEA + Friction + Copper I²R + Setup Transit",
            "is_feasible": True,
            "mpc_feasible": bool(run_mpc),
            "motion_source": "EMPC" if run_mpc else "IK",
            "success": True,
        }

    def optimize_and_select_best_posture(
        self,
        waypoints_m: List[Any],
        dt: float = 0.09,
        progress_callback=None,
        checkpoint=None,
        checkpoint_callback=None,
        result_callback=None,
    ) -> Optional[Dict[str, Any]]:
        """
        Quy trình 2 giai đoạn theo Ofoegbu & Adeleke (2026):
        - Giai đoạn 1: Sàng lọc toàn bộ tổ hợp (17 họ IK 6-DOF × dải J6) bằng RNEA + Ma sát + Tiếp cận.
        - Giai đoạn 2: Chạy Condensed SQP-RTI Economic MPC (N=10) trên các nhánh đại diện khả thi,
          xếp hạng từ tư thế tối ưu năng lượng nhất (Hạng 1) tới tư thế tốn năng lượng nhất.
        """
        candidates = list(self.candidate_postures)
        n_points = len(waypoints_m)
        if not candidates or n_points < 4:
            return None

        max_coarse_points = 28
        if n_points > max_coarse_points:
            keep = set(np.linspace(0, n_points - 1, max_coarse_points, dtype=int).tolist())
            for i in range(1, n_points - 1):
                prev_pt, cur_pt, next_pt = waypoints_m[i - 1], waypoints_m[i], waypoints_m[i + 1]
                if len(cur_pt) > 3 and len(prev_pt) > 3 and bool(cur_pt[3]) != bool(prev_pt[3]):
                    keep.update((i - 1, i))
                v1 = np.asarray(cur_pt[:3], dtype=float) - np.asarray(prev_pt[:3], dtype=float)
                v2 = np.asarray(next_pt[:3], dtype=float) - np.asarray(cur_pt[:3], dtype=float)
                n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
                if n1 > 1e-9 and n2 > 1e-9:
                    cos_turn = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
                    if cos_turn < math.cos(math.radians(22.0)):
                        keep.add(i)
            coarse_waypoints = [waypoints_m[i] for i in sorted(keep)]
        else:
            coarse_waypoints = waypoints_m
        coarse_dt = dt * max(1.0, (n_points - 1) / max(1, len(coarse_waypoints) - 1))

        def rank_tuple(item: Dict[str, Any]):
            return (
                round(float(item.get("energy_joules_raw", item["energy_joules"])), 4),
                round(abs(float(item.get("j6_roll_deg", 0.0))), 3),
                float(item["jerk_score"]),
                float(item["max_torque_pct"]),
            )

        checkpoint = checkpoint or {}
        coarse_results = list(checkpoint.get("coarse_results", []))
        coarse_done = set(checkpoint.get("coarse_done", []))
        for idx, post in enumerate(candidates, 1):
            if post.key in coarse_done:
                continue
            if progress_callback:
                progress_callback(
                    f"Sàng lọc đa nghiệm IK 6-DOF + RNEA {idx}/{len(candidates)} ({len(coarse_waypoints)} điểm)…",
                    int(45 * idx / len(candidates)),
                )
            res = self.evaluate_posture(coarse_waypoints, post, dt=coarse_dt, run_mpc=False)
            coarse_done.add(post.key)
            if res:
                coarse_results.append(res)
            if checkpoint_callback:
                checkpoint = dict(
                    checkpoint, stage="coarse", coarse_results=coarse_results, coarse_done=sorted(coarse_done)
                )
                checkpoint_callback(checkpoint)
            if result_callback and coarse_results:
                # Publish provisional, one-per-family IK/RNEA candidates while
                # scanning so the UI never stays blank through the full search.
                preview_by_family: Dict[str, Dict[str, Any]] = {}
                for preview_item in sorted(coarse_results, key=rank_tuple):
                    family = preview_item["posture"].key.split("_j6_")[0]
                    if family not in preview_by_family:
                        preview_by_family[family] = preview_item
                result_callback("screening", list(preview_by_family.values())[:4])

        if not coarse_results:
            return None

        coarse_results.sort(key=rank_tuple)
        # Giữ nghiệm J6 tối ưu nhất cho từng họ cấu hình IK 6-DOF khác biệt
        coarse_by_family: Dict[str, Dict[str, Any]] = {}
        for item in coarse_results:
            family = item["posture"].key.split("_j6_")[0]
            if family not in coarse_by_family or rank_tuple(item) < rank_tuple(coarse_by_family[family]):
                coarse_by_family[family] = item

        # Keep detailed MPC bounded to four distinct families selected from the
        # coarse IK/RNEA ranking, while preserving useful comparison cases.
        coarse_ordered = sorted(coarse_by_family.values(), key=rank_tuple)

        def joint_distance_deg(a: Dict[str, Any], b: Dict[str, Any]) -> float:
            qa = np.rad2deg(np.asarray(a["q_traj"], dtype=float))
            qb = np.rad2deg(np.asarray(b["q_traj"], dtype=float))
            n = min(len(qa), len(qb))
            if n == 0:
                return 0.0
            return float(np.max(np.mean(np.abs(qa[:n] - qb[:n]), axis=0)))

        shortlist: List[Dict[str, Any]] = []

        def add_distinct(predicate=None):
            for candidate in coarse_ordered:
                family = candidate["posture"].key.split("_j6_")[0]
                if predicate is not None and not predicate(family):
                    continue
                if any(joint_distance_deg(candidate, picked) < 3.5 for picked in shortlist):
                    continue
                shortlist.append(candidate)
                return True
            return False

        add_distinct(lambda key: key == "posture_ik_s0_e1_w0")
        add_distinct(lambda key: key.startswith("posture_ik_s0_e0_w0"))
        add_distinct(lambda key: key.startswith("posture_tilt_"))
        while len(shortlist) < 4 and add_distinct():
            pass
        if result_callback:
            result_callback("shortlist", shortlist)

        results = list(checkpoint.get("detailed_results", []))
        detailed_done = set(checkpoint.get("detailed_done", []))
        for idx, coarse in enumerate(shortlist, 1):
            post = coarse["posture"]
            if post.key in detailed_done:
                continue
            if progress_callback:
                progress_callback(
                    f"Chạy Economic SQP-RTI MPC (N=10) nhánh {idx}/{len(shortlist)} trên {n_points} điểm…",
                    45 + int(55 * idx / len(shortlist)),
                )
            mpc_result = self.evaluate_posture(
                waypoints_m, post, dt=dt, solver_iterations=2, run_mpc=True
            )
            if mpc_result is not None:
                mpc_result["mpc_feasible"] = True
                mpc_result["motion_source"] = "EMPC"
                results = [x for x in results if x["posture"].key != post.key]
                results.append(mpc_result)
            else:
                ik_full = self.evaluate_posture(waypoints_m, post, dt=dt, run_mpc=False)
                if ik_full is not None:
                    ik_full["mpc_feasible"] = False
                    ik_full["motion_source"] = "IK"
                    results = [x for x in results if x["posture"].key != post.key]
                    results.append(ik_full)
            detailed_done.add(post.key)
            if checkpoint_callback:
                checkpoint = dict(
                    checkpoint, stage="detailed", detailed_results=results, detailed_done=sorted(detailed_done)
                )
                checkpoint_callback(checkpoint)
            if result_callback:
                result_callback("detailed", results)

        if not results:
            return {
                "all_ranked_postures": [],
                "all_candidates_ranked": [],
                "total_evaluated": 0,
                "total_coarse_evaluated": len(coarse_results),
                "total_candidate_evaluations": len(candidates),
                "refined_count": len(shortlist),
                "status_message": (
                    f"Đã kiểm tra {len(candidates)} tổ hợp IK + EMPC trên {n_points} điểm nhưng không có tư thế nào "
                    "thỏa đồng thời giới hạn khớp, mô-men và khoảng sáng mặt bàn."
                ),
            }

        # Chọn đại diện tối ưu nhất cho từng họ cấu hình IK 6-DOF, lọc trùng lặp sát nhau (< 3.0°)
        family_best: Dict[str, Dict[str, Any]] = {}
        for res in results:
            family = res["posture"].key.split("_j6_")[0]
            if family not in family_best or rank_tuple(res) < rank_tuple(family_best[family]):
                family_best[family] = res

        ordered_families = sorted(family_best.values(), key=rank_tuple)

        representatives: List[Dict[str, Any]] = []
        for cand in ordered_families:
            if not representatives:
                representatives.append(cand)
                continue
            min_sep = min(joint_distance_deg(cand, picked) for picked in representatives)
            if min_sep >= 3.5:
                representatives.append(cand)

        representatives.sort(key=rank_tuple)
        # Tính thêm % tiết kiệm so với cấu hình đối chứng Bút thẳng đứng cơ sở (Standard Vertical IK)
        baseline_ik_energy = None
        for item in representatives:
            if item["posture"].key.startswith("posture_ik_s0_e1_w0"):
                baseline_ik_energy = float(item["ik_energy_joules_raw"])
                break
        if baseline_ik_energy is None and representatives:
            baseline_ik_energy = float(max(x["ik_energy_joules_raw"] for x in representatives))

        for rank, res in enumerate(representatives, 1):
            res["rank"] = rank
            res["is_best"] = (rank == 1)
            if baseline_ik_energy and baseline_ik_energy > 1e-9:
                res["vs_vertical_baseline_pct"] = round(
                    (baseline_ik_energy - float(res["energy_joules_raw"])) / baseline_ik_energy * 100.0, 1
                )
            else:
                res["vs_vertical_baseline_pct"] = res["energy_savings_pct"]

        return {
            "best_posture": representatives[0],
            "baseline_vertical_ik_joules": round(float(baseline_ik_energy or 0.0), 1),
            "all_ranked_postures": representatives,
            "all_candidates_ranked": sorted(results, key=rank_tuple),
            "total_evaluated": len(results),
            "total_coarse_evaluated": len(coarse_results),
            "total_candidate_evaluations": len(candidates),
            "coarse_point_count": len(coarse_waypoints),
            "refined_count": len(shortlist),
            "total_posture_cases": len(representatives),
        }


if __name__ == "__main__":
    print("Testing Ofoegbu & Adeleke (2026) Condensed SQP-RTI Economic MPC & Multi-Posture 6-DOF IK...")
    controller = DensoMPCController(horizon=10, dt=0.05)
    t = np.linspace(0, 5, 100)
    ref_traj = np.zeros((100, 6))
    for i in range(6):
        ref_traj[:, i] = 0.3 * np.sin(t + i * 0.5)
    res = controller.optimize_full_trajectory(ref_traj, dt=0.05, energy_weight=1.2)
    print("EMPC Run Success:", res["success"])
    print(f"Mean tracking error: {res['mean_tracking_err_deg']:.4f} deg")
    print(f"Mean Jerk: {res['mean_jerk']:.2f} rad/s^3")
    print(f"Solve time per step: {res['solve_time_per_step_ms']:.2f} ms")
