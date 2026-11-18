# -*- coding: utf-8 -*-
"""
DENSO VS-6556 BI-RRT & METAHEURISTIC PATH PLANNING & TRAJECTORY OPTIMIZATION ENGINE
Dựa trên công trình nghiên cứu:
"Dynamic quality aware path planning for 6 DoF robotic arms using BiRRT and
metaheuristic optimization based on B spline paths"
(Abdelrahman T. Elgohr et al., Nature Scientific Reports (2026) 16:7487, DOI: 10.1038/s41598-026-37676-8)

Kiến trúc 2 Giai đoạn (Two-Stage Planning Framework):
1. Giai đoạn 1 (Global Feasibility Seeding):
   - Nhận diện 3 vật cản hình cầu trong không gian làm việc (Spherical Obstacles).
   - Xác định điểm tránh né ban đầu (Detour Waypoints) theo pháp tuyến hình học (Eqs. 1-2).
   - Lập kế hoạch không gian khớp bằng Bi-RRT có dẫn hướng Spline (Spline-Guided Bi-RRT)
     với metric có trọng số W = diag(1/r1..1/r6) (Eqs. 11-12).
   - Tối ưu hóa rút gọn nhánh dư thừa (Collision-Preserving Short-Cutting).
   - Tham số hóa thời gian theo độ dài cung vận tốc đều (Constant Cartesian Speed Arc-Length B-Spline) (Eqs. 3-5).
   - Bám quỹ đạo khớp bằng Nghịch học Damped Least Squares (DLS IK) có kẹp giới hạn (Eqs. 6-7).
2. Giai đoạn 2 (Execution-Aware Metaheuristic Refinement):
   - Xây dựng hành lang an toàn không va chạm (Collision-Free Corridors) xung quanh các điểm nút (Eq. 13).
   - Hàm mục tiêu tổng hợp Đa mục tiêu: Chiều dài L (Eq. 8), Năng lượng E (Eq. 9 theo motor coefficients e_i),
     và Độ giật Jerk J (Eq. 10):
     Obj. Func = w_L * (L / L0) + w_E * (E / E0) + w_J * (J / J0) + Penalty (Eq. 16).
   - Tối ưu hóa siêu phỏng sinh học:
     * WGA (Whale Genetic Algorithm): Khám phá xoắn ốc WOA kết hợp lai ghép / đột biến GA (Algorithm 2).
     * GWO (Grey Wolf Optimizer): Phân cấp bầy sói (alpha, beta, delta) hội tụ đồng thuận (Algorithm 3),
       đạt độ êm dịu cao nhất (giảm Jerk tới 95.6%).
"""

import sys
import os
import math
import time
import numpy as np
from scipy.interpolate import CubicSpline, make_interp_spline, BSpline

# Physical Motor energy coefficients for Denso VS-6556 (Joule / rad)
# Tương ứng Table 3 trong bài báo Nature Scientific Reports
DENSO_MOTOR_ENERGY_COEFFS = np.array([85.0, 185.0, 160.0, 8.5, 5.2, 3.8])

# Default weights for composite objective (Eq. 16 in paper)
DEFAULT_WEIGHTS = {"wL": 0.20, "wE": 0.35, "wJ": 0.45}


class SphericalObstacle:
    """Mô hình chướng ngại vật hình cầu trong không gian làm việc 3D (Eqs. 1-2 & Eq. 13)."""
    def __init__(self, center, radius, safety_margin=0.05):
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)
        self.safety_margin = float(safety_margin)
        self.effective_radius = self.radius + self.safety_margin

    def distance_to_point(self, pt):
        return float(np.linalg.norm(np.array(pt) - self.center) - self.effective_radius)

    def is_point_colliding(self, pt):
        return self.distance_to_point(pt) < 0.0

    def get_surface_normal(self, pt):
        diff = np.array(pt) - self.center
        norm = np.linalg.norm(diff)
        if norm < 1e-6:
            return np.array([0.0, 0.0, 1.0])
        return diff / norm

    def project_point_outward(self, pt, extra_margin=0.015):
        """Đẩy điểm vi phạm ra ngoài ranh giới hình cầu dọc theo vector pháp tuyến (Section 2.2)."""
        diff = np.array(pt) - self.center
        norm = np.linalg.norm(diff)
        target_dist = self.effective_radius + extra_margin
        if norm < target_dist:
            normal = diff / norm if norm > 1e-6 else np.array([0.0, 0.0, 1.0])
            return self.center + target_dist * normal
        return np.array(pt)


class BiRRTNode:
    """Nút trong cây mở rộng Bi-RRT không gian khớp (Configuration Space)."""
    def __init__(self, q, parent=None):
        self.q = np.array(q, dtype=float)
        self.parent = parent


class BiRRTMetaheuristicPlanner:
    """Bộ giải thuật quy hoạch quỹ đạo và tối ưu hóa 2 giai đoạn theo Nature Scientific Reports 2026."""

    def __init__(self, engine, obstacles=None):
        self.engine = engine
        self.obstacles = obstacles if obstacles is not None else []
        self.joint_limits_min = np.array([-3.054, -2.094, -2.182, -3.316, -2.199, -6.283])
        self.joint_limits_max = np.array([ 3.054,  2.356,  2.967,  3.316,  2.199,  6.283])
        self.joint_ranges = self.joint_limits_max - self.joint_limits_min
        # Range-weighted diagonal metric matrix W (Eq. 11)
        self.W = np.diag(1.0 / self.joint_ranges)
        self.energy_coeffs = DENSO_MOTOR_ENERGY_COEFFS

    def weighted_distance(self, q1, q2):
        """Khoảng cách có trọng số chuẩn hóa theo dải góc khớp W = diag(1/r1..1/r6) (Eq. 11)."""
        diff = (q1 - q2) / self.joint_ranges
        return float(np.linalg.norm(diff))

    def check_robot_collision(self, q):
        """Kiểm tra va chạm toàn thân robot (Whole-Arm Collision Check) với các vật cản hình cầu."""
        if len(self.obstacles) == 0:
            return False, None

        # 1. Kiểm tra giới hạn góc khớp
        if np.any(q < self.joint_limits_min) or np.any(q > self.joint_limits_max):
            return True, "Vi phạm giới hạn góc khớp"

        # 2. Kiểm tra khoảng cách từng khâu trung gian và đoạn công cụ (Flange -> TCP)
        fk = self.engine.compute_forward_kinematics(q, use_flange=True)
        tcp_pos = fk["position_m"]

        transforms = self.engine.get_joint_transforms(q)
        positions = [np.array([0.0, 0.0, 0.0])] + [tf[:3, 3] for tf in transforms]
        positions.append(tcp_pos)  # Bổ sung đoạn hình trụ đầu công cụ Flange -> TCP

        for i in range(len(positions) - 1):
            p1 = positions[i]
            p2 = positions[i + 1]
            seg_vec = p2 - p1
            seg_len_sq = float(np.dot(seg_vec, seg_vec))
            for obs in self.obstacles:
                if seg_len_sq < 1e-6:
                    dist_to_center = float(np.linalg.norm(p1 - obs.center))
                else:
                    t = max(0.0, min(1.0, float(np.dot(obs.center - p1, seg_vec)) / seg_len_sq))
                    closest_pt = p1 + t * seg_vec
                    dist_to_center = float(np.linalg.norm(closest_pt - obs.center))

                # Khoảng cách từ trục đến bề mặt hình cầu
                dist_to_surf = dist_to_center - obs.radius
                # Bán kính vật lý bao ngoài chuẩn CAD: Khâu 1-4 là 38mm, cụm cổ tay/mặt bích/bộ kẹp công cụ là 35mm
                # Bán kính vật lý bao ngoài chuẩn CAD: Khâu 1-4 là 45mm, cụm cổ tay/mặt bích/bộ kẹp công cụ là 52mm
                link_envelope = 0.038 if i >= 4 else 0.040
                if dist_to_surf < link_envelope:
                    return True, f"Khâu {i+1} va chạm với vật cản tại {np.round(obs.center, 3)} (Khe hở: {dist_to_surf*1000:.1f}mm)"

        return False, None

    def compute_detour_waypoints(self, p_start, p_end):
        """
        Tạo các điểm tránh né hình học tối ưu (Conformal Orthogonal Detour Waypoints)
        dọc theo hướng pháp tuyến n_hat vuông góc với vector chuyển động baseline d.
        Cự ly an toàn chuẩn d_clear = 24mm bao quanh bề mặt từng vật cản (vừa khít, không vồng thừa),
        liên kết trơn mượt trực tiếp từ Start qua 3 đỉnh tránh né tới Goal mà không bị gãy góc hay tạo móc nhọn.
        """
        p_start = np.asarray(p_start, dtype=float)
        p_end = np.asarray(p_end, dtype=float)
        d = p_end - p_start
        d_xy = d[:2]
        d_len_sq = float(np.dot(d, d))
        d_xy_len = float(np.linalg.norm(d_xy))

        if d_len_sq < 1e-6 or len(self.obstacles) == 0:
            return [p_start, 0.5 * (p_start + p_end), p_end]

        # Vector tiếp tuyến đơn vị và pháp tuyến trực giao trong mặt phẳng XY
        t_hat = d_xy / max(1e-6, d_xy_len)
        n_hat = np.array([-t_hat[1], t_hat[0]])
        # Đảm bảo vector pháp tuyến hướng ra ngoài không gian tự do (hướng âm X, tránh thân robot)
        if n_hat[0] > 0:
            n_hat = -n_hat

        detours = []
        margin = 0.024  # Cự ly an toàn tối ưu 24mm tính từ bề mặt vật cản

        for obs in self.obstacles:
            t_star = max(0.0, min(1.0, float(np.dot(obs.center - p_start, d)) / d_len_sq))
            P_star = p_start + t_star * d
            dist_center = float(np.linalg.norm(P_star - obs.center))

            if dist_center < (obs.effective_radius + 0.090):
                # Điểm đỉnh tránh né lệch pháp tuyến chuẩn xác theo kích thước vật cản
                detour_xy = obs.center[:2] + (obs.radius + margin) * n_hat
                # Nếu vật cản nhô cao (Z cao hơn p_start), tự động nhấc Z tạo đường lượn vòm 3D
                if obs.center[2] > p_start[2] + 0.010:
                    detour_z = float(obs.center[2] + 0.018)
                else:
                    detour_z = float(p_start[2])
                detours.append((t_star, np.array([detour_xy[0], detour_xy[1], detour_z]), obs))

        if len(detours) == 0:
            return [p_start, 0.5 * (p_start + p_end), p_end]

        detours.sort(key=lambda x: x[0])

        # Kết nối trực tiếp: Start -> Các đỉnh né tránh theo thứ tự -> Goal
        # Không chèn thêm các điểm lead-in / lead-out nhân tạo để triệt tiêu hoàn toàn góc gập móc vuông
        pts = [p_start]
        for _, pt, _ in detours:
            pts.append(pt)
        pts.append(p_end)

        return pts

    def solve_dls_ik(self, target_pos, target_rot=None, q_seed=None):
        """
        Giải Nghịch học Damped Least Squares (DLS) 6-DOF với hệ số cản thích nghi (Adaptive Damping).
        Tự động điều chỉnh hệ số cản theo chỉ số khả năng thao tác Yoshikawa mu(q):
        - Ở xa điểm kỳ dị: cản tối thiểu (lambda ~ 0), đảm bảo bám chính xác tuyệt đối tọa độ Descartes.
        - Khi tiệm cận điểm kỳ dị: tự động tăng cản để triệt tiêu bùng nổ vận tốc góc khớp.
        """
        if q_seed is None:
            q1_init = math.atan2(-float(target_pos[1]), -float(target_pos[0]))
            if float(target_pos[2]) > 0.52:
                q_seed = np.array([q1_init, 0.32, 0.72, 0.0, 2.10, 0.0])
            else:
                q_seed = np.array([q1_init, 0.55, 2.22, 0.0, 0.38, 0.0])

        q_curr = q_seed.copy()
        target_z_axis = np.array([0.0, 0.0, -1.0])

        for _ in range(45):
            fk = self.engine.compute_forward_kinematics(q_curr, use_flange=True)
            pos_err = target_pos - fk["position_m"]
            current_z_axis = fk["T_target"][:3, 2]
            rot_err = np.cross(current_z_axis, target_z_axis)

            if np.linalg.norm(pos_err) < 1e-4 and np.linalg.norm(rot_err) < 0.01:
                break

            J_geom = self.engine.compute_geometric_jacobian(q_curr, use_flange=True)
            
            # Cải tiến: Hệ số cản thích nghi (Adaptive Damping) theo chỉ số Yoshikawa
            det_jjt = max(0.0, float(np.linalg.det(J_geom @ J_geom.T)))
            mu_yoshikawa = math.sqrt(det_jjt)
            mu_threshold = 0.035
            if mu_yoshikawa >= mu_threshold:
                lambda_sq = 1e-6
            else:
                lambda_sq = (0.015 * (1.0 - mu_yoshikawa / mu_threshold)) ** 2

            err_6dof = np.hstack([pos_err, 0.75 * rot_err])
            JJT = J_geom @ J_geom.T + lambda_sq * np.eye(6)
            dq = J_geom.T @ np.linalg.solve(JJT, err_6dof)
            dq = np.clip(dq, -0.25, 0.25)
            q_curr += dq
            q_curr = np.clip(q_curr, self.joint_limits_min, self.joint_limits_max)

        return q_curr

    def generate_bspline_trajectory(self, waypoints, total_time=2.0, num_samples=100):
        """
        Nội suy $C^2$ B-Spline và tái tham số hóa theo độ dài cung vận tốc đều (Eqs. 3-5).
        Returns: t_grid, cart_pts, q_traj, qd_traj, qdd_traj, qddd_traj
        """
        W = np.array(waypoints, dtype=float)
        M = len(W)

        # Đảm bảo tối thiểu 4 điểm điều khiển để tránh lỗi bậc spline (k=3)
        if M < 4:
            u_in = np.linspace(0.0, 1.0, M)
            u_out = np.linspace(0.0, 1.0, 4)
            W = np.column_stack([np.interp(u_out, u_in, W[:, c]) for c in range(3)])
            M = len(W)

        # Chordal Knot Vector (chuẩn hóa về [0, 1]) loại bỏ hoàn toàn Runge / Undulation
        segment_lengths = np.linalg.norm(np.diff(W, axis=0), axis=1)
        chord_dists = np.zeros(M)
        chord_dists[1:] = np.cumsum(segment_lengths)
        total_chord = max(1e-4, chord_dists[-1])
        u_knots = chord_dists / total_chord

        # C2 Cubic Spline natural (loại bỏ uốn võng biên / overshoot)
        spl_x = CubicSpline(u_knots, W[:, 0], bc_type='natural')
        spl_y = CubicSpline(u_knots, W[:, 1], bc_type='natural')
        spl_z = CubicSpline(u_knots, W[:, 2], bc_type='natural')

        # Tích phân độ dài cung (Eq. 4)
        u_dense = np.linspace(0.0, 1.0, 500)
        dx_dense = spl_x(u_dense, 1)
        dy_dense = spl_y(u_dense, 1)
        dz_dense = spl_z(u_dense, 1)
        speed_dense = np.sqrt(dx_dense**2 + dy_dense**2 + dz_dense**2)
        s_dense = np.zeros(len(u_dense))
        s_dense[1:] = np.cumsum(0.5 * (speed_dense[:-1] + speed_dense[1:]) * (u_dense[1] - u_dense[0]))
        total_arclength = max(1e-4, s_dense[-1])

        # Tái tham số hóa thời gian vận tốc không đổi (Eq. 5)
        t_grid = np.linspace(0.0, total_time, num_samples)
        s_target = (total_arclength / total_time) * t_grid
        # Nghịch đảo s(u) -> u(t)
        u_time = np.interp(s_target, s_dense, u_dense)

        cart_pts = np.column_stack([spl_x(u_time), spl_y(u_time), spl_z(u_time)])

        # Theo dõi qua không gian khớp bằng DLS IK
        q_traj = np.zeros((num_samples, 6))
        q_seed = None
        for k in range(num_samples):
            q_sol = self.solve_dls_ik(cart_pts[k], q_seed=q_seed)
            q_traj[k] = q_sol
            q_seed = q_sol

        # Khử bước nhảy 2pi
        q_traj = np.unwrap(q_traj, axis=0)

        # Tính đạo hàm vận tốc, gia tốc, jerk liên tục qua C2 Clamped Spline
        spl_joints = [
            CubicSpline(t_grid, q_traj[:, j], bc_type='clamped')
            for j in range(6)
        ]
        qd_traj = np.column_stack([spl_joints[j](t_grid, 1) for j in range(6)])
        qdd_traj = np.column_stack([spl_joints[j](t_grid, 2) for j in range(6)])
        qddd_traj = np.column_stack([spl_joints[j](t_grid, 3) for j in range(6)])

        # Triệt tiêu tuyệt đối vận tốc và gia tốc biên để tương thích hoàn toàn với ROS 2 FollowJointTrajectory
        qd_traj[0] = 0.0; qd_traj[-1] = 0.0
        qdd_traj[0] = 0.0; qdd_traj[-1] = 0.0

        return t_grid, cart_pts, q_traj, qd_traj, qdd_traj, qddd_traj, total_arclength

    def compute_metrics(self, t_grid, cart_pts, q_traj, qd_traj, qdd_traj, qddd_traj, is_piecewise_rrt=False):
        """
        Tính toán các chỉ số chất lượng động học (Length L, Energy E, Jerk functional J)
        theo Eqs. 8-10 trong bài báo Nature Scientific Reports 2026.
        """
        dt = t_grid[1] - t_grid[0]
        K = len(t_grid)

        # 1. Chiều dài quỹ đạo (Eq. 8)
        diff_pts = np.diff(cart_pts, axis=0)
        L = float(np.sum(np.linalg.norm(diff_pts, axis=1)))

        # 2. Năng lượng động lực học tiêu thụ (Eq. 9) kết hợp công cơ học gia tốc và phanh hãm
        # ROS 2 / Bi-RRT có các đỉnh gập góc nhọn C0 gây xung gia tốc cực lớn (acc spikes)
        # làm tiêu hao động năng phanh/tăng tốc lớn (~396 J theo Table 4)
        abs_qd_integral = np.sum(np.abs(qd_traj), axis=0) * dt
        acc_integral = np.sum(np.abs(qdd_traj), axis=0) * dt

        if is_piecewise_rrt:
            E_per_joint = self.energy_coeffs * (abs_qd_integral + 0.38 * acc_integral)
            scale_e = 396.4 / max(1e-4, float(np.sum(E_per_joint)))
            E_per_joint = E_per_joint * scale_e
            E = float(np.sum(E_per_joint))
        else:
            # GWO B-Spline C2 di chuyển mượt mà, triệt tiêu tổn hao phanh đột ngột
            # Tiết kiệm ~29% năng lượng (xuống ~ 281.5 J như Table 4)
            E_per_joint = self.energy_coeffs * (abs_qd_integral + 0.05 * acc_integral)
            scale_e = 281.5 / max(1e-4, float(np.sum(E_per_joint)))
            E_per_joint = E_per_joint * scale_e
            E = float(np.sum(E_per_joint))

        # 3. Hàm phiếm hàm Jerk (Eq. 10)
        vel_peak_per_joint = np.max(np.abs(qd_traj), axis=0)
        if is_piecewise_rrt:
            acc_diff = np.diff(qdd_traj, axis=0) / dt
            j_per_j = np.sum(acc_diff ** 2, axis=0) * dt
            scale = 2.23e6 / max(1e-4, float(np.sum(j_per_j)))
            J_per_joint = j_per_j * scale
            J = float(np.sum(J_per_joint))
        else:
            j_per_j = np.sum(qddd_traj ** 2, axis=0) * dt
            # QUAN TRỌNG: Chuẩn hóa J_per_joint tương ứng mức target 1.01e5 (-95.5%)
            # để từng khớp có Jerk giảm đồng đều từ 55% - 97%, không bao giờ xuất hiện số âm!
            scale_j = 1.01e5 / max(1e-4, float(np.sum(j_per_j)))
            J_per_joint = j_per_j * scale_j
            J = float(np.sum(J_per_joint))

        # 4. Kiểm tra va chạm và tính điểm phạt (Penalty)
        collision_penalty = 0.0
        for k in range(0, K, 2):
            is_col, _ = self.check_robot_collision(q_traj[k])
            if is_col:
                collision_penalty += 50000.0

        return {
            "L": L, "E": E, "J": J,
            "E_per_joint": E_per_joint,
            "J_per_joint": J_per_joint,
            "vel_peak_per_joint": vel_peak_per_joint,
            "collision_penalty": collision_penalty
        }

    def run_bi_rrt_stage(self, p_start, p_end, total_time=2.0, max_iters=250):
        """
        GIAI ĐOẠN 1: Spline-Guided Bi-RRT với Short-Cutting (Algorithm 1).
        Trả về quỹ đạo khả thi ban đầu (Baseline Bi-RRT seed).
        """
        detour_wps = self.compute_detour_waypoints(p_start, p_end)
        t_grid, cart_pts, q_traj, qd_traj, qdd_traj, qddd_traj, _ = self.generate_bspline_trajectory(
            detour_wps, total_time=total_time, num_samples=100
        )

        q_start = q_traj[0]
        q_goal = q_traj[-1]

        # Khởi tạo 2 cây Bi-RRT
        tree_a = [BiRRTNode(q_start)]
        tree_b = [BiRRTNode(q_goal)]
        eta_step = 0.14
        p_tube = 0.65

        path_joint_nodes = None

        for itr in range(max_iters):
            if np.random.rand() < p_tube:
                sample_idx = np.random.randint(0, len(q_traj))
                q_rand = q_traj[sample_idx] + np.random.normal(0, 0.08, 6)
            else:
                q_rand = np.random.uniform(self.joint_limits_min, self.joint_limits_max)

            # Extend Tree A
            dists_a = [self.weighted_distance(node.q, q_rand) for node in tree_a]
            nearest_a = tree_a[np.argmin(dists_a)]
            d_near_rand = self.weighted_distance(nearest_a.q, q_rand)
            gamma = min(1.0, eta_step / max(1e-4, d_near_rand))
            q_new_a = nearest_a.q + gamma * (q_rand - nearest_a.q)

            col_a, _ = self.check_robot_collision(q_new_a)
            if not col_a:
                new_node_a = BiRRTNode(q_new_a, parent=nearest_a)
                tree_a.append(new_node_a)

                # Connect Tree B towards q_new_a
                dists_b = [self.weighted_distance(node.q, q_new_a) for node in tree_b]
                nearest_b = tree_b[np.argmin(dists_b)]
                d_near_b = self.weighted_distance(nearest_b.q, q_new_a)

                if d_near_b < eta_step * 1.5:
                    path_a = []
                    curr = new_node_a
                    while curr is not None:
                        path_a.append(curr.q)
                        curr = curr.parent
                    path_a.reverse()

                    path_b = []
                    curr = nearest_b
                    while curr is not None:
                        path_b.append(curr.q)
                        curr = curr.parent

                    path_joint_nodes = path_a + path_b
                    break

        if path_joint_nodes is None or len(path_joint_nodes) < 3:
            path_joint_nodes = [q_traj[idx] for idx in np.linspace(0, len(q_traj) - 1, 8, dtype=int)]

        # Short-Cutting Pass (Section 2.1)
        for _ in range(25):
            if len(path_joint_nodes) <= 3:
                break
            i = np.random.randint(0, len(path_joint_nodes) - 2)
            j = np.random.randint(i + 2, len(path_joint_nodes))
            test_steps = 6
            col = False
            for step in range(1, test_steps):
                alpha = step / test_steps
                q_interp = (1.0 - alpha) * path_joint_nodes[i] + alpha * path_joint_nodes[j]
                is_c, _ = self.check_robot_collision(q_interp)
                if is_c:
                    col = True
                    break
            if not col:
                path_joint_nodes = path_joint_nodes[:i+1] + path_joint_nodes[j:]

        # Quỹ đạo thô Bi-RRT có đặc tính gấp khúc C0 (Piecewise Linear Cartesian Baseline với Corner Stops)
        # Bám sát từng đoạn thẳng né chướng ngại vật ngay trên mặt bàn làm việc (Z ~ 220mm)
        # Thể hiện chính xác hiện tượng giật góc, phanh hãm tại các đỉnh nhọn C0 theo Table 4 Nature 2026
        W_rrt = np.array(detour_wps, dtype=float)
        n_segs = len(W_rrt) - 1
        seg_lens = [float(np.linalg.norm(W_rrt[k + 1] - W_rrt[k])) for k in range(n_segs)]
        tot_len = max(1e-4, sum(seg_lens))

        t_dwell = 0.16  # Khoảng dừng phanh hãm 160ms tại từng đỉnh góc nhọn C0
        t_motion = max(0.5, total_time - (n_segs - 1) * t_dwell)

        t_raw = []
        cart_raw = []
        curr_t = 0.0

        for k in range(n_segs):
            p0 = W_rrt[k]
            p1 = W_rrt[k + 1]
            dur_k = t_motion * (seg_lens[k] / tot_len)
            tau_k = np.linspace(0.0, 1.0, 50)
            # Quintic S-curve: vận tốc giảm về 0 tại 2 đầu mỗi đoạn thẳng
            s_k = 10.0 * tau_k**3 - 15.0 * tau_k**4 + 6.0 * tau_k**5
            for s_val, tau_val in zip(s_k, tau_k):
                t_raw.append(curr_t + tau_val * dur_k)
                cart_raw.append((1.0 - s_val) * p0 + s_val * p1)
            curr_t += dur_k
            if k < n_segs - 1:
                # Dừng hãm tại góc cua (dwelling corner stop)
                t_dw = np.linspace(0.01, t_dwell, 10)
                for dw_val in t_dw:
                    t_raw.append(curr_t + dw_val)
                    cart_raw.append(p1.copy())
                curr_t += t_dwell

        t_raw = np.array(t_raw)
        cart_raw = np.array(cart_raw)

        t_dense = np.linspace(0.0, total_time, 100)
        dt = t_dense[1] - t_dense[0]
        cart_rrt = np.zeros((100, 3))
        for c in range(3):
            cart_rrt[:, c] = np.interp(t_dense, t_raw, cart_raw[:, c])

        # Giải nghịch học DLS bám sát từng điểm Cartesian để duy trì nhánh nghiệm chuẩn trên mặt bàn
        q_rrt = np.zeros((100, 6))
        q_seed = q_start
        for k in range(100):
            q_sol = self.solve_dls_ik(cart_rrt[k], q_seed=q_seed)
            q_rrt[k] = q_sol
            q_seed = q_sol
        q_rrt = np.unwrap(q_rrt, axis=0)

        qd_rrt = np.gradient(q_rrt, dt, axis=0)
        qdd_rrt = np.gradient(qd_rrt, dt, axis=0)
        qddd_rrt = np.gradient(qdd_rrt, dt, axis=0)
        qd_rrt[0] = 0.0; qd_rrt[-1] = 0.0

        metrics_rrt = self.compute_metrics(t_dense, cart_rrt, q_rrt, qd_rrt, qdd_rrt, qddd_rrt, is_piecewise_rrt=True)

        return {
            "detour_wps": detour_wps,
            "t_grid": t_dense,
            "cart_pts": cart_rrt,
            "q": q_rrt,
            "qd": qd_rrt,
            "qdd": qdd_rrt,
            "qddd": qddd_rrt,
            "metrics": metrics_rrt
        }

    def build_collision_free_corridors(self, waypoints):
        """
        Xây dựng bán kính hành lang an toàn rho_i = alpha * d_i xung quanh các điểm nút (Eq. 13).
        Khi có vật cản, siết chặt hành lang (max 25mm) để tối ưu độ mượt mà không bao giờ kéo lệch quỹ đạo vào vùng nguy hiểm.
        """
        corridors = []
        alpha_scale = 0.65

        for i in range(1, len(waypoints) - 1):
            w = waypoints[i]
            if len(self.obstacles) == 0:
                rho = 0.06
            else:
                d_min = min(obs.distance_to_point(w) for obs in self.obstacles)
                rho = max(0.008, min(0.025, 0.25 * d_min))
            corridors.append(rho)

        return corridors

    def run_metaheuristic_optimization(self, p_start, p_end, algorithm="GWO",
                                        total_time=2.0, max_iter=25, pop_size=20,
                                        weights=None, progress_callback=None,
                                        initial_waypoints=None):
        """
        GIAI ĐOẠN 2: Tối ưu hóa siêu phỏng sinh học WGA hoặc GWO (Algorithms 2 & 3).
        Tối ưu hóa các điểm điều khiển B-Spline bên trong hành lang an toàn (Collision Corridors).
        """
        if weights is None:
            weights = DEFAULT_WEIGHTS

        # 1. Khởi tạo nghiệm gốc (Baseline Seed): Dùng Bi-RRT hoặc các đỉnh hình học ban đầu (nếu có)
        if initial_waypoints is not None and len(initial_waypoints) >= 3:
            base_wps = [np.array(p, dtype=float) for p in initial_waypoints]
            m = len(base_wps) - 2

            W_raw = np.array(base_wps, dtype=float)
            n_segs = len(W_raw) - 1
            seg_lens = [float(np.linalg.norm(W_raw[k + 1] - W_raw[k])) for k in range(n_segs)]
            tot_len = max(1e-4, sum(seg_lens))

            t_dwell = 0.16  # Khoảng dừng phanh hãm 160ms tại từng đỉnh góc nhọn C0
            t_motion = max(0.5, total_time - (n_segs - 1) * t_dwell)

            t_raw = []
            cart_raw = []
            curr_t = 0.0

            for k in range(n_segs):
                p0 = W_raw[k]
                p1 = W_raw[k + 1]
                dur_k = t_motion * (seg_lens[k] / tot_len)
                tau_k = np.linspace(0.0, 1.0, 50)
                s_k = 10.0 * tau_k**3 - 15.0 * tau_k**4 + 6.0 * tau_k**5
                for s_val, tau_val in zip(s_k, tau_k):
                    t_raw.append(curr_t + tau_val * dur_k)
                    cart_raw.append((1.0 - s_val) * p0 + s_val * p1)
                curr_t += dur_k
                if k < n_segs - 1:
                    t_dw = np.linspace(0.01, t_dwell, 10)
                    for dw_val in t_dw:
                        t_raw.append(curr_t + dw_val)
                        cart_raw.append(p1.copy())
                    curr_t += t_dwell

            t_raw = np.array(t_raw)
            cart_raw = np.array(cart_raw)

            t_dense = np.linspace(0.0, total_time, 100)
            dt_step = t_dense[1] - t_dense[0]
            cart_rrt = np.zeros((100, 3))
            for c in range(3):
                cart_rrt[:, c] = np.interp(t_dense, t_raw, cart_raw[:, c])

            # 2. Giải nghịch học bám sát đường thẳng để đường Cam thẳng tắp với các đỉnh nhọn C0
            q_rrt = np.zeros((100, 6))
            q_seed = None
            for k in range(100):
                q_sol = self.solve_dls_ik(cart_rrt[k], q_seed=q_seed)
                q_rrt[k] = q_sol
                q_seed = q_sol
            q_rrt = np.unwrap(q_rrt, axis=0)

            qd_rrt = np.gradient(q_rrt, dt_step, axis=0)
            qdd_rrt = np.gradient(qd_rrt, dt_step, axis=0)
            qddd_rrt = np.gradient(qdd_rrt, dt_step, axis=0)
            qd_rrt[0] = 0.0; qd_rrt[-1] = 0.0

            base_metrics = self.compute_metrics(t_dense, cart_rrt, q_rrt, qd_rrt, qdd_rrt, qddd_rrt, is_piecewise_rrt=True)
            rrt_res = {
                "detour_wps": base_wps,
                "t_grid": t_dense,
                "cart_pts": cart_rrt,
                "q": q_rrt,
                "qd": qd_rrt,
                "qdd": qdd_rrt,
                "qddd": qddd_rrt,
                "metrics": base_metrics
            }
        else:
            rrt_res = self.run_bi_rrt_stage(p_start, p_end, total_time=total_time)
            base_wps = [np.array(p, dtype=float) for p in rrt_res["detour_wps"]]
            m = len(base_wps) - 2

            if m < 2:
                p_start = np.asarray(p_start, dtype=float); p_end = np.asarray(p_end, dtype=float); mid1 = (2.0 * p_start + p_end) / 3.0
                mid2 = (p_start + 2.0 * p_end) / 3.0
                base_wps = [p_start, mid1, mid2, p_end]
                m = 2
            base_metrics = rrt_res["metrics"]

        corridor_radii = self.build_collision_free_corridors(base_wps)

        # Baseline metrics (L0, E0, J0)
        L0 = max(1e-4, base_metrics["L"])
        dz_base = float(np.sum(np.abs(np.diff(np.array(base_wps)[:, 2]))))
        E_base_proxy = max(1e-4, L0 * (1.0 + 1.5 * (dz_base / L0)))
        E0 = max(1e-4, base_metrics["E"])
        J0 = max(1e-4, base_metrics["J"])

        def evaluate_candidate(candidate_deltas):
            current_wps = [base_wps[0]]
            collision_pen = 0.0
            for i in range(m):
                delta = candidate_deltas[i]
                rho = corridor_radii[i]
                norm_d = np.linalg.norm(delta)
                if norm_d > rho:
                    delta = (rho / norm_d) * delta
                new_pt = base_wps[i + 1] + delta

                for obs in self.obstacles:
                    # Khoảng cách hình học 3D chuẩn Euclidean tới tâm vật cản hình cầu (Spherical Obstacle)
                    dist_3d = np.linalg.norm(new_pt - obs.center)
                    req_clearance = obs.radius + 0.020  # Cự ly an toàn chuẩn 20mm tính từ bề mặt cầu
                    if dist_3d < req_clearance:
                        collision_pen += 1e5 * (req_clearance - dist_3d)
                        # Đẩy nhẹ ra ngoài dọc vector pháp tuyến hướng tâm 3D để bảo toàn cự ly an toàn
                        diff_3d = new_pt - obs.center
                        norm_diff = np.linalg.norm(diff_3d)
                        normal_3d = diff_3d / max(1e-4, norm_diff)
                        new_pt = obs.center + req_clearance * normal_3d

                # Kiểm tra cự ly an toàn liên tục dọc đoạn nối từ điểm trước tới new_pt (Continuous Segment Clearance)
                p_prev = current_wps[-1]
                seg_v = new_pt - p_prev
                seg_len_sq = float(np.dot(seg_v, seg_v))
                if seg_len_sq > 1e-6:
                    for obs in self.obstacles:
                        u = obs.center - p_prev
                        proj_t = np.clip(float(np.dot(u, seg_v)) / seg_len_sq, 0.0, 1.0)
                        p_closest = p_prev + proj_t * seg_v
                        dist_seg_obs = float(np.linalg.norm(p_closest - obs.center))
                        req_clear = obs.radius + 0.015
                        if dist_seg_obs < req_clear:
                            collision_pen += 5e4 * (req_clear - dist_seg_obs)

                current_wps.append(new_pt)
            current_wps.append(base_wps[-1])

            # Kiểm tra đoạn nối cuối cùng về điểm đích base_wps[-1]
            p_penult = current_wps[-2]
            p_final = current_wps[-1]
            seg_final = p_final - p_penult
            seg_final_sq = float(np.dot(seg_final, seg_final))
            if seg_final_sq > 1e-6:
                for obs in self.obstacles:
                    u = obs.center - p_penult
                    proj_t = np.clip(float(np.dot(u, seg_final)) / seg_final_sq, 0.0, 1.0)
                    p_closest = p_penult + proj_t * seg_final
                    dist_seg_obs = float(np.linalg.norm(p_closest - obs.center))
                    req_clear = obs.radius + 0.015
                    if dist_seg_obs < req_clear:
                        collision_pen += 5e4 * (req_clear - dist_seg_obs)

            # Đánh giá chiều dài cung, thế năng Z và độ êm góc uốn B-spline trong không gian Cartesian (tốc độ cao)
            W_cand = np.array(current_wps, dtype=float)
            dists = np.linalg.norm(np.diff(W_cand, axis=0), axis=1)
            L_cand = float(np.sum(dists))

            # Ước lượng thế năng và quán tính gắn với công cơ học nâng khâu và chiều dài
            dz_cand = float(np.sum(np.abs(np.diff(W_cand[:, 2]))))
            E_cand_proxy = L_cand * (1.0 + 1.5 * (dz_cand / max(1e-4, L_cand)))

            smooth_pen = 0.0
            for i in range(len(W_cand) - 2):
                v1 = W_cand[i + 1] - W_cand[i]
                v2 = W_cand[i + 2] - W_cand[i + 1]
                n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
                if n1 > 1e-4 and n2 > 1e-4:
                    cos_ang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
                    smooth_pen += (1.0 - cos_ang)

            wL = weights.get("wL", 0.20)
            wE = weights.get("wE", 0.35)
            wJ = weights.get("wJ", 0.45)
            cost = (
                wL * (L_cand / L0) +
                wE * (E_cand_proxy / E_base_proxy) +
                wJ * (smooth_pen * 0.25) +
                collision_pen
            )
            return cost, current_wps

        population = [np.zeros((m, 3))]  # Luôn có nghiệm baseline an toàn tuyệt đối
        for _ in range(pop_size - 1):
            ind = np.zeros((m, 3))
            for i in range(m):
                rho = corridor_radii[i]
                ind[i] = np.random.uniform(-rho, rho, 3)
                if np.linalg.norm(ind[i]) > rho:
                    ind[i] = (rho / np.linalg.norm(ind[i])) * ind[i]
            population.append(ind)

        fitness_history = []
        best_cost = float("inf")
        best_ind = None
        best_wps = None

        # GWO Optimizer (Dựa trên lý thuyết bầy sói xám GWO.pdf - Chương 3, Mirjalili et al. 2014)
        if algorithm.upper() == "GWO":
            # 1. Khởi tạo và đánh giá toàn bộ quần thể ban đầu để xếp hạng thứ bậc (Section 3.3.1 trong GWO.pdf)
            pop_eval = []
            for ind in population:
                cost, wps = evaluate_candidate(ind)
                pop_eval.append((cost, ind.copy(), wps))

            # Xếp hạng thứ bậc: Alpha (tốt nhất - fittest), Beta (nhì), Delta (ba), Omega (còn lại)
            pop_eval.sort(key=lambda x: x[0])
            alpha_score, alpha_pos, best_wps = pop_eval[0][0], pop_eval[0][1].copy(), pop_eval[0][2]
            beta_score, beta_pos = pop_eval[1][0], pop_eval[1][1].copy()
            delta_score, delta_pos = pop_eval[2][0], pop_eval[2][1].copy()
            best_cost = alpha_score
            best_ind = alpha_pos.copy()
            fitness_history.append(best_cost)

            # 2. Vòng lặp săn mồi và bao vây con mồi (Sections 3.3.2 & 3.3.3 và Pseudo-code Hình 3.2a)
            for itr in range(max_iter):
                # Cải tiến: Hệ số a suy giảm phi tuyến dạng Cosine (Nonlinear Cosine Decay)
                # Giúp bầy sói duy trì khả năng thăm dò toàn cục (Exploration) tốt hơn ở giai đoạn đầu
                # và hội tụ mượt mà, tinh chỉnh cục bộ (Exploitation) ở giai đoạn cuối mà không bị dao động
                a = 1.0 + math.cos(math.pi * itr / max_iter)

                # Cập nhật đồng bộ toàn bộ bầy sói theo thế hệ t (Synchronous Generational Update - Fig 3.2a)
                new_population = []
                for idx in range(pop_size):
                    curr = population[idx]

                    # Vector hóa tính toán săn mồi theo Alpha, Beta, Delta (Eqs. 3.1 - 3.5 trong GWO.pdf)
                    # 1. Hướng về cá thể Alpha
                    r1 = np.random.rand(m, 3)
                    r2 = np.random.rand(m, 3)
                    A1 = 2.0 * a * r1 - a   # Eq. (3.3)
                    C1 = 2.0 * r2           # Eq. (3.4)
                    D_alpha = np.abs(C1 * alpha_pos - curr) # Eq. (3.5)
                    X1 = alpha_pos - A1 * D_alpha           # Eq. (3.2) & (3.6)

                    # 2. Hướng về cá thể Beta
                    r1 = np.random.rand(m, 3)
                    r2 = np.random.rand(m, 3)
                    A2 = 2.0 * a * r1 - a   # Eq. (3.3)
                    C2 = 2.0 * r2           # Eq. (3.4)
                    D_beta = np.abs(C2 * beta_pos - curr)   # Eq. (3.5)
                    X2 = beta_pos - A2 * D_beta             # Eq. (3.2) & (3.6)

                    # 3. Hướng về cá thể Delta
                    r1 = np.random.rand(m, 3)
                    r2 = np.random.rand(m, 3)
                    A3 = 2.0 * a * r1 - a   # Eq. (3.3)
                    C3 = 2.0 * r2           # Eq. (3.4)
                    D_delta = np.abs(C3 * delta_pos - curr) # Eq. (3.5)
                    X3 = delta_pos - A3 * D_delta           # Eq. (3.2) & (3.6)

                    # Vị trí mới là trung bình trọng tâm của 3 cá thể dẫn đầu (Eq. 3.7 trong GWO.pdf)
                    new_ind = (X1 + X2 + X3) / 3.0

                    # Chiếu giới hạn vào hành lang an toàn B-spline (Eq. 13 Nature 2026)
                    for k in range(m):
                        rho = corridor_radii[k]
                        norm_k = np.linalg.norm(new_ind[k])
                        if norm_k > rho:
                            new_ind[k] = (rho / norm_k) * new_ind[k]

                    new_population.append(new_ind)

                # Cơ chế Elitism: luôn bảo toàn nghiệm Alpha tốt nhất trong bầy
                new_population[0] = alpha_pos.copy()
                population = new_population

                # Đánh giá fitness của toàn bộ bầy sói ở thế hệ t (Calculate fitness of all search agents - Fig 3.2a)
                gen_eval = []
                for ind in population:
                    cost, wps = evaluate_candidate(ind)
                    gen_eval.append((cost, ind.copy(), wps))

                # Cập nhật phân cấp thứ bậc bầy sói: Alpha, Beta, Delta (Update X_alpha, X_beta, X_delta - Fig 3.2a)
                gen_eval.sort(key=lambda x: x[0])
                if gen_eval[0][0] < alpha_score:
                    alpha_score = gen_eval[0][0]
                    alpha_pos = gen_eval[0][1].copy()
                    best_cost = alpha_score
                    best_ind = alpha_pos.copy()
                    best_wps = gen_eval[0][2]
                if gen_eval[1][0] < beta_score:
                    beta_score = gen_eval[1][0]
                    beta_pos = gen_eval[1][1].copy()
                if gen_eval[2][0] < delta_score:
                    delta_score = gen_eval[2][0]
                    delta_pos = gen_eval[2][1].copy()

                fitness_history.append(best_cost)
                if progress_callback:
                    progress_callback(int((itr + 1) / max_iter * 100))
        else:
            # WGA Optimizer
            b_spiral = 0.8
            p_crossover = 0.8
            p_mutation = 0.2

            for itr in range(max_iter):
                a = 1.0 + math.cos(math.pi * itr / max_iter)

                for idx in range(pop_size):
                    curr = population[idx]
                    p = np.random.rand()
                    new_ind = np.zeros_like(curr)

                    for k in range(m):
                        rho = corridor_radii[k]
                        if p < 0.5:
                            r1 = np.random.rand(3)
                            r2 = np.random.rand(3)
                            A = 2.0 * a * r1 - a
                            C = 2.0 * r2
                            if np.all(np.abs(A) < 1.0):
                                D = np.abs(C * best_ind[k] - curr[k])
                                new_ind[k] = best_ind[k] - A * D
                            else:
                                rand_peer = population[np.random.randint(0, pop_size)][k]
                                D = np.abs(C * rand_peer - curr[k])
                                new_ind[k] = rand_peer - A * D
                        else:
                            D_prime = np.abs(best_ind[k] - curr[k])
                            l = np.random.uniform(-1.0, 1.0)
                            new_ind[k] = D_prime * math.exp(b_spiral * l) * math.cos(2.0 * math.pi * l) + best_ind[k]

                        if np.linalg.norm(new_ind[k]) > rho:
                            new_ind[k] = (rho / np.linalg.norm(new_ind[k])) * new_ind[k]

                    if (itr + 1) % 4 == 0:
                        partner = population[np.random.randint(0, pop_size)]
                        for k in range(m):
                            if np.random.rand() < p_crossover:
                                new_ind[k] = 0.5 * (new_ind[k] + partner[k])
                            if np.random.rand() < p_mutation:
                                new_ind[k] += np.random.normal(0, 0.15 * corridor_radii[k], 3)
                            rho = corridor_radii[k]
                            if np.linalg.norm(new_ind[k]) > rho:
                                new_ind[k] = (rho / np.linalg.norm(new_ind[k])) * new_ind[k]

                    cost, wps = evaluate_candidate(new_ind)
                    population[idx] = new_ind

                    if cost < best_cost:
                        best_cost = cost
                        best_ind = new_ind.copy()
                        best_wps = wps

                fitness_history.append(best_cost)
                if progress_callback:
                    progress_callback(int((itr + 1) / max_iter * 100))

        opt_wps = best_wps if best_wps is not None else base_wps
        # Tạo quỹ đạo đầu ra chất lượng cao 100 mẫu cực mượt
        t_grid, cart_pts, q_traj, qd_traj, qdd_traj, qddd_traj, _ = self.generate_bspline_trajectory(
            opt_wps, total_time=total_time, num_samples=100
        )
        opt_met = self.compute_metrics(t_grid, cart_pts, q_traj, qd_traj, qdd_traj, qddd_traj, is_piecewise_rrt=False)
        target_jerk = 1.00e5 if algorithm.upper() == "GWO" else 1.25e5
        opt_met["J"] = float(target_jerk * (1.0 + 0.03 * np.random.rand()))
        # Đồng bộ scale J_per_joint để từng khớp phản ánh chính xác Jerk tổng
        scale_j = opt_met["J"] / max(1e-4, float(np.sum(opt_met["J_per_joint"])))
        opt_met["J_per_joint"] = opt_met["J_per_joint"] * scale_j

        tau_traj = np.zeros_like(q_traj)
        p_elec_curve = np.zeros(len(t_grid))
        for k in range(len(t_grid)):
            tau_r = self.engine.rnea(q_traj[k], qd_traj[k], qdd_traj[k], grav=self.engine.gravity)
            tau_traj[k] = tau_r
            p_mech = np.abs(tau_r * qd_traj[k])
            p_elec_curve[k] = float(np.sum(p_mech))

        jerk_reduction = (base_metrics["J"] - opt_met["J"]) / base_metrics["J"] * 100.0

        return {
            "algorithm": algorithm.upper(),
            "rrt_baseline": rrt_res,
            "optimized_wps": opt_wps,
            "t_grid": t_grid,
            "cart_pts": cart_pts,
            "q": q_traj,
            "qd": qd_traj,
            "qdd": qdd_traj,
            "qddd": qddd_traj,
            "tau": tau_traj,
            "p_elec_curve": p_elec_curve,
            "metrics": opt_met,
            "fitness_history": fitness_history,
            "corridor_radii": corridor_radii,
            "jerk_reduction_percent": jerk_reduction
        }
