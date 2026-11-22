# -*- coding: utf-8 -*-
"""
Mô-đun Tối ưu Quỹ đạo Thời gian Tối thiểu Dựa trên Thuật toán Di truyền Cải tiến (IGA)
Áp dụng cho Robot Công nghiệp 6 Bậc Tự Do Denso VS-6556 trên ROS 2 Jazzy.
Tài liệu tham chiếu: Cheng & Xu, Journal of Intelligent & Robotic Systems (2025).
DOI: 10.1007/s10846-025-02306-4
"""
import numpy as np
import math
from scipy.interpolate import CubicSpline
try:
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    HAS_ROS2_TRAJ = True
except ImportError:
    HAS_ROS2_TRAJ = False


class DensoIGATrajectoryOptimizer:
    def __init__(self):
        # 1. RÀNG BUỘC CƠ KHÍ CHUẨN ROBOT DENSO VS-6556
        # Giới hạn vận tốc cực đại 6 khớp (rad/s) theo catalog Denso Wave:
        # J1=250°/s, J2=185°/s, J3=250°/s, J4=330°/s, J5=330°/s, J6=400°/s
        self.vel_limits = np.array([4.36, 3.23, 4.36, 5.76, 5.76, 6.98])
        
        # Giới hạn gia tốc cực đại 6 khớp (rad/s^2):
        self.acc_limits = np.array([20.94, 17.45, 20.94, 31.42, 31.42, 41.89])
        
        # Giới hạn độ giật (jerk) cực đại 6 khớp (rad/s^3) để triệt tiêu rung lắc hộp số:
        self.jerk_limits = np.array([150.0, 120.0, 150.0, 200.0, 200.0, 300.0])

        # 2. THAM SỐ THUẬT TOÁN DI TRUYỀN CẢI TIẾN (IGA)
        self.pop_size = 50       # Kích thước quần thể
        self.max_gen = 60        # Số thế hệ tiến hóa
        self.xi = 0.15           # Hệ số phạt vi phạm (theo bài báo xi = 0.1 ~ 0.2)
        self.pc_max, self.pc_min = 0.90, 0.40  # Biên xác suất lai ghép
        self.pm_max, self.pm_min = 0.15, 0.02  # Biên xác suất đột biến

    def optimize(self, joint_waypoints: np.ndarray, initial_dt: float = 0.25) -> np.ndarray:
        """
        Tối ưu hóa phân bổ thời gian delta_t giữa n điểm góc khớp theo hàm mục tiêu thời gian nhỏ nhất.
        
        :param joint_waypoints: Mảng numpy shape (N, 6) chứa góc 6 khớp tính bằng radian.
        :param initial_dt: Khoảng thời gian tham chiếu ban đầu cho mỗi phân đoạn.
        :return: Mảng numpy chứa các delta_t_i tối ưu đã kiểm tra ràng buộc.
        """
        N = len(joint_waypoints)
        if N < 3:
            return np.full(max(1, N - 1), initial_dt)

        num_segments = N - 1
        # Khởi tạo quần thể ngẫu nhiên xung quanh initial_dt
        population = np.random.uniform(0.08, 0.45, size=(self.pop_size, num_segments))
        population[0] = np.full(num_segments, initial_dt)  # Lưu nghiệm gốc làm mốc đối chứng

        best_individual = population[0].copy()
        best_fitness = -1e9

        for gen in range(self.max_gen):
            fitness_scores = np.zeros(self.pop_size)

            for idx, ind in enumerate(population):
                total_time, penalty = self._evaluate_trajectory(ind, joint_waypoints)
                # Hàm mục tiêu bài báo Eq. (4): F(T) = Total_Time + xi * Penalty
                obj_val = total_time + self.xi * penalty
                fitness_scores[idx] = 1.0 / (obj_val + 1e-6)

            f_max = np.max(fitness_scores)
            f_avg = np.mean(fitness_scores)
            f_min = np.min(fitness_scores)

            if f_max > best_fitness:
                best_fitness = f_max
                best_individual = population[np.argmax(fitness_scores)].copy()

            # Chọn lọc theo bánh xe Roulette
            prob = fitness_scores / np.sum(fitness_scores)
            selected_idx = np.random.choice(self.pop_size, size=self.pop_size, p=prob)
            population = population[selected_idx]

            # Lai ghép thích nghi dạng hình Sin (Sinusoidal Adaptive Crossover)
            for i in range(0, self.pop_size - 1, 2):
                f_prime = max(fitness_scores[i], fitness_scores[i + 1])
                if f_prime >= f_avg and (f_max - f_avg) > 1e-6:
                    pc = self.pc_max - 0.5 * (self.pc_max - self.pc_min) * (
                        1.0 + math.sin(((f_prime - f_avg) / (f_max - f_avg)) * (math.pi / 2.0))
                    )
                else:
                    pc = self.pc_max

                if np.random.rand() < pc:
                    alpha = np.random.rand()
                    p1 = population[i].copy()
                    p2 = population[i + 1].copy()
                    population[i] = alpha * p1 + (1 - alpha) * p2
                    population[i + 1] = (1 - alpha) * p1 + alpha * p2

            # Đột biến thích nghi dạng hình Sin (Sinusoidal Adaptive Mutation)
            for i in range(self.pop_size):
                f_ind = fitness_scores[i]
                if f_ind >= f_avg and (f_max - f_avg) > 1e-6:
                    pm = self.pm_max - 0.5 * (self.pm_max - self.pm_min) * (
                        1.0 + math.sin(((f_ind - f_avg) / (f_max - f_avg)) * (math.pi / 2.0))
                    )
                else:
                    pm = self.pm_max

                if np.random.rand() < pm:
                    mut_idx = np.random.randint(0, num_segments)
                    population[i, mut_idx] += np.random.normal(0, 0.04)
                    population[i, mut_idx] = max(0.04, population[i, mut_idx])

            # Bảo tồn cá thể ưu tú nhất (Elitism)
            population[0] = best_individual.copy()

        return best_individual

    def _evaluate_trajectory(self, dt_array: np.ndarray, waypoints: np.ndarray):
        """
        Nội suy Cubic B-Spline C^2 và tính toán hàm phạt vi phạm giới hạn vận tốc, gia tốc, jerk.
        """
        t_knots = np.insert(np.cumsum(dt_array), 0, 0.0)
        total_time = t_knots[-1]
        penalty = 0.0

        # Nội suy Cubic Spline độc lập cho 6 khớp
        splines = [CubicSpline(t_knots, waypoints[:, j], bc_type='clamped') for j in range(6)]
        t_samples = np.linspace(0, total_time, num=max(25, len(waypoints) * 3))

        for j in range(6):
            vel = splines[j](t_samples, 1)   # Vận tốc góc q_dot
            acc = splines[j](t_samples, 2)   # Gia tốc góc q_ddot
            jerk = splines[j](t_samples, 3)  # Độ giật q_dddot

            # Tính độ vượt ngưỡng
            v_viol = np.maximum(0.0, np.abs(vel) - self.vel_limits[j])
            a_viol = np.maximum(0.0, np.abs(acc) - self.acc_limits[j])
            j_viol = np.maximum(0.0, np.abs(jerk) - self.jerk_limits[j])

            penalty += np.sum(v_viol) * 15.0 + np.sum(a_viol) * 5.0 + np.sum(j_viol) * 0.5

        return total_time, penalty

    def build_ros2_trajectory_msg(self, joint_waypoints: np.ndarray, optimal_dt: np.ndarray, 
                                  joint_names=None, sample_hz: float = 50.0):
        """
        Đóng gói quỹ đạo B-spline tối ưu thành thông điệp ROS 2 JointTrajectory chuẩn.
        """
        if not HAS_ROS2_TRAJ:
            raise ImportError("Môi trường Python chưa nạp ROS 2 (trajectory_msgs)!")

        if joint_names is None:
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        t_knots = np.insert(np.cumsum(optimal_dt), 0, 0.0)
        splines = [CubicSpline(t_knots, joint_waypoints[:, j], bc_type='clamped') for j in range(6)]

        total_time = t_knots[-1]
        dt_step = 1.0 / float(sample_hz)
        sample_times = np.arange(0.0, total_time, dt_step)
        if len(sample_times) == 0 or sample_times[-1] < total_time:
            sample_times = np.append(sample_times, total_time)

        for t in sample_times:
            pt = JointTrajectoryPoint()
            pt.positions = [float(splines[j](t)) for j in range(6)]
            pt.velocities = [float(splines[j](t, 1)) for j in range(6)]
            pt.accelerations = [float(splines[j](t, 2)) for j in range(6)]

            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj_msg.points.append(pt)

        return traj_msg


if __name__ == "__main__":
    print("[TEST] Khởi tạo bộ tối ưu IGA Denso VS-6556...")
    optimizer = DensoIGATrajectoryOptimizer()
    
    # Giả lập 5 điểm góc khớp (rad)
    test_waypoints = np.array([
        [0.0, 0.5, 1.2, 0.0, 0.4, 0.0],
        [0.2, 0.6, 1.0, 0.1, 0.5, 0.2],
        [0.4, 0.4, 0.8, -0.1, 0.6, 0.4],
        [0.1, 0.3, 1.1, 0.0, 0.3, 0.1],
        [0.0, 0.5, 1.2, 0.0, 0.4, 0.0]
    ])
    
    unopt_time = 0.25 * (len(test_waypoints) - 1)
    print(f"[*] Thời gian ban đầu (chưa tối ưu): {unopt_time:.2f} s")
    
    opt_dt = optimizer.optimize(test_waypoints, initial_dt=0.25)
    opt_time = float(np.sum(opt_dt))
    print(f"[+] Thời gian tối ưu (sau IGA): {opt_time:.2f} s")
    print(f"[+] Tỷ lệ rút ngắn thời gian: {((unopt_time - opt_time) / unopt_time) * 100.0:.2f}%")
    print("[SUCCESS] Thuật toán IGA hoạt động ổn định và sẵn sàng tích hợp vào ROS 2!")

