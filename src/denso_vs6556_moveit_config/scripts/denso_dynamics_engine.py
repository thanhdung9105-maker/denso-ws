#!/usr/bin/env python3
"""
DENSO VS-6556 DYNAMICS ENGINE
Thuật toán tính toán Động Lực Học Nghịch (Inverse Dynamics - ID) và Động Lực Học Thuận (Forward Dynamics - FD)
kèm tạo visual marker 3D cho RViz2 tại các khớp của robot Denso VS-6556 (5-DOF).
"""

import math
import numpy as np

# -------------------------------------------------------------
# ROBOT CONSTANTS & DYNAMICS SPECIFICATIONS
# -------------------------------------------------------------
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
JOINT_LIMITS = [
    (-2.967, 2.967),   # Joint 1: Base Z [-170°, +170°]
    (-2.094, 2.356),   # Joint 2: Shoulder Y [-120°, +135°]
    (-2.077, 2.897),   # Joint 3: Elbow Y [-119°, +166°]
    (-3.316, 3.316),   # Joint 4: Forearm X [-190°, +190°]
    (-2.094, 2.094),   # Joint 5: Wrist Y [-120°, +120°]
]

# Torque limits (N*m) for Denso VS-6556 industrial motors
TORQUE_LIMITS = np.array([80.0, 100.0, 60.0, 25.0, 20.0])

# Friction parameters: viscous friction B (N*m*s/rad) and Coulomb friction Fc (N*m)
FRICTION_B = np.array([0.8, 1.2, 0.9, 0.3, 0.2])
FRICTION_FC = np.array([0.5, 0.8, 0.6, 0.2, 0.15])

# Link masses (kg)
LINK_MASSES = np.array([4.5, 4.2, 3.1, 2.0, 1.2])

# Centers of mass (m) in each link frame
COMS = [
    np.array([-0.04, 0.0, 0.08]),    # link_1
    np.array([0.0, 0.003, 0.135]),   # link_2
    np.array([-0.04, 0.02, 0.04]),   # link_3
    np.array([-0.10, 0.0, 0.0]),     # link_4
    np.array([-0.03, 0.0, 0.0]),     # link_5
]

# Inertia tensors (kg*m^2) at CoM
INERTIAS = [
    np.diag([0.035, 0.035, 0.025]),  # link_1
    np.diag([0.045, 0.040, 0.015]),  # link_2
    np.diag([0.020, 0.020, 0.010]),  # link_3
    np.diag([0.005, 0.015, 0.015]),  # link_4
    np.diag([0.003, 0.003, 0.003]),  # link_5
]

# Joint axes in local joint frames
AXES = [
    np.array([0.0, 0.0, 1.0]),  # J1: Z
    np.array([0.0, 1.0, 0.0]),  # J2: Y
    np.array([0.0, 1.0, 0.0]),  # J3: Y
    np.array([1.0, 0.0, 0.0]),  # J4: X
    np.array([0.0, 1.0, 0.0]),  # J5: Y
]

# Origins of joint i in frame i-1 (from URDF)
P_ORIGINS = [
    np.array([0.0, 0.0, 0.185]),          # Base -> Joint 1
    np.array([-0.075, -0.0545, 0.150]),   # Link 1 -> Joint 2
    np.array([0.0, 0.0055, 0.270]),       # Link 2 -> Joint 3
    np.array([-0.088, 0.049, 0.090]),     # Link 3 -> Joint 4
    np.array([-0.207, 0.0, 0.0]),         # Link 4 -> Joint 5
]

def rot_x(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

def rot_y(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

def rot_z(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

ROT_FUNCS = [rot_z, rot_y, rot_y, rot_x, rot_y]


class DensoDynamicsEngine:
    """High-performance RNEA and Forward Dynamics Engine for Denso VS-6556."""

    def __init__(self):
        self.gravity = np.array([0.0, 0.0, -9.81])
        self.num_joints = 5

    def get_joint_transforms(self, q):
        """
        Computes 4x4 homogenous transforms T_i for each joint in base frame.
        Returns list of 4x4 numpy arrays [T0, T1, T2, T3, T4, T5].
        """
        T = np.eye(4)
        transforms = [T.copy()]

        for i in range(self.num_joints):
            R_rot = ROT_FUNCS[i](q[i])
            T_step = np.eye(4)
            T_step[:3, 3] = P_ORIGINS[i]
            T_step[:3, :3] = R_rot
            T = T @ T_step
            transforms.append(T.copy())

        return transforms

    def rnea(self, q, qd, qdd, grav=None, payload_mass=0.0, f_ext=None, n_ext=None):
        """
        Recursive Newton-Euler Algorithm (RNEA) for Inverse Dynamics.
        Returns joint torque vector tau (5,).
        """
        if grav is None:
            grav = self.gravity

        n = self.num_joints
        R_rel = [ROT_FUNCS[i](q[i]) for i in range(n)]

        w = [np.zeros(3) for _ in range(n)]
        wd = [np.zeros(3) for _ in range(n)]
        vd = [np.zeros(3) for _ in range(n)]
        a_c = [np.zeros(3) for _ in range(n)]

        w_prev = np.zeros(3)
        wd_prev = np.zeros(3)
        vd_prev = -grav  # Base upward acceleration naturally accounts for gravity

        # Forward recursion: Kinematics and inertial forces
        for i in range(n):
            R_i = R_rel[i]
            z_i = AXES[i]
            p_i = P_ORIGINS[i]

            w[i] = R_i.T @ w_prev + z_i * qd[i]
            wd[i] = R_i.T @ wd_prev + z_i * qdd[i] + np.cross(w[i], z_i * qd[i])
            acc_origin_prev = vd_prev + np.cross(wd_prev, p_i) + np.cross(w_prev, np.cross(w_prev, p_i))
            vd[i] = R_i.T @ acc_origin_prev
            a_c[i] = vd[i] + np.cross(wd[i], COMS[i]) + np.cross(w[i], np.cross(w[i], COMS[i]))

            w_prev = w[i]
            wd_prev = wd[i]
            vd_prev = vd[i]

        # Backward recursion: Wrench propagation and joint torques
        f = [np.zeros(3) for _ in range(n)]
        n_mom = [np.zeros(3) for _ in range(n)]
        tau = np.zeros(n)

        f_next = np.zeros(3) if f_ext is None else np.array(f_ext, dtype=float)
        n_next = np.zeros(3) if n_ext is None else np.array(n_ext, dtype=float)

        for i in range(n - 1, -1, -1):
            m_i = LINK_MASSES[i]
            I_i = INERTIAS[i]
            if i == n - 1 and payload_mass > 0.0:
                m_i += payload_mass

            F_i = m_i * a_c[i]
            N_i = I_i @ wd[i] + np.cross(w[i], I_i @ w[i])

            if i == n - 1:
                f[i] = F_i + f_next
                n_mom[i] = N_i + n_next + np.cross(COMS[i], F_i)
            else:
                R_child = R_rel[i + 1]
                p_child = P_ORIGINS[i + 1]
                f_from_child = R_child @ f[i + 1]
                n_from_child = R_child @ n_mom[i + 1]
                f[i] = F_i + f_from_child
                n_mom[i] = N_i + n_from_child + np.cross(COMS[i], F_i) + np.cross(p_child, f_from_child)

            tau[i] = float(np.dot(n_mom[i], AXES[i]))

        return tau

    def compute_mass_matrix(self, q, payload_mass=0.0):
        """Computes 5x5 Mass/Inertia matrix M(q)."""
        M = np.zeros((self.num_joints, self.num_joints))
        zeros = np.zeros(self.num_joints)
        for j in range(self.num_joints):
            ej = np.zeros(self.num_joints)
            ej[j] = 1.0
            M[:, j] = self.rnea(q, zeros, ej, grav=np.zeros(3), payload_mass=payload_mass)
        return 0.5 * (M + M.T)

    def inverse_dynamics(self, q, qd, qdd, payload_mass=0.0, f_ext=None, include_friction=True):
        """
        Solves Inverse Dynamics: returns total torque and breakdown dictionary.
        tau = M(q)*qdd + C(q, qd)*qd + g(q) + tau_payload + tau_friction
        """
        q = np.array(q, dtype=float)
        qd = np.array(qd, dtype=float)
        qdd = np.array(qdd, dtype=float)

        zeros = np.zeros(self.num_joints)

        # 1. Gravity torque g(q) with no payload
        tau_g = self.rnea(q, zeros, zeros, grav=self.gravity, payload_mass=0.0)

        # 2. Coriolis & Centrifugal C(q, qd)*qd
        tau_c = self.rnea(q, qd, zeros, grav=np.zeros(3), payload_mass=0.0)

        # 3. Inertial torque M(q)*qdd
        tau_m = self.rnea(q, zeros, qdd, grav=np.zeros(3), payload_mass=0.0)

        # 4. Additional torque due to payload
        if payload_mass > 0.0 or f_ext is not None:
            tau_with_load = self.rnea(q, qd, qdd, grav=self.gravity, payload_mass=payload_mass, f_ext=f_ext)
            tau_no_load = self.rnea(q, qd, qdd, grav=self.gravity, payload_mass=0.0)
            tau_payload = tau_with_load - tau_no_load
        else:
            tau_payload = zeros

        # 5. Friction torque
        if include_friction:
            tau_friction = FRICTION_B * qd + FRICTION_FC * np.tanh(10.0 * qd)
        else:
            tau_friction = zeros

        total_tau = tau_m + tau_c + tau_g + tau_payload + tau_friction
        percent_load = np.abs(total_tau) / TORQUE_LIMITS * 100.0

        return {
            'tau': total_tau,
            'M_qdd': tau_m,
            'C_qd': tau_c,
            'g': tau_g,
            'payload': tau_payload,
            'friction': tau_friction,
            'percent_load': percent_load,
            'is_overload': np.any(percent_load > 100.0)
        }

    def forward_dynamics(self, q, qd, tau, payload_mass=0.0, f_ext=None, include_friction=True):
        """
        Solves Forward Dynamics: returns joint acceleration vector qdd (5,).
        qdd = M(q)^-1 * (tau - C(q, qd)*qd - g(q) - tau_payload - tau_friction)
        """
        q = np.array(q, dtype=float)
        qd = np.array(qd, dtype=float)
        tau = np.array(tau, dtype=float)
        zeros = np.zeros(self.num_joints)

        M = self.compute_mass_matrix(q, payload_mass=payload_mass)

        # h(q, qd) = C*qd + g + payload
        h = self.rnea(q, qd, zeros, grav=self.gravity, payload_mass=payload_mass, f_ext=f_ext)

        if include_friction:
            tau_friction = FRICTION_B * qd + FRICTION_FC * np.tanh(10.0 * qd)
        else:
            tau_friction = zeros

        tau_effective = tau - h - tau_friction
        qdd = np.linalg.solve(M, tau_effective)
        return qdd

    def gravity_compensation(self, q, payload_mass=0.0):
        """Returns static gravity holding torques (N*m)."""
        zeros = np.zeros(self.num_joints)
        return self.rnea(q, zeros, zeros, grav=self.gravity, payload_mass=payload_mass)

    def integrate_step(self, q, qd, tau, dt=0.05, payload_mass=0.0):
        """
        Semi-Implicit Euler integration step for real-time physics simulation.
        Includes soft boundary springs/dampers at joint limits.
        """
        q = np.array(q, dtype=float)
        qd = np.array(qd, dtype=float)
        tau = np.array(tau, dtype=float)

        # Add joint limit restoration spring/damper if near limits
        tau_limit_spring = np.zeros(self.num_joints)
        for i in range(self.num_joints):
            q_min, q_max = JOINT_LIMITS[i]
            margin = 0.05  # ~3 deg margin
            if q[i] < q_min + margin:
                dist = (q_min + margin) - q[i]
                tau_limit_spring[i] += 200.0 * dist - 20.0 * qd[i]
            elif q[i] > q_max - margin:
                dist = q[i] - (q_max - margin)
                tau_limit_spring[i] -= 200.0 * dist + 20.0 * qd[i]

        effective_tau = tau + tau_limit_spring
        qdd = self.forward_dynamics(q, qd, effective_tau, payload_mass=payload_mass)

        # Update velocity & position
        qd_new = qd + qdd * dt
        # Viscous drag damping to keep system stable
        qd_new *= math.exp(-0.4 * dt)
        q_new = q + qd_new * dt

        # Hard limit enforcement
        for i in range(self.num_joints):
            q_min, q_max = JOINT_LIMITS[i]
            if q_new[i] < q_min:
                q_new[i] = q_min
                qd_new[i] = 0.0
            elif q_new[i] > q_max:
                q_new[i] = q_max
                qd_new[i] = 0.0

        return q_new, qd_new, qdd

    def build_marker_array(self, q, tau, qdd=None, frame_id="world"):
        """
        Generates visualization_msgs/MarkerArray for RViz2 containing:
        - 5 Torque vector arrows located at the joints
        - 5 3D Text labels displaying tau (N*m), % load, and qdd
        """
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA

        marker_array = MarkerArray()
        transforms = self.get_joint_transforms(q)

        joint_names_display = ["J1 (Base)", "J2 (Vai)", "J3 (Khuỷu)", "J4 (Cẳng tay)", "J5 (Cổ tay)"]

        for i in range(self.num_joints):
            T_joint = transforms[i + 1]
            pos = T_joint[:3, 3]
            R_rot = T_joint[:3, :3]
            z_axis_world = R_rot @ AXES[i]

            t_val = float(tau[i])
            t_limit = float(TORQUE_LIMITS[i])
            load_pct = min(150.0, abs(t_val) / t_limit * 100.0)

            # Determine color based on load percentage
            # Green (<50%), Yellow/Orange (50-80%), Red (>80%)
            if load_pct < 50.0:
                color = ColorRGBA(r=0.0, g=0.95, b=0.4, a=0.9)  # Green
            elif load_pct < 80.0:
                color = ColorRGBA(r=1.0, g=0.75, b=0.1, a=0.95) # Yellow/Orange
            else:
                color = ColorRGBA(r=1.0, g=0.15, b=0.25, a=1.0) # Red warning

            # -------------------------------------------------------------
            # 1. Torque Arrow Marker
            # -------------------------------------------------------------
            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame_id
            arrow_marker.ns = "joint_torques"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD

            # Scale length proportional to load (0.05m to 0.25m)
            arrow_len = 0.06 + 0.18 * (load_pct / 100.0)
            sign = 1.0 if t_val >= 0 else -1.0
            p_start = pos
            p_end = pos + (sign * arrow_len) * z_axis_world

            p0 = Point(x=float(p_start[0]), y=float(p_start[1]), z=float(p_start[2]))
            p1 = Point(x=float(p_end[0]), y=float(p_end[1]), z=float(p_end[2]))
            arrow_marker.points = [p0, p1]

            arrow_marker.scale.x = 0.016  # Shaft diameter
            arrow_marker.scale.y = 0.035  # Head diameter
            arrow_marker.scale.z = 0.04   # Head length
            arrow_marker.color = color

            marker_array.markers.append(arrow_marker)

            # Explicitly delete old scattered per-joint text markers
            del_old = Marker()
            del_old.header.frame_id = frame_id
            del_old.ns = "joint_dynamics_labels"
            del_old.id = 100 + i
            del_old.action = Marker.DELETE
            marker_array.markers.append(del_old)

        # -------------------------------------------------------------
        # 2. UNIFIED DYNAMICS TELEMETRY TABLE (ONE CLEAN TABLE IN RVIZ2)
        # -------------------------------------------------------------
        joint_labels = ["J1", "J2", "J3", "J4", "J5"]
        table_lines = [
            "+-------------------------------------------------------------+",
            "|            DENSO VS-6556 JOINT DYNAMICS TELEMETRY           |",
            "+-------+-------------+-------------+-------------+-----------+",
            "| Joint | Torque (Nm) | Limit (Nm)  | Motor Load  | Accel     |",
            "+-------+-------------+-------------+-------------+-----------+",
        ]

        max_load = 0.0
        for i in range(self.num_joints):
            t_val = float(tau[i])
            t_lim = float(TORQUE_LIMITS[i])
            pct = min(150.0, abs(t_val) / t_lim * 100.0)
            if pct > max_load:
                max_load = pct

            tag = "[OK] " if pct < 50.0 else ("[WARN]" if pct < 80.0 else "[OVER]")
            acc_str = f"{qdd[i]:+4.1f} r/s2" if qdd is not None else " +0.0 r/s2"
            row = f"|  {joint_labels[i]}   |  {t_val:+6.1f} Nm |   {t_lim:5.1f} Nm   | {pct:4.1f}% {tag} | {acc_str:9s} |"
            table_lines.append(row)

        table_lines.append("+-------+-------------+-------------+-------------+-----------+")
        if max_load < 50.0:
            status_line = "| STATUS: NORMAL LOAD (<50% MOTOR CAPACITY)                   |"
            table_color = ColorRGBA(r=0.0, g=0.92, b=1.0, a=0.98)  # Cyan
        elif max_load < 80.0:
            status_line = "| STATUS: MODERATE LOAD (50% - 80% MOTOR CAPACITY)            |"
            table_color = ColorRGBA(r=1.0, g=0.82, b=0.15, a=0.98) # Yellow
        else:
            status_line = "| STATUS: ALERT - HIGH TORQUE (>80% MOTOR CAPACITY)           |"
            table_color = ColorRGBA(r=1.0, g=0.25, b=0.30, a=1.0)  # Red

        table_lines.append(status_line)
        table_lines.append("+-------------------------------------------------------------+")

        table_marker = Marker()
        table_marker.header.frame_id = frame_id
        table_marker.ns = "dynamics_dashboard"
        table_marker.id = 200
        table_marker.type = Marker.TEXT_VIEW_FACING
        table_marker.action = Marker.ADD

        # Position neatly beside the robot workspace in world coordinates
        table_marker.pose.position.x = -0.35
        table_marker.pose.position.y = 0.55
        table_marker.pose.position.z = 0.45

        table_marker.text = "\n".join(table_lines)
        table_marker.scale.z = 0.024  # Monospace text height
        table_marker.color = table_color

        marker_array.markers.append(table_marker)

        return marker_array


# -------------------------------------------------------------
# CLI SELF-TEST
# -------------------------------------------------------------
if __name__ == '__main__':
    print("=== DENSO VS-6556 DYNAMICS ENGINE TEST ===")
    engine = DensoDynamicsEngine()

    q_home = np.zeros(5)
    qd_zero = np.zeros(5)
    qdd_zero = np.zeros(5)

    print("\n1. Gravity Torques at Home (q=0):")
    res_home = engine.inverse_dynamics(q_home, qd_zero, qdd_zero)
    for i, name in enumerate(JOINT_NAMES):
        print(f"  {name}: {res_home['tau'][i]:+6.2f} N*m ({res_home['percent_load'][i]:4.1f}% of {TORQUE_LIMITS[i]} N*m)")

    print("\n2. Symmetric Mass Matrix M(q=0):")
    M = engine.compute_mass_matrix(q_home)
    print(np.round(M, 4))
    eigvals = np.linalg.eigvals(M)
    print("  Eigenvalues:", np.round(eigvals, 4))
    assert np.all(eigvals > 0), "Error: Mass matrix is not positive definite!"

    print("\n3. Consistency Check (FD of ID torque):")
    q_test = np.array([0.2, 0.4, -0.3, 0.5, -0.2])
    qd_test = np.array([0.1, -0.2, 0.3, -0.1, 0.2])
    qdd_desired = np.array([0.5, -0.8, 1.2, -0.4, 0.6])

    tau_id = engine.rnea(q_test, qd_test, qdd_desired)
    qdd_recovered = engine.forward_dynamics(q_test, qd_test, tau_id, include_friction=False)
    err = np.max(np.abs(qdd_desired - qdd_recovered))
    print(f"  Max Acceleration Error: {err:.2e}")
    assert err < 1e-6, "Error: Forward & Inverse Dynamics mismatch!"

    print("\nAll Dynamics Engine Tests PASSED successfully!")
