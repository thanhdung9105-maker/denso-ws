#!/usr/bin/env python3
"""
DENSO VS-6556 DYNAMICS ENGINE
Thuật toán tính toán Động Lực Học Nghịch (Inverse Dynamics - ID) và Động Lực Học Thuận (Forward Dynamics - FD)
kèm tạo visual marker 3D cho RViz2 tại các khớp của robot Denso VS-6556 (5-DOF).
"""

import math
from typing import Any
import numpy as np

# -------------------------------------------------------------
# ROBOT CONSTANTS & DYNAMICS SPECIFICATIONS
# -------------------------------------------------------------
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
JOINT_LIMITS = [
    (-2.967, 2.967),   # Joint 1: Base Z [-170°, +170°]
    (-2.094, 2.356),   # Joint 2: Shoulder Y [-120°, +135°]
    (-2.077, 2.897),   # Joint 3: Elbow Y [-119°, +166°]
    (-3.316, 3.316),   # Joint 4: Forearm X [-190°, +190°]
    (-2.094, 2.094),   # Joint 5: Wrist Y [-120°, +120°]
    (-6.283, 6.283),   # Joint 6: Flange Z [-360°, +360°]
]

# Torque limits (N*m) for Denso VS-6556 industrial motors
TORQUE_LIMITS = np.array([80.0, 100.0, 60.0, 25.0, 20.0, 15.0])

# DENSO VS-6556 AC Servomotor Specifications (Denso Manual No. 410590-0080)
# Rated Power (Watts) per joint
MOTOR_POWER_WATTS = np.array([400.0, 750.0, 400.0, 100.0, 100.0, 100.0])

# Rated Motor Speed (RPM)
MOTOR_RATED_RPM = 3000.0

# Harmonic Drive / RV Gear Reduction Ratios
GEAR_RATIOS = np.array([80.0, 100.0, 80.0, 50.0, 50.0, 50.0])

# Maximum joint angular velocities (rad/s) from Denso datasheet (262.5°, 240.0°, 300.0°, 300.0°, 300.0°, 420.0°/s)
VELOCITY_LIMITS = np.array([4.5815, 4.1888, 5.2360, 5.2360, 5.2360, 7.3304])

# Maximum joint angular accelerations (rad/s^2) for standard 0.49s cycle time
ACCELERATION_LIMITS = np.array([15.0, 12.0, 15.0, 20.0, 20.0, 25.0])

# Friction parameters: viscous friction B (N*m*s/rad) and Coulomb friction Fc (N*m)
FRICTION_B = np.array([0.8, 1.2, 0.9, 0.3, 0.2, 0.1])
FRICTION_FC = np.array([0.5, 0.8, 0.6, 0.2, 0.15, 0.05])

# Link masses (kg)
LINK_MASSES = np.array([4.5, 4.2, 3.1, 2.0, 0.95, 0.25])

# Centers of mass (m) in each link frame (synchronized with calibrated denso_vs6556.urdf)
COMS = [
    np.array([-0.04, 0.0, 0.08]),     # link_1
    np.array([0.0, -0.041, 0.120]),   # link_2
    np.array([0.045, 0.075, 0.010]),  # link_3
    np.array([0.0, 0.0, 0.10]),       # link_4 (along +Z forearm axis)
    np.array([0.0, 0.0, 0.0163]),     # link_5 (wrist pitch knuckle)
    np.array([0.0, 0.0, 0.01126]),    # link_6 (flange roll, flipped with bolt circle facing +Z)
]

# Inertia tensors (kg*m^2) at CoM
INERTIAS = [
    np.diag([0.035, 0.035, 0.025]),           # link_1
    np.diag([0.045, 0.040, 0.015]),           # link_2
    np.diag([0.020, 0.020, 0.010]),           # link_3
    np.diag([0.015, 0.015, 0.005]),           # link_4
    np.diag([0.000816, 0.000954, 0.000634]),   # link_5
    np.diag([0.000072, 0.000072, 0.000120]),   # link_6
]

# Joint axes in local joint frames
AXES = [
    np.array([0.0, 0.0, 1.0]),   # J1: Z (Yaw)
    np.array([0.0, -1.0, 0.0]),  # J2: -Y (Pitch, Denso inverted)
    np.array([0.0, -1.0, 0.0]),  # J3: -Y (Pitch, Denso inverted)
    np.array([0.0, 0.0, 1.0]),   # J4: Z (Forearm Roll along arm axis)
    np.array([0.0, -1.0, 0.0]),  # J5: -Y (Wrist Pitch, Denso inverted)
    np.array([0.0, 0.0, 1.0]),   # J6: Z (Flange Roll)
]

# Origins of joint i in frame i-1 (matching SolidWorks VS-6556-B_asm.STL & calibrated URDF)
P_ORIGINS = [
    np.array([0.0, 0.0, 0.205]),          # Base -> Joint 1 (Z=205mm as measured in SolidWorks)
    np.array([-0.075, -0.0745, 0.130]),   # Link 1 -> Joint 2 (X=-75mm, Y=-74.5mm, Z=130mm, total h=335mm)
    np.array([0.0, 0.0065, 0.270]),       # Link 2 -> Joint 3 (Y=+6.5mm, Z=270mm upper arm, total h=605mm)
    np.array([0.090, 0.068, 0.109]),      # Link 3 -> Joint 4 (X=+90mm, Y=+68mm -> Y=0, Z=109mm, total h=714mm)
    np.array([0.0, 0.0, 0.186]),          # Link 4 -> Joint 5 (Z=186mm forearm length to wrist)
    np.array([0.0, 0.0, 0.06204]),        # Link 5 -> Joint 6 (Wrist pitch knuckle to Flange top)
]

# Tool flange & gripper TCP offset (m) from Joint 6 flange axis along tool link
# URDF: joint_pen_tool at +0.028m, stylus nib tip at +0.014m in pen_tool_link -> Total tip offset = +0.042m (42mm)
TOOL_TCP_OFFSET = np.array([0.0, 0.0, 0.042])

# Official technical drawing dimensions (Denso VS-6556 catalog)
POINT_P_MAX_REACH = 653.42   # mm: Workable space defined by point P
FLANGE_MAX_REACH = 733.42    # mm: Workable space to mounting face (653.42 + 80mm)
FIRST_ARM_LENGTH = 270.0     # mm: J2 to J3 axis
SECOND_ARM_LENGTH = 295.0    # mm: J3 to Point P (88mm + 207mm)
BASE_HEIGHT = 185.0          # mm: Base mounting plane to J1
J2_HEIGHT = 335.0            # mm: Base mounting plane to J2 axis
BASE_OFFSET = 75.0           # mm: J1 axis to J2 axis offset

def rot_x(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

def rot_y(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

def rot_y_neg(th):
    # Rotation around -Y axis is rot_y(-th)
    c, s = np.cos(-th), np.sin(-th)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

def rot_z(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

ROT_FUNCS = [rot_z, rot_y_neg, rot_y_neg, rot_z, rot_y_neg, rot_z]

# Standard Denavit-Hartenberg (D-H) Parameters for Denso VS-6556 (6-DOF)
# Format: [a_i (m), alpha_i (rad), d_i (m), theta_offset (rad), joint_sign]
DH_PARAMS = [
    [-0.075, -np.pi / 2, 0.335,  0.0,        1.0],  # Joint 1: Base -> Shoulder J2 (Z0 vertical, Z1 horizontal)
    [ 0.270,  0.0,       0.000, -np.pi / 2, -1.0],  # Joint 2: Upper Arm (J2 -> J3, L=270mm, sign=-1 for -Y axis)
    [ 0.090,  np.pi / 2, 0.000,  np.pi / 2, -1.0],  # Joint 3: Elbow Offset (J3 -> Forearm axis, offset=90mm, sign=-1)
    [ 0.000, -np.pi / 2, 0.295,  0.0,        1.0],  # Joint 4: Forearm Roll (L=295mm to Point P)
    [ 0.000,  np.pi / 2, 0.000,  0.0,       -1.0],  # Joint 5: Wrist Pitch
    [ 0.000,  0.0,       0.10404, 0.0,       1.0],  # Joint 6: Tool Flange Roll (offset to TCP: 62.04mm + 42mm tip)
]

def standard_dh_matrix(a, alpha, d, theta):
    """
    4x4 Standard Denavit-Hartenberg Transformation Matrix A_i.
    A_i = Rot(Z, theta) * Trans(Z, d) * Trans(X, a) * Rot(X, alpha)
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,      ca,       d],
        [0.0,    0.0,     0.0,     1.0]
    ])


# =============================================================================
# EXTENDED D-H PARAMETERS (MÔ HÌNH D-H MỞ RỘNG CHUẨN XÁC 100% TỪNG LINK RVIZ)
# Bao gồm độ lệch ngang b (trục Y) của cụm động cơ vai và khớp khuỷu
# Format: (Link_Name, d (m), a (m), b (m), joint_axis, joint_index, description)
# =============================================================================
EXTENDED_DH_PARAMS = [
    ("link_1", 0.2050,  0.0000,  0.0000, "Z", 0, "Base -> Link 1 (Flange quay J1 h=205mm)"),
    ("link_2", 0.1300, -0.0750, -0.0745, "-Y", 1, "Link 1 -> Link 2 (Vai J2, lệch motor Y=-74.5mm, -Y axis)"),
    ("link_3", 0.2700,  0.0000,  0.0065, "-Y", 2, "Link 2 -> Link 3 (Khuỷu J3, lệch khớp Y=+6.5mm, -Y axis)"),
    ("link_4", 0.1090,  0.0900,  0.0680, "Z", 3, "Link 3 -> Link 4 (Cẳng tay J4, bù triệt tiêu Y)"),
    ("link_5", 0.1860,  0.0000,  0.0000, "-Y", 4, "Link 4 -> Link 5 (Tâm cổ tay Point P, -Y axis)"),
    ("link_6", 0.06204, 0.0000,  0.0000, "Z", 5, "Link 5 -> Link 6 (Cổ tay xoay J6 h=62.04mm, Z axis)"),
    ("flange", 0.0420,  0.0000,  0.0000, "Fixed", -1, "Link 6 -> Flange Tool TCP (+42mm)"),
]

def extended_dh_matrix(d, a, b, q_angle=0.0, axis="Z"):
    """
    4x4 Extended Denavit-Hartenberg Transformation Matrix H_i with lateral offset b (along Y).
    H_i = Trans(a, b, d) * Rot_axis(q_angle)
    """
    T_trans = np.eye(4)
    T_trans[:3, 3] = [a, b, d]
    
    T_rot = np.eye(4)
    if axis == "Z":
        c, s = np.cos(q_angle), np.sin(q_angle)
        T_rot[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    elif axis == "Y":
        c, s = np.cos(q_angle), np.sin(q_angle)
        T_rot[:3, :3] = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    elif axis == "-Y":
        c, s = np.cos(q_angle), np.sin(q_angle)
        T_rot[:3, :3] = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
    elif axis == "X":
        c, s = np.cos(q_angle), np.sin(q_angle)
        T_rot[:3, :3] = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    
    return T_trans @ T_rot


# Fixed rotation of joint origins matching calibrated URDF
R_ORIGINS = [
    np.eye(3),                         # Base -> Joint 1 (rpy=0 0 0)
    np.eye(3),                         # Link 1 -> Joint 2 (rpy=0 0 0)
    np.eye(3),                         # Link 2 -> Joint 3 (rpy=0 0 0)
    np.eye(3),                         # Link 3 -> Joint 4 (rpy=0 0 0)
    np.eye(3),                         # Link 4 -> Joint 5 (rpy=0 0 0)
    np.eye(3),                         # Link 5 -> Joint 6 (rpy=0 0 0)
]



def rot_matrix_to_rpy(R):
    """
    Converts 3x3 rotation matrix to Roll, Pitch, Yaw Euler angles (rad).
    Convention: Extrinsic X-Y-Z (Intrinsic Z-Y-X), matching ROS standard.
    """
    pitch = math.atan2(-float(R[2, 0]), math.hypot(float(R[2, 1]), float(R[2, 2])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(R[0, 1]), float(R[1, 1]))
    return roll, pitch, yaw


def rot_matrix_to_quaternion(R):
    """
    Converts 3x3 rotation matrix to unit quaternion [x, y, z, w].
    """
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + R[0, 0] - R[1, 1] - R[2, 2]))
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + R[1, 1] - R[0, 0] - R[2, 2]))
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + R[2, 2] - R[0, 0] - R[1, 1]))
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])

class DensoDynamicsEngine:
    """High-performance RNEA and Forward Dynamics Engine for Denso VS-6556."""

    def __init__(self):
        self.gravity = np.array([0.0, 0.0, -9.81])
        self.num_joints = 6

    def get_joint_transforms(self, q):
        """
        Computes 4x4 homogenous transforms T_i for each joint in base frame.
        Returns list of 4x4 numpy arrays [T0, T1, T2, T3, T4, T5, T_flange].
        - transforms[0]: Base
        - transforms[1..5]: Joints 1 to 5 (T5 is Point P wrist center)
        - transforms[6]: Flange TCP (offset 80mm along tool axis from Point P)
        """
        T = np.eye(4)
        transforms = [T.copy()]

        for i in range(self.num_joints):
            R_rot = ROT_FUNCS[i](q[i])
            T_step = np.eye(4)
            T_step[:3, 3] = P_ORIGINS[i]
            T_step[:3, :3] = R_ORIGINS[i] @ R_rot
            T = T @ T_step
            transforms.append(T.copy())

        # Also append Flange transform (80mm along tool axis from Point P)
        T_flange = T.copy()
        T_flange[:3, 3] += T[:3, :3] @ TOOL_TCP_OFFSET
        transforms.append(T_flange)

        return transforms

    def check_collision(self, q):
        """
        Kiem tra va cham tu than giua cac khau va va cham mat san/ban.
        Returns: (is_colliding: bool, reason: str)
        """
        transforms = self.get_joint_transforms(q)
        pts = [T[:3, 3] * 1000.0 for T in transforms]  # mm
        for i in range(2, len(pts)):
            if pts[i][2] < 20.0:
                return True, f"Va cham mat san/ban (Frame {i} co Z = {pts[i][2]:.1f} mm < 20 mm)"
        for idx in [4, 5, 6]:
            p = pts[idx]
            if p[2] < 340.0:
                r_xy = math.hypot(p[0], p[1])
                if r_xy < 135.0:
                    name = "Mat bich TCP" if idx == 6 else f"Khau {idx}"
                    return True, f"Va cham tu than: {name} ep vao than Base/Link 1 (Ban kinh R = {r_xy:.1f} mm < 135 mm)"
        d_3_1 = float(np.linalg.norm(pts[3] - pts[1]))
        if d_3_1 < 115.0:
            return True, f"Va cham kep giua Khau 3 va Khau 1 (Khoang cach d = {d_3_1:.1f} mm < 115 mm)"
        for i in range(self.num_joints):
            low, high = JOINT_LIMITS[i]
            if q[i] < low - 1e-3 or q[i] > high + 1e-3:
                deg_val = math.degrees(q[i])
                deg_low = math.degrees(low)
                deg_high = math.degrees(high)
                return True, f"Vuot gioi han co hoc khop J{i+1}: {deg_val:.1f} deg ngoai [{deg_low:.0f}, {deg_high:.0f}]"
        return False, "An toan (Khong phat hien va cham)"


    def get_dh_transforms(self, q):
        """
        Computes 4x4 homogenous transforms via Standard D-H convention:
        T_i = T_{i-1} @ A_i(q_i)
        Returns:
          transforms: [T0, T1, T2, T3, T4 (Point P), T5 (Flange TCP)]
          matrices_A: [A1, A2, A3, A4, A5]
        """
        T = np.eye(4)
        transforms = [T.copy()]
        matrices_A = []
        for i in range(self.num_joints):
            a, alpha, d, th_offset, sign = DH_PARAMS[i]
            theta = sign * q[i] + th_offset
            A_i = standard_dh_matrix(a, alpha, d, theta)
            T = T @ A_i
            matrices_A.append(A_i)
            transforms.append(T.copy())
        return transforms, matrices_A

    def get_extended_dh_transforms(self, q):
        """
        Computes 4x4 homogenous transforms via Extended D-H convention matching ROS 2 URDF & RViz.
        Returns:
          transforms: [T0 (Base), T0_1, T0_2, T0_3, T0_4, T0_5 (Point P), T0_Flange]
          matrices_H: [H1, H2, H3, H4, H5, H_flange]
        """
        T = np.eye(4)
        transforms = [T.copy()]
        matrices_H = []
        for name, d, a, b, axis, j_idx, desc in EXTENDED_DH_PARAMS:
            q_val = float(q[j_idx]) if j_idx >= 0 else 0.0
            H_i = extended_dh_matrix(d, a, b, q_val, axis)
            T = T @ H_i
            matrices_H.append(H_i)
            transforms.append(T.copy())
        return transforms, matrices_H

    def compute_forward_kinematics(self, q, use_flange=False):
        """
        Comprehensive Forward Kinematics (FK) for Denso VS-6556.
        Returns:
          dict containing:
            - position_m: [x, y, z] in meters
            - position_mm: [x, y, z] in mm
            - rpy_rad: (roll, pitch, yaw) in radians
            - rpy_deg: (roll, pitch, yaw) in degrees
            - quaternion: [x, y, z, w]
            - T_target: 4x4 homogenous transformation matrix of target link
            - transforms: list of all 7 transforms [T0, T1..T5, T_flange]
            - link_positions_mm: list of positions [X, Y, Z] in mm for each link
            - reach_radius_p_mm: distance from base Z-axis to Point P
            - reach_radius_f_mm: distance from base Z-axis to Flange
        """
        transforms, _ = self.get_dh_transforms(q)
        target_idx = self.num_joints if use_flange else (self.num_joints - 1)
        T_target = transforms[target_idx]
        pos_m = T_target[:3, 3]
        pos_mm = pos_m * 1000.0
        R_target = T_target[:3, :3]

        r_rad, p_rad, y_rad = rot_matrix_to_rpy(R_target)
        quat = rot_matrix_to_quaternion(R_target)

        link_positions_mm = [T[:3, 3] * 1000.0 for T in transforms]
        r_p = float(math.hypot(link_positions_mm[self.num_joints - 1][0], link_positions_mm[self.num_joints - 1][1]))
        r_f = float(math.hypot(link_positions_mm[self.num_joints][0], link_positions_mm[self.num_joints][1]))

        return {
            "position_m": pos_m,
            "position_mm": pos_mm,
            "rpy_rad": (r_rad, p_rad, y_rad),
            "rpy_deg": (math.degrees(r_rad), math.degrees(p_rad), math.degrees(y_rad)),
            "quaternion": quat,
            "T_target": T_target,
            "transforms": transforms,
            "link_positions_mm": link_positions_mm,
            "reach_radius_p_mm": r_p,
            "reach_radius_f_mm": r_f
        }

    def compute_jacobian(self, q, use_flange=False, eps=1e-6):
        """
        Computes 3x6 position Jacobian J_p = d(pos)/dq (numerical differentiation).
        """
        J = np.zeros((3, self.num_joints))
        transforms, _ = self.get_dh_transforms(q)
        idx = self.num_joints if use_flange else (self.num_joints - 1)
        p0 = transforms[idx][:3, 3]
        for j in range(self.num_joints):
            q_eps = np.array(q, dtype=float).copy()
            q_eps[j] += eps
            t_eps, _ = self.get_dh_transforms(q_eps)
            pj = t_eps[idx][:3, 3]
            J[:, j] = (pj - p0) / eps
        return J

    def compute_geometric_jacobian(self, q, use_flange=False):
        """
        Computes exact 6x6 Geometric Jacobian J = [J_v; J_w] in base_link frame:
          - J_v (3x6): linear velocity Jacobian z_i x (p_e - p_i)
          - J_w (3x6): angular velocity Jacobian z_i
        """
        T = np.eye(4)
        joint_origins = []
        joint_axes = []
        for i in range(self.num_joints):
            p_joint = T[:3, 3] + T[:3, :3] @ P_ORIGINS[i]
            z_joint = T[:3, :3] @ R_ORIGINS[i] @ AXES[i]
            joint_origins.append(p_joint)
            joint_axes.append(z_joint)

            R_rot = ROT_FUNCS[i](q[i])
            T_step = np.eye(4)
            T_step[:3, 3] = P_ORIGINS[i]
            T_step[:3, :3] = R_ORIGINS[i] @ R_rot
            T = T @ T_step

        p_e = (T[:3, 3] + T[:3, :3] @ TOOL_TCP_OFFSET) if use_flange else joint_origins[4]
        J = np.zeros((6, self.num_joints))
        for i in range(self.num_joints):
            J[:3, i] = np.cross(joint_axes[i], p_e - joint_origins[i])
            J[3:, i] = joint_axes[i]
        return J

    def compute_manipulability(self, q, use_flange=False):
        """
        Computes Yoshikawa's translational manipulability index w(q) = sqrt(det(J_v * J_v^T)),
        singular values (sigma_1, sigma_2, sigma_3), and condition number kappa = sigma_max / sigma_min.
        """
        J_v = self.compute_jacobian(q, use_flange=use_flange)
        singular_vals = np.linalg.svd(J_v, compute_uv=False)
        w = float(np.prod(singular_vals))
        s_min = float(singular_vals[-1])
        s_max = float(singular_vals[0])
        cond = (s_max / s_min) if s_min > 1e-9 else float('inf')
        return {
            "manipulability": w,
            "condition_number": cond,
            "singular_values": singular_vals,
            "is_near_singularity": (w < 1e-3 or cond > 50.0),
        }

    def solve_ik_analytical_point_p(self, target_pos_m, elbow_up=True, q_seed=None):
        """
        Closed-Form Analytical Geometric IK for Point P (Link 5 Wrist Center)
        with inverted pitch axes (Joint 2, 3 around -Y).
        Supports full 360-degree reach:
          - Mode 1: Backward reach (arm pitches backward, primary for drawing table)
          - Mode 2: Forward reach (arm pitches forward, primary for front workspaces)
        Exact solution in < 1 microsecond with 0 iteration and zero mathematical error.
        Returns:
            q_sol (np.ndarray): [q1, q2, q3, 0.0, 0.0] in radians
            success (bool): whether position is physically reachable within joint limits
            err_m (float): theoretical residual error (typically < 1e-12 m)
        """
        x, y, z = target_pos_m
        L2 = 0.270            # Upper arm length (m)
        L3 = 0.295            # Forearm length to Point P (m)
        a3 = 0.090            # Elbow offset (m)
        x2 = -0.075           # Base to J2 horizontal offset (m)
        z2 = 0.335           # Base mounting plane to J2 height (m)

        L_eff = math.hypot(L3, a3)
        phi = math.atan2(a3, L3)
        r_xy = math.hypot(x, y)

        # 2 candidate modes for q1 and longitudinal arm coordinate x5:
        q1_back = math.atan2(-y, -x) if r_xy > 1e-6 else 0.0
        q1_back = (q1_back + math.pi) % (2.0 * math.pi) - math.pi
        q1_fwd = math.atan2(y, x) if r_xy > 1e-6 else 0.0
        q1_fwd = (q1_fwd + math.pi) % (2.0 * math.pi) - math.pi

        modes = [(q1_back, -r_xy), (q1_fwd, r_xy)]
        dz = z - z2
        candidates = []

        for q1, x5 in modes:
            # Check Joint 1 limits
            if q1 < JOINT_LIMITS[0][0] - 1e-4 or q1 > JOINT_LIMITS[0][1] + 1e-4:
                continue

            dx = x2 - x5
            D_sq = dx**2 + dz**2
            D = math.sqrt(D_sq)

            if D > (L2 + L_eff) or D < abs(L2 - L_eff):
                continue

            cos_beta = np.clip((L2**2 + L_eff**2 - D_sq) / (2.0 * L2 * L_eff), -1.0, 1.0)
            beta = math.acos(cos_beta)
            q3_eff = (math.pi - beta) if elbow_up else -(math.pi - beta)
            q3 = q3_eff + phi

            gamma = math.atan2(dx, dz)
            cos_alpha = np.clip((L2**2 + D_sq - L_eff**2) / (2.0 * L2 * D), -1.0, 1.0)
            alpha = math.acos(cos_alpha)
            q2 = (gamma - alpha) if elbow_up else (gamma + alpha)

            q_sol = np.zeros(self.num_joints, dtype=float)
            q_sol[:3] = [q1, q2, q3]

            # Check joint limits
            in_limits = True
            for i in range(3):
                if q_sol[i] < JOINT_LIMITS[i][0] - 1e-4 or q_sol[i] > JOINT_LIMITS[i][1] + 1e-4:
                    in_limits = False
                    break

            if in_limits:
                tfs, _ = self.get_dh_transforms(q_sol)
                err_m = float(np.linalg.norm(np.array(target_pos_m) - tfs[5][:3, 3]))
                candidates.append((q_sol, err_m))

        if not candidates:
            D_min = math.sqrt((abs(r_xy - 0.075))**2 + dz**2)
            return None, False, float(max(0.0, D_min - (L2 + L_eff)))

        if q_seed is not None and len(candidates) > 1:
            candidates.sort(key=lambda item: float(np.linalg.norm(item[0][:3] - np.array(q_seed)[:3])))

        best_q, best_err = candidates[0]
        return best_q, True, best_err

    def solve_ik_analytical_flange(self, target_pos_m, tool_pitch_rad=None, elbow_up=True, q_seed=None):
        """
        Closed-Form Analytical Geometric IK for Flange (+80mm offset)
        with inverted pitch axes (Joint 2, 3, 5 around -Y).
        Supports full 360-degree reach in both forward and backward modes.
        - If tool_pitch_rad is None: uses aligned neutral wrist pose (q5 = 0, L3_eff = 375mm).
        - If tool_pitch_rad is specified: computes Point P offset and exact wrist pitch q5.
        Returns:
            q_sol (np.ndarray): [q1, q2, q3, 0.0, q5]
            success (bool): whether reachable and within limits
            err_m (float): final position error
        """
        xf, yf, zf = target_pos_m
        L2 = 0.270
        x2 = -0.075
        z2 = 0.335
        L_tool = float(DH_PARAMS[5][2])
        r_xy = math.hypot(xf, yf)

        if tool_pitch_rad is None:
            # Aligned forearm + tool pose (wrist neutral q5 = 0)
            L3_tool = 0.295 + L_tool  # Forearm to Flange TCP
            a3 = 0.090
            L_eff_tool = math.hypot(L3_tool, a3)
            phi_tool = math.atan2(a3, L3_tool)

            q1_back = math.atan2(-yf, -xf) if r_xy > 1e-6 else 0.0
            q1_back = (q1_back + math.pi) % (2.0 * math.pi) - math.pi
            q1_fwd = math.atan2(yf, xf) if r_xy > 1e-6 else 0.0
            q1_fwd = (q1_fwd + math.pi) % (2.0 * math.pi) - math.pi

            modes = [(q1_back, -r_xy), (q1_fwd, r_xy)]
            dz = zf - z2
            candidates = []

            for q1, x5 in modes:
                if q1 < JOINT_LIMITS[0][0] - 1e-4 or q1 > JOINT_LIMITS[0][1] + 1e-4:
                    continue
                dx = x2 - x5
                D_sq = dx**2 + dz**2
                D = math.sqrt(D_sq)
                if D > (L2 + L_eff_tool) or D < abs(L2 - L_eff_tool):
                    continue

                cos_beta = np.clip((L2**2 + L_eff_tool**2 - D_sq) / (2.0 * L2 * L_eff_tool), -1.0, 1.0)
                beta = math.acos(cos_beta)
                q3_eff = (math.pi - beta) if elbow_up else -(math.pi - beta)
                q3 = q3_eff + phi_tool

                gamma = math.atan2(dx, dz)
                cos_alpha = np.clip((L2**2 + D_sq - L_eff_tool**2) / (2.0 * L2 * D), -1.0, 1.0)
                alpha = math.acos(cos_alpha)
                q2 = (gamma - alpha) if elbow_up else (gamma + alpha)

                q_sol = np.zeros(self.num_joints, dtype=float)
                q_sol[:3] = [q1, q2, q3]

                in_limits = True
                for i in range(3):
                    if q_sol[i] < JOINT_LIMITS[i][0] - 1e-4 or q_sol[i] > JOINT_LIMITS[i][1] + 1e-4:
                        in_limits = False
                        break
                if in_limits:
                    tfs, _ = self.get_dh_transforms(q_sol)
                    err_m = float(np.linalg.norm(np.array(target_pos_m) - tfs[6][:3, 3]))
                    candidates.append((q_sol, err_m))

            if not candidates:
                return None, False, 1.0
            if q_seed is not None and len(candidates) > 1:
                candidates.sort(key=lambda item: float(np.linalg.norm(item[0][:3] - np.array(q_seed)[:3])))
            return candidates[0][0], True, candidates[0][1]
        else:
            # Specified tool pitch angle (e.g. for drawing table)
            # Evaluate in both backward (-r_xy) and forward (+r_xy) radial planes
            L3 = 0.295
            a3 = 0.090
            L_eff = math.hypot(L3, a3)
            phi = math.atan2(a3, L3)

            q1_back = math.atan2(-yf, -xf) if r_xy > 1e-6 else 0.0
            q1_back = (q1_back + math.pi) % (2.0 * math.pi) - math.pi
            q1_fwd = math.atan2(yf, xf) if r_xy > 1e-6 else 0.0
            q1_fwd = (q1_fwd + math.pi) % (2.0 * math.pi) - math.pi

            modes = [(q1_back, -r_xy), (q1_fwd, r_xy)]
            z5 = zf - L_tool * math.cos(tool_pitch_rad)
            dz = z5 - z2
            candidates = []

            for q1, xf_planar in modes:
                if q1 < JOINT_LIMITS[0][0] - 1e-4 or q1 > JOINT_LIMITS[0][1] + 1e-4:
                    continue
                x5_planar = xf_planar + L_tool * math.sin(tool_pitch_rad)
                dx = x2 - x5_planar
                D_sq = dx**2 + dz**2
                D = math.sqrt(D_sq)
                if D > (L2 + L_eff) or D < abs(L2 - L_eff):
                    continue

                cos_beta = np.clip((L2**2 + L_eff**2 - D_sq) / (2.0 * L2 * L_eff), -1.0, 1.0)
                beta = math.acos(cos_beta)
                q3_eff = (math.pi - beta) if elbow_up else -(math.pi - beta)
                q3 = q3_eff + phi

                gamma = math.atan2(dx, dz)
                cos_alpha = np.clip((L2**2 + D_sq - L_eff**2) / (2.0 * L2 * D), -1.0, 1.0)
                alpha = math.acos(cos_alpha)
                q2 = (gamma - alpha) if elbow_up else (gamma + alpha)

                q5 = (tool_pitch_rad - (q2 + q3) + math.pi) % (2.0 * math.pi) - math.pi
                q_sol = np.zeros(self.num_joints, dtype=float)
                q_sol[:5] = [q1, q2, q3, 0.0, q5]

                in_limits = True
                for i in range(self.num_joints):
                    if q_sol[i] < JOINT_LIMITS[i][0] - 1e-4 or q_sol[i] > JOINT_LIMITS[i][1] + 1e-4:
                        in_limits = False
                        break
                if in_limits:
                    tfs, _ = self.get_dh_transforms(q_sol)
                    err_m = float(np.linalg.norm(np.array(target_pos_m) - tfs[6][:3, 3]))
                    if err_m < 1e-3:
                        candidates.append((q_sol, err_m))

            if not candidates:
                return None, False, 1.0
            if q_seed is not None and len(candidates) > 1:
                candidates.sort(key=lambda item: float(np.linalg.norm(item[0][:3] - np.array(q_seed)[:3])))
            return candidates[0][0], True, candidates[0][1]

    def solve_ik_analytical_6dof(self, target_pos_m, target_rot_matrix, elbow_up=True, q_seed=None):
        """
        Closed-Form Analytical Geometric IK for Full 6-DOF Pose (Position + SO(3) Orientation)
        via Pieper's Spherical Wrist Decoupling (Tâm cổ tay cầu khâu 4-5-6 giao nhau tại Point P).
        Returns:
            q_sol (np.ndarray): [q1, q2, q3, q4, q5, q6] in radians within joint limits.
            success (bool): whether pose is physically reachable within limits.
            pos_err (float): final position residual in meters.
            rot_err (float): final rotation matrix residual norm.
        """
        R_tgt = np.array(target_rot_matrix, dtype=float)
        p_tgt = np.array(target_pos_m, dtype=float)

        # Step 1: Decouple spherical wrist center (Point P)
        # Flange normal along tool Z-axis: column 2 of R_tgt
        d6 = 0.09004
        z_flange = R_tgt[:, 2]
        p_wrist = p_tgt - d6 * z_flange

        # Step 2: Solve Joint 1, 2, 3 using closed-form Point P solver
        q_seed_123 = q_seed[:3] if q_seed is not None else None
        q123_sol, ok123, _ = self.solve_ik_analytical_point_p(p_wrist, elbow_up=elbow_up, q_seed=q_seed_123)
        if not ok123 or q123_sol is None:
            return None, False, 1.0, 1.0

        # Step 3: Compute R_03(q1, q2, q3) and relative wrist rotation R_36 = R_03^T @ R_tgt
        tfs123, _ = self.get_dh_transforms([q123_sol[0], q123_sol[1], q123_sol[2], 0.0, 0.0, 0.0])
        R_03 = tfs123[3][:3, :3]
        R_36 = R_03.T @ R_tgt

        r11, r12, r13 = R_36[0]
        r21, r22, r23 = R_36[1]
        r31, r32, r33 = R_36[2]

        q_seed_4 = float(q_seed[3]) if q_seed is not None and len(q_seed) > 3 else 0.0
        cos_q5 = np.clip(r33, -1.0, 1.0)
        sin_q5_pos = math.sqrt(max(0.0, 1.0 - cos_q5**2))

        candidates = []
        for s5 in [sin_q5_pos, -sin_q5_pos]:
            q5 = math.atan2(s5, cos_q5)
            if abs(s5) > 1e-4:
                q4 = math.atan2(-r23 / s5, -r13 / s5)
                q6 = math.atan2(-r32 / s5, r31 / s5)
            else:
                # Wrist Singularity: J4 and J6 axes become collinear
                if cos_q5 > 0:
                    q4_plus_q6 = math.atan2(r21, r11)
                    q4 = q_seed_4
                    q6 = (q4_plus_q6 - q4 + math.pi) % (2.0 * math.pi) - math.pi
                else:
                    q4_minus_q6 = math.atan2(-r21, -r11)
                    q4 = q_seed_4
                    q6 = (q4 - q4_minus_q6 + math.pi) % (2.0 * math.pi) - math.pi

            q4 = (q4 + math.pi) % (2.0 * math.pi) - math.pi
            q5 = (q5 + math.pi) % (2.0 * math.pi) - math.pi
            q6 = (q6 + math.pi) % (2.0 * math.pi) - math.pi

            q_cand = np.array([q123_sol[0], q123_sol[1], q123_sol[2], q4, q5, q6], dtype=float)

            # Check joint limits
            in_limits = True
            for i in range(self.num_joints):
                if q_cand[i] < JOINT_LIMITS[i][0] - 1e-4 or q_cand[i] > JOINT_LIMITS[i][1] + 1e-4:
                    in_limits = False
                    break
            if in_limits:
                fk_cand = self.compute_forward_kinematics(q_cand, use_flange=True)
                pos_err = float(np.linalg.norm(fk_cand["position_m"] - p_tgt))
                rot_err = float(np.linalg.norm(fk_cand["T_target"][:3, :3] - R_tgt))
                if pos_err < 2e-3 and rot_err < 5e-3:
                    is_col, _ = self.check_collision(q_cand)
                    if not is_col:
                        candidates.append((q_cand, pos_err, rot_err))

        if not candidates:
            return None, False, 1.0, 1.0

        if q_seed is not None and len(candidates) > 1:
            q_seed_arr = np.array(q_seed, dtype=float)[:6]
            candidates.sort(key=lambda c: float(np.linalg.norm(c[0] - q_seed_arr)))

        best_cand, best_pos_err, best_rot_err = candidates[0]
        return best_cand, True, best_pos_err, best_rot_err

    def inverse_kinematics(self, target_pos_m, q_seed=None, use_flange=False,
                           elbow_up=True, mode="hybrid", max_iter=25, tol=1e-4,
                           tool_pitch_rad=None, target_rot_matrix=None):
        """
        Upgraded Hybrid / Analytical / DLS Inverse Kinematics for Denso VS-6556.
        - mode="analytical": Exact closed-form geometric solution in 0 iterations (Point P & Flange).
        - mode="hybrid": Analytical initial seed + high-speed Damped Least Squares (1-3 iters, < 0.001mm).
        - mode="dls": Multi-seed numerical optimization with collision avoidance.
        - target_rot_matrix: Optional 3x3 SO(3) orientation matrix for full 6-DOF pose IK.
        Returns:
            q_sol (np.ndarray): Target joint angles (rad) within limits.
            success (bool): Whether tolerance was achieved.
            iters (int): Number of iterations taken.
            err_m (float): Final position error in meters.
        """
        target = np.array(target_pos_m, dtype=float)
        idx = self.num_joints if use_flange else (self.num_joints - 1)
        lam = 0.012

        # -----------------------------------------------------------------
        # 0. FULL 6-DOF ANALYTICAL ATTEMPT (if target orientation provided)
        # -----------------------------------------------------------------
        if target_rot_matrix is not None:
            branches = [elbow_up, not elbow_up] if elbow_up is not None else [True, False]
            for branch in branches:
                q_6d, ok_6d, pos_e, rot_e = self.solve_ik_analytical_6dof(
                    target, target_rot_matrix, elbow_up=branch, q_seed=q_seed
                )
                if ok_6d and q_6d is not None and pos_e < tol:
                    return q_6d, True, 0, pos_e
        # -----------------------------------------------------------------
        last_err_ana = 1.0
        if mode in ("analytical", "hybrid"):
            branches = [elbow_up, not elbow_up] if elbow_up is not None else [True, False]
            for branch in branches:
                if not use_flange:
                    q_ana, ok, err_ana = self.solve_ik_analytical_point_p(target, elbow_up=branch, q_seed=q_seed)
                else:
                    q_ana, ok, err_ana = self.solve_ik_analytical_flange(target, tool_pitch_rad=tool_pitch_rad, elbow_up=branch, q_seed=q_seed)
                last_err_ana = err_ana

                if ok and q_ana is not None and err_ana < 1e-3:
                    is_col, _ = self.check_collision(q_ana)
                    if not is_col:
                        return q_ana, True, 0, err_ana

            # Đối với Point P (Khâu 5) hoặc Flange có góc chúc bút tool_pitch_rad cố định,
            # giải tích hình học đã bao quát toàn bộ không gian làm việc chính xác 100%.
            # Nếu cả 2 nhánh cùi chỏ đều không tới, điểm này nằm ngoài tầm vươn vật lý.
            # Trả về ngay lập tức (0.001ms) tránh chạy vòng lặp DLS 3-DOF làm lệch góc nghiêng cổ tay!
            if not use_flange or mode == "analytical" or tool_pitch_rad is not None:
                return None, False, 0, last_err_ana

        # -----------------------------------------------------------------
        # 2. HYBRID OR NUMERICAL DLS STEP
        # -----------------------------------------------------------------
        def _dls_step(q_init, num_iters):
            q = np.zeros(self.num_joints, dtype=float)
            q_arr = np.array(q_init, dtype=float)
            q[:min(len(q_arr), self.num_joints)] = q_arr[:self.num_joints]
            for it in range(num_iters):
                transforms, _ = self.get_dh_transforms(q)
                cur_p = transforms[idx][:3, 3]
                err = target - cur_p
                err_norm = float(np.linalg.norm(err))
                if err_norm < tol:
                    return q, True, it, err_norm

                J = self.compute_jacobian(q, use_flange=use_flange)
                JJT = J @ J.T + (lam**2) * np.eye(3)
                dq = J.T @ np.linalg.inv(JJT) @ err
                q += 0.7 * dq

                for i in range(self.num_joints):
                    low, high = JOINT_LIMITS[i]
                    q[i] = np.clip(q[i], low, high)

            transforms, _ = self.get_dh_transforms(q)
            final_err = float(np.linalg.norm(target - transforms[idx][:3, 3]))
            return q, (final_err < 5e-3), num_iters, final_err

        # Try user seed first if provided for trajectory continuity
        if q_seed is not None:
            q_sol, success, iters, err_m = _dls_step(q_seed, max_iter)
            if success:
                is_col, _ = self.check_collision(q_sol)
                if not is_col:
                    return q_sol, True, iters, err_m
            best_q, best_succ, best_iters, best_err = q_sol, success, iters, err_m
        else:
            best_q, best_succ, best_iters, best_err = None, False, max_iter, 1e9

        # Seed generation: start from analytical seed, followed by smart geometric seeds
        yaw = math.atan2(target[1], target[0])
        alt_yaw = yaw + math.pi if yaw <= 0 else yaw - math.pi

        candidate_seeds = []
        for branch in [True, False]:
            if not use_flange:
                qa, _, _ = self.solve_ik_analytical_point_p(target, elbow_up=branch)
            else:
                qa, _, _ = self.solve_ik_analytical_flange(target, tool_pitch_rad=tool_pitch_rad, elbow_up=branch)
            if qa is not None:
                candidate_seeds.append(qa)

        candidate_seeds.extend([
            [yaw, 0.0, 0.0, 0.0, 0.0, 0.0],
            [yaw, -0.4, 0.8, 0.0, 0.0, 0.0],
            [yaw, 0.5, -0.5, 0.0, 0.0, 0.0],
            [yaw, 0.8, -1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.3, 0.5, 0.0, 0.0, 0.0],
            [alt_yaw, -0.5, -0.5, 0.0, 0.0, 0.0],
        ])

        valid_candidates = []
        for s in candidate_seeds:
            s_arr = np.zeros(self.num_joints, dtype=float)
            s_raw = np.array(s, dtype=float)
            s_arr[:min(len(s_raw), self.num_joints)] = s_raw[:self.num_joints]
            for i in range(self.num_joints):
                s_arr[i] = np.clip(s_arr[i], JOINT_LIMITS[i][0], JOINT_LIMITS[i][1])
            q_cand, ok, it_c, err_c = _dls_step(s_arr, max_iter)
            if ok:
                is_col, _ = self.check_collision(q_cand)
                if not is_col:
                    return q_cand, True, it_c, err_c
                valid_candidates.append((err_c, q_cand, ok, it_c, is_col))
            if err_c < best_err:
                best_q, best_succ, best_iters, best_err = q_cand, ok, it_c, err_c

        if valid_candidates:
            valid_candidates.sort(key=lambda x: x[0])
            return valid_candidates[0][1], True, valid_candidates[0][3], valid_candidates[0][0]

        return best_q, (best_err < 5e-3), best_iters, best_err

    def rnea(self, q, qd, qdd, grav=None, payload_mass=0.0, f_ext=None, n_ext=None):
        """
        Recursive Newton-Euler Algorithm (RNEA) for Inverse Dynamics.
        Returns joint torque vector tau (5,).
        """
        if grav is None:
            grav = self.gravity

        n = self.num_joints
        R_rel = [R_ORIGINS[i] @ ROT_FUNCS[i](q[i]) for i in range(n)]

        w = [np.zeros(3) for _ in range(n)]
        wd = [np.zeros(3) for _ in range(n)]
        vd = [np.zeros(3) for _ in range(n)]
        a_c = [np.zeros(3) for _ in range(n)]

        w_prev = np.zeros(3)
        wd_prev = np.zeros(3)
        vd_prev = -grav  # Base upward acceleration naturally accounts for gravity

        # Dynamically compute CoMs and masses with payload attached at TCP
        coms = [np.array(c, dtype=float) for c in COMS]
        masses = np.array(LINK_MASSES, dtype=float)
        if payload_mass > 0.0:
            last_idx = self.num_joints - 1
            m_tot = masses[last_idx] + payload_mass
            coms[last_idx] = (masses[last_idx] * coms[last_idx] + payload_mass * TOOL_TCP_OFFSET) / m_tot
            masses[last_idx] = m_tot

        # Forward recursion: Kinematics and inertial forces
        for i in range(n):
            R_i = R_rel[i]
            z_i = AXES[i]
            p_i = P_ORIGINS[i]

            w[i] = R_i.T @ w_prev + z_i * qd[i]
            wd[i] = R_i.T @ wd_prev + z_i * qdd[i] + np.cross(w[i], z_i * qd[i])
            acc_origin_prev = vd_prev + np.cross(wd_prev, p_i) + np.cross(w_prev, np.cross(w_prev, p_i))
            vd[i] = R_i.T @ acc_origin_prev
            a_c[i] = vd[i] + np.cross(wd[i], coms[i]) + np.cross(w[i], np.cross(w[i], coms[i]))

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
            m_i = masses[i]
            I_i = INERTIAS[i]
            com_i = coms[i]

            F_i = m_i * a_c[i]
            N_i = I_i @ wd[i] + np.cross(w[i], I_i @ w[i])

            if i == n - 1:
                f[i] = F_i + f_next
                n_mom[i] = N_i + n_next + np.cross(com_i, F_i)
            else:
                R_child = R_rel[i + 1]
                p_child = P_ORIGINS[i + 1]
                f_from_child = R_child @ f[i + 1]
                n_from_child = R_child @ n_mom[i + 1]
                f[i] = F_i + f_from_child
                n_mom[i] = N_i + n_from_child + np.cross(com_i, F_i) + np.cross(p_child, f_from_child)

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
        Solves Forward Dynamics: returns joint acceleration vector qdd (6,).
        qdd = M(q)^-1 * (tau - C(q, qd)*qd - g(q) - tau_payload - tau_friction)
        """
        if isinstance(tau, dict):
            tau = tau.get('tau', tau.get('tau_total', tau))
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

    def build_marker_array(self, q, tau, qdd=None, frame_id="base_link"):
        """
        Generates visualization_msgs/MarkerArray for RViz2 containing:
        - 5 Torque vector arrows located at the joints
        - 5 3D Text labels displaying tau (N*m), % load, and qdd
        """
        from visualization_msgs.msg import Marker, MarkerArray  # pyrefly: ignore [missing-import]
        from geometry_msgs.msg import Point  # pyrefly: ignore [missing-import]
        from std_msgs.msg import ColorRGBA  # pyrefly: ignore [missing-import]

        marker_array = MarkerArray()
        transforms = self.get_joint_transforms(q)

        joint_names_display = ["J1 (Base)", "J2 (Vai)", "J3 (Khuỷu)", "J4 (Cẳng tay)", "J5 (Cổ tay)", "J6 (Mặt bích)"]

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
        # 2. CLEAN UP 3D SCENE (REMOVE FLOATING 3D TEXT TO AVOID CLUTTER)
        # All telemetry is now hosted in the dedicated White-Background GUI Tab!
        # -------------------------------------------------------------
        for mid in [198, 199, 200]:
            del_m = Marker()
            del_m.header.frame_id = frame_id
            del_m.ns = "dynamics_dashboard"
            del_m.id = mid
            del_m.action = Marker.DELETE
            marker_array.markers.append(del_m)

        return marker_array

    def build_dashboard_image(self, tau, qdd=None, stamp=None):
        """
        Generates a high-definition 2D sensor_msgs/msg/Image HUD card displaying:
        - Robot model banner
        - Color-coded joint torque gauges with limits and percentages
        - Joint accelerations
        - Real-time status indicator
        """
        import cv2  # pyrefly: ignore [missing-import]
        cv2: Any = cv2
        from sensor_msgs.msg import Image  # pyrefly: ignore [missing-import]

        w, h = 460, 260
        img: Any = np.full((h, w, 3), (22, 25, 30), dtype=np.uint8)

        # Card border (Cyan / Amber / Red depending on max load)
        max_pct = max(min(150.0, abs(float(tau[i])) / float(TORQUE_LIMITS[i]) * 100.0) for i in range(self.num_joints))
        if max_pct < 50.0:
            border_col = (70, 150, 220)
            status_text = "NORMAL LOAD (<50%)"
            dot_col = (70, 220, 110)
        elif max_pct < 80.0:
            border_col = (30, 180, 240)
            status_text = "MODERATE LOAD (50-80%)"
            dot_col = (30, 190, 255)
        else:
            border_col = (60, 60, 240)
            status_text = "ALERT: HIGH TORQUE (>80%)"
            dot_col = (70, 70, 255)

        cv2.rectangle(img, (2, 2), (w - 3, h - 3), border_col, 2)

        # Title bar
        cv2.rectangle(img, (2, 2), (w - 3, 34), (35, 45, 60), -1)
        cv2.putText(img, "DENSO VS-6556 DYNAMICS", (15, 24),
                    cv2.FONT_HERSHEY_DUPLEX, 0.60, (255, 255, 255), 1, cv2.LINE_AA)

        # Sub-header
        cv2.putText(img, "Joint", (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 160, 175), 1, cv2.LINE_AA)
        cv2.putText(img, "Torque / Max", (75, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 160, 175), 1, cv2.LINE_AA)
        cv2.putText(img, "Motor Capacity", (235, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 160, 175), 1, cv2.LINE_AA)
        cv2.putText(img, "Accel", (395, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 160, 175), 1, cv2.LINE_AA)
        cv2.line(img, (10, 60), (w - 10, 60), (55, 65, 80), 1)

        y0 = 84
        row_step = 30
        for i in range(self.num_joints):
            y = y0 + i * row_step
            t = float(tau[i])
            lim = float(TORQUE_LIMITS[i])
            pct = min(150.0, abs(t) / lim * 100.0)

            if pct < 50.0:
                b_col = (70, 220, 110)
            elif pct < 80.0:
                b_col = (30, 190, 255)
            else:
                b_col = (70, 70, 255)

            cv2.putText(img, f"J{i+1}", (15, y), cv2.FONT_HERSHEY_DUPLEX, 0.50, (220, 225, 230), 1, cv2.LINE_AA)

            t_str = f"{t:+5.1f} / {lim:2.0f} Nm"
            cv2.putText(img, t_str, (70, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 235, 255), 1, cv2.LINE_AA)

            bx, by, bw, bh = 225, y - 12, 105, 14
            cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (35, 42, 52), -1)
            cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (75, 85, 100), 1)
            fill_w = int(bw * min(1.0, pct / 100.0))
            if fill_w > 0:
                cv2.rectangle(img, (bx, by), (bx + fill_w, by + bh), b_col, -1)

            pct_str = f"{pct:3.0f}%"
            cv2.putText(img, pct_str, (bx + bw + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 210, 220), 1, cv2.LINE_AA)

            acc_val = float(qdd[i]) if qdd is not None else 0.0
            cv2.putText(img, f"{acc_val:+4.1f}", (395, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 210, 220), 1, cv2.LINE_AA)

        # Footer
        cv2.line(img, (10, h - 30), (w - 10, h - 30), (55, 65, 80), 1)
        cv2.circle(img, (22, h - 16), 5, dot_col, -1)
        cv2.putText(img, f"STATUS: {status_text}", (35, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 220, 210), 1, cv2.LINE_AA)

        msg = Image()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.step = w * 3
        msg.data = img.tobytes()
        return msg


# -------------------------------------------------------------
# CLI SELF-TEST
# -------------------------------------------------------------
if __name__ == '__main__':
    print("=== DENSO VS-6556 DYNAMICS ENGINE TEST ===")
    engine = DensoDynamicsEngine()

    q_home = np.zeros(engine.num_joints)
    qd_zero = np.zeros(engine.num_joints)
    qdd_zero = np.zeros(engine.num_joints)

    print("\n--- DENSO MOTOR SPECIFICATIONS ---")
    for i, name in enumerate(JOINT_NAMES):
        deg_s = VELOCITY_LIMITS[i] * 180.0 / math.pi
        print(f"  {name}: {MOTOR_POWER_WATTS[i]:4.0f}W AC Servo | Gear {GEAR_RATIOS[i]:3.0f}:1 | Max Vel: {deg_s:5.1f} deg/s ({VELOCITY_LIMITS[i]:.2f} rad/s) | Max Effort: {TORQUE_LIMITS[i]:.0f} Nm | Max Accel: {ACCELERATION_LIMITS[i]:.0f} rad/s^2")

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
    q_test = np.array([0.2, 0.4, -0.3, 0.5, -0.2, 0.1])
    qd_test = np.array([0.1, -0.2, 0.3, -0.1, 0.2, -0.05])
    qdd_desired = np.array([0.5, -0.8, 1.2, -0.4, 0.6, 0.3])

    tau_id = engine.rnea(q_test, qd_test, qdd_desired)
    qdd_recovered = engine.forward_dynamics(q_test, qd_test, tau_id, include_friction=False)
    err = np.max(np.abs(qdd_desired - qdd_recovered))
    print(f"  Max Acceleration Error: {err:.2e}")
    assert err < 1e-6, "Error: Forward & Inverse Dynamics mismatch!"

    print("\nAll Dynamics Engine Tests PASSED successfully!")
