import sys, os, math, numpy as np

# -----------------------------------------------------------------------------
# 1. LINK-BY-LINK KINEMATICS (CHÍNH XÁC 100% TỪNG LINK THEO RVIZ / URDF)
# -----------------------------------------------------------------------------
def link_transform(d, theta_deg, a, b, alpha_deg):
    th = math.radians(theta_deg)
    al = math.radians(alpha_deg)
    return np.array([
        [math.cos(th), -math.sin(th)*math.cos(al),  math.sin(th)*math.sin(al), a*math.cos(th) - b*math.sin(th)],
        [math.sin(th),  math.cos(th)*math.cos(al), -math.cos(th)*math.sin(al), a*math.sin(th) + b*math.cos(th)],
        [0.0,           math.sin(al),               math.cos(al),              d],
        [0.0,           0.0,                        0.0,                       1.0]
    ])

# Table matching ROS 2 URDF links exactly at q = 0:
# [Link Name, d (m), theta_offset (deg), a (m), b_dy (m), alpha (deg)]
LINK_PARAMS = [
    ("link_1", 0.2050,   0.0,  0.0000,  0.0000,   0.0),
    ("link_2", 0.1300,   0.0, -0.0750, -0.0745,   0.0),
    ("link_3", 0.2700,   0.0,  0.0000,  0.0065,   0.0),
    ("link_4", 0.1090,   0.0,  0.0900,  0.0680,   0.0),
    ("link_5", 0.1860,   0.0,  0.0000,  0.0000,   0.0),
    ("link_6", 0.06204,  0.0,  0.0000,  0.0000,   0.0),
    ("flange", 0.0280,   0.0,  0.0000,  0.0000,   0.0),
]

def verify_rviz_links():
    print("=== 1. KIỂM CHỨNG TỌA ĐỘ TỪNG LINK SO VỚI RVIZ (q = 0) ===")
    H = np.eye(4)
    for name, d, th, a, b, al in LINK_PARAMS:
        H_step = link_transform(d, th, a, b, al)
        H = H @ H_step
        pos = H[:3, 3]
        print(f"  {name:8s}: X = {pos[0]:+7.4f} m, Y = {pos[1]:+7.4f} m, Z = {pos[2]:+7.4f} m  ({pos[0]*1000:+7.1f}, {pos[1]*1000:+7.1f}, {pos[2]*1000:+7.1f} mm)")

# -----------------------------------------------------------------------------
# 2. STANDARD DENAVIT-HARTENBERG (6-DOF JOINT-AXES CONVENTION)
# -----------------------------------------------------------------------------
def standard_dh(a, alpha, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0]
    ])

def fk_dh_steps(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5] if len(q) > 5 else 0.0
    A1 = standard_dh(-0.075, -math.pi/2, 0.335,   q1)
    A2 = standard_dh( 0.270,  0.0,       0.0,    -q2 - math.pi/2)
    A3 = standard_dh( 0.090,  math.pi/2, 0.0,    -q3 + math.pi/2)
    A4 = standard_dh( 0.000, -math.pi/2, 0.295,   q4)
    A5 = standard_dh( 0.000,  math.pi/2, 0.0,    -q5)
    A6 = standard_dh( 0.000,  0.0,       0.09004, q6)
    
    T0 = np.eye(4)
    T1 = T0 @ A1
    T2 = T1 @ A2
    T3 = T2 @ A3
    T4 = T3 @ A4
    T5 = T4 @ A5
    T6 = T5 @ A6
    return [T0, T1, T2, T3, T4, T5, T6], [A1, A2, A3, A4, A5, A6]


def fk_urdf(q):
    from denso_dynamics_engine import DensoDynamicsEngine
    eng = DensoDynamicsEngine()
    q_full = np.zeros(eng.num_joints)
    q_full[:min(len(q), eng.num_joints)] = q[:eng.num_joints]
    tfs = eng.get_joint_transforms(q_full)
    return tfs[5][:3, 3] * 1000, tfs[-1][:3, 3] * 1000

def fk_planar_sincos(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    L2, a3, L3, L_tool = 270.0, 90.0, 295.0, 90.04
    x2, z2 = -75.0, 335.0
    x3 = x2 - L2 * math.sin(q2)
    z3 = z2 + L2 * math.cos(q2)
    angle_arm2 = q2 + q3
    x5 = x3 - L3 * math.sin(angle_arm2) + a3 * math.cos(angle_arm2)
    z5 = z3 + L3 * math.cos(angle_arm2) + a3 * math.sin(angle_arm2)

    # Tool vector accounting for forearm roll q4 and wrist pitch q5
    dx_in_plane = -L_tool * (math.sin(angle_arm2) * math.cos(q5) + math.cos(angle_arm2) * math.cos(q4) * math.sin(q5))
    dz_in_plane =  L_tool * (math.cos(angle_arm2) * math.cos(q5) - math.sin(angle_arm2) * math.cos(q4) * math.sin(q5))
    dy_out_plane = -L_tool * math.sin(q4) * math.sin(q5)

    xf = x5 + dx_in_plane
    zf = z5 + dz_in_plane

    cq1, sq1 = math.cos(q1), math.sin(q1)
    P5_3d = np.array([x5 * cq1, x5 * sq1, z5])
    Pf_3d = np.array([xf * cq1 - dy_out_plane * sq1, xf * sq1 + dy_out_plane * cq1, zf])
    return P5_3d, Pf_3d

if __name__ == "__main__":
    verify_rviz_links()
    
    print("\n=== 2. KIỂM CHỨNG GÓC ĐỘNG HỌC THUẬN VỚI CÁC TƯ THẾ TEST ===")
    test_poses = [
        ("Home (0 deg)", [0, 0, 0, 0, 0, 0]),
        ("J2 = 90 deg", [0, math.pi/2, 0, 0, 0, 0]),
        ("J3 = 90 deg", [0, 0, math.pi/2, 0, 0, 0]),
        ("J1=30, J2=45, J3=-30, J5=20, J6=45", [math.radians(30), math.radians(45), math.radians(-30), 0, math.radians(20), math.radians(45)]),
        ("Roll J4=60, J2=30, J3=40, J6=90", [math.radians(15), math.radians(30), math.radians(40), math.radians(60), math.radians(10), math.radians(90)]),
    ]

    for name, q in test_poses:
        tfs, As = fk_dh_steps(q)
        P5_dh = tfs[5][:3, 3] * 1000
        Pf_dh = tfs[6][:3, 3] * 1000
        P5_geo, Pf_geo = fk_planar_sincos(q)
        print(f"=== {name} ===")
        print(f"  DH    Point P: [{P5_dh[0]:+7.2f}, {P5_dh[1]:+7.2f}, {P5_dh[2]:+7.2f}] mm | Flange: [{Pf_dh[0]:+7.2f}, {Pf_dh[1]:+7.2f}, {Pf_dh[2]:+7.2f}] mm")
        print(f"  SinCos Point P: [{P5_geo[0]:+7.2f}, {P5_geo[1]:+7.2f}, {P5_geo[2]:+7.2f}] mm | Flange: [{Pf_geo[0]:+7.2f}, {Pf_geo[1]:+7.2f}, {Pf_geo[2]:+7.2f}] mm")
        P5_u, Pf_u = fk_urdf(q)
        print(f"  Diff Point P: {np.linalg.norm(P5_dh - P5_u):.6f} mm | Diff Flange (DH vs URDF): {np.linalg.norm(Pf_dh - Pf_u):.6f} mm")
