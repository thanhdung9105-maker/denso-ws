
import sys, os, math, numpy as np

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
    q1, q2, q3, q4, q5 = q
    A1 = standard_dh(-0.075, -math.pi/2, 0.335, q1)
    A2 = standard_dh( 0.270,  0.0,       0.0,   q2 - math.pi/2)
    A3 = standard_dh( 0.090,  math.pi/2, 0.0,   q3 + math.pi/2)
    A4 = standard_dh( 0.000, -math.pi/2, 0.295, -q4)
    A5 = standard_dh( 0.080,  0.0,       0.0,   q5 - math.pi/2)
    
    T0 = np.eye(4)
    T1 = T0 @ A1
    T2 = T1 @ A2
    T3 = T2 @ A3
    T4 = T3 @ A4
    T5 = T4 @ A5
    return [T0, T1, T2, T3, T4, T5], [A1, A2, A3, A4, A5]

def fk_planar_sincos(q):
    q1, q2, q3, q4, q5 = q
    L2, a3, L3, L_tool = 270.0, 90.0, 295.0, 80.0
    x2, z2 = -75.0, 335.0
    x3 = x2 + L2 * math.sin(q2)
    z3 = z2 + L2 * math.cos(q2)
    angle_arm2 = q2 + q3
    x5 = x3 + L3 * math.sin(angle_arm2) + a3 * math.cos(angle_arm2)
    z5 = z3 + L3 * math.cos(angle_arm2) - a3 * math.sin(angle_arm2)
    angle_tool = q2 + q3 + q5
    xf = x5 + L_tool * math.sin(angle_tool)
    zf = z5 + L_tool * math.cos(angle_tool)
    
    cq1, sq1 = math.cos(q1), math.sin(q1)
    P5_3d = np.array([x5 * cq1, x5 * sq1, z5])
    Pf_3d = np.array([xf * cq1, xf * sq1, zf])
    return P5_3d, Pf_3d

# Print verification
test_poses = [
    ("Home (0 deg)", [0, 0, 0, 0, 0]),
    ("J2 = 90 deg", [0, math.pi/2, 0, 0, 0]),
    ("J3 = 90 deg", [0, 0, math.pi/2, 0, 0]),
    ("J1=30, J2=45, J3=-30, J5=20", [math.radians(30), math.radians(45), math.radians(-30), 0, math.radians(20)])
]

for name, q in test_poses:
    tfs, As = fk_dh_steps(q)
    P5_dh = tfs[4][:3, 3] * 1000
    Pf_dh = tfs[5][:3, 3] * 1000
    P5_geo, Pf_geo = fk_planar_sincos(q)
    print(f"=== {name} ===")
    print(f"  DH    Point P: [{P5_dh[0]:+7.2f}, {P5_dh[1]:+7.2f}, {P5_dh[2]:+7.2f}] mm | Flange: [{Pf_dh[0]:+7.2f}, {Pf_dh[1]:+7.2f}, {Pf_dh[2]:+7.2f}] mm")
    print(f"  SinCos Point P: [{P5_geo[0]:+7.2f}, {P5_geo[1]:+7.2f}, {P5_geo[2]:+7.2f}] mm | Flange: [{Pf_geo[0]:+7.2f}, {Pf_geo[1]:+7.2f}, {Pf_geo[2]:+7.2f}] mm")
    print(f"  Diff Point P: {np.linalg.norm(P5_dh - P5_geo):.6f} mm | Diff Flange: {np.linalg.norm(Pf_dh - Pf_geo):.6f} mm")
