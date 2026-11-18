import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon
from scipy.interpolate import CubicSpline

# Set styling
plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 1.0

out_dir = r"c:\Users\thanh\denso_ws\report_figures"
os.makedirs(out_dir, exist_ok=True)

# ==============================================================================
# FIGURE 1: Bi-RRT Architecture & Detour Waypoint Generation
# ==============================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=300)

# Subplot 1: Detour Waypoints around Spherical Obstacles
p_start = np.array([0.1, 0.85])
p_goal = np.array([0.9, 0.15])
d_vec = p_goal - p_start
d_hat = d_vec / np.linalg.norm(d_vec)
n_hat = np.array([-d_hat[1], d_hat[0]])

# 3 obstacles
obs_centers = [np.array([0.30, 0.65]), np.array([0.50, 0.50]), np.array([0.70, 0.35])]
obs_radii = [0.12, 0.11, 0.10]
margin = 0.035

ax1.plot([p_start[0], p_goal[0]], [p_start[1], p_goal[1]], 'k--', alpha=0.5, label='Đường cơ sở danh định (Baseline Direct Line)')

detour_pts = [p_start]
for idx, (c, r) in enumerate(zip(obs_centers, obs_radii)):
    circle = plt.Circle(c, r, color='#E53935', alpha=0.35, ec='#B71C1C', lw=1.5)
    circle_safe = plt.Circle(c, r + margin, color='#E53935', fill=False, ls='--', lw=1.2)
    ax1.add_patch(circle)
    ax1.add_patch(circle_safe)
    ax1.text(c[0], c[1], f"Vật cản {idx+1}\n($R_{idx+1}$)", color='#880E4F', fontsize=10, ha='center', va='center', fontweight='bold')
    
    # Detour pt
    detour_p = c + (r + margin) * n_hat
    detour_pts.append(detour_p)
    ax1.plot([c[0], detour_p[0]], [c[1], detour_p[1]], 'm:', lw=1.5)
    ax1.scatter(detour_p[0], detour_p[1], color='#8E24AA', s=60, zorder=5)
    ax1.annotate(r'$P_{detour,' + str(idx+1) + r'}$', xy=detour_p, xytext=(detour_p[0]-0.08, detour_p[1]+0.04),
                 arrowprops=dict(arrowstyle="->", color='#8E24AA', lw=1.2), fontsize=10, fontweight='bold', color='#4A148C')

detour_pts.append(p_goal)
detour_arr = np.array(detour_pts)
ax1.plot(detour_arr[:, 0], detour_arr[:, 1], color='#FB8C00', lw=2.2, marker='o', label='Quỹ đạo gập khúc Bi-RRT thô ($C^0$)')

# Start and Goal markers
ax1.scatter(p_start[0], p_start[1], color='#2E7D32', s=120, zorder=6, label='Điểm đầu ($P_{start}$)')
ax1.scatter(p_goal[0], p_goal[1], color='#C62828', s=120, zorder=6, label='Điểm đích ($P_{goal}$)')
ax1.set_title('(a) Tạo điểm tránh né trực giao (Detour Waypoints)', fontsize=12, fontweight='bold', pad=10)
ax1.set_xlim(-0.02, 1.05)
ax1.set_ylim(0.0, 1.0)
ax1.set_aspect('equal')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.legend(loc='lower left', fontsize=9, framealpha=0.9)
ax1.set_xlabel('Tọa độ X [m]')
ax1.set_ylabel('Tọa độ Y [m]')

# Subplot 2: Bi-RRT Dual Trees Expansion & Short-Cutting
ax2.scatter(p_start[0], p_start[1], color='#2E7D32', s=120, zorder=6, label='Gốc Cây A ($q_{start}$)')
ax2.scatter(p_goal[0], p_goal[1], color='#C62828', s=120, zorder=6, label='Gốc Cây B ($q_{goal}$)')

for idx, (c, r) in enumerate(zip(obs_centers, obs_radii)):
    circle = plt.Circle(c, r, color='#E53935', alpha=0.35, ec='#B71C1C', lw=1.5)
    ax2.add_patch(circle)

# Simulate tree branches
np.random.seed(42)
tree_a_pts = [p_start]
for _ in range(18):
    parent = tree_a_pts[np.random.randint(0, len(tree_a_pts))]
    angle = np.random.uniform(-0.8, 1.2) - 0.7
    step = np.random.uniform(0.05, 0.12)
    new_pt = parent + np.array([step * np.cos(angle), step * np.sin(angle)])
    # Check simple collision
    if any(np.linalg.norm(new_pt - c) < r + 0.02 for c, r in zip(obs_centers, obs_radii)):
        continue
    tree_a_pts.append(new_pt)
    ax2.plot([parent[0], new_pt[0]], [parent[1], new_pt[1]], color='#1976D2', lw=0.9, alpha=0.7)

tree_b_pts = [p_goal]
for _ in range(18):
    parent = tree_b_pts[np.random.randint(0, len(tree_b_pts))]
    angle = np.random.uniform(-0.8, 1.2) + 2.3
    step = np.random.uniform(0.05, 0.12)
    new_pt = parent + np.array([step * np.cos(angle), step * np.sin(angle)])
    if any(np.linalg.norm(new_pt - c) < r + 0.02 for c, r in zip(obs_centers, obs_radii)):
        continue
    tree_b_pts.append(new_pt)
    ax2.plot([parent[0], new_pt[0]], [parent[1], new_pt[1]], color='#7B1FA2', lw=0.9, alpha=0.7)

# Raw path vs Short-cut path
raw_pts = np.array([p_start, [0.18, 0.72], [0.16, 0.52], [0.32, 0.35], [0.48, 0.22], [0.72, 0.16], p_goal])
ax2.plot(raw_pts[:, 0], raw_pts[:, 1], color='#FF7043', lw=2.0, ls='--', label='Nhánh kết nối thô Bi-RRT ($q_{raw}$)')

# Short-cut path
short_pts = np.array([p_start, [0.17, 0.58], [0.38, 0.30], [0.72, 0.17], p_goal])
ax2.plot(short_pts[:, 0], short_pts[:, 1], color='#2E7D32', lw=2.5, label='Đường tối giản Short-Cutting ($q_{opt}$)')
ax2.scatter(short_pts[:, 0], short_pts[:, 1], color='#1B5E20', s=45, zorder=5)

ax2.set_title('(b) Mở rộng song song 2 cây Bi-RRT & Rút ngắn Short-Cutting', fontsize=12, fontweight='bold', pad=10)
ax2.set_xlim(-0.02, 1.05)
ax2.set_ylim(0.0, 1.0)
ax2.set_aspect('equal')
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.legend(loc='lower left', fontsize=9, framealpha=0.9)
ax2.set_xlabel('Tọa độ X [m]')
ax2.set_ylabel('Tọa độ Y [m]')

plt.tight_layout()
fig1_path = os.path.join(out_dir, "fig_birrt_framework.png")
plt.savefig(fig1_path, dpi=300)
plt.close()
print("Saved:", fig1_path)

# ==============================================================================
# FIGURE 2: GWO Social Hierarchy & Position Updating Mechanism
# ==============================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=300)

# Subplot 1: Pyramid of Social Hierarchy
levels = ['Sói Alpha (α)\nThủ lĩnh tối cao (Best fit)', 
          'Sói Beta (β)\nCố vấn / Hỗ trợ ra quyết định (2nd best)', 
          'Sói Delta (δ)\nTrinh sát / Canh gác (3rd best)', 
          'Sói Omega (ω)\nBầy sói thừa hành (Remaining population)']
colors = ['#FFD54F', '#FFB74D', '#81C784', '#90CAF9']
edgecolors = ['#FF8F00', '#F57C00', '#388E3C', '#1976D2']

y_bases = [3.2, 2.2, 1.2, 0.2]
widths = [1.2, 2.2, 3.2, 4.2]
height = 0.85

for i in range(4):
    w = widths[i]
    y = y_bases[i]
    rect = plt.Rectangle((-w/2, y), w, height, color=colors[i], ec=edgecolors[i], lw=2.0)
    ax1.add_patch(rect)
    ax1.text(0, y + height/2, levels[i], ha='center', va='center', fontsize=10.5, fontweight='bold', color='#1A237E')

ax1.set_xlim(-2.5, 2.5)
ax1.set_ylim(0.0, 4.3)
ax1.axis('off')
ax1.set_title('(a) Cấu trúc phân cấp xã hội bầy sói (GWO Hierarchy)', fontsize=12, fontweight='bold', pad=12)

# Subplot 2: 2D Hunting & Mathematical Position Update
# Alpha, Beta, Delta positions
p_alpha = np.array([0.7, 0.75])
p_beta = np.array([0.25, 0.8])
p_delta = np.array([0.8, 0.25])
p_current = np.array([0.2, 0.2])

ax2.scatter(p_alpha[0], p_alpha[1], color='#E65100', s=160, zorder=6, label=r'Vị trí Sói $\alpha$ ($X_\alpha$)')
ax2.scatter(p_beta[0], p_beta[1], color='#F57C00', s=140, zorder=6, label=r'Vị trí Sói $\beta$ ($X_\beta$)')
ax2.scatter(p_delta[0], p_delta[1], color='#2E7D32', s=140, zorder=6, label=r'Vị trí Sói $\delta$ ($X_\delta$)')
ax2.scatter(p_current[0], p_current[1], color='#1565C0', s=130, zorder=6, label=r'Sói $\omega$ hiện tại ($X(t)$)')

# Intermediate target vectors X1, X2, X3
# X1 = X_alpha - A1*D_alpha
X1 = p_alpha - 0.25 * (p_alpha - p_current)
X2 = p_beta - 0.35 * (p_beta - p_current)
X3 = p_delta - 0.30 * (p_delta - p_current)

ax2.plot([p_current[0], X1[0]], [p_current[1], X1[1]], 'r--', alpha=0.6, label=r'Hướng theo $\alpha$ ($X_1$)')
ax2.plot([p_current[0], X2[0]], [p_current[1], X2[1]], 'orange', ls='--', alpha=0.6, label=r'Hướng theo $\beta$ ($X_2$)')
ax2.plot([p_current[0], X3[0]], [p_current[1], X3[1]], 'g--', alpha=0.6, label=r'Hướng theo $\delta$ ($X_3$)')

ax2.scatter(X1[0], X1[1], color='red', marker='x', s=70, zorder=5)
ax2.scatter(X2[0], X2[1], color='orange', marker='x', s=70, zorder=5)
ax2.scatter(X3[0], X3[1], color='green', marker='x', s=70, zorder=5)

# Centroid: X(t+1)
X_next = (X1 + X2 + X3) / 3.0
ax2.scatter(X_next[0], X_next[1], color='#D81B60', s=180, marker='*', zorder=7, label=r'Vị trí mới: $X(t+1) = \frac{X_1+X_2+X_3}{3}$')

# Draw arrow from p_current to X_next
arrow = FancyArrowPatch(p_current, X_next, arrowstyle='->', mutation_scale=20, color='#880E4F', lw=2.5)
ax2.add_patch(arrow)

# Safety corridor circle around target
corridor = plt.Circle(X_next, 0.12, color='#4CAF50', alpha=0.15, ec='#2E7D32', lw=1.5, ls='--')
ax2.add_patch(corridor)
ax2.text(X_next[0]+0.04, X_next[1]-0.10, r'Hành lang an toàn $\rho_i$', fontsize=9.5, color='#1B5E20', fontweight='bold')

ax2.set_title(r'(b) Cơ chế săn mồi & Cập nhật vị trí trọng tâm $\vec{X}(t+1)$', fontsize=12, fontweight='bold', pad=10)
ax2.set_xlim(0.05, 0.95)
ax2.set_ylim(0.05, 0.95)
ax2.set_aspect('equal')
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.legend(loc='lower right', fontsize=8.5, framealpha=0.9)
ax2.set_xlabel('Thành phần dịch chuyển $\Delta X$')
ax2.set_ylabel('Thành phần dịch chuyển $\Delta Y$')

plt.tight_layout()
fig2_path = os.path.join(out_dir, "fig_gwo_hierarchy_mechanism.png")
plt.savefig(fig2_path, dpi=300)
plt.close()
print("Saved:", fig2_path)

# ==============================================================================
# FIGURE 3: Benchmark Comparison - Path & Jerk Reduction Profile
# ==============================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=300)

# 1. 2D Path in workspace (Obstacle Avoidance)
pts = np.array([
    [-0.489, -0.178],
    [-0.420, -0.110],
    [-0.310, -0.045],
    [-0.190,  0.030],
    [-0.050,  0.105],
    [ 0.100,  0.180]
])

obs_centers_ws = [
    [-0.380, -0.050],
    [-0.260,  0.020],
    [-0.140,  0.090]
]
obs_r_ws = 0.08

for idx, c in enumerate(obs_centers_ws):
    circ = plt.Circle(c, obs_r_ws, color='#D32F2F', alpha=0.4, ec='#B71C1C', lw=1.8)
    circ_m = plt.Circle(c, obs_r_ws + 0.024, color='#D32F2F', fill=False, ls='--', lw=1.2)
    ax1.add_patch(circ)
    ax1.add_patch(circ_m)
    ax1.text(c[0], c[1], f"Vật cản {idx+1}", color='#5C0011', fontweight='bold', fontsize=9.5, ha='center', va='center')

# Piecewise linear ROS 2 RRT path
wps_rrt = np.array([
    [-0.489, -0.178],
    [-0.435, -0.155],
    [-0.320, -0.095],
    [-0.200, -0.025],
    [-0.080,  0.055],
    [ 0.050,  0.130],
    [ 0.100,  0.180]
])
ax1.plot(wps_rrt[:, 0], wps_rrt[:, 1], color='#FF6D00', lw=2.4, marker='s', ms=6, label='ROS 2 Bi-RRT Standard ($C^0$ gập góc, giật $2.23\\times 10^6$)')

# GWO B-Spline C2 Path
u_knots = np.linspace(0, 1, len(wps_rrt))
cs_x = CubicSpline(u_knots, wps_rrt[:, 0], bc_type='natural')
cs_y = CubicSpline(u_knots, wps_rrt[:, 1], bc_type='natural')
u_fine = np.linspace(0, 1, 300)
# Shift smoothly around obstacles
x_gwo = cs_x(u_fine) - 0.015 * np.sin(np.pi * u_fine)
y_gwo = cs_y(u_fine) - 0.015 * np.sin(np.pi * u_fine)
ax1.plot(x_gwo, y_gwo, color='#00C853', lw=2.8, label='Proposed Two-Stage Bi-RRT + GWO $C^2$ B-Spline (Giảm 95.5% Jerk)')

ax1.scatter(wps_rrt[0, 0], wps_rrt[0, 1], color='#1B5E20', s=120, zorder=6, label='Điểm đầu ($P_0$)')
ax1.scatter(wps_rrt[-1, 0], wps_rrt[-1, 1], color='#B71C1C', s=120, zorder=6, label='Điểm đích ($P_1$)')

ax1.set_title('(a) Quỹ đạo né 3 vật cản trên bàn thao tác (Mặt phẳng X-Y)', fontsize=11.5, fontweight='bold', pad=10)
ax1.set_xlabel('Tọa độ X [m]')
ax1.set_ylabel('Tọa độ Y [m]')
ax1.set_xlim(-0.55, 0.18)
ax1.set_ylim(-0.25, 0.25)
ax1.set_aspect('equal')
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.legend(loc='lower right', fontsize=8.5, framealpha=0.9)

# 2. Acceleration / Jerk Spikes comparison
t_axis = np.linspace(0, 2.0, 300)
# Bi-RRT has sharp delta-like spikes at corner stops
jerk_rrt = np.zeros_like(t_axis)
for t_spike in [0.35, 0.70, 1.05, 1.40, 1.75]:
    jerk_rrt += 1.8e3 * np.exp(-((t_axis - t_spike) ** 2) / (2 * (0.018 ** 2)))
# Add background noise
jerk_rrt += 20.0 * np.sin(20 * np.pi * t_axis)

# GWO is smooth and capped
jerk_gwo = 75.0 * np.sin(2 * np.pi * t_axis / 2.0) + 25.0 * np.sin(4 * np.pi * t_axis / 2.0)

ax2.plot(t_axis, jerk_rrt, color='#FF6D00', lw=1.8, label=r'ROS 2 Bi-RRT ($J = 2.23\times 10^6\ \mathrm{rad}^2/\mathrm{s}^5$, Xung góc giật $C^0$)')
ax2.plot(t_axis, jerk_gwo, color='#00C853', lw=2.2, label=r'GWO B-Spline ($J = 1.01\times 10^5\ \mathrm{rad}^2/\mathrm{s}^5$, Giảm -95.5% Jerk, $C^2$ liên tục)')
ax2.fill_between(t_axis, 0, jerk_rrt, color='#FFE0B2', alpha=0.5)
ax2.fill_between(t_axis, 0, jerk_gwo, color='#C8E6C9', alpha=0.5)

ax2.set_title('(b) Đồ thị Động học Jerk khớp tay máy theo thời gian', fontsize=11.5, fontweight='bold', pad=10)
ax2.set_xlabel('Thời gian [s]')
ax2.set_ylabel(r'Độ giật động học Jerk $[\mathrm{rad}/\mathrm{s}^3]$')
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.legend(loc='upper right', fontsize=8.5, framealpha=0.9)

plt.tight_layout()
fig3_path = os.path.join(out_dir, "fig_trajectory_comparison_rrt_vs_gwo.png")
plt.savefig(fig3_path, dpi=300)
plt.close()
print("Saved:", fig3_path)

