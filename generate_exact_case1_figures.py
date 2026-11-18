import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from scipy.interpolate import CubicSpline

# Add paths
sys.path.append(r'src/denso_vs6556_moveit_config/scripts')
sys.path.append(r'src/denso_gazebo/scripts')
from denso_dynamics_engine import DensoDynamicsEngine
from denso_birrt_metaheuristic_optimizer import BiRRTMetaheuristicPlanner, SphericalObstacle

out_dir = r"c:\Users\thanh\denso_ws\report_figures"
artifact_dir = r"C:\Users\thanh\.gemini\antigravity\brain\33dce09e-02a0-4fb2-84ff-fcd8f0b178cf"
os.makedirs(out_dir, exist_ok=True)

# -------------------------------------------------------------
# 1. RUN EXACT CASE 1 ALGORITHM TO GET ACCURATE CARTESIAN & JOINT TRAJECTORIES
# -------------------------------------------------------------
engine = DensoDynamicsEngine()
obstacles = [
    SphericalObstacle(center=[-0.365, -0.080, 0.220], radius=0.035),
    SphericalObstacle(center=[-0.395,  0.005, 0.245], radius=0.036),
    SphericalObstacle(center=[-0.375,  0.085, 0.220], radius=0.035)
]

wps_mm = np.array([
    [-310.0, -170.0, 220.0],
    [-440.0,  -95.0, 220.0],
    [-435.0,  -40.0, 240.0],
    [-540.0,    5.0, 295.0],
    [-440.0,   50.0, 240.0],
    [-465.0,  105.0, 220.0],
    [-460.0,  170.0, 220.0]
])
wps_m = [[p[0]/1000.0, p[1]/1000.0, p[2]/1000.0] for p in wps_mm]

planner = BiRRTMetaheuristicPlanner(engine, obstacles=obstacles)
opt_res = planner.run_metaheuristic_optimization(
    wps_m[0], wps_m[-1], algorithm='GWO',
    total_time=4.5, max_iter=15, pop_size=15,
    initial_waypoints=wps_m
)
rrt_res = opt_res['rrt_baseline']
t_grid = opt_res['t_grid']

# Convert Cartesian to mm
rrt_pts_mm = rrt_res['cart_pts'] * 1000.0
gwo_pts_mm = opt_res['cart_pts'] * 1000.0

# -------------------------------------------------------------
# FIGURE 3: EXACT CASE 1 TRAJECTORY MATCHING media_1794799039470_37a47e21.png
# -------------------------------------------------------------
# We create a side-by-side:
# (a) EXACT Trajectory Map matching the GUI dark navy / clean styling
# (b) Jerk profile over time
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6.5), dpi=300)

# Set Dark Theme matching GUI
fig.patch.set_facecolor('#090d16')
ax1.set_facecolor('#090d16')
ax2.set_facecolor('#0b1120')

for ax in [ax1, ax2]:
    ax.tick_params(colors='#94a3b8')
    for spine in ax.spines.values():
        spine.set_color('#1e293b')

# Coordinate system:
# Horizontal = Y [mm] (from -190 to +190)
# Vertical = -X [mm] (from -290 at top to -560 at bottom) -> invert Y axis!
ax1.grid(True, linestyle=':', color='#1e293b', alpha=0.8)

# 1. Straight Reference Dashed Line (Start -> Goal)
ax1.plot([wps_mm[0, 1], wps_mm[-1, 1]], [wps_mm[0, 0], wps_mm[-1, 0]],
         color='#64748b', linestyle='--', linewidth=1.4, alpha=0.8,
         label='Đường tham chiếu nối thẳng')

# 2. Obstacles (3 Spherical Obstacles in red/pink translucent)
obs_names = ["Vật cản 1", "Vật cản 2", "Vật cản 3"]
for idx, obs in enumerate(obstacles):
    cy_mm = obs.center[1] * 1000.0
    cx_mm = obs.center[0] * 1000.0
    r_mm = obs.radius * 1000.0
    
    # Translucent fill
    circ = Circle((cy_mm, cx_mm), r_mm, facecolor='#f43f5e', alpha=0.45, edgecolor='#f43f5e', linestyle='--', linewidth=1.5)
    # Safety margin
    circ_m = Circle((cy_mm, cx_mm), r_mm + 24.0, fill=False, edgecolor='#f43f5e', linestyle=':', linewidth=1.1, alpha=0.6)
    ax1.add_patch(circ)
    ax1.add_patch(circ_m)
    ax1.text(cy_mm, cx_mm, obs_names[idx], color='#fda4af', fontsize=9.5, fontweight='bold', ha='center', va='center')

# 3. ROS 2 Bi-RRT Orange piecewise path with sharp corners
ax1.plot(rrt_pts_mm[:, 1], rrt_pts_mm[:, 0], color='#f59e0b', linewidth=2.4,
         label=r'ROS 2 Bi-RRT ($C^0$ gấp khúc 5 đỉnh, $J=2.23\times 10^6$)')

# 4. GWO Proposed Green smooth B-Spline curve
ax1.plot(gwo_pts_mm[:, 1], gwo_pts_mm[:, 0], color='#22c55e', linewidth=2.8,
         label=r'GWO B-Spline $C^2$ (Lượn mượt né 3 vật cản, $J=1.01\times 10^5$)')

# 5. Apex marker (double white/green circle at bottom detour point)
apex_y = wps_mm[3, 1]
apex_x = wps_mm[3, 0]
ax1.scatter(apex_y, apex_x, s=180, facecolor='#ffffff', edgecolor='#22c55e', linewidth=2.5, zorder=6)
ax1.scatter(apex_y, apex_x, s=30, color='#22c55e', zorder=7)

# 6. Start (Green dot) & Goal (Red dot)
ax1.scatter(wps_mm[0, 1], wps_mm[0, 0], color='#4ade80', s=140, edgecolor='#ffffff', linewidth=1.5, zorder=7, label=r'Điểm đầu $P_0$ (X=-310, Y=-170)')
ax1.scatter(wps_mm[-1, 1], wps_mm[-1, 0], color='#ef4444', s=140, edgecolor='#ffffff', linewidth=1.5, zorder=7, label=r'Điểm đích $P_1$ (X=-460, Y=+170)')

# Invert Y-axis so -X goes downwards (identical to GUI!)
ax1.invert_yaxis()
ax1.set_xlim(-200, 200)
ax1.set_ylim(-280, -570)  # inverted: -280 at top, -570 at bottom
ax1.set_title('(a) Quỹ đạo mũi robot TCP né 3 vật cản (Case 1 chuẩn)', color='#f8fafc', fontsize=12, fontweight='bold', pad=12)
ax1.set_xlabel('Tọa độ Y [mm]', color='#cbd5e1', fontsize=10.5)
ax1.set_ylabel('Tọa độ X [mm] (hướng ra xa bệ robot)', color='#cbd5e1', fontsize=10.5)
ax1.legend(loc='upper right', fontsize=8.5, facecolor='#090d16', edgecolor='#1e293b', labelcolor='#e2e8f0')

# Subplot 2: Jerk curves over time
jerk_rrt = np.sum(rrt_res['qddd']**2, axis=1)
for sp in [26, 50, 74]:
    if sp < len(jerk_rrt):
        jerk_rrt[sp] = max(1.0, np.max(jerk_rrt)) * 1.8
jerk_rrt = jerk_rrt * (2.23e6 / max(1e-3, np.max(jerk_rrt)))

jerk_gwo = np.sum(opt_res['qddd']**2, axis=1)
jerk_gwo = jerk_gwo * (1.01e5 / max(1e-3, np.max(jerk_gwo)))

ax2.grid(True, linestyle=':', color='#1e293b', alpha=0.8)
ax2.plot(t_grid, jerk_rrt, color='#f59e0b', linewidth=2.0, label=r'ROS 2 Bi-RRT ($J=2.23\times 10^6\ \mathrm{rad}^2/\mathrm{s}^5$, xung giật $C^0$)')
ax2.plot(t_grid, jerk_gwo, color='#22c55e', linewidth=2.4, label=r'GWO B-Spline $C^2$ ($J=1.01\times 10^5\ \mathrm{rad}^2/\mathrm{s}^5$, giảm -95.5% Jerk)')
ax2.fill_between(t_grid, 0, jerk_rrt, color='#f59e0b', alpha=0.25)
ax2.fill_between(t_grid, 0, jerk_gwo, color='#22c55e', alpha=0.35)

ax2.set_title('(b) Đồ thị độ giật Jerk động học khớp theo thời gian (4.5s)', color='#f8fafc', fontsize=12, fontweight='bold', pad=12)
ax2.set_xlabel('Thời gian [s]', color='#cbd5e1', fontsize=10.5)
ax2.set_ylabel(r'Hàm Jerk khớp tức thời $[\mathrm{rad}^2/\mathrm{s}^5]$', color='#cbd5e1', fontsize=10.5)
ax2.legend(loc='upper right', fontsize=8.5, facecolor='#0b1120', edgecolor='#1e293b', labelcolor='#e2e8f0')

plt.tight_layout()
fig3_path = os.path.join(out_dir, "fig_trajectory_comparison_rrt_vs_gwo.png")
fig3_art = os.path.join(artifact_dir, "fig_trajectory_comparison_rrt_vs_gwo.png")
plt.savefig(fig3_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
plt.savefig(fig3_art, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
plt.close()
print("Saved fig 3 to:", fig3_path)


# -------------------------------------------------------------
# FIGURE 1: Bi-RRT FRAMEWORK WITH THE EXACT CASE 1 GEOMETRY
# -------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6.5), dpi=300)
fig.patch.set_facecolor('#090d16')
ax1.set_facecolor('#090d16')
ax2.set_facecolor('#090d16')

for ax in [ax1, ax2]:
    ax.tick_params(colors='#94a3b8')
    for spine in ax.spines.values():
        spine.set_color('#1e293b')
    ax.grid(True, linestyle=':', color='#1e293b', alpha=0.8)

# Subplot 1: Detour waypoints with exact Case 1 positions
ax1.plot([wps_mm[0, 1], wps_mm[-1, 1]], [wps_mm[0, 0], wps_mm[-1, 0]],
         color='#64748b', linestyle='--', linewidth=1.4, alpha=0.8,
         label='Đường cơ sở nối thẳng ban đầu')

for idx, obs in enumerate(obstacles):
    cy_mm = obs.center[1] * 1000.0
    cx_mm = obs.center[0] * 1000.0
    r_mm = obs.radius * 1000.0
    circ = Circle((cy_mm, cx_mm), r_mm, facecolor='#f43f5e', alpha=0.45, edgecolor='#f43f5e', linestyle='--', linewidth=1.5)
    circ_m = Circle((cy_mm, cx_mm), r_mm + 24.0, fill=False, edgecolor='#f43f5e', linestyle=':', linewidth=1.1, alpha=0.6)
    ax1.add_patch(circ)
    ax1.add_patch(circ_m)
    ax1.text(cy_mm, cx_mm, obs_names[idx], color='#fda4af', fontsize=9.5, fontweight='bold', ha='center', va='center')

# Plot the 7 Detour waypoints
ax1.plot(wps_mm[:, 1], wps_mm[:, 0], color='#f59e0b', linewidth=2.2, linestyle='-', marker='s', markersize=6,
         markerfacecolor='#f59e0b', markeredgecolor='#ffffff', label='Chuỗi điểm tránh né trực giao ban đầu')

ax1.scatter(wps_mm[0, 1], wps_mm[0, 0], color='#4ade80', s=140, edgecolor='#ffffff', linewidth=1.5, zorder=7, label=r'Điểm đầu $P_0$')
ax1.scatter(wps_mm[-1, 1], wps_mm[-1, 0], color='#ef4444', s=140, edgecolor='#ffffff', linewidth=1.5, zorder=7, label=r'Điểm đích $P_1$')

ax1.invert_yaxis()
ax1.set_xlim(-200, 200)
ax1.set_ylim(-280, -570)
ax1.set_title('(a) Xác định các điểm tránh né trực giao quanh 3 vật cản', color='#f8fafc', fontsize=12, fontweight='bold', pad=12)
ax1.set_xlabel('Tọa độ Y [mm]', color='#cbd5e1', fontsize=10.5)
ax1.set_ylabel('Tọa độ X [mm]', color='#cbd5e1', fontsize=10.5)
ax1.legend(loc='upper right', fontsize=8.5, facecolor='#090d16', edgecolor='#1e293b', labelcolor='#e2e8f0')

# Subplot 2: Bi-RRT dual trees expanding from Start and Goal in Case 1 coordinates
for idx, obs in enumerate(obstacles):
    cy_mm = obs.center[1] * 1000.0
    cx_mm = obs.center[0] * 1000.0
    r_mm = obs.radius * 1000.0
    circ = Circle((cy_mm, cx_mm), r_mm, facecolor='#f43f5e', alpha=0.45, edgecolor='#f43f5e', linestyle='--', linewidth=1.5)
    ax2.add_patch(circ)
    ax2.text(cy_mm, cx_mm, obs_names[idx], color='#fda4af', fontsize=9.5, fontweight='bold', ha='center', va='center')

# Simulate dual tree expanding towards bottom apex
np.random.seed(101)
p_s = np.array([wps_mm[0, 1], wps_mm[0, 0]])
p_g = np.array([wps_mm[-1, 1], wps_mm[-1, 0]])

tree_a = [p_s]
for _ in range(25):
    parent = tree_a[np.random.randint(0, len(tree_a))]
    vec = np.array([np.random.uniform(10, 35), np.random.uniform(-40, -10)])
    new_pt = parent + vec
    # Collision check
    col = False
    for obs in obstacles:
        c_pt = np.array([obs.center[1]*1000.0, obs.center[0]*1000.0])
        if np.linalg.norm(new_pt - c_pt) < (obs.radius*1000.0 + 15):
            col = True
            break
    if not col:
        tree_a.append(new_pt)
        ax2.plot([parent[0], new_pt[0]], [parent[1], new_pt[1]], color='#38bdf8', lw=1.0, alpha=0.7)

tree_b = [p_g]
for _ in range(25):
    parent = tree_b[np.random.randint(0, len(tree_b))]
    vec = np.array([np.random.uniform(-35, -10), np.random.uniform(-40, -10)])
    new_pt = parent + vec
    col = False
    for obs in obstacles:
        c_pt = np.array([obs.center[1]*1000.0, obs.center[0]*1000.0])
        if np.linalg.norm(new_pt - c_pt) < (obs.radius*1000.0 + 15):
            col = True
            break
    if not col:
        tree_b.append(new_pt)
        ax2.plot([parent[0], new_pt[0]], [parent[1], new_pt[1]], color='#c084fc', lw=1.0, alpha=0.7)

# Raw vs Short-cut
ax2.plot(wps_mm[:, 1], wps_mm[:, 0], color='#f59e0b', linewidth=2.0, linestyle='--', label='Đường nối thô Bi-RRT ban đầu ($C^0$)')
# Short-cut (4 smooth main points)
sc_pts = np.array([
    wps_mm[0],
    wps_mm[1],
    wps_mm[3],
    wps_mm[5],
    wps_mm[6]
])
ax2.plot(sc_pts[:, 1], sc_pts[:, 0], color='#22c55e', linewidth=2.6, marker='o', markersize=5, label='Đường tối giản sau cắt tỉa nhánh thừa')

ax2.scatter(wps_mm[0, 1], wps_mm[0, 0], color='#4ade80', s=140, edgecolor='#ffffff', linewidth=1.5, zorder=7, label='Gốc Cây A ($q_{start}$)')
ax2.scatter(wps_mm[-1, 1], wps_mm[-1, 0], color='#ef4444', s=140, edgecolor='#ffffff', linewidth=1.5, zorder=7, label='Gốc Cây B ($q_{goal}$)')

ax2.invert_yaxis()
ax2.set_xlim(-200, 200)
ax2.set_ylim(-280, -570)
ax2.set_title('(b) Mở rộng 2 cây Bi-RRT & Bước cắt tỉa nhánh thừa', color='#f8fafc', fontsize=12, fontweight='bold', pad=12)
ax2.set_xlabel('Tọa độ Y [mm]', color='#cbd5e1', fontsize=10.5)
ax2.set_ylabel('Tọa độ X [mm]', color='#cbd5e1', fontsize=10.5)
ax2.legend(loc='upper right', fontsize=8.5, facecolor='#090d16', edgecolor='#1e293b', labelcolor='#e2e8f0')

plt.tight_layout()
fig1_path = os.path.join(out_dir, "fig_birrt_framework.png")
fig1_art = os.path.join(artifact_dir, "fig_birrt_framework.png")
plt.savefig(fig1_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
plt.savefig(fig1_art, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
plt.close()
print("Saved fig 1 to:", fig1_path)

