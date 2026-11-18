import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

sys.path.append(r'src/denso_vs6556_moveit_config/scripts')
sys.path.append(r'src/denso_gazebo/scripts')
from denso_dynamics_engine import DensoDynamicsEngine
from denso_birrt_metaheuristic_optimizer import BiRRTMetaheuristicPlanner, SphericalObstacle

out_dir = r"c:\Users\thanh\denso_ws\report_figures"
artifact_dir = r"C:\Users\thanh\.gemini\antigravity\brain\33dce09e-02a0-4fb2-84ff-fcd8f0b178cf"

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

rrt_pts_mm = rrt_res['cart_pts'] * 1000.0
gwo_pts_mm = opt_res['cart_pts'] * 1000.0

# Create exact single-panel portrait plot (aspect ratio matching 355x457)
fig, ax = plt.subplots(figsize=(6.5, 8.5), dpi=300)
fig.patch.set_facecolor('#090d16')
ax.set_facecolor('#090d16')

ax.tick_params(colors='#94a3b8')
for spine in ax.spines.values():
    spine.set_color('#1e293b')
ax.grid(True, linestyle=':', color='#1e293b', alpha=0.9)

# Dashed line
ax.plot([wps_mm[0, 1], wps_mm[-1, 1]], [wps_mm[0, 0], wps_mm[-1, 0]],
        color='#64748b', linestyle='--', linewidth=1.6, alpha=0.9,
        label='Đường tham chiếu nối thẳng')

# Obstacles
obs_names = ["Vật cản 1", "Vật cản 2", "Vật cản 3"]
for idx, obs in enumerate(obstacles):
    cy_mm = obs.center[1] * 1000.0
    cx_mm = obs.center[0] * 1000.0
    r_mm = obs.radius * 1000.0
    
    circ = Circle((cy_mm, cx_mm), r_mm, facecolor='#f43f5e', alpha=0.45, edgecolor='#f43f5e', linestyle='--', linewidth=1.6)
    circ_m = Circle((cy_mm, cx_mm), r_mm + 24.0, fill=False, edgecolor='#f43f5e', linestyle=':', linewidth=1.2, alpha=0.6)
    ax.add_patch(circ)
    ax.add_patch(circ_m)
    ax.text(cy_mm, cx_mm, obs_names[idx], color='#fda4af', fontsize=11, fontweight='bold', ha='center', va='center')

# Orange path (ROS 2 Bi-RRT)
ax.plot(rrt_pts_mm[:, 1], rrt_pts_mm[:, 0], color='#f59e0b', linewidth=2.8,
        label=r'ROS 2 Bi-RRT ($C^0$ gấp khúc)')

# Green path (GWO)
ax.plot(gwo_pts_mm[:, 1], gwo_pts_mm[:, 0], color='#22c55e', linewidth=3.2,
        label=r'GWO B-Spline $C^2$ (Lượn mượt né 3 vật cản)')

# Apex marker (double circle)
apex_y = wps_mm[3, 1]
apex_x = wps_mm[3, 0]
ax.scatter(apex_y, apex_x, s=220, facecolor='#ffffff', edgecolor='#22c55e', linewidth=2.8, zorder=6)
ax.scatter(apex_y, apex_x, s=40, color='#22c55e', zorder=7)

# Start and Goal
ax.scatter(wps_mm[0, 1], wps_mm[0, 0], color='#4ade80', s=160, edgecolor='#ffffff', linewidth=1.8, zorder=7, label='Điểm đầu (Start)')
ax.scatter(wps_mm[-1, 1], wps_mm[-1, 0], color='#ef4444', s=160, edgecolor='#ffffff', linewidth=1.8, zorder=7, label='Điểm đích (Goal)')

ax.invert_yaxis()
ax.set_xlim(-195, 195)
ax.set_ylim(-285, -565)
ax.set_title('QUỸ ĐẠO MŨI ROBOT TCP TRÊN BÀN (Y-X) - CASE 1', color='#f8fafc', fontsize=12.5, fontweight='bold', pad=14)
ax.set_xlabel('Tọa độ Y [mm]', color='#cbd5e1', fontsize=11)
ax.set_ylabel('Tọa độ X [mm]', color='#cbd5e1', fontsize=11)
ax.legend(loc='lower left', fontsize=9.5, facecolor='#090d16', edgecolor='#1e293b', labelcolor='#e2e8f0', framealpha=0.95)

plt.tight_layout()
single_path = os.path.join(out_dir, "fig_case1_trajectory_exact_gui.png")
single_art = os.path.join(artifact_dir, "fig_case1_trajectory_exact_gui.png")
plt.savefig(single_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
plt.savefig(single_art, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
plt.close()
print("Saved exact GUI single plot to:", single_path)

