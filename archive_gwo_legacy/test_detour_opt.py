import numpy as np
from scipy.interpolate import CubicSpline

p_start = np.array([-0.310, -0.170, 0.220])
p_end = np.array([-0.460,  0.170, 0.220])

obstacles = [
    {"center": np.array([-0.370, -0.070, 0.220]), "radius": 0.036},
    {"center": np.array([-0.395,  0.010, 0.250]), "radius": 0.038},
    {"center": np.array([-0.430,  0.090, 0.220]), "radius": 0.036}
]

d_vec = p_end[:2] - p_start[:2]
d_len = np.linalg.norm(d_vec)
t_hat = d_vec / d_len
n_hat = np.array([-t_hat[1], t_hat[0]])
if n_hat[0] > 0:
    n_hat = -n_hat

# Calculate optimal waypoints
wps = [p_start]
margin = 0.024
for obs in obstacles:
    c = obs["center"]
    r = obs["radius"]
    apex_xy = c[:2] + (r + margin) * n_hat
    apex_z = float(max(p_start[2], c[2] + 0.018)) if c[2] > p_start[2] + 0.010 else float(p_start[2])
    wps.append(np.array([apex_xy[0], apex_xy[1], apex_z]))
wps.append(p_end)

print("Waypoints:")
for i, w in enumerate(wps):
    print(f"  W{i}: {np.round(w, 4)}")

W = np.array(wps)
dists = [0.0]
for i in range(len(W) - 1):
    dists.append(dists[-1] + float(np.linalg.norm(W[i+1] - W[i])))
s_cum = np.array(dists) / dists[-1]

spl_x = CubicSpline(s_cum, W[:, 0], bc_type='clamped')
spl_y = CubicSpline(s_cum, W[:, 1], bc_type='clamped')
spl_z = CubicSpline(s_cum, W[:, 2], bc_type='clamped')

u = np.linspace(0, 1, 100)
traj = np.column_stack([spl_x(u), spl_y(u), spl_z(u)])

print("\nClearances:")
for i, obs in enumerate(obstacles):
    c = obs["center"]
    r = obs["radius"]
    min_d = np.min(np.linalg.norm(traj - c, axis=1))
    print(f"  Obs {i+1} (R={r*1000:.1f}mm): min_dist = {min_d*1000:.1f}mm | clearance = {(min_d - r)*1000:.1f}mm")
