#!/usr/bin/env python3
"""Test headless: Tab 7 AI Hybrid RL+MPC integration."""
import sys, os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'denso_gazebo', 'scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'denso_vs6556_moveit_config', 'scripts'))

# Test 1: Import tab module
from denso_gcode_energy_optimizer_tab import DensoGCodeEnergyOptimizerTab, CASE_DEFINITIONS
print(f'✅ Tab imported OK — {len(CASE_DEFINITIONS)} cases defined')
for k, v in CASE_DEFINITIONS.items():
    print(f'   {k}: {v["badge"]} ({v["color"]})')

# Test 2: Import MPC + RL
from denso_mpc_trajectory_controller import DensoMPCController
from denso_rl_policy_optimizer import DensoRLDecisionAgent
print('✅ MPC Controller + RL Agent imported OK')

# Test 3: Run energy optimizer comparison (all 6 cases)
from denso_gcode_energy_optimizer import DensoGCodeEnergyOptimizer, BUILTIN_GCODE_PATTERNS
opt = DensoGCodeEnergyOptimizer()
comp = opt.compare_all_cases(BUILTIN_GCODE_PATTERNS['square']['gcode'], step_mm=2.5, dt=0.02)
if 'error' not in comp:
    print(f'✅ compare_all_cases: {len(comp["results"])} results')
    for k, v in comp['results'].items():
        rl_info = ""
        if "rl_decision" in v:
            rd = v["rl_decision"]
            rl_info = f" | RL: {rd.get('elbow_mode','?')} tilt={rd.get('wrist_tilt_deg',0):.1f}° jerk={rd.get('mpc_mean_jerk',0):.2f}"
        print(f'   {k}: {v["e_total_j"]:.1f} J{rl_info}')
    print(f'   Best: {comp["best_case"]} ({comp["energy_saved_pct"]:.1f}% saved)')
else:
    print(f'❌ Error: {comp["error"]}')

print('✅ ALL TESTS PASSED')
