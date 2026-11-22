import numpy as np

R0 = 61.50
cor = 0.83407
kEnv = 1.74 # hoac 0.57 - 1.74
RL = 10.0

print("--- With base 400.0 and kEnv = 1.74 ---")
for ppm in [400, 650, 950, 1300, 2000]:
    rs = (cor * R0 / kEnv) * (ppm / 400.0) ** (-1.0 / 2.76903)
    vout = 5.0 * RL / (rs + RL)
    eps = 0.01
    rs_p = (cor * R0 / kEnv) * ((ppm + eps) / 400.0) ** (-1.0 / 2.76903)
    vout_p = 5.0 * RL / (rs_p + RL)
    dv_dppm = (vout_p - vout) / eps
    dppm_dlsb = (5.0 / 1023.0) / dv_dppm
    print(f"ppm={ppm:4d}: Rs={rs:5.2f} kOhm, Vout={vout:.3f} V, dppm/LSB={dppm_dlsb:4.1f}")

