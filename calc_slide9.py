import numpy as np

# R0 = 61.50
# COR = (0.00035 * 31.4**2 - 0.02718 * 31.4 + 1.39538) - ((62.4 - 33.0) * 0.0018)
# cor = 0.8340
# kEnvAdapt = 1.74
# RL = 10.0
# ppm = 415 * (Rs * kEnvAdapt / (COR * R0)) ** (-2.769)
# -> (ppm / 415) = (Rs * kEnvAdapt / (COR * R0)) ** (-2.769)
# -> Rs = (COR * R0 / kEnvAdapt) * (ppm / 415) ** (-1 / 2.769)
# -> Vout = 5.0 * RL / (Rs + RL)

R0 = 61.50
cor = 0.83407
kEnv = 1.74
RL = 10.0

for ppm in [415, 1000, 1400, 2000]:
    rs = (cor * R0 / kEnv) * (ppm / 415.0) ** (-1.0 / 2.769)
    vout = 5.0 * RL / (rs + RL)
    # dppm / dLSB: LSB = 5.0 / 1023
    # dV/dppm or dppm/dV
    # V = 5*RL / (Rs + RL)
    # dV/dRs = -5*RL / (Rs + RL)^2
    # dRs/dppm = (-1/2.769) * (cor*R0/kEnv) / 415 * (ppm/415)**(-1/2.769 - 1)
    # dV/dppm = dV/dRs * dRs/dppm
    # dppm/dLSB = (1 / dV/dppm) * (5.0 / 1023.0)
    eps = 0.01
    rs_p = (cor * R0 / kEnv) * ((ppm + eps) / 415.0) ** (-1.0 / 2.769)
    vout_p = 5.0 * RL / (rs_p + RL)
    dv_dppm = (vout_p - vout) / eps
    dppm_dlsb = (5.0 / 1023.0) / dv_dppm
    print(f"ppm={ppm}: Rs={rs:.2f} kOhm, Vout={vout:.3f} V, dppm/LSB={dppm_dlsb:.1f}")

