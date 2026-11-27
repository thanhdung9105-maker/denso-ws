#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared motor and actuator parameters used by MPC and energy analysis."""
import numpy as np

# User supplied nominal ratings in joint order J1..J6.
MOTOR_RATED_POWERS_W = np.array([750.0, 400.0, 200.0, 80.0, 50.0, 50.0], dtype=float)
COPPER_LOSS_COEFFICIENTS = np.array([1.2, 1.5, 1.8, 2.2, 2.5, 2.5], dtype=float)
TORQUE_LIMITS_NM = np.array([80.0, 100.0, 60.0, 25.0, 20.0, 15.0], dtype=float)

def validate_energy_parameters():
    """Fail early if shared per-joint arrays become inconsistent."""
    expected = (6,)
    for name, values in (
        ("MOTOR_RATED_POWERS_W", MOTOR_RATED_POWERS_W),
        ("COPPER_LOSS_COEFFICIENTS", COPPER_LOSS_COEFFICIENTS),
        ("TORQUE_LIMITS_NM", TORQUE_LIMITS_NM),
    ):
        if values.shape != expected or not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError(f"{name} must contain six finite positive values.")
