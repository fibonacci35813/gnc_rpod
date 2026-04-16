"""
sim/params_utils.py — JSON param I/O helpers for Python optimiser.
Phase 3.
"""

import json
import os
from pathlib import Path


DEFAULT_PARAMS = {
    "kp_x": 0.30, "kp_y": 0.20, "kp_z": 0.30,
    "kd_x": 30.0, "kd_y": 25.0, "kd_z": 30.0,
    "kp_terminal_x": 0.50, "kp_terminal_y": 0.50, "kp_terminal_z": 0.50,
    "kd_terminal_x": 42.0, "kd_terminal_y": 42.0, "kd_terminal_z": 42.0,
    "K_V": 0.010,
    "v_phase_0_max": 1.00, "v_phase_1_max": 0.50,
    "v_phase_2_max": 0.08, "v_phase_3_max": 0.04,
    "fdir_hold_timeout_s": 60.0, "fdir_dropout_limit": 3,
    "mib_normal_ns": 0.100, "mib_terminal_ns": 0.005,
}


def read_params_json(path: str) -> dict:
    """Read params from a JSON file; missing keys get DEFAULT_PARAMS values."""
    params = dict(DEFAULT_PARAMS)
    if os.path.exists(path):
        with open(path, "r") as f:
            data = json.load(f)
        params.update(data)
    return params


def write_params_json(params: dict, path: str) -> None:
    """Write params dict to a JSON file (creates parent dirs if needed)."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(params, f, indent=2)
        f.write("\n")
