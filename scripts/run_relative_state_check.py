#!/usr/bin/env python3
"""check relative state adapter functionality"""

from __future__ import annotations

import numpy as np

from rpod_gnc.bsk.relative_state_adapter import compute_relative_state

def main() -> None:
    target_r = np.array([7000e3, 0.0, 0.0])
    target_v = np.array([0.0, 7.5e3, 0.0])

    chaser_r = target_r + np.array([0.0, -10.0, 0.0])
    chaser_v = target_v + np.array([0.0, 0.01, 0.0])

    relative_state = compute_relative_state( target_r, target_v, chaser_r, chaser_v)

    print("Relative state check: OK")
    print(f"Relative position (m): {relative_state.position_m}")
    print(f"Relative velocity (m/s): {relative_state.velocity_mps}")
    print(f"Range (m): {relative_state.range_m}")
    print(f"Closing speed (m/s): {relative_state.closing_speed_mps}")

if __name__ == "__main__":
    main()