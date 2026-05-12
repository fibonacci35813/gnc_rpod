#!/usr/bin/env python3
"""Run reusable Basilisk two-spacecraft relative motion demo."""

from __future__ import annotations

from rpod_gnc.bsk.two_spacecraft_runner import run_two_spacecraft_demo
from rpod_gnc.validation.metrics import compute_range_metrics


def main() -> None:
    result = run_two_spacecraft_demo(duration_s=60.0, dt_s=1.0)
    final_eci = result.relative_states_eci[-1]
    final_lvlh = result.relative_states_lvlh[-1]
    metrics = compute_range_metrics(result.ranges_m)

    print("Basilisk two-spacecraft demo: OK")
    print(f"Final relative position ECI (m): {final_eci.position_m}")
    print(f"Final relative position LVLH (m): {final_lvlh.position_m}")
    print(f"Final relative velocity LVLH (m/s): {final_lvlh.velocity_mps}")
    print(f"Final range (m): {metrics.final_range_m}")
    print(f"Logged samples: {metrics.samples}")
    print(f"Initial range (m): {metrics.initial_range_m:.6f}")
    print(f"Minimum range (m): {metrics.min_range_m:.6f}")
    print(f"Maximum range (m): {metrics.max_range_m:.6f}")


if __name__ == "__main__":
    main()
