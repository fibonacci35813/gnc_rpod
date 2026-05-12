#!/usr/bin/env python3

"""Run the first closed-loop Basilisk SIL demo."""

from __future__ import annotations

from rpod_gnc.validation.metrics import compute_range_metrics


def main() -> None:
    try:
        from rpod_gnc.bsk.closed_loop_runner import ClosedLoopConfig, run_closed_loop_demo
    except ImportError as exc:
        raise SystemExit(
            "Basilisk closed-loop runner import failed. Install or activate Basilisk to run this demo."
        ) from exc

    result = run_closed_loop_demo(ClosedLoopConfig())
    metrics = compute_range_metrics(result.ranges_m)

    print("Closed-loop Basilisk SIL demo: OK")
    print(f"Initial range (m): {metrics.initial_range_m:.6f}")
    print(f"Final range (m): {metrics.final_range_m:.6f}")
    print(f"Minimum range (m): {metrics.min_range_m:.6f}")
    print(f"Maximum range (m): {metrics.max_range_m:.6f}")
    print(f"Total delta-v (m/s): {result.total_delta_v_mps:.6f}")
    print(f"Final relative position LVLH (m): {result.relative_positions_lvlh_m[-1]}")
    print(f"Final relative velocity LVLH (m/s): {result.relative_velocities_lvlh_mps[-1]}")


if __name__ == "__main__":
    main()
