#!/usr/bin/env python3
"""Run reusable Basilisk two-spacecraft relative motion demo."""

from __future__ import annotations

from rpod_gnc.bsk.two_spacecraft_runner import run_two_spacecraft_demo


def main() -> None:
    result = run_two_spacecraft_demo(duration_s=60.0, dt_s=1.0)
    final = result.relative_states[-1]

    print("Basilisk two-spacecraft demo: OK")
    print(f"Final relative position (m): {final.position_m}")
    print(f"Final relative velocity (m/s): {final.velocity_mps}")
    print(f"Final range (m): {final.range_m}")
    print(f"Final closing speed (m/s): {final.closing_speed_mps}")
    print(f"Logged samples: {len(result.times_s)}")
    print(f"Initial range (m): {result.ranges_m[0]:.6f}")
    print(f"Minimum range (m): {result.ranges_m.min():.6f}")
    print(f"Maximum range (m): {result.ranges_m.max():.6f}")


if __name__ == "__main__":
    main()