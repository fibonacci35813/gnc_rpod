"""Validation metrics for RPOD relative motion simulations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RelativeMotionMetrics:
    initial_range_m: float
    final_range_m: float
    min_range_m: float
    max_range_m: float
    final_closing_speed_mps: float | None
    samples: int


def compute_range_metrics(
    ranges_m: np.ndarray,
    final_closing_speed_mps: float | None = None,
) -> RelativeMotionMetrics:
    """Return basic range metrics for a relative-motion trajectory."""

    ranges_m = np.asarray(ranges_m, dtype=float)
    if ranges_m.size == 0:
        raise ValueError("ranges_m must not be empty.")

    return RelativeMotionMetrics(
        initial_range_m=float(ranges_m[0]),
        final_range_m=float(ranges_m[-1]),
        min_range_m=float(np.min(ranges_m)),
        max_range_m=float(np.max(ranges_m)),
        final_closing_speed_mps=final_closing_speed_mps,
        samples=int(ranges_m.size),
    )
