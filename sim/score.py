"""
sim/score.py — Scoring functions for Bayesian optimiser.
Phase 3.
"""

SCENARIO_GATES = {
    "nominal":        0.95,
    "off_axis":       0.90,
    "high_vel":       0.90,
    "sensor_dropout": 0.90,
    "stuck_closed":   0.80,
    "high_drag":      0.85,
    "combined_stress": 0.80,
}


def score_scenario(metrics: dict, scenario: str) -> float:
    """
    Score one scenario run.

    metrics must contain:
        dock_rate        : float in [0, 1]
        mean_dv_mps      : float
        mean_final_pos_m : float
        mean_steps       : float
    """
    dock_rate = metrics["dock_rate"]
    mean_dv   = metrics["mean_dv_mps"]
    mean_pos  = metrics["mean_final_pos_m"]
    mean_time = metrics["mean_steps"]

    if dock_rate < 0.80:
        return dock_rate * 10.0   # heavily penalise failures

    score = (dock_rate * 100.0
             - mean_dv   *  5.0
             - mean_pos  * 20.0
             - (mean_time / 10000.0) * 2.0)

    gate = SCENARIO_GATES.get(scenario, 0.90)
    if dock_rate > gate + 0.05:
        score += 5.0   # bonus for clear margin

    return score


def aggregate_scores(scenario_scores: dict) -> float:
    """Return mean score across all scenarios."""
    if not scenario_scores:
        return 0.0
    return sum(scenario_scores.values()) / len(scenario_scores)
