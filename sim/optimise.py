"""
sim/optimise.py — Bayesian parameter optimisation with Optuna TPE.
Phase 3.

Usage:
    python sim/optimise.py --trials=200        # fast (default)
    python sim/optimise.py --trials=1000       # thorough
    python sim/optimise.py --resume            # continue existing study
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

import optuna
from params_utils import DEFAULT_PARAMS, write_params_json
from score import score_scenario, aggregate_scores

# ---------------------------------------------------------------------------
# Search space
# ---------------------------------------------------------------------------
PARAM_SPACE = {
    "kp_x":            (0.10, 2.00),
    "kp_y":            (0.05, 1.50),
    "kp_z":            (0.10, 2.00),
    "kd_x":            (8.0,  80.0),
    "kd_y":            (5.0,  60.0),
    "kd_z":            (8.0,  80.0),
    "kp_terminal_x":   (0.50, 4.00),
    "kp_terminal_y":   (0.50, 4.00),
    "kp_terminal_z":   (0.50, 4.00),
    "kd_terminal_x":   (20.0, 100.0),
    "kd_terminal_y":   (20.0, 100.0),
    "kd_terminal_z":   (20.0, 100.0),
    "K_V":             (0.003, 0.030),
    "v_phase_0_max":   (0.30,  2.00),
    "v_phase_1_max":   (0.10,  0.80),
    "v_phase_2_max":   (0.02,  0.15),
    "v_phase_3_max":   (0.005, 0.08),
    "mib_terminal_ns": (0.001, 0.020),
}

# Scenarios included in objective (not stuck_open — it gates on abort_rate).
OPTIMISE_SCENARIOS = [
    "nominal", "off_axis", "high_vel", "sensor_dropout", "high_drag",
]

SCENARIO_RUNNER = "./sim/scenario_runner"
RESULTS_DIR = "sim/results"
BEST_PARAMS_PATH = "sim/best_params.json"
STUDY_DB = "sqlite:///sim/optuna.db"
STUDY_NAME = "gnc_docking_v1"


# ---------------------------------------------------------------------------
# Parse --json output from scenario_runner
# ---------------------------------------------------------------------------
def _run_scenario_json(params_path: str, scenario: str, n_seeds: int) -> dict:
    """Run scenario_runner for one scenario; return parsed metrics dict.

    The runner emits one JSON object per line (NDJSON format) when --json
    is passed.  Each line looks like:
        {"nominal":{"dock_rate":1.0000,...}}
    """
    cmd = [
        SCENARIO_RUNNER,
        f"--params={params_path}",
        f"--scenario={scenario}",
        f"--n={n_seeds}",
        "--json",
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=300
        )
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                if scenario in obj:
                    return obj[scenario]
            except json.JSONDecodeError:
                continue
    except (subprocess.TimeoutExpired, Exception):
        pass
    return {}


# ---------------------------------------------------------------------------
# Objective
# ---------------------------------------------------------------------------
def objective(trial: optuna.Trial) -> float:
    # Sample parameters
    params = dict(DEFAULT_PARAMS)
    for name, (lo, hi) in PARAM_SPACE.items():
        params[name] = trial.suggest_float(name, lo, hi)

    # Write to a temp file
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", dir="sim", delete=False
    ) as f:
        tmp_path = f.name
    try:
        write_params_json(params, tmp_path)

        scenario_scores = {}
        for sc in OPTIMISE_SCENARIOS:
            metrics = _run_scenario_json(tmp_path, sc, n_seeds=10)
            if not metrics:
                return -100.0   # runner failed: penalise hard

            score = score_scenario(
                {
                    "dock_rate":        metrics.get("dock_rate", 0.0),
                    "mean_dv_mps":      metrics.get("mean_dv_mps", 999.0),
                    "mean_final_pos_m": metrics.get("mean_final_pos_m", 999.0),
                    "mean_steps":       metrics.get("mean_steps", 10000.0),
                },
                sc,
            )
            scenario_scores[sc] = score

        return aggregate_scores(scenario_scores)

    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="GNC Bayesian optimiser")
    parser.add_argument("--trials", type=int, default=200,
                        help="Number of Optuna trials (default: 200)")
    parser.add_argument("--resume", action="store_true",
                        help="Continue existing study from DB")
    args = parser.parse_args()

    os.makedirs("sim", exist_ok=True)

    # Silence Optuna info logs unless debugging
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    load_existing = True  # always load if exists; prevents crash on re-run

    study = optuna.create_study(
        direction="maximize",
        storage=STUDY_DB,
        study_name=STUDY_NAME,
        load_if_exists=load_existing,
        sampler=optuna.samplers.TPESampler(seed=42),
    )

    # Seed with the known-good baseline (only on first run)
    if not load_existing and len(study.trials) == 0:
        baseline = {k: DEFAULT_PARAMS[k] for k in PARAM_SPACE}
        study.enqueue_trial(baseline)
        print("[OPTIMISE] Seeded with baseline params")

    n_trials = args.trials
    print(f"[OPTIMISE] Running {n_trials} trials (study: {STUDY_NAME})")

    study.optimize(
        objective,
        n_trials=n_trials,
        n_jobs=1,           # single job: avoids temp-file race conditions
        show_progress_bar=True,
    )

    best = dict(DEFAULT_PARAMS)
    best.update(study.best_params)
    write_params_json(best, BEST_PARAMS_PATH)

    print(f"\n[OPTIMISE] Best score : {study.best_value:.4f}")
    print(f"[OPTIMISE] Best params written to {BEST_PARAMS_PATH}")

    # Quick summary of improvement
    baseline_score = None
    for t in study.trials:
        if t.number == 0:
            baseline_score = t.value
            break
    if baseline_score is not None:
        delta = study.best_value - baseline_score
        print(f"[OPTIMISE] Improvement over baseline: {delta:+.4f}")


if __name__ == "__main__":
    # Change to project root so relative paths work from any CWD
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(project_root)
    sys.path.insert(0, os.path.join(project_root, "sim"))
    main()
