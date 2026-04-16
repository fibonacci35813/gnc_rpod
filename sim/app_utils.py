"""
sim/app_utils.py — Helper functions for the Streamlit demo app.
Phase 5.
"""

import csv
import json
import os
import subprocess
import sys

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIO_RUNNER = os.path.join(_ROOT, "sim", "scenario_runner")
TELEM_CSV       = os.path.join(_ROOT, "sim", "telem.csv")
RESULTS_DIR     = os.path.join(_ROOT, "sim", "results")
BSK_RESULTS_CSV = os.path.join(_ROOT, "bsk", "scenario_results.csv")
COMPARISON_CSV  = os.path.join(_ROOT, "bsk", "comparison_report.csv")
DEFAULT_PARAMS  = os.path.join(_ROOT, "sim", "default_params.json")
BEST_PARAMS     = os.path.join(_ROOT, "sim", "best_params.json")

SCENARIO_NAMES = [
    "nominal", "off_axis", "high_vel", "sensor_dropout",
    "stuck_closed", "stuck_open", "high_drag", "combined_stress",
]

SCENARIO_GATES = {
    "nominal": 0.95, "off_axis": 0.90, "high_vel": 0.90,
    "sensor_dropout": 0.90, "stuck_closed": 0.80,
    "stuck_open": 0.0, "high_drag": 0.85, "combined_stress": 0.80,
}


# ---------------------------------------------------------------------------
# BSK run (via Python CW runner)
# ---------------------------------------------------------------------------

def run_bsk_scenario(scenario: str, params_file: str,
                     enable_vizard: bool = False) -> dict:
    """Run one BSK scenario; return result dict."""
    sys.path.insert(0, _ROOT)
    try:
        from bsk.run_all_scenarios import run_scenario, SCENARIOS
    except (ImportError, OSError) as e:
        return {
            "error": (
                f"{e}\n\n"
                "Build the shared library first with: make shared"
            )
        }

    sc_list = [s for s in SCENARIOS if s["name"] == scenario]
    if not sc_list:
        return {"error": f"Unknown scenario: {scenario}"}

    params_abs = os.path.abspath(params_file) if params_file else None
    try:
        return run_scenario(sc_list[0], params_abs,
                            seed=42, enable_vizard=enable_vizard)
    except RuntimeError as e:
        return {"error": str(e)}


# ---------------------------------------------------------------------------
# C scenario runner
# ---------------------------------------------------------------------------

def run_mc_scenario(scenario: str, params_file: str,
                    n_seeds: int = 20) -> dict:
    """Run C MC runner for one scenario; return parsed metrics."""
    if not os.path.exists(SCENARIO_RUNNER):
        return {"error": "scenario_runner not built — run: make mc-all"}

    cmd = [
        SCENARIO_RUNNER,
        f"--params={params_file}",
        f"--scenario={scenario}",
        f"--n={n_seeds}",
        "--json",
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        for line in res.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                if scenario in obj:
                    return obj[scenario]
            except json.JSONDecodeError:
                continue
    except Exception as e:
        return {"error": str(e)}
    return {"error": "no output from runner"}


# ---------------------------------------------------------------------------
# Telemetry CSV reader
# ---------------------------------------------------------------------------

def read_telem_csv(path: str = TELEM_CSV):
    """Read sim/telem.csv; return dict of lists."""
    if not os.path.exists(path):
        return None
    data = {
        "step": [], "time_s": [], "range_m": [], "delta_v": [],
        "true_pos_x": [], "true_pos_y": [], "true_pos_z": [],
    }
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for k in data:
                    try:
                        data[k].append(float(row[k]))
                    except (KeyError, ValueError):
                        data[k].append(0.0)
    except Exception:
        return None
    return data


# ---------------------------------------------------------------------------
# Comparison CSV reader
# ---------------------------------------------------------------------------

def read_comparison_csv(path: str = COMPARISON_CSV):
    """Read bsk/comparison_report.csv; return list of row dicts."""
    if not os.path.exists(path):
        return None
    rows = []
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(dict(row))
    except Exception:
        return None
    return rows


# ---------------------------------------------------------------------------
# Params helpers
# ---------------------------------------------------------------------------

def params_path(optimised: bool) -> str:
    """Return path to params JSON (default or optimised)."""
    p = BEST_PARAMS if optimised else DEFAULT_PARAMS
    return p if os.path.exists(p) else DEFAULT_PARAMS


def vizard_bin_path(scenario: str) -> str:
    """Return path to vizard .bin file for a scenario."""
    return os.path.join(_ROOT, "bsk", f"vizard_{scenario}.bin")
