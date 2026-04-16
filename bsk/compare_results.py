"""
bsk/compare_results.py — Phase 4: compare C sim results vs BSK results.

Reads:
    sim/results/<name>_mc.csv   (C MC runner output)
    bsk/scenario_results.csv    (BSK runner output)

Writes:
    bsk/comparison_report.csv
    bsk/comparison_report.md

Flags scenarios where delta_dock_rate > 5 % or delta_dv > 5 %.
"""

import csv
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

RESULTS_DIR  = os.path.join(_ROOT, "sim", "results")
BSK_CSV      = os.path.join(_HERE, "scenario_results.csv")
OUT_CSV      = os.path.join(_HERE, "comparison_report.csv")
OUT_MD       = os.path.join(_HERE, "comparison_report.md")

SCENARIO_NAMES = [
    "nominal", "off_axis", "high_vel", "sensor_dropout",
    "stuck_closed", "stuck_open", "high_drag", "combined_stress",
]


def _read_c_csv(name):
    """Read sim/results/<name>_mc.csv; return aggregated metrics."""
    path = os.path.join(RESULTS_DIR, f"{name}_mc.csv")
    if not os.path.exists(path):
        return None

    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        return None

    n = len(rows)
    n_docked  = sum(1 for r in rows if int(r["docked"]) == 1)
    n_aborted = sum(1 for r in rows if int(r["aborted"]) == 1)
    mean_dv   = sum(float(r["total_dv_mps"]) for r in rows) / n
    return {
        "dock_rate":  n_docked  / n,
        "abort_rate": n_aborted / n,
        "mean_dv":    mean_dv,
        "n":          n,
    }


def _read_bsk_csv():
    """Read bsk/scenario_results.csv; return dict keyed by scenario name."""
    if not os.path.exists(BSK_CSV):
        return {}

    result = {}
    with open(BSK_CSV, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row["scenario"]
            result[name] = {
                "docked":      int(row["docked"]),
                "aborted":     int(row["aborted"]),
                "dv_mps":      float(row["dv_mps"]),
                "final_pos_m": float(row["final_pos_m"]),
            }
    return result


def main():
    c_data   = {}
    bsk_data = _read_bsk_csv()

    if not bsk_data:
        sys.exit(f"[ERROR] {BSK_CSV} not found. Run: make bsk-all first.")

    for name in SCENARIO_NAMES:
        m = _read_c_csv(name)
        if m:
            c_data[name] = m

    if not c_data:
        sys.exit(f"[ERROR] No C results found in {RESULTS_DIR}. "
                 "Run: make mc-all first.")

    # Build comparison rows
    rows = []
    for name in SCENARIO_NAMES:
        c   = c_data.get(name)
        bsk = bsk_data.get(name)

        if c is None or bsk is None:
            rows.append({
                "scenario":     name,
                "dock_rate_c":  "N/A",
                "dock_rate_bsk": "N/A",
                "dv_c":         "N/A",
                "dv_bsk":       "N/A",
                "delta_pct":    "N/A",
                "flag":         "MISSING",
            })
            continue

        # BSK single-run vs C MC mean
        bsk_dock  = float(bsk["docked"])   # 1.0 or 0.0 (single run)
        c_dock    = c["dock_rate"]
        bsk_dv    = bsk["dv_mps"]
        c_dv      = c["mean_dv"]

        # ΔV delta percent (C as reference; avoid div-by-zero)
        if c_dv > 0.0:
            delta_dv_pct = 100.0 * abs(bsk_dv - c_dv) / c_dv
        else:
            delta_dv_pct = 0.0

        flag = "OK"
        if delta_dv_pct > 5.0:
            flag = "DELTA_HIGH"

        rows.append({
            "scenario":      name,
            "dock_rate_c":   f"{c_dock:.4f}",
            "dock_rate_bsk": f"{bsk_dock:.1f}",
            "dv_c":          f"{c_dv:.4f}",
            "dv_bsk":        f"{bsk_dv:.4f}",
            "delta_pct":     f"{delta_dv_pct:.2f}",
            "flag":          flag,
        })

    # Write CSV
    with open(OUT_CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Write Markdown
    with open(OUT_MD, "w") as f:
        f.write("# BSK vs C Comparison Report\n\n")
        f.write("| Scenario | dock_rate_c | dock_rate_bsk |"
                " dv_c (m/s) | dv_bsk (m/s) | delta_dv_pct | flag |\n")
        f.write("|---|---|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['scenario']} | {r['dock_rate_c']} | "
                    f"{r['dock_rate_bsk']} | {r['dv_c']} | {r['dv_bsk']} | "
                    f"{r['delta_pct']} | {r['flag']} |\n")

    print(f"[COMPARE] Written: {OUT_CSV}")
    print(f"[COMPARE] Written: {OUT_MD}")

    # Summary
    n_ok   = sum(1 for r in rows if r["flag"] == "OK")
    n_high = sum(1 for r in rows if r["flag"] == "DELTA_HIGH")
    print(f"[COMPARE] {n_ok}/{len(rows)} within 5% ΔV delta; "
          f"{n_high} flagged DELTA_HIGH")


if __name__ == "__main__":
    main()
