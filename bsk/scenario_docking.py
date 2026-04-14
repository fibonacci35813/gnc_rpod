"""
scenario_docking.py
===================
Basilisk scenario: autonomous rendezvous & docking simulation.

Architecture
------------
  BSK spacecraft (chaser ECI dynamics)
        ↕  extForceTorque (LVLH → ECI force transform)
  BSK spacecraft (target  ECI dynamics, passive circular orbit)
        ↕  relative state computation
  Python LVLH bridge (compute meas noise, call GncBridge)
  GncBridge → libgnc.so (our P10-compliant C GNC algorithms)
        ↕  telemetry logger + Vizard data recorder

Install
-------
    pip install bsk matplotlib numpy
    # (in gnc_docking/)  make shared

Run
---
    python bsk/scenario_docking.py
    python bsk/scenario_docking.py --vizard   # open Vizard 3D viewer
    python bsk/scenario_docking.py --mc 50    # 50-run Monte Carlo
"""

import argparse
import sys
import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless rendering
import matplotlib.pyplot as plt

# ── Basilisk imports ──────────────────────────────────────────────────────
try:
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
    from Basilisk.simulation import spacecraft, extForceTorque, gravityEffector
except ImportError as exc:
    sys.exit(f"[ERROR] Basilisk not installed: {exc}\n  pip install bsk")

# ── Our GNC bridge ────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bsk.gnc_ctypes import GncBridge

# ── Constants ─────────────────────────────────────────────────────────────
MU_EARTH    = 3.986004418e14   # m^3/s^2
R_EARTH     = 6.371e6          # m
ISS_ALT     = 4.0e5            # m
SMA         = R_EARTH + ISS_ALT
INC_RAD     = math.radians(51.6)
CHASER_MASS = 500.0            # kg
SIM_DT_S    = 1.0              # control step (s)
SIM_MAX_S   = 12000.0          # hard sim ceiling (s)

# Sensor noise: sigma = max(FLOOR, K * range)
SENSOR_K     = 0.003
SENSOR_FLOOR = 0.002


# ---------------------------------------------------------------------------
# Coordinate utilities
# ---------------------------------------------------------------------------

def as_vector3(value) -> np.ndarray:
    """Convert Basilisk vector-like values to a flat NumPy 3-vector."""
    return np.asarray(value, dtype=float).reshape(3)


def eci_to_lvlh(r_tgt_eci: np.ndarray, v_tgt_eci: np.ndarray,
                r_chs_eci: np.ndarray, v_chs_eci: np.ndarray):
    """
    Compute LVLH relative position and velocity of chaser w.r.t. target.

    Returns
    -------
    pos_lvlh : np.ndarray [3]  — relative position in LVLH (m)
    vel_lvlh : np.ndarray [3]  — relative velocity in LVLH (m/s)
    R_eci2lvlh : np.ndarray [3,3] — rotation matrix ECI → LVLH
    """
    r_hat = r_tgt_eci / np.linalg.norm(r_tgt_eci)          # radial
    h_vec = np.cross(r_tgt_eci, v_tgt_eci)
    h_hat = h_vec / np.linalg.norm(h_vec)                  # orbit normal
    y_hat = np.cross(h_hat, r_hat)                         # along-track
    # LVLH frame: x=radial, y=along-track, z=cross-track (= -h_hat in std)
    R = np.vstack([r_hat, y_hat, h_hat])                   # rows = frame axes

    dr_eci = r_chs_eci - r_tgt_eci
    dv_eci = v_chs_eci - v_tgt_eci

    omega = np.linalg.norm(h_vec) / (np.linalg.norm(r_tgt_eci) ** 2)
    omega_vec = h_hat * omega
    pos_lvlh = R @ dr_eci
    vel_lvlh = R @ (dv_eci - np.cross(omega_vec, dr_eci))
    return pos_lvlh, vel_lvlh, R


def lvlh_force_to_eci(force_lvlh: np.ndarray, R_eci2lvlh: np.ndarray) -> np.ndarray:
    """Transform a force vector from LVLH to ECI."""
    return R_eci2lvlh.T @ force_lvlh


def range_dependent_sigma(range_m: float) -> float:
    """LIDAR noise model: sigma = max(FLOOR, K * range)."""
    return max(SENSOR_FLOOR, SENSOR_K * abs(range_m))


def add_measurement_noise(pos_true: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Add range-proportional Gaussian noise to position measurement."""
    r = np.linalg.norm(pos_true)
    sig = range_dependent_sigma(r)
    return pos_true + rng.normal(0.0, sig, 3)


def make_earth_gravity_body():
    """Create the point-mass Earth gravity body expected by Basilisk 2.x."""
    earth = gravityEffector.GravBodyData()
    earth.mu = MU_EARTH
    earth.radEquator = R_EARTH
    earth.isCentralBody = True
    earth.planetName = "earth"
    return earth


# ---------------------------------------------------------------------------
# Single simulation run
# ---------------------------------------------------------------------------

def run_sim(seed: int = 42, show_progress: bool = True) -> dict:
    """
    Run one docking simulation. Returns a result dict.
    """
    rng = np.random.default_rng(seed)

    # ── BSK simulation setup ─────────────────────────────────────────────
    scSim = SimulationBaseClass.SimBaseClass()
    dt_ns = macros.sec2nano(SIM_DT_S)
    earth_gravity = make_earth_gravity_body()

    dynProc = scSim.CreateNewProcess("dynProcess", 10)
    dynTask = scSim.CreateNewTask("dynTask", dt_ns)
    dynProc.addTask(dynTask)

    # ── Target spacecraft (passive circular orbit) ────────────────────────
    tgt = spacecraft.Spacecraft()
    tgt.ModelTag = "target"
    tgt.gravField.gravBodies = gravityEffector.GravBodyVector([earth_gravity])

    oe_tgt = orbitalMotion.ClassicElements()
    oe_tgt.a     = SMA
    oe_tgt.e     = 0.0
    oe_tgt.i     = INC_RAD
    oe_tgt.Omega = math.radians(0.0)
    oe_tgt.omega = math.radians(0.0)
    oe_tgt.f     = math.radians(0.0)

    r_tgt0, v_tgt0 = orbitalMotion.elem2rv(MU_EARTH, oe_tgt)
    tgt.hub.r_CN_NInit = r_tgt0.tolist()
    tgt.hub.v_CN_NInit = v_tgt0.tolist()
    tgt.hub.mHub = 100000.0   # 100 t station
    scSim.AddModelToTask("dynTask", tgt, None, 20)

    # ── Chaser spacecraft (starts 200 m behind in along-track) ────────────
    chs = spacecraft.Spacecraft()
    chs.ModelTag = "chaser"
    chs.gravField.gravBodies = gravityEffector.GravBodyVector([earth_gravity])

    # Initial LVLH offset: [x=0, y=+200, z=0] m, zero relative velocity
    r_tgt0_np = as_vector3(r_tgt0)
    v_tgt0_np = as_vector3(v_tgt0)

    # Build LVLH frame at t=0 and transform initial LVLH offset to ECI
    r_hat0 = r_tgt0_np / np.linalg.norm(r_tgt0_np)
    h_vec0 = np.cross(r_tgt0_np, v_tgt0_np)
    h_hat0 = h_vec0 / np.linalg.norm(h_vec0)
    y_hat0 = np.cross(h_hat0, r_hat0)
    R0_inv  = np.vstack([r_hat0, y_hat0, h_hat0]).T   # LVLH → ECI

    lvlh_offset = np.array([0.0, 200.0, 0.0])
    r_chs0_np   = r_tgt0_np + R0_inv @ lvlh_offset
    v_chs0_np   = v_tgt0_np.copy()    # same orbital velocity = relative rest

    chs.hub.r_CN_NInit = r_chs0_np.tolist()
    chs.hub.v_CN_NInit = v_chs0_np.tolist()
    chs.hub.mHub       = CHASER_MASS
    scSim.AddModelToTask("dynTask", chs, None, 21)

    # ── External force module on chaser ───────────────────────────────────
    extFT = extForceTorque.ExtForceTorque()
    extFT.ModelTag = "extForceTorque"
    chs.addDynamicEffector(extFT)
    scSim.AddModelToTask("dynTask", extFT, None, 22)

    # ── Data loggers ──────────────────────────────────────────────────────
    tgt_log = tgt.scStateOutMsg.recorder()
    chs_log = chs.scStateOutMsg.recorder()
    scSim.AddModelToTask("dynTask", tgt_log)
    scSim.AddModelToTask("dynTask", chs_log)

    scSim.InitializeSimulation()

    # ── GNC bridge initialise ─────────────────────────────────────────────
    gnc = GncBridge(sma_m=SMA, mass_kg=CHASER_MASS)
    gnc.init(pos0=lvlh_offset.tolist(), vel0=[0.0, 0.0, 0.0])

    # ── Telemetry accumulators ─────────────────────────────────────────────
    telem = {
        "time_s":    [],
        "range_m":   [],
        "phase":     [],
        "dv_mps":    [],
        "pos_lvlh":  [],
        "vel_lvlh":  [],
        "force":     [],
    }

    # ── Main simulation loop ───────────────────────────────────────────────
    max_steps = int(SIM_MAX_S / SIM_DT_S)
    docked    = False
    step      = 0

    for step in range(max_steps):
        t_s = step * SIM_DT_S

        # Advance BSK dynamics one step
        scSim.ConfigureStopTime(macros.sec2nano(t_s + SIM_DT_S))
        scSim.ExecuteSimulation()

        # Read current ECI states
        r_tgt = as_vector3(tgt.hub.r_CN_NInit if step == 0
                           else tgt_log.r_BN_N[-1])
        v_tgt = as_vector3(tgt.hub.v_CN_NInit if step == 0
                           else tgt_log.v_BN_N[-1])
        r_chs = as_vector3(chs.hub.r_CN_NInit if step == 0
                           else chs_log.r_BN_N[-1])
        v_chs = as_vector3(chs.hub.v_CN_NInit if step == 0
                           else chs_log.v_BN_N[-1])

        # Compute true LVLH relative state
        pos_lvlh_true, vel_lvlh_true, R_eci2lvlh = eci_to_lvlh(
            r_tgt, v_tgt, r_chs, v_chs)

        # Sensor measurement with range-proportional noise
        meas = add_measurement_noise(pos_lvlh_true, rng)
        sigma = range_dependent_sigma(np.linalg.norm(pos_lvlh_true))

        # GNC step
        force_lvlh, range_m, dock_flag = gnc.step(meas, sigma, SIM_DT_S)

        if dock_flag:
            docked = True
            if show_progress:
                st = gnc.get_state()
                print(f"\n[{step:5d}] DOCKED  range={range_m:.4f} m  "
                      f"dv={st['dv_mps']:.3f} m/s")
            break

        # Transform LVLH force to ECI and apply to chaser
        force_eci = lvlh_force_to_eci(np.array(force_lvlh), R_eci2lvlh)
        extFT.extForce_N = force_eci.tolist()

        # Telemetry
        st = gnc.get_state()
        telem["time_s"].append(t_s)
        telem["range_m"].append(range_m)
        telem["phase"].append(gnc.phase)
        telem["dv_mps"].append(st["dv_mps"])
        telem["pos_lvlh"].append(st["pos_lvlh"][:])
        telem["vel_lvlh"].append(st["vel_lvlh"][:])
        telem["force"].append(force_lvlh[:])

        if show_progress and (step % 100 == 0):
            print(f"[{step:5d}] t={t_s:.0f}s  range={range_m:.3f}m  "
                  f"phase={gnc.phase}  dv={st['dv_mps']:.3f}m/s")

    st = gnc.get_state()
    return {
        "docked":      docked,
        "steps":       step,
        "time_s":      step * SIM_DT_S,
        "range_m":     range_m,
        "dv_mps":      st["dv_mps"],
        "prop_kg":     st["prop_kg"],
        "telem":       telem,
        "seed":        seed,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(result: dict, out_dir: str = "bsk/plots") -> None:
    os.makedirs(out_dir, exist_ok=True)
    t  = result["telem"]["time_s"]
    r  = result["telem"]["range_m"]
    ph = result["telem"]["phase"]
    dv = result["telem"]["dv_mps"]

    pos = np.array(result["telem"]["pos_lvlh"])   # N×3
    vel = np.array(result["telem"]["vel_lvlh"])

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("BSK Docking Simulation — GNC Performance", fontsize=13)

    # Range vs time
    ax = axes[0, 0]
    ax.semilogy(t, r, "steelblue", lw=1.5)
    ax.axhline(0.05, color="red", ls="--", lw=1, label="5 cm tolerance")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Range (m)"); ax.legend()
    ax.set_title("Range to Docking Port")

    # Phase vs time
    ax = axes[0, 1]
    ax.step(t, ph, "darkorange", where="post", lw=1.5)
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(["Hold", "Mid", "Close", "Terminal"])
    ax.set_xlabel("Time (s)"); ax.set_title("Guidance Phase")

    # ΔV vs time
    ax = axes[1, 0]
    ax.plot(t, dv, "green", lw=1.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("ΔV (m/s)")
    ax.set_title("Cumulative ΔV")

    # LVLH trajectory (x-y plane)
    ax = axes[1, 1]
    ax.plot(pos[:, 1], pos[:, 0], "steelblue", lw=1)
    ax.plot(0, 0, "r*", ms=12, label="Docking port")
    ax.set_xlabel("Along-track y (m)"); ax.set_ylabel("Radial x (m)")
    ax.legend(); ax.set_title("LVLH Trajectory")
    ax.invert_xaxis()

    plt.tight_layout()
    path = os.path.join(out_dir, "docking_results.pdf")
    fig.savefig(path, dpi=150)
    print(f"[PLOT] Saved → {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Monte Carlo
# ---------------------------------------------------------------------------

def run_monte_carlo(n_runs: int = 50) -> None:
    print(f"\n=== Monte Carlo: {n_runs} runs ===")
    results = []
    for seed in range(n_runs):
        r = run_sim(seed=seed, show_progress=False)
        results.append(r)
        status = "DOCK" if r["docked"] else "FAIL"
        print(f"  seed={seed:3d}  {status}  steps={r['steps']:5d}  "
              f"range={r['range_m']:.4f}m  dv={r['dv_mps']:.3f}m/s")

    docked_n   = sum(1 for r in results if r["docked"])
    success_p  = 100.0 * docked_n / n_runs
    dv_vals    = [r["dv_mps"]  for r in results if r["docked"]]
    rng_vals   = [r["range_m"] for r in results if r["docked"]]

    print(f"\n--- MC Summary ({n_runs} runs) ---")
    print(f"  P(dock)      = {success_p:.1f}%  ({docked_n}/{n_runs})")
    if dv_vals:
        print(f"  ΔV mean/max  = {np.mean(dv_vals):.3f} / {np.max(dv_vals):.3f} m/s")
        print(f"  range mean   = {np.mean(rng_vals)*100:.1f} cm")
    print("----------------------------------")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BSK docking scenario")
    parser.add_argument("--mc",     type=int, default=0,
                        help="Run Monte Carlo with N seeds (0 = single run)")
    parser.add_argument("--seed",   type=int, default=42,
                        help="RNG seed for single run")
    parser.add_argument("--vizard", action="store_true",
                        help="Launch Vizard 3D visualiser after run")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip matplotlib output")
    args = parser.parse_args()

    if args.mc > 0:
        run_monte_carlo(args.mc)
    else:
        print(f"=== Single run (seed={args.seed}) ===")
        result = run_sim(seed=args.seed, show_progress=True)

        print(f"\n{'='*44}")
        print(f"  Docked        : {'YES' if result['docked'] else 'NO'}")
        print(f"  Steps         : {result['steps']}")
        print(f"  Mission time  : {result['time_s']:.0f} s")
        print(f"  Final range   : {result['range_m']*100:.1f} cm")
        print(f"  Total ΔV      : {result['dv_mps']:.3f} m/s")
        print(f"  Propellant    : {result['prop_kg']:.3f} kg")
        print(f"{'='*44}\n")

        if not args.no_plot:
            plot_results(result)
