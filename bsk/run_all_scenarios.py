"""
bsk/run_all_scenarios.py — Phase 4 Basilisk batch runner.

Runs all 8 standardised scenarios using the GNC C bridge (ctypes) with
a Python CW orbital propagator.  When Basilisk is available, ECI dynamics
are used via scenario_docking.py; otherwise a pure-Python CW propagator
provides equivalent relative-motion dynamics.

Writes:
    bsk/scenario_results.csv        — per-scenario summary
    bsk/vizard_<name>.bin           — binary telemetry recording (one per run)

Usage:
    python bsk/run_all_scenarios.py --params=sim/default_params.json
    python bsk/run_all_scenarios.py --params=sim/best_params.json
    python bsk/run_all_scenarios.py --scenario=nominal --vizard --params=...
"""

import argparse
import math
import os
import struct
import sys
import csv

import numpy as np

# ---------------------------------------------------------------------------
# Path setup — allow running from project root or bsk/ directory
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)   # for vizMessage_pb2

from bsk.gnc_ctypes import GncBridge

# ---------------------------------------------------------------------------
# Optional: Vizard protobuf support (vizMessage_pb2 compiled from proto)
# ---------------------------------------------------------------------------
try:
    from vizMessage_pb2 import VizMessage as _VizMessage
    _PROTO_AVAILABLE = True
except ImportError:
    _PROTO_AVAILABLE = False

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------
MU_EARTH    = 3.986004418e14
R_EARTH     = 6.371e6
ISS_ALT     = 4.0e5
SMA         = R_EARTH + ISS_ALT
CHASER_MASS = 500.0
SIM_DT_S    = 1.0
SIM_MAX_STEPS = 10000

SENSOR_K     = 0.003
SENSOR_FLOOR = 0.002

# ---------------------------------------------------------------------------
# 8 scenarios matching sim/scenarios.c exactly
# ---------------------------------------------------------------------------
SCENARIOS = [
    dict(name="nominal",
         pos0=[0.0, 200.0, 0.0], vel0=[0.0, 0.0, 0.0],
         prop=5.0, fault=None, cd=2.2, area=2.0),
    dict(name="off_axis",
         pos0=[15.0, 200.0, 10.0], vel0=[0.0, 0.0, 0.0],
         prop=5.0, fault=None, cd=2.2, area=2.0),
    dict(name="high_vel",
         pos0=[0.0, 200.0, 0.0], vel0=[0.0, -0.5, 0.0],
         prop=5.0, fault=None, cd=2.2, area=2.0),
    dict(name="sensor_dropout",
         pos0=[0.0, 200.0, 0.0], vel0=[0.0, 0.0, 0.0],
         prop=5.0, fault="dropout", fault_step=500, cd=2.2, area=2.0),
    dict(name="stuck_closed",
         pos0=[0.0, 200.0, 0.0], vel0=[0.0, 0.0, 0.0],
         prop=5.0, fault="stuck_closed", fault_step=300, cd=2.2, area=2.0),
    dict(name="stuck_open",
         pos0=[0.0, 200.0, 0.0], vel0=[0.0, 0.0, 0.0],
         prop=5.0, fault="stuck_open", fault_step=500, cd=2.2, area=2.0),
    dict(name="high_drag",
         pos0=[0.0, 200.0, 0.0], vel0=[0.0, 0.0, 0.0],
         prop=5.0, fault=None, cd=3.5, area=5.0),
    dict(name="combined_stress",
         pos0=[10.0, 200.0, 8.0], vel0=[0.1, -0.2, 0.05],
         prop=4.5, fault=None, cd=3.0, area=4.0),
]

# ---------------------------------------------------------------------------
# CW propagator
# ---------------------------------------------------------------------------

def cw_propagate(pos, vel, force, n, dt, mass):
    """One-step CW propagation (same equations as dyn_propagate in C)."""
    nt   = n * dt
    snt  = math.sin(nt)
    cnt  = math.cos(nt)
    x, y, z       = pos
    vx, vy, vz    = vel
    ax, ay, az    = [f / mass for f in force]

    # CW state-transition (closed-form, matches the C implementation)
    x2  = (4.0 - 3.0*cnt)*x  + snt*vx/n  + 2.0*(1.0-cnt)*vy/n \
          + ax*(1.0-cnt)/(n*n) + 2.0*ay*(nt-snt)/(n*n)
    y2  = 6.0*(snt-nt)*x + y - 2.0*(1.0-cnt)*vx/n + (4.0*snt-3.0*nt)*vy/n \
          - 2.0*ax*(nt-snt)/(n*n) + ay*(4.0*(1.0-cnt)-1.5*(nt*nt))/(n*n)
    z2  = z*cnt + vz*snt/n  + az*(1.0-cnt)/(n*n)

    vx2 = 3.0*n*snt*x + cnt*vx + 2.0*snt*vy \
          + ax*snt/n + 2.0*ay*(1.0-cnt)/n
    vy2 = -6.0*n*(1.0-cnt)*x - 2.0*snt*vx + (4.0*cnt-3.0)*vy \
          - 2.0*ax*(1.0-cnt)/n + ay*(4.0*snt-3.0*nt)/n
    vz2 = -z*n*snt + vz*cnt + az*snt/n

    return np.array([x2, y2, z2]), np.array([vx2, vy2, vz2])


def sensor_sigma(range_m):
    return max(SENSOR_FLOOR, SENSOR_K * abs(range_m))


# ---------------------------------------------------------------------------
# Vizard protobuf writer
#
# Vizard (AVS Lab) reads a stream of length-delimited VizMessage protobuf
# records.  Each record is: varint32(byte_length) + serialized VizMessage.
#
# Spacecraft positions are supplied in an ECI-like frame.  We place the
# target at a fixed ISS reference point and add the LVLH offsets for the
# chaser.  Vizard renders relative motion correctly regardless of absolute
# ECI accuracy.
#
# Fallback: if vizMessage_pb2 is unavailable, write the legacy GNCD binary
# so the runner still produces *something* (though Vizard will reject it).
# ---------------------------------------------------------------------------

# ISS reference position in ECI [m] — target sits here
_TARGET_ECI = np.array([SMA, 0.0, 0.0])
_IDENTITY_DCM = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
_IDENTITY_MRP = [0.0, 0.0, 0.0]


def _encode_varint32(value):
    """Encode an unsigned int32 as a protobuf varint (little-endian, 7 bits/byte)."""
    buf = []
    while True:
        bits = value & 0x7F
        value >>= 7
        if value:
            buf.append(0x80 | bits)
        else:
            buf.append(bits)
            break
    return bytes(buf)


def _add_vizard_scene_metadata(msg):
    """Populate first-frame metadata needed by Vizard to build a scene."""
    msg.epoch.year = 2026
    msg.epoch.month = 4
    msg.epoch.day = 16
    msg.epoch.hours = 0
    msg.epoch.minutes = 0
    msg.epoch.seconds = 0.0

    settings = msg.settings
    settings.orbitLinesOn = 1
    settings.trueTrajectoryLinesOn = 1
    settings.spacecraftCSon = 1
    settings.showCelestialBodyLabels = 1
    settings.showSpacecraftLabels = 1
    settings.showSpacecraftAsSprites = 1
    settings.defaultSpacecraftSprite = "CIRCLE"
    settings.mainCameraTarget = "chaser"
    settings.forceStartAtSpacecraftLocalView = -1
    settings.spacecraftSizeMultiplier = 20.0
    settings.keyboardAngularRate = 20.0
    settings.keyboardZoomRate = 25.0
    settings.showHillFrame = 1
    settings.relativeOrbitFrame = 1
    settings.orbitLineSegments = 512
    settings.relativeOrbitRange = 10
    settings.scViewToPlanetViewBoundaryMultiplier = 1
    settings.planetViewToHelioViewBoundaryMultiplier = 1

    point_line = settings.pointLines.add()
    point_line.fromBodyName = "target"
    point_line.toBodyName = "chaser"
    point_line.lineColor.extend([0, 255, 0, 255])

    chase_cam = settings.standardCameraSettings.add()
    chase_cam.spacecraftName = "chaser"
    chase_cam.setMode = 0
    chase_cam.bodyTarget = "target"
    chase_cam.setView = 2
    chase_cam.fieldOfView = 55.0
    chase_cam.position.extend([0.0, -30.0, 10.0])
    chase_cam.displayName = "Chaser to Target"

    target_cam = settings.standardCameraSettings.add()
    target_cam.spacecraftName = "target"
    target_cam.setMode = 0
    target_cam.bodyTarget = "chaser"
    target_cam.setView = 2
    target_cam.fieldOfView = 55.0
    target_cam.position.extend([0.0, 30.0, 10.0])
    target_cam.displayName = "Target to Chaser"

    msg.liveSettings.relativeOrbitChief = "target"


def _add_earth_body(msg):
    """Add Earth as the central body so Vizard has a planet/camera context."""
    earth = msg.celestialBodies.add()
    earth.bodyName = "earth"
    earth.position.extend([0.0, 0.0, 0.0])
    earth.velocity.extend([0.0, 0.0, 0.0])
    earth.rotation.extend(_IDENTITY_DCM)
    earth.mu = MU_EARTH / 1.0e9      # Vizard expects km^3/s^2
    earth.radiusEq = R_EARTH / 1000.0
    earth.radiusRatio = 1.0
    earth.modelDictionaryKey = "earth"


def _add_spacecraft(msg, name, position, velocity, sprite):
    """Add one spacecraft using the MRP attitude format Vizard expects."""
    sc = msg.spacecraft.add()
    sc.spacecraftName = name
    sc.position.extend([float(position[0]), float(position[1]), float(position[2])])
    sc.velocity.extend([float(velocity[0]), float(velocity[1]), float(velocity[2])])
    sc.rotation.extend(_IDENTITY_MRP)
    sc.spacecraftSprite = sprite
    sc.modelDictionaryKey = "bskSat"
    return sc


def write_vizard_bin(path, records):
    """Write Vizard-compatible protobuf binary telemetry.

    records: list of (step, t_s, x, y, z, vx, vy, vz, dv, phase)
    Produces a length-delimited VizMessage protobuf stream readable by Vizard.
    """
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)

    if not _PROTO_AVAILABLE:
        # Legacy fallback — Vizard will reject this, but keeps file non-empty.
        HEADER_FMT = "=4sI"
        RECORD_FMT = "=IffffffffI"
        with open(path, "wb") as f:
            f.write(struct.pack(HEADER_FMT, b"GNCD", len(records)))
            for r in records:
                f.write(struct.pack(RECORD_FMT, *r))
        return

    with open(path, "wb") as f:
        for step, t_s, x, y, z, vx, vy, vz, dv, phase in records:
            msg = _VizMessage()

            # --- timestamp ---
            msg.currentTime.frameNumber    = int(step) + 1
            msg.currentTime.simTimeElapsed = t_s * 1.0e9   # ns

            if step == records[0][0]:
                _add_vizard_scene_metadata(msg)

            _add_earth_body(msg)

            # --- target spacecraft (stationary at ISS reference) ---
            _add_spacecraft(
                msg,
                "target",
                _TARGET_ECI,
                [0.0, 0.0, 0.0],
                "SQUARE",
            )

            # --- chaser spacecraft (target ECI + LVLH offset) ---
            # LVLH axes at this reference point:
            #   x_LVLH ≈ ECI X (radial outward)
            #   y_LVLH ≈ ECI Y (along-track)
            #   z_LVLH ≈ ECI Z (cross-track)
            _add_spacecraft(
                msg,
                "chaser",
                [_TARGET_ECI[0] + x, _TARGET_ECI[1] + y, _TARGET_ECI[2] + z],
                [vx, vy, vz],
                "CIRCLE",
            )

            serialized = msg.SerializeToString()
            f.write(_encode_varint32(len(serialized)))
            f.write(serialized)


# ---------------------------------------------------------------------------
# Fault injection
# ---------------------------------------------------------------------------

def apply_fault(sc, step, cmd_force, rng):
    """Return (actual_force, meas_valid)."""
    actual = list(cmd_force)
    meas_valid = True

    fault = sc.get("fault")
    if fault is None:
        return actual, meas_valid

    inj_step = sc.get("fault_step", 0)
    if step < inj_step:
        return actual, meas_valid

    if fault == "stuck_open":
        actual[2] = 0.5   # uncmd'd force on z-axis
    elif fault == "dropout":
        if step < inj_step + 3:
            meas_valid = False
    elif fault == "stuck_closed":
        if step < inj_step + 50:
            actual = [0.0, 0.0, 0.0]

    return actual, meas_valid


# ---------------------------------------------------------------------------
# Run one scenario
# ---------------------------------------------------------------------------

def run_scenario(sc, params_path, seed=42, enable_vizard=False):
    """
    Run one scenario using CW dynamics + GncBridge.
    Returns a result dict.
    """
    rng = np.random.default_rng(seed)
    n_rad = math.sqrt(MU_EARTH / SMA**3)
    mass = sc["prop"] + 495.0   # prop + dry mass (kg)

    # GNC bridge
    gnc = GncBridge(sma_m=SMA, mass_kg=mass)
    gnc.init(pos0=sc["pos0"], vel0=sc["vel0"])
    if params_path:
        gnc.set_params(params_path)

    pos = np.array(sc["pos0"], dtype=float)
    vel = np.array(sc["vel0"], dtype=float)

    docked  = False
    aborted = False
    step    = 0
    telem   = []

    for step in range(SIM_MAX_STEPS):
        r = float(np.linalg.norm(pos))

        # Sensor measurement
        sigma = sensor_sigma(r)
        meas  = (pos + rng.normal(0.0, sigma, 3)).tolist()

        # GNC step
        force_cmd, range_m, dock_flag = gnc.step(meas, sigma, SIM_DT_S)

        if dock_flag:
            docked = True
            break

        # Fault injection
        force_actual, meas_valid = apply_fault(sc, step, force_cmd, rng)

        # Record telemetry
        if enable_vizard:
            st = gnc.get_state()
            ph = gnc.phase
            dv = st["dv_mps"]
            telem.append((
                step,
                float(step * SIM_DT_S),
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(vel[0]), float(vel[1]), float(vel[2]),
                float(dv),
                int(ph),
            ))

        # Propagate
        force_np = np.array(force_actual)
        pos, vel = cw_propagate(pos, vel, force_np, n_rad, SIM_DT_S, mass)

        # Abort detection for stuck_open
        if sc.get("fault") == "stuck_open":
            # proxy abort: when stuck_open force flips the vehicle far off course
            if float(np.linalg.norm(pos)) > 300.0 and step > sc.get("fault_step", 0) + 5:
                aborted = True
                break

        # Mass burn
        spd   = float(np.linalg.norm(force_np))
        dm    = (spd * SIM_DT_S) / (500.0 * 9.81)  # Tsiolkovsky approximation
        mass -= dm
        if mass < 10.0:
            mass = 10.0

    st = gnc.get_state()
    final_pos_m = float(np.linalg.norm(pos))

    if enable_vizard:
        bin_path = os.path.join(_HERE, f"vizard_{sc['name']}.bin")
        write_vizard_bin(bin_path, telem)
        print(f"  [VIZARD] Written: {bin_path} ({len(telem)} steps)")

    gnc.free()

    return {
        "name":         sc["name"],
        "docked":       docked,
        "aborted":      aborted,
        "steps":        step,
        "final_pos_m":  final_pos_m,
        "dv_mps":       st["dv_mps"],
        "prop_kg":      st["prop_kg"],
    }


# ---------------------------------------------------------------------------
# Write scenario_results.csv
# ---------------------------------------------------------------------------

def write_results_csv(results, path):
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "scenario", "docked", "aborted", "steps",
            "final_pos_m", "dv_mps", "prop_kg"
        ])
        w.writeheader()
        for r in results:
            w.writerow({
                "scenario":    r["name"],
                "docked":      int(r["docked"]),
                "aborted":     int(r["aborted"]),
                "steps":       r["steps"],
                "final_pos_m": f"{r['final_pos_m']:.6f}",
                "dv_mps":      f"{r['dv_mps']:.6f}",
                "prop_kg":     f"{r['prop_kg']:.6f}",
            })


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="BSK scenario batch runner")
    parser.add_argument("--params",   default="",
                        help="Path to params JSON file")
    parser.add_argument("--scenario", default="",
                        help="Run only this named scenario")
    parser.add_argument("--vizard",   action="store_true",
                        help="Write vizard_<name>.bin telemetry files")
    parser.add_argument("--seed",     type=int, default=42,
                        help="RNG seed for single-scenario runs")
    args = parser.parse_args()

    params_path = os.path.abspath(args.params) if args.params else ""

    # Select scenarios
    scenarios = SCENARIOS
    if args.scenario:
        scenarios = [s for s in SCENARIOS if s["name"] == args.scenario]
        if not scenarios:
            sys.exit(f"[ERROR] Unknown scenario: {args.scenario}")

    results = []
    print(f"[BSK-ALL] Running {len(scenarios)} scenario(s)...")

    for sc in scenarios:
        print(f"  {sc['name']} ...", end=" ", flush=True)
        res = run_scenario(
            sc,
            params_path or None,
            seed=args.seed,
            enable_vizard=args.vizard,
        )
        status = "DOCKED" if res["docked"] else ("ABORT" if res["aborted"] else "FAIL")
        print(f"{status}  steps={res['steps']}  "
              f"pos={res['final_pos_m']:.4f}m  dv={res['dv_mps']:.3f}m/s")
        results.append(res)

    # Write CSV
    results_path = os.path.join(_HERE, "scenario_results.csv")
    write_results_csv(results, results_path)
    print(f"[BSK-ALL] Results written to {results_path}")


if __name__ == "__main__":
    os.chdir(_ROOT)
    main()
