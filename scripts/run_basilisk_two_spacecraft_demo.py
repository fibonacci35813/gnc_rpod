#!/usr/bin/env python
"""Minimal basilisk two spacecraft relative motion demo"""


from __future__ import annotations

import numpy as np

from Basilisk.simulation import spacecraft
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion

from rpod_gnc.bsk.relative_state_adapter import compute_relative_state

def make_spacecraft(name: str, r_n_m: np.ndarray, v_n_mps: np.ndarray):
    sc = spacecraft.Spacecraft()
    sc.ModelTag = name
    sc.hub.r_CN_NInit = r_n_m.tolist()
    sc.hub.v_CN_NInit = v_n_mps.tolist()
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    return sc

def main():
    sim = SimulationBaseClass.SimBaseClass()

    process_name = "rpod_process"
    task_name = "rpod_task"
    dt_s = 1.0

    process= sim.CreateNewProcess(process_name)
    task = sim.CreateNewTask(task_name, macros.sec2nano(dt_s))
    process.addTask(task)

    mu = orbitalMotion.MU_EARTH
    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True

    oe = orbitalMotion.ClassicElements()
    oe.a = 7000e3
    oe.e = 0.0
    oe.i = np.deg2rad(51.6)
    oe.Omega = 0.0
    oe.omega = 0.0
    oe.f = 0.0

    r_target, v_target = orbitalMotion.elem2rv(mu, oe)

    r_chaser = np.array(r_target) + np.array([0.0, -10.0, 0.0])
    v_chaser = np.array(v_target)

    target = make_spacecraft("target", np.array(r_target), np.array(v_target))
    chaser = make_spacecraft("chaser", r_chaser, v_chaser)

    grav_factory.addBodiesTo(target)
    grav_factory.addBodiesTo(chaser)

    sim.AddModelToTask(task_name, target)
    sim.AddModelToTask(task_name, chaser)

    target_recorder = target.scStateOutMsg.recorder(macros.sec2nano(dt_s))
    chaser_recorder = chaser.scStateOutMsg.recorder(macros.sec2nano(dt_s))

    sim.AddModelToTask(task_name, target_recorder)
    sim.AddModelToTask(task_name, chaser_recorder)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(60.0))
    sim.ExecuteSimulation()

    times_s = target_recorder.times() * macros.NANO2SEC

    target_positions = np.array(target_recorder.r_BN_N)
    target_velocities = np.array(target_recorder.v_BN_N)
    chaser_positions = np.array(chaser_recorder.r_BN_N)
    chaser_velocities = np.array(chaser_recorder.v_BN_N)

    ranges_m = []

    for k in range(len(times_s)):
        rel_k = compute_relative_state(
            target_positions[k],
            target_velocities[k],
            chaser_positions[k],
            chaser_velocities[k],
        )
        ranges_m.append(rel_k.range_m)

    rel = compute_relative_state(
        target_positions[-1],
        target_velocities[-1],
        chaser_positions[-1],
        chaser_velocities[-1],
    )

    print("Relative state at end of simulation:")
    print(f"Relative position (m): {rel.position_m}")
    print(f"Relative velocity (m/s): {rel.velocity_mps}")
    print(f"Range (m): {rel.range_m}")
    print(f"Closing speed (m/s): {rel.closing_speed_mps}")
    print(f"Logged samples: {len(times_s)}")
    print(f"Initial range (m): {ranges_m[0]:.6f}")
    print(f"Minimum range (m): {min(ranges_m):.6f}")
    print(f"Maximum range (m): {max(ranges_m):.6f}")

if __name__ == "__main__":
    main()