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

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(60.0))
    sim.ExecuteSimulation()

    target_state = target.scStateOutMsg.read()
    chaser_state = chaser.scStateOutMsg.read()




    rel = compute_relative_state(
        target_state.r_BN_N,
        target_state.v_BN_N,
        chaser_state.r_BN_N,
        chaser_state.v_BN_N,
    )

    print("Relative state at end of simulation:")
    print(f"Relative position (m): {rel.position_m}")
    print(f"Relative velocity (m/s): {rel.velocity_mps}")
    print(f"Range (m): {rel.range_m}")
    print(f"Closing speed (m/s): {rel.closing_speed_mps}")

if __name__ == "__main__":
    main()