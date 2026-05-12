"""Reusable Basilisk two-spacecraft propagation runner."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody

from rpod_gnc.bsk.frames import relative_state_eci_to_lvlh
from rpod_gnc.bsk.relative_state_adapter import RelativeState, compute_relative_state


@dataclass(frozen=True)
class LvlhRelativeState:
    """Relative state of the chaser with respect to the target in LVLH."""

    position_m: np.ndarray
    velocity_mps: np.ndarray

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.position_m))


@dataclass(frozen=True)
class TwoSpacecraftResult:
    """Logged open-loop truth states for the two-spacecraft Basilisk demo.

    `relative_states` is kept as a backward-compatible alias for
    `relative_states_eci`.
    """

    times_s: np.ndarray
    relative_states_eci: list[RelativeState]
    relative_states_lvlh: list[LvlhRelativeState]
    ranges_m: np.ndarray

    @property
    def relative_states(self) -> list[RelativeState]:
        """Backward-compatible alias for inertial relative states."""

        return self.relative_states_eci


def make_spacecraft(name: str, r_n_m: np.ndarray, v_n_mps: np.ndarray):
    sc = spacecraft.Spacecraft()
    sc.ModelTag = name
    sc.hub.r_CN_NInit = np.asarray(r_n_m, dtype=float).reshape(3).tolist()
    sc.hub.v_CN_NInit = np.asarray(v_n_mps, dtype=float).reshape(3).tolist()
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    return sc


def run_two_spacecraft_demo(duration_s: float = 60.0, dt_s: float = 1.0) -> TwoSpacecraftResult:
    sim = SimulationBaseClass.SimBaseClass()

    process = sim.CreateNewProcess("rpod_process")
    task = sim.CreateNewTask("rpod_task", macros.sec2nano(dt_s))
    process.addTask(task)

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

    r_target, v_target = orbitalMotion.elem2rv(orbitalMotion.MU_EARTH, oe)

    r_chaser = np.array(r_target) + np.array([0.0, -10.0, 0.0])
    v_chaser = np.array(v_target)

    target = make_spacecraft("target", np.array(r_target), np.array(v_target))
    chaser = make_spacecraft("chaser", r_chaser, v_chaser)

    grav_factory.addBodiesTo(target)
    grav_factory.addBodiesTo(chaser)

    sim.AddModelToTask("rpod_task", target)
    sim.AddModelToTask("rpod_task", chaser)

    target_recorder = target.scStateOutMsg.recorder(macros.sec2nano(dt_s))
    chaser_recorder = chaser.scStateOutMsg.recorder(macros.sec2nano(dt_s))

    sim.AddModelToTask("rpod_task", target_recorder)
    sim.AddModelToTask("rpod_task", chaser_recorder)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration_s))
    sim.ExecuteSimulation()

    times_s = target_recorder.times() * macros.NANO2SEC

    target_positions = np.array(target_recorder.r_BN_N)
    target_velocities = np.array(target_recorder.v_BN_N)
    chaser_positions = np.array(chaser_recorder.r_BN_N)
    chaser_velocities = np.array(chaser_recorder.v_BN_N)

    relative_states_eci: list[RelativeState] = []
    relative_states_lvlh: list[LvlhRelativeState] = []
    for k in range(len(times_s)):
        relative_states_eci.append(
            compute_relative_state(
                target_positions[k],
                target_velocities[k],
                chaser_positions[k],
                chaser_velocities[k],
            )
        )
        relative_position_lvlh_m, relative_velocity_lvlh_mps, _ = relative_state_eci_to_lvlh(
            target_positions[k],
            target_velocities[k],
            chaser_positions[k],
            chaser_velocities[k],
        )
        relative_states_lvlh.append(
            LvlhRelativeState(
                position_m=relative_position_lvlh_m,
                velocity_mps=relative_velocity_lvlh_mps,
            )
        )

    ranges_m = np.array([state.range_m for state in relative_states_lvlh])

    return TwoSpacecraftResult(
        times_s=times_s,
        relative_states_eci=relative_states_eci,
        relative_states_lvlh=relative_states_lvlh,
        ranges_m=ranges_m,
    )
