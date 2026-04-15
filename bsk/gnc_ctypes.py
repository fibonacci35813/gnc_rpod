"""
gnc_ctypes.py
=============
Python ctypes interface to the compiled GNC shared library (libgnc.so).

Usage:
    from bsk.gnc_ctypes import GncBridge

    gnc = GncBridge(sma_m=6.771e6, mass_kg=500.0)
    gnc.init(pos0=[0, 200, 0], vel0=[0, 0, 0])

    for step in range(N):
        meas = sensor.measure()           # [x, y, z] LVLH metres
        sigma = sensor.sigma(range_m)
        force, range_m, docked = gnc.step(meas, sigma, dt_s=1.0)
        dynamics.apply_force(force)       # LVLH newtons
        if docked:
            break
"""

import ctypes
import os
import sys
from typing import Tuple, List


# ---------------------------------------------------------------------------
# Locate the shared library relative to this file
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB_PATH = os.path.join(_HERE, "..", "build", "libgnc.so")

if not os.path.exists(_LIB_PATH):
    raise ImportError(
        f"libgnc.so not found at {_LIB_PATH}\n"
        "Run:  make shared   in the gnc_docking/ directory first."
    )

_lib = ctypes.CDLL(_LIB_PATH)

# ---------------------------------------------------------------------------
# C function signatures
# ---------------------------------------------------------------------------
_D3 = ctypes.c_double * 3

# GncContext* gnc_bridge_init(const double[3], const double[3], double, double)
_lib.gnc_bridge_init.restype  = ctypes.c_void_p
_lib.gnc_bridge_init.argtypes = [
    ctypes.POINTER(ctypes.c_double),   # pos0_lvlh[3]
    ctypes.POINTER(ctypes.c_double),   # vel0_lvlh[3]
    ctypes.c_double,                   # mass_kg
    ctypes.c_double,                   # sma_m
]

# int gnc_bridge_step(ctx, meas[3], sigma, dt, force[3]*, range*, docked*)
_lib.gnc_bridge_step.restype  = ctypes.c_int
_lib.gnc_bridge_step.argtypes = [
    ctypes.c_void_p,                   # ctx
    ctypes.POINTER(ctypes.c_double),   # meas_lvlh[3]
    ctypes.c_double,                   # meas_sigma
    ctypes.c_double,                   # dt_s
    ctypes.POINTER(ctypes.c_double),   # force_lvlh[3]  (output)
    ctypes.POINTER(ctypes.c_double),   # range_m        (output)
    ctypes.POINTER(ctypes.c_int),      # docked         (output)
]

# int gnc_bridge_get_state(ctx, pos[3]*, vel[3]*, dv*, prop*)
_lib.gnc_bridge_get_state.restype  = ctypes.c_int
_lib.gnc_bridge_get_state.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
]

# int gnc_bridge_get_phase(ctx)
_lib.gnc_bridge_get_phase.restype  = ctypes.c_int
_lib.gnc_bridge_get_phase.argtypes = [ctypes.c_void_p]

# void gnc_bridge_free(ctx)
_lib.gnc_bridge_free.restype  = None
_lib.gnc_bridge_free.argtypes = [ctypes.c_void_p]


# ---------------------------------------------------------------------------
# Python wrapper class
# ---------------------------------------------------------------------------
class GncBridge:
    """
    Thin Python wrapper over the C GNC shared library.

    All position/velocity in LVLH (target-centred):
        [0] = radial, [1] = along-track, [2] = cross-track
    """

    def __init__(self, sma_m: float, mass_kg: float) -> None:
        self._sma_m    = float(sma_m)
        self._mass_kg  = float(mass_kg)
        self._ctx: ctypes.c_void_p = None

    def init(self,
             pos0: List[float],
             vel0: List[float]) -> None:
        """Initialise the GNC context with initial LVLH state."""
        p = (_D3)(*pos0)
        v = (_D3)(*vel0)
        self._ctx = _lib.gnc_bridge_init(p, v, self._mass_kg, self._sma_m)
        if self._ctx is None:
            raise RuntimeError("gnc_bridge_init failed — static pool may be full")

    def step(self,
             meas_lvlh: List[float],
             meas_sigma: float,
             dt_s: float) -> Tuple[List[float], float, bool]:
        """
        Run one GNC update step.

        Parameters
        ----------
        meas_lvlh   : [x, y, z] LIDAR measurement in LVLH (m)
        meas_sigma  : 1-sigma measurement noise per axis (m)
        dt_s        : timestep (s)

        Returns
        -------
        force_lvlh  : [fx, fy, fz] commanded force (N, LVLH)
        range_m     : estimated range to docking port (m)
        docked      : True when docking declared
        """
        if self._ctx is None:
            raise RuntimeError("GncBridge.init() must be called first")

        m         = (_D3)(*meas_lvlh)
        force_out = (_D3)(0.0, 0.0, 0.0)
        range_out = ctypes.c_double(0.0)
        dock_out  = ctypes.c_int(0)

        rc = _lib.gnc_bridge_step(
            self._ctx, m,
            ctypes.c_double(meas_sigma),
            ctypes.c_double(dt_s),
            force_out, range_out, dock_out)

        if rc != 0:
            raise RuntimeError(f"gnc_bridge_step returned error {rc}")

        return (list(force_out), float(range_out.value), bool(dock_out.value))

    def get_state(self) -> dict:
        """Return current estimated nav state and fuel usage."""
        if self._ctx is None:
            raise RuntimeError("GncBridge.init() must be called first")

        pos    = (_D3)(0.0, 0.0, 0.0)
        vel    = (_D3)(0.0, 0.0, 0.0)
        dv     = ctypes.c_double(0.0)
        prop   = ctypes.c_double(0.0)
        rc     = _lib.gnc_bridge_get_state(self._ctx, pos, vel, dv, prop)
        if rc != 0:
            raise RuntimeError(f"gnc_bridge_get_state returned error {rc}")

        return {
            "pos_lvlh": list(pos),
            "vel_lvlh": list(vel),
            "dv_mps":   float(dv.value),
            "prop_kg":  float(prop.value),
        }

    @property
    def phase(self) -> int:
        """Current guidance phase index (0–3)."""
        if self._ctx is None:
            return -1
        return int(_lib.gnc_bridge_get_phase(self._ctx))

    def free(self) -> None:
        """Release this context back to the static pool (required for MC runs)."""
        if self._ctx is not None:
            _lib.gnc_bridge_free(self._ctx)
            self._ctx = None

    def __del__(self) -> None:
        self.free()
