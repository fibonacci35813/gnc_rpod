"""
sim/app.py — Streamlit demo application for the GNC Docking Testbed.
Phase 5.

Usage:
    streamlit run sim/app.py
"""

import os
import subprocess
import sys

import streamlit as st
import pandas as pd
import plotly.graph_objects as go

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)

from app_utils import (
    run_bsk_scenario, run_mc_scenario, read_telem_csv, read_comparison_csv,
    params_path, vizard_bin_path,
    SCENARIO_NAMES, SCENARIO_GATES,
    DEFAULT_PARAMS, BEST_PARAMS,
)

# ---------------------------------------------------------------------------
# Page config
# ---------------------------------------------------------------------------
st.set_page_config(page_title="GNC Docking Testbed", layout="wide")
st.title("Autonomous Spacecraft Docking — GNC Testbed")
st.caption(
    "Flight-software GNC running live: CW dynamics · EKF nav · PD control · FDIR"
)

# ---------------------------------------------------------------------------
# Sidebar
# ---------------------------------------------------------------------------
with st.sidebar:
    st.header("Configuration")
    scenario = st.selectbox(
        "Scenario", SCENARIO_NAMES, index=0,
        help="Select one of the 8 standardised test scenarios"
    )
    use_opt = st.toggle(
        "Use optimised params",
        value=False,
        help="Switch between default gains and Bayesian-optimised gains",
        disabled=not os.path.exists(BEST_PARAMS),
    )
    n_mc = st.number_input("MC seeds", min_value=5, max_value=200, value=20, step=5)

    st.divider()
    run_bsk_btn  = st.button("▶ Run BSK + Vizard", use_container_width=True)
    run_mc_btn   = st.button("▶ Run Monte Carlo", use_container_width=True)
    st.divider()
    run_opt_btn  = st.button("⚙ Launch Optimiser (200 trials)",
                              use_container_width=True)

p_file = params_path(use_opt)

# ---------------------------------------------------------------------------
# Session state for last results
# ---------------------------------------------------------------------------
if "last_bsk" not in st.session_state:
    st.session_state.last_bsk = None
if "last_mc" not in st.session_state:
    st.session_state.last_mc = None
if "opt_running" not in st.session_state:
    st.session_state.opt_running = False

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
if run_bsk_btn:
    with st.spinner(f"Running BSK scenario: {scenario} …"):
        result = run_bsk_scenario(scenario, p_file, enable_vizard=True)
    st.session_state.last_bsk = result

if run_mc_btn:
    with st.spinner(f"Running MC ({n_mc} seeds): {scenario} …"):
        result = run_mc_scenario(scenario, p_file, n_seeds=int(n_mc))
    st.session_state.last_mc = result

if run_opt_btn:
    st.session_state.opt_running = True
    with st.spinner("Running Optuna optimiser (200 trials) …"):
        opt_script = os.path.join(_ROOT, "sim", "optimise.py")
        subprocess.run(
            [sys.executable, opt_script, "--trials=200"],
            cwd=_ROOT
        )
    st.session_state.opt_running = False
    st.success("Optimisation complete — best_params.json updated.")
    st.rerun()

# ---------------------------------------------------------------------------
# Main area — 3 columns
# ---------------------------------------------------------------------------
col1, col2, col3 = st.columns([1, 1, 1.5])

with col1:
    st.subheader("Last BSK Run")
    bsk = st.session_state.last_bsk
    if bsk is None:
        st.info("Press 'Run BSK + Vizard' to execute a scenario.")
    elif "error" in bsk:
        st.error(bsk["error"])
    else:
        docked  = bsk.get("docked", False)
        aborted = bsk.get("aborted", False)
        status  = "DOCKED ✓" if docked else ("ABORTED (safe)" if aborted else "FAILED ✗")
        color   = "green" if docked else ("orange" if aborted else "red")
        st.markdown(f"**Status:** :{color}[{status}]")
        st.metric("Steps",        bsk.get("steps", "—"))
        st.metric("Final pos (m)", f"{bsk.get('final_pos_m', 0):.4f}")
        st.metric("ΔV (m/s)",     f"{bsk.get('dv_mps', 0):.3f}")

        bin_path = vizard_bin_path(scenario)
        if os.path.exists(bin_path):
            st.success(f"Vizard file: `{bin_path}`")
            with open(bin_path, "rb") as f:
                st.download_button(
                    "Download .bin",
                    data=f,
                    file_name=os.path.basename(bin_path),
                    mime="application/octet-stream",
                )

with col2:
    st.subheader("Monte Carlo")
    mc = st.session_state.last_mc
    if mc is None:
        st.info("Press 'Run Monte Carlo' to evaluate docking probability.")
    elif "error" in mc:
        st.error(mc["error"])
    else:
        dr = mc.get("dock_rate", 0.0)
        ar = mc.get("abort_rate", 0.0)
        gate = SCENARIO_GATES.get(scenario, 0.90)

        color = "green" if (
            (scenario == "stuck_open" and ar >= 1.0) or
            (scenario != "stuck_open" and dr >= gate)
        ) else "red"

        if scenario == "stuck_open":
            st.markdown(f"**Abort rate:** :{color}[{ar:.3f}] (gate=1.00)")
        else:
            st.markdown(f"**Dock rate:** :{color}[{dr:.3f}] (gate≥{gate})")

        st.metric("Mean ΔV (m/s)", f"{mc.get('mean_dv_mps', 0):.3f}")
        st.metric("Mean pos (m)",  f"{mc.get('mean_final_pos_m', 0):.4f}")
        st.metric("Mean steps",    f"{mc.get('mean_steps', 0):.0f}")

with col3:
    st.subheader("Scenario Comparison Matrix")
    comp = read_comparison_csv()
    if comp is None:
        st.info("Run `make bsk-compare` to generate the comparison table.")
    else:
        df = pd.DataFrame(comp)
        st.dataframe(
            df,
            use_container_width=True,
            hide_index=True,
        )

# ---------------------------------------------------------------------------
# Telemetry plots
# ---------------------------------------------------------------------------
st.divider()
telem = read_telem_csv()

tab_range, tab_traj, tab_dv = st.tabs(["Range", "3D Trajectory", "ΔV"])

with tab_range:
    if telem is None:
        st.info("Run a simulation to see telemetry (make run generates sim/telem.csv).")
    else:
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=telem["time_s"], y=telem["range_m"],
            name="Range", line=dict(color="steelblue", width=1.5)
        ))
        fig.add_hline(y=0.05, line_dash="dash", line_color="red",
                      annotation_text="5 cm dock tolerance")
        fig.update_layout(
            title="Range to Docking Port",
            xaxis_title="Time (s)", yaxis_title="Range (m)",
            yaxis_type="log", height=380
        )
        st.plotly_chart(fig, use_container_width=True)

with tab_traj:
    if telem is None:
        st.info("No telemetry data available.")
    else:
        fig = go.Figure(data=[go.Scatter3d(
            x=telem["true_pos_x"],
            y=telem["true_pos_y"],
            z=telem["true_pos_z"],
            mode="lines",
            line=dict(color=telem["time_s"], colorscale="Viridis", width=3),
        )])
        fig.add_trace(go.Scatter3d(
            x=[0], y=[0], z=[0],
            mode="markers",
            marker=dict(size=8, color="red"),
            name="Docking port"
        ))
        fig.update_layout(
            title="LVLH Trajectory",
            scene=dict(
                xaxis_title="Radial x (m)",
                yaxis_title="Along-track y (m)",
                zaxis_title="Cross-track z (m)",
            ),
            height=440
        )
        st.plotly_chart(fig, use_container_width=True)

with tab_dv:
    if telem is None:
        st.info("No telemetry data available.")
    else:
        dv_cumsum = []
        total = 0.0
        for d in telem["delta_v"]:
            total += d
            dv_cumsum.append(total)

        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=telem["time_s"], y=dv_cumsum,
            name="Cumulative ΔV",
            line=dict(color="green", width=1.5)
        ))
        fig.update_layout(
            title="Cumulative ΔV",
            xaxis_title="Time (s)", yaxis_title="ΔV (m/s)",
            height=380
        )
        st.plotly_chart(fig, use_container_width=True)

# ---------------------------------------------------------------------------
# Vizard instructions
# ---------------------------------------------------------------------------
with st.expander("Vizard 3D Visualisation — Opening Instructions"):
    st.markdown("""
    ### Opening a Vizard .bin file

    **Step 1:** Download and install the Vizard application from
    [AVS Lab Basilisk website](https://hanspeterschaub.info/basilisk/Vizard/vizardDownload.html).

    **Step 2:** Run a scenario with Vizard enabled:
    ```bash
    make bsk-vizard SC=nominal P=sim/default_params.json
    ```

    **Step 3:** Launch Vizard and use **File → Open Playback** to select the `.bin` file
    from the `bsk/` directory.

    The recording contains the spacecraft trajectory, attitude, and approach axis overlay.

    > **Note:** The current `.bin` files use a custom telemetry binary format
    > (not the native Vizard protobuf format, as Basilisk is not installed).
    > To generate native Vizard files, install the Basilisk astrodynamics framework
    > and run with the `--vizard` flag.
    """)

# ---------------------------------------------------------------------------
# Footer
# ---------------------------------------------------------------------------
st.divider()
st.caption(
    "GNC Testbed · NASA Power of 10 · "
    f"Params: {'optimised' if use_opt else 'default'} | "
    f"Scenario: {scenario}"
)
