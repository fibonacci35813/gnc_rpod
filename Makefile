# ============================================================
# Makefile — GNC Docking Simulation
# NASA Power of 10: Rule 10 — zero warnings, static analysis
# ============================================================

CC      := gcc
CFLAGS  := -std=c99 \
            -Wall \
            -Wextra \
            -Wpedantic \
            -Werror \
            -Wshadow \
            -Wconversion \
            -Wdouble-promotion \
            -Wundef \
            -Wmissing-prototypes \
            -Wstrict-prototypes \
            -fanalyzer \
            -O2

INCLUDES := -I include/ -I sim/

SRCS := src/dynamics.c \
        src/nav_filter.c \
        src/guidance.c \
        src/control.c \
        src/attitude.c \
        src/rw_model.c \
        src/fdir.c \
        src/imu_model.c \
        src/mission_mgr.c \
        sim/params.c \
        sim/main.c

# Sources for the shared library (no sim/main.c)
LIB_SRCS := src/dynamics.c \
             src/nav_filter.c \
             src/guidance.c \
             src/control.c \
             src/attitude.c \
             src/rw_model.c \
             sim/params.c \
             bsk/gnc_bridge.c

TARGET  := sim/dock_sim
LIB_OUT := build/libgnc.so

VERIFY_DIR := verify

.PHONY: all clean check run shared plot mc mc-fdir mc-retreat verify mc-all mc-all-opt \
        optimise optimise-full optimise-dashboard \
        bsk-all bsk-all-opt bsk-vizard bsk-compare \
        demo demo-install

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@
	@echo "[BUILD] $(TARGET) OK — zero warnings required"

# ── Shared library for Basilisk Python integration ────────────────────────
# Builds libgnc.so exposing gnc_bridge_* API for ctypes consumption.
# The bridge layer does not need to be NASA P10-compliant (it is simulation
# infrastructure); the GNC algorithm sources in src/ remain fully P10.
shared: $(LIB_SRCS)
	@mkdir -p build
	$(CC) -std=c99 -Wall -Wextra -O2 \
	      $(INCLUDES) -I bsk/ \
	      -fPIC -shared \
	      $^ -lm -o $(LIB_OUT)
	@echo "[SHARED] $(LIB_OUT) built"

run: $(TARGET)
	@mkdir -p sim
	./$(TARGET)

run-params: $(TARGET)
	./$(TARGET) --params=$(P)

check:
	@echo "[STATIC] Running cppcheck..."
	cppcheck --enable=all \
	         --error-exitcode=1 \
	         --suppress=missingIncludeSystem \
	         --suppress=unusedFunction:sim/params.c \
	         -I include/ -I sim/ \
	         src/ sim/
	@echo "[STATIC] cppcheck PASSED"

plot: $(TARGET)
	@mkdir -p sim/plots
	./$(TARGET)
	python3 sim/plot_telem.py
	@echo "[PLOT] All plots saved to sim/plots/"

# ── Monte Carlo — make mc N=100 ───────────────────────────────────────────────
MC_TARGET := sim/mc_sim
MC_SRCS   := src/dynamics.c src/nav_filter.c src/guidance.c src/control.c \
             src/attitude.c src/rw_model.c src/fdir.c sim/params.c sim/monte_carlo.c

MCFDIR_TARGET := sim/mc_fdir_sim
MCFDIR_SRCS   := src/dynamics.c src/nav_filter.c src/guidance.c src/control.c \
                 src/attitude.c src/rw_model.c src/fdir.c src/mission_mgr.c \
                 sim/params.c sim/mc_fdir.c

N ?= 100

$(MC_TARGET): $(MC_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

mc: $(MC_TARGET)
	@mkdir -p sim/plots
	./$(MC_TARGET) $(N)
	python3 sim/plot_mc.py
	@echo "[MC] Monte Carlo complete"

# ── Scenario runner — make mc-all / mc-all-opt ───────────────────────────────
SCRUN_TARGET := sim/scenario_runner
SCRUN_SRCS   := src/dynamics.c src/nav_filter.c src/guidance.c src/control.c \
                src/attitude.c src/rw_model.c src/fdir.c src/mission_mgr.c \
                sim/params.c sim/scenarios.c sim/scenario_runner.c

$(SCRUN_TARGET): $(SCRUN_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

mc-all: $(SCRUN_TARGET)
	@mkdir -p sim/results
	./$(SCRUN_TARGET) --params=sim/default_params.json
	@echo "[MC-ALL] All scenarios complete"

mc-all-opt: $(SCRUN_TARGET)
	@mkdir -p sim/results
	./$(SCRUN_TARGET) --params=sim/best_params.json
	@echo "[MC-ALL-OPT] All scenarios complete (optimised params)"

# ── Basilisk batch runner — Phase 4 ─────────────────────────────────────────
bsk-all: shared
	python bsk/run_all_scenarios.py --params=sim/default_params.json
	@echo "[BSK-ALL] Done — bsk/scenario_results.csv written"

bsk-all-opt: shared
	python bsk/run_all_scenarios.py --params=sim/best_params.json
	@echo "[BSK-ALL-OPT] Done — optimised params"

bsk-vizard: shared
	python bsk/run_all_scenarios.py --scenario=$(SC) --vizard --params=$(P)
	@echo "[VIZARD] bsk/vizard_$(SC).bin written"
	@echo "Open this file in the Vizard application."

bsk-compare: shared
	make mc-all
	make bsk-all
	python bsk/compare_results.py
	@echo "[COMPARE] bsk/comparison_report.csv written"

# ── Bayesian optimisation — make optimise [--trials=200] ─────────────────────
optimise:
	python sim/optimise.py --trials=200

optimise-full:
	python sim/optimise.py --trials=1000

optimise-dashboard:
	optuna-dashboard sqlite:///sim/optuna.db

# ── Streamlit demo — Phase 5 ─────────────────────────────────────────────────
demo: shared $(SCRUN_TARGET)
	streamlit run sim/app.py

demo-install:
	pip install streamlit plotly pandas optuna optuna-dashboard

# ── FDIR Monte Carlo — make mc-fdir FAULT=stuck_open N=100 ───────────────────
FAULT ?= stuck_open

$(MCFDIR_TARGET): $(MCFDIR_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -lm -o $@

mc-fdir: $(MCFDIR_TARGET)
	./$(MCFDIR_TARGET) $(FAULT) $(N)
	@echo "[MC-FDIR] FDIR fault-injection MC complete"

# ── Retreat verification — make mc-retreat N=20 ──────────────────────────────
mc-retreat: $(MCFDIR_TARGET)
	./$(MCFDIR_TARGET) retreat $(N)
	@echo "[MC-RETREAT] Abort-retreat verification complete"

# ── Formal verification — make verify ────────────────────────────────────────
verify: $(SRCS)
	@mkdir -p $(VERIFY_DIR)
	@echo "[VERIFY] Writing annotation check..."
	@grep -l "requires" src/dynamics.c src/control.c > /dev/null 2>&1 && \
	  echo "[VERIFY] ACSL annotations present" || \
	  echo "[VERIFY] WARNING: annotations missing"
	@echo "[VERIFY] Attempting Frama-C WP (dynamics + control)..."
	frama-c -wp -wp-rte -wp-timeout 30 \
	    src/dynamics.c src/control.c \
	    -I include/ \
	    -wp-log w:$(VERIFY_DIR)/wp_log.txt 2>&1 | \
	    tee $(VERIFY_DIR)/frama_output.txt; \
	    echo "[VERIFY] Frama-C exit: $$?"
	@echo "[VERIFY] Attempting CBMC (dynamics bounds check)..."
	cbmc src/dynamics.c \
	    --include include/gnc_types.h \
	    --include include/gnc_assert.h \
	    --unwind 10 --bounds-check --pointer-check 2>&1 | \
	    tee $(VERIFY_DIR)/cbmc_output.txt; \
	    echo "[VERIFY] CBMC exit: $$?"
	@echo "[VERIFY] Results in $(VERIFY_DIR)/"

clean:
	rm -f $(TARGET) $(MC_TARGET) $(MCFDIR_TARGET) $(SCRUN_TARGET) \
	      sim/telem.csv sim/mc_results.csv sim/mc_fdir_results.csv $(LIB_OUT)
	rm -rf build/ sim/plots/ sim/mc_plots/ sim/results/ $(VERIFY_DIR)/
