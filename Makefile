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

INCLUDES := -I include/

SRCS := src/dynamics.c \
        src/nav_filter.c \
        src/guidance.c \
        src/control.c \
        sim/main.c

# Sources for the shared library (no sim/main.c)
LIB_SRCS := src/dynamics.c \
             src/nav_filter.c \
             src/guidance.c \
             src/control.c \
             bsk/gnc_bridge.c

TARGET  := sim/dock_sim
LIB_OUT := build/libgnc.so

.PHONY: all clean check run shared

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

check:
	@echo "[STATIC] Running cppcheck..."
	cppcheck --enable=all \
	         --error-exitcode=1 \
	         --suppress=missingIncludeSystem \
	         --suppress=checkersReport \
	         -I include/ \
	         src/ sim/
	@echo "[STATIC] cppcheck PASSED"

clean:
	rm -f $(TARGET) sim/telem.csv $(LIB_OUT)
	rm -rf build/
