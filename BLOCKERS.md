# BLOCKERS

## Phase 10 — Formal Verification Tools Not Installed

**Status:** PARTIAL PASS

**Root cause:** Neither `frama-c` nor `cbmc` is installed on the host system.

```
$ which frama-c 2>/dev/null && frama-c -version || echo "FRAMA-C NOT FOUND"
FRAMA-C NOT FOUND

$ which cbmc 2>/dev/null && cbmc --version || echo "CBMC NOT FOUND"
CBMC NOT FOUND
```

**What was completed:**
- All ACSL annotations written in source files (valid C comments, zero impact on compilation):
  - `src/dynamics.c`: dyn_mean_motion, dyn_range, dyn_build_phi, dyn_propagate
  - `src/control.c`: ctrl_init_gains, ctrl_vec3_norm, ctrl_vec3_sub, ctrl_compute
  - `src/nav_filter.c`: nav_update
  - `src/guidance.c`: guid_init_plan
- `make verify` target created in Makefile
- `make verify` runs successfully, detects tools missing, reports "[VERIFY] ACSL annotations present"
- `verify/` directory created with placeholder output files

**What is blocked:**
- Frama-C WP formal proof of absence of: null dereference, divide-by-zero, array OOB
- CBMC bounds-checking verification of dynamics.c

**Recommended resolution:**
```bash
# Ubuntu/Debian:
sudo apt-get install frama-c
sudo apt-get install cbmc

# Then re-run:
make verify
```

After installation, `make verify` will attempt proofs with 30-second WP timeout.
Expect partial proof coverage; full automation requires additional ACSL loop invariants
on the propagation loops in dyn_propagate.
