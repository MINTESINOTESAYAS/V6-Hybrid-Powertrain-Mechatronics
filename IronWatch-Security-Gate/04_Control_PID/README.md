# Boom-Arm Dynamics & PID Controller

This module continues the IronWatch project from the point the previous session
stopped: designing the **motorised swing-arm boom barrier** — its mechanical
sizing, dynamic model, and a **PID position controller** for smooth, non-slamming
open/close motion. This is the piece that makes the project satisfy the
electromechanical internship spec (mechanical sizing + dynamic modelling +
PID control), with the existing metal-detection + facial-recognition work acting
as the authorisation layer that triggers it.

## Contents

| File | Purpose |
|------|---------|
| `boom_arm_pid.py` | Full nonlinear model + PID design + auto-tune + figure generation (Python) |
| `matlab/boom_arm_pid.m` | Equivalent linearised model for MATLAB / PID Tuner |
| `arduino/boom_arm_pid_controller.ino` | Real-hardware PID position loop for the Arduino |
| `figures/` | Generated result plots |
| `DESIGN_NOTES.md` | The written engineering derivation for the report |

## How to run (Python)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install numpy scipy matplotlib
python boom_arm_pid.py
```

Figures are written to `figures/`.

## The mechanism

A single boom arm (like a short pedestrian parking barrier) pivots about one end,
driven by a 12 V DC gearmotor:

- `theta = 0`   → **CLOSED** (arm horizontal → worst-case gravity torque)
- `theta = 90°` → **OPEN** (arm vertical → zero gravity torque)

## Key results (from the model)

**Sizing** — uniform slender-rod arm, 1.5 kg × 1.0 m:

| Quantity | Value |
|----------|-------|
| Arm inertia about pivot `J = ⅓ m L²` | 0.500 kg·m² |
| Worst-case gravity torque (θ = 0) | 7.36 N·m |
| Stall torque at arm (12 V, 70:1, η = 0.7) | 12.35 N·m |
| **Holding safety factor** | **1.68×** ✅ |
| No-load arm speed | 234 °/s (free 90° sweep ≈ 0.38 s) |

**Controller** — recommended gains **Kp = 70, Ki = 60, Kd = 4**:

| Metric (open 0→90°) | P-only (Kp = 70) | Tuned PID |
|---------------------|------------------|-----------|
| Rise time (10–90 %) | 0.72 s | 0.72 s |
| Overshoot | 5.6 % (slams) | **1.5 %** |
| Settling (±2 %) | 1.40 s | **1.03 s** |
| Steady-state error | ~0° | ~0° |

**Why the integral term** — holding at an intermediate 45° against gravity:

| Controller | Steady-state droop |
|------------|--------------------|
| PD only (Ki = 0) | 4.45° |
| Full PID (Ki = 60) | **0.02°** ✅ |

See `DESIGN_NOTES.md` for the full derivation and `figures/` for the plots.
