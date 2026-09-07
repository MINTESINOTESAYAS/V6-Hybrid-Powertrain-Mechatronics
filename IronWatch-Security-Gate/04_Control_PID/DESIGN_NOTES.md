# Boom-Arm Dynamic Model & PID Controller — Engineering Design Notes

> IronWatch Smart Security Access Gate — Control subsystem
> Continuation of the design started in the previous session (which stopped
> at "derive the equation of motion for the arm").

---

## 1. Purpose and system context

The IronWatch gate has three layers built in earlier phases:

1. **Identification** — facial recognition (OpenCV) + RFID/badge, logging
   entry/exit attendance.
2. **Ferrous-metal theft detection** — coil + CD4046 phase-discrimination
   circuit + Arduino, distinguishing iron from harmless non-ferrous metal.
3. **Physical barrier** — a single motorised swing-arm boom gate.

This document covers **layer 3's control system**. The detection/identification
layers act purely as the *authorisation input*: a valid, metal-clear crossing
commands the barrier to open; anything else keeps it closed and/or alarms.

The barrier is the component that makes the project a true **electromechanical**
system — it has a real moving mass under continuous feedback control, which is
what the internship specification requires (mechanical sizing, dynamic modelling,
PID control).

---

## 2. Mechanism definition

A single **swing-arm boom barrier** pivots about one end at the base of one
upright, rotating ~90°:

- **θ = 0°** — arm horizontal → **CLOSED**. This is the worst case for the motor:
  gravity torque is maximum.
- **θ = 90°** — arm vertical → **OPEN**. Gravity torque is zero.

One clean rotational degree of freedom → one PID loop, one motor to size. No
turnstile, no sliding panels (deliberately scoped down).

---

## 3. Mechanical sizing

The arm is modelled as a **uniform slender rod** pivoting about one end.

### 3.1 Moment of inertia

$$ J_{arm} = \tfrac{1}{3} m L^2 $$

With `m = 1.5 kg`, `L = 1.0 m`:

$$ J_{arm} = \tfrac{1}{3}(1.5)(1.0)^2 = 0.500 \ \text{kg·m}^2 $$

### 3.2 Gravity torque (angle-dependent)

The centre of mass is at `r_cg = L/2 = 0.5 m`. The gravity torque opposing
opening is:

$$ \tau_g(\theta) = m g\, r_{cg} \cos\theta $$

- Maximum at θ = 0 (closed): `τ_g = 1.5 × 9.81 × 0.5 = 7.36 N·m`
- Zero at θ = 90° (open).

### 3.3 Motor + gearbox selection

A **12 V DC gearmotor** with `Kt = Ke = 0.042 N·m/A`, `R = 2 Ω`, reduction
`N = 70:1`, efficiency `η = 0.70`:

- Motor stall torque: `τ_stall,m = Kt·V/R = 0.042 × 12 / 2 = 0.252 N·m`
- Reflected to the arm: `τ_stall,arm = τ_stall,m · N · η = 12.35 N·m`

**Holding safety factor:**

$$ SF = \frac{\tau_{stall,arm}}{\tau_{g,max}} = \frac{12.35}{7.36} = 1.68\times $$

> 1.68× exceeds the usual ≥1.3× guideline, so the motor holds the closed arm
> against gravity and still has torque margin to accelerate it.

- No-load arm speed: `ω = V/(Ke·N) = 4.08 rad/s = 234 °/s`
  → a free 90° sweep would take ≈ 0.38 s, so a **loaded, controlled 1–1.5 s**
  open time is comfortably feasible.

### 3.4 Inertia & damping reflected to the arm shaft

$$ J_{tot} = J_{arm} + N^2 J_{motor} = 0.500 + 70^2(1\times10^{-5}) = 0.549 \ \text{kg·m}^2 $$
$$ b_{tot} = b_{pivot} + N^2 b_{motor} = 0.30 + 70^2(1\times10^{-5}) = 0.349 \ \text{N·m·s/rad} $$

---

## 4. Equation of motion (the part the previous session was about to derive)

Newton's second law for rotation about the pivot:

$$ J_{tot}\,\ddot\theta = \tau_{drive} - b_{tot}\,\dot\theta - m g\, r_{cg}\cos\theta $$

The drive torque comes from the DC motor. With the electrical time constant
`L/R ≈ 0.25 ms` far faster than the mechanical response, the armature current is
quasi-static:

$$ i = \frac{V - K_e\,\omega_{motor}}{R}, \qquad \omega_{motor} = N\dot\theta $$
$$ \tau_{drive} = N\,\eta\,K_t\,i $$

Substituting gives the full nonlinear plant:

$$ J_{tot}\ddot\theta = \frac{N\eta K_t}{R}V - \left(b_{tot} + \frac{N^2\eta K_t K_e}{R}\right)\dot\theta - m g\, r_{cg}\cos\theta $$

This is a **second-order nonlinear system** (nonlinear only through the
`cos θ` gravity term). It is implemented and integrated in `boom_arm_pid.py`.

### Linearised transfer function (for PID Tuner)

Dropping the gravity disturbance, `θ(s)/V(s)`:

$$ P(s) = \frac{K_v}{J_{tot}s^2 + b_e s}, \quad K_v = \frac{N\eta K_t}{R},\ b_e = b_{tot} + \frac{N^2\eta K_t K_e}{R} $$

Used in `matlab/boom_arm_pid.m`.

---

## 5. PID controller design

Control law (derivative-on-measurement to avoid setpoint kick, with a
first-order derivative filter and integrator anti-windup):

$$ u(t) = K_p e(t) + K_i\!\int e\,dt - K_d \frac{d\,\theta_{meas}}{dt} $$

where `e = θ_ref − θ`, and `u` is the motor voltage, saturated at ±12 V.

### 5.1 Tuning

A coarse grid search (`auto_tune()` in the Python script) minimises a weighted
cost of overshoot + settling time + steady-state error, evaluated on the harder
**closing** move (gravity-assisted, where overshoot means the arm slams toward
the 0° hard stop). Combined with the intermediate-hold requirement, the final
recommended gains are:

| Gain | Value |
|------|-------|
| **Kp** | **70** |
| **Ki** | **60** |
| **Kd** | **4** |

### 5.2 Results

**Step response, open 0 → 90°** (`figures/01_step_response.png`):

| Metric | P-only (Kp = 70) | **Tuned PID** |
|--------|------------------|---------------|
| Rise time (10–90 %) | 0.72 s | 0.72 s |
| Overshoot | 5.6 % — *slams past target* | **1.5 %** |
| Peak angle | 95.0° | 91.4° |
| Settling (±2 %) | 1.40 s | **1.03 s** |
| Steady-state error | ~0° | ~0° |

**Control effort** (`figures/02_control_effort.png`) — voltage stays within the
±12 V rail and armature current peaks are bounded, confirming the actuator is not
being over-commanded.

**Full access cycle** (`figures/03_full_cycle.png`) — open → hold → close, all
under the same PID, showing smooth motion in both directions.

**Why the integral term is needed** (`figures/04_integral_hold.png`) — holding at
an intermediate 45° against gravity:

| Controller | Steady-state droop |
|------------|--------------------|
| PD only (Ki = 0) | **4.45°** |
| Full PID (Ki = 60) | **0.02°** |

Without integral action the arm droops ~4.5° below the commanded angle because
the constant gravity torque needs a constant holding effort that only the
integrator can supply. This is the concrete justification for a full PID rather
than a PD controller.

---

## 6. Implementation path

1. **Simulation (done)** — `boom_arm_pid.py` proves the plant + controller.
2. **MATLAB/Simulink** — `matlab/boom_arm_pid.m` reproduces the linear design and
   is ready for the **PID Tuner** app; a Simscape physical model (DC motor → gear
   → revolute joint with gravity load) can be dropped in for a higher-fidelity
   study, seeding the tuner with Kp = 70, Ki = 60, Kd = 4.
3. **Firmware** — `arduino/boom_arm_pid_controller.ino` implements the same loop
   at 100 Hz with anti-windup and auto-close. Gains are rescaled for a
   degrees→PWM actuator and should be re-verified on real hardware.

## 7. PLC / sequencing hook

The PID loop is *downstream* of the access logic. The intended control sequence:

```
person approaches
  -> face/RFID authorised?
       no  -> keep CLOSED
       yes -> ENTRY:  command OPEN -> hold -> auto-close
              EXIT:   check ferrous-metal sensor
                        clear  -> command OPEN -> hold -> auto-close
                        metal  -> keep CLOSED + ALARM
safety interlocks: obstruction detection freezes/reopens the arm;
                   power loss fails the arm to unlocked/open.
```

The `commandOpen()` / `commandClose()` calls in the Arduino sketch are the exact
integration points for this sequence (or a PLC ladder equivalent).

---

## 8. Honest modelling caveats

- Gravity is modelled; aerodynamic drag and stiction are not (viscous friction
  only) — reasonable for a slow pedestrian barrier.
- The quasi-static armature approximation is valid because `L/R ≪` mechanical
  time constants; a full electrical state can be added if a current-limited
  driver is modelled.
- Arduino gains are **rescaled** from SI and are a validated *starting point*,
  not a substitute for on-hardware tuning.
