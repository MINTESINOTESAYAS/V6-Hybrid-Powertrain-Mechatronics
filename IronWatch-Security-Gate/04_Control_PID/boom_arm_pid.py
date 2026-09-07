"""
IronWatch Security Gate - Boom Arm Dynamic Model & PID Controller Design
=======================================================================
Models the motorised swing-arm boom barrier of the IronWatch factory
security gate and designs a PID position controller for smooth,
non-slamming open/close motion.

Physical concept
----------------
A single boom arm (like a short pedestrian parking barrier) pivots about
one end, driven by a 12 V DC gearmotor. It rotates ~90 degrees between:
    theta = 0     -> CLOSED (arm horizontal, worst-case gravity load)
    theta = 90deg -> OPEN   (arm vertical, zero gravity torque)

The plant is a DC motor + gearbox + rigid arm with gravity, viscous
friction and reflected rotor inertia. A PID loop commands motor voltage
(saturated at +/- 12 V) to track a reference angle.

Author: continuation of the IronWatch project
Run:    python boom_arm_pid.py
Outputs: printed sizing report + PNG figures in ./figures/
"""

import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(HERE, "figures")
os.makedirs(FIGDIR, exist_ok=True)

# =====================================================================
# 1. DESIGN PARAMETERS  (defaults - pedestrian boom arm)
# =====================================================================
g = 9.81                     # gravity [m/s^2]

# --- Arm (modelled as a uniform slender rod pivoting about one end) ---
m_arm = 1.5                  # arm mass [kg]  (aluminium tube + skin)
L_arm = 1.0                  # arm length [m]
# Moment of inertia of a rod about its end: J = (1/3) m L^2
J_arm = (1.0 / 3.0) * m_arm * L_arm**2       # [kg m^2]
# Distance from pivot to centre of mass = L/2
r_cg = L_arm / 2.0

# --- 12 V DC gearmotor (typical small industrial gearmotor) ---
V_supply = 12.0              # supply voltage [V]
R_m = 2.0                    # armature resistance [ohm]
L_m = 0.5e-3                 # armature inductance [H]
Kt = 0.042                   # torque constant [N m / A]
Ke = 0.042                   # back-emf constant [V s / rad]
J_motor = 1.0e-5             # rotor inertia [kg m^2]
b_motor = 1.0e-5             # rotor viscous damping [N m s / rad]

# --- Gearbox ---
N_gear = 70.0                # gear reduction ratio (motor:arm)
eta_gear = 0.70              # gearbox efficiency

# --- Pivot / bearing friction referred to the arm shaft ---
b_pivot = 0.30               # viscous friction at arm pivot [N m s / rad]

# Inertia and damping reflected to the ARM shaft
J_total = J_arm + (N_gear**2) * J_motor       # [kg m^2] seen at arm
b_total = b_pivot + (N_gear**2) * b_motor     # [N m s/rad] seen at arm

# =====================================================================
# 2. STATIC / SIZING CALCULATIONS
# =====================================================================
# Worst-case gravity torque (arm horizontal, theta = 0)
tau_grav_max = m_arm * g * r_cg               # [N m] at the arm
# Motor no-load speed and stall torque
omega_nl_motor = V_supply / Ke                # [rad/s] no-load motor speed
tau_stall_motor = Kt * V_supply / R_m         # [N m] motor stall torque
# Reflected to the arm
omega_nl_arm = omega_nl_motor / N_gear        # [rad/s] arm no-load speed
tau_stall_arm = tau_stall_motor * N_gear * eta_gear   # [N m] arm stall torque
# Holding-torque safety factor against gravity
SF = tau_stall_arm / tau_grav_max


def print_sizing_report():
    print("=" * 66)
    print(" IRONWATCH BOOM ARM - SIZING & DYNAMIC MODEL REPORT")
    print("=" * 66)
    print("\n--- Arm ---")
    print(f"  mass                 m_arm     = {m_arm:.3f} kg")
    print(f"  length               L_arm     = {L_arm:.3f} m")
    print(f"  inertia (about end)  J_arm     = {J_arm:.4f} kg m^2   [ (1/3) m L^2 ]")
    print(f"  CG distance          r_cg      = {r_cg:.3f} m")
    print("\n--- Gearmotor (12 V DC) ---")
    print(f"  Kt = Ke              = {Kt:.4f} N m/A")
    print(f"  R_armature           = {R_m:.2f} ohm")
    print(f"  gear ratio N         = {N_gear:.0f}:1   efficiency = {eta_gear*100:.0f}%")
    print("\n--- Reflected to arm shaft ---")
    print(f"  total inertia        J_total   = {J_total:.4f} kg m^2")
    print(f"  total damping        b_total   = {b_total:.4f} N m s/rad")
    print("\n--- Static torque / speed check ---")
    print(f"  worst-case gravity torque (theta=0)  = {tau_grav_max:.3f} N m")
    print(f"  motor stall torque at arm            = {tau_stall_arm:.3f} N m")
    print(f"  --> holding safety factor            = {SF:.2f}x  "
          f"({'OK' if SF > 1.3 else 'TOO LOW'})")
    print(f"  motor no-load speed at arm           = {omega_nl_arm:.2f} rad/s "
          f"= {np.degrees(omega_nl_arm):.0f} deg/s")
    print(f"  (=> free-run 90 deg sweep ~ {np.radians(90)/omega_nl_arm:.2f} s, "
          f"loaded target 1.5 s is feasible)")
    print("=" * 66)


# =====================================================================
# 3. PLANT + PID SIMULATION
# =====================================================================
def simulate(Kp, Ki, Kd, theta_ref_deg=90.0, theta0_deg=0.0, t_end=4.0, dt=0.001,
             derivative_on_measurement=True, tau_d=0.02):
    """
    Integrate the full nonlinear plant with a discrete-ish PID loop.
    theta0_deg : initial arm angle (0=closed, 90=open).
    Returns time histories.
    """
    theta_ref = np.radians(theta_ref_deg)
    theta0 = np.radians(theta0_deg)

    n = int(t_end / dt) + 1
    t = np.linspace(0.0, t_end, n)
    theta = np.zeros(n)
    omega = np.zeros(n)
    i_arm = np.zeros(n)
    V_cmd = np.zeros(n)
    theta[0] = theta0

    integ = 0.0
    e_prev = theta_ref - theta0
    d_filt = 0.0

    for k in range(n - 1):
        th, om, ia = theta[k], omega[k], i_arm[k]

        # ---- PID controller ----
        e = theta_ref - th
        integ += e * dt
        if derivative_on_measurement:
            # derivative of measurement (avoids setpoint kick), low-pass filtered
            dmeas = (th - theta[k - 1]) / dt if k > 0 else 0.0
            d_filt += (dt / (tau_d + dt)) * (-dmeas - d_filt)
            deriv = d_filt
        else:
            deriv = (e - e_prev) / dt
        e_prev = e

        V = Kp * e + Ki * integ + Kd * deriv
        # Voltage saturation + simple anti-windup (clamp integrator on sat)
        V_sat = np.clip(V, -V_supply, V_supply)
        if V != V_sat:
            integ -= e * dt      # back-calculate: undo this step's integration
        V_cmd[k] = V_sat

        # ---- Plant dynamics (arm-shaft referred) ----
        # Quasi-static armature: L/R (~0.25 ms) << mechanical time constants,
        # so the current settles almost instantly -> algebraic solution.
        om_motor = N_gear * om
        ia_next = (V_sat - Ke * om_motor) / R_m
        tau_motor = Kt * ia_next
        tau_arm_drive = N_gear * eta_gear * tau_motor
        tau_gravity = m_arm * g * r_cg * np.cos(th)      # +ve pulls toward closed
        # mechanical (arm)
        dom = (tau_arm_drive - b_total * om - tau_gravity) / J_total
        dth = om

        # ---- Euler integrate (fine dt) ----
        theta[k + 1] = th + dth * dt
        omega[k + 1] = om + dom * dt
        i_arm[k + 1] = ia_next

        # hard stops at 0 and 95 deg (mechanical limit)
        if theta[k + 1] < 0.0:
            theta[k + 1] = 0.0
            omega[k + 1] = 0.0
        if theta[k + 1] > np.radians(95.0):
            theta[k + 1] = np.radians(95.0)
            omega[k + 1] = 0.0

    V_cmd[-1] = V_cmd[-2]
    return dict(t=t, theta=np.degrees(theta), omega=np.degrees(omega),
                i=i_arm, V=V_cmd, ref=theta_ref_deg)


def step_metrics(t, y, ref, y0=0.0):
    """Rise time (10-90%), overshoot %, settling time (2%), steady-state err.
    Works for both rising (open) and falling (close) steps."""
    y = np.asarray(y)
    span = ref - y0
    if abs(span) < 1e-9:
        return dict(rise=np.nan, overshoot=0.0, settle=0.0, sse=ref - y[-1], peak=y[-1])
    frac = (y - y0) / span            # 0 at start, 1 at target regardless of direction
    try:
        t10 = t[np.where(frac >= 0.1)[0][0]]
        t90 = t[np.where(frac >= 0.9)[0][0]]
        rise = t90 - t10
    except IndexError:
        rise = np.nan
    peak_frac = frac.max()
    overshoot = max(0.0, (peak_frac - 1.0) * 100.0)
    band = 0.02 * abs(span)
    outside = np.where(np.abs(y - ref) > band)[0]
    settle = t[outside[-1]] if len(outside) else 0.0
    sse = ref - y[-1]
    peak = y0 + peak_frac * span
    return dict(rise=rise, overshoot=overshoot, settle=settle, sse=sse, peak=peak)


def auto_tune(t_end=4.0):
    """Coarse grid search minimising a weighted cost of overshoot + settling +
    steady-state error, evaluated on the harder CLOSING move (gravity-assisted,
    where overshoot toward the 0 deg hard stop = slamming)."""
    best = None
    for Kp in (20, 30, 40, 55, 70):
        for Ki in (0, 5, 10, 20):
            for Kd in (4, 8, 14, 22):
                s = simulate(Kp, Ki, Kd, theta_ref_deg=0.0, theta0_deg=90.0,
                             t_end=t_end)
                m = step_metrics(s["t"], s["theta"], 0.0, y0=90.0)
                cost = (m["overshoot"] * 1.0
                        + m["settle"] * 8.0
                        + abs(m["sse"]) * 5.0
                        + (m["rise"] if not np.isnan(m["rise"]) else 5.0) * 3.0)
                if best is None or cost < best[0]:
                    best = (cost, (Kp, Ki, Kd), m)
    return best


# =====================================================================
# 4. MAIN
# =====================================================================
if __name__ == "__main__":
    print_sizing_report()

    # ---- Automatic coarse tuning on the harder CLOSING move ----
    cost, (Kp, Ki, Kd), _ = auto_tune()
    print("\n--- AUTO-TUNE RESULT (grid search, closing move) ---")
    print(f"  best gains:  Kp = {Kp}   Ki = {Ki}   Kd = {Kd}   (cost = {cost:.2f})")

    # ---- Aggressive P-only baseline: fast but SLAMS (large overshoot) ----
    Kp_slam = 70.0
    p_only = simulate(Kp=Kp_slam, Ki=0.0, Kd=0.0, theta_ref_deg=90.0)
    m_p = step_metrics(p_only["t"], p_only["theta"], 90.0, y0=0.0)

    # ---- Tuned PID (opening move for the headline figure) ----
    pid = simulate(Kp, Ki, Kd, theta_ref_deg=90.0)
    m_pid = step_metrics(pid["t"], pid["theta"], 90.0, y0=0.0)

    print("\n--- CONTROLLER PERFORMANCE (open 0->90 deg command) ---")
    print(f"{'metric':<24}{'P-only (Kp=70)':>18}{'Tuned PID':>16}")
    print(f"{'Kp,Ki,Kd':<24}{f'{Kp_slam:.0f}, 0, 0':>18}{f'{Kp:.0f}, {Ki:.0f}, {Kd:.0f}':>16}")
    print(f"{'rise time 10-90% [s]':<24}{m_p['rise']:>18.3f}{m_pid['rise']:>16.3f}")
    print(f"{'overshoot [%]':<24}{m_p['overshoot']:>18.2f}{m_pid['overshoot']:>16.2f}")
    print(f"{'peak angle [deg]':<24}{m_p['peak']:>18.2f}{m_pid['peak']:>16.2f}")
    print(f"{'settling 2% [s]':<24}{m_p['settle']:>18.3f}{m_pid['settle']:>16.3f}")
    print(f"{'steady-state err [deg]':<24}{m_p['sse']:>18.3f}{m_pid['sse']:>16.3f}")

    # =============================================================
    # FIGURE 1: step response comparison
    # =============================================================
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.axhline(90, color="gray", ls="--", lw=1, label="reference (90 deg)")
    ax.axhspan(90 * 0.98, 90 * 1.02, color="green", alpha=0.10,
               label="+/-2% settling band")
    ax.plot(p_only["t"], p_only["theta"], color="#c0392b", lw=2,
            label=f"P-only (Kp={Kp_slam:.0f}): SLAMS, overshoot {m_p['overshoot']:.0f}%")
    ax.plot(pid["t"], pid["theta"], color="#1f77b4", lw=2.5,
            label=f"Tuned PID: overshoot {m_pid['overshoot']:.1f}%, "
                  f"settle {m_pid['settle']:.2f}s")
    ax.set_xlabel("time [s]"); ax.set_ylabel("arm angle [deg]")
    ax.set_ylim(0, 115)
    ax.set_title("IronWatch Boom Arm - PID Position Step Response (Open 0->90 deg)")
    ax.legend(loc="lower right"); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "01_step_response.png"), dpi=130)

    # =============================================================
    # FIGURE 2: control effort (voltage) & motor current
    # =============================================================
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    a1.plot(pid["t"], pid["V"], color="#8e44ad", lw=2)
    a1.axhline(V_supply, color="r", ls=":", lw=1); a1.axhline(-V_supply, color="r", ls=":", lw=1)
    a1.set_ylabel("motor voltage [V]"); a1.grid(alpha=0.3)
    a1.set_title("Control Effort (PID output, saturated at +/-12 V)")
    a2.plot(pid["t"], pid["i"], color="#e67e22", lw=2)
    a2.set_ylabel("armature current [A]"); a2.set_xlabel("time [s]"); a2.grid(alpha=0.3)
    a2.set_title("Motor Armature Current")
    fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "02_control_effort.png"), dpi=130)

    # =============================================================
    # FIGURE 3: full open -> hold -> close cycle
    # =============================================================
    # open to 90 (from closed), hold, then close back to 0 (from open)
    openc = simulate(Kp, Ki, Kd, theta_ref_deg=90.0, theta0_deg=0.0, t_end=2.5)
    closec = simulate(Kp, Ki, Kd, theta_ref_deg=0.0, theta0_deg=90.0, t_end=2.5)
    hold_t = np.linspace(0, 1.5, 200)
    t_all = np.concatenate([openc["t"],
                            openc["t"][-1] + hold_t,
                            openc["t"][-1] + hold_t[-1] + closec["t"]])
    th_all = np.concatenate([openc["theta"],
                             np.full_like(hold_t, openc["theta"][-1]),
                             closec["theta"]])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t_all, th_all, color="#16a085", lw=2.5)
    ax.axhline(90, color="gray", ls="--", lw=1)
    ax.axhline(0, color="gray", ls="--", lw=1)
    ax.text(0.5, 82, "OPEN", color="#16a085")
    ax.text(3.0, 82, "HOLD", color="gray")
    ax.text(5.2, 8, "CLOSE", color="#16a085")
    ax.set_xlabel("time [s]"); ax.set_ylabel("arm angle [deg]")
    ax.set_title("Full Access Cycle: Open -> Hold -> Close (PID controlled)")
    ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "03_full_cycle.png"), dpi=130)

    # =============================================================
    # FIGURE 4: why the integral term is needed
    # Holding the arm at an intermediate 45 deg (gravity torque non-zero)
    # PD alone droops below target; adding I removes the steady-state error.
    # =============================================================
    Ki_final = 60.0
    hold_pd = simulate(Kp, 0.0, Kd, theta_ref_deg=45.0, theta0_deg=0.0, t_end=6.0)
    hold_pid = simulate(Kp, Ki_final, Kd, theta_ref_deg=45.0, theta0_deg=0.0, t_end=6.0)
    droop_pd = 45.0 - hold_pd["theta"][-1]
    droop_pid = 45.0 - hold_pid["theta"][-1]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.axhline(45, color="gray", ls="--", lw=1, label="reference (45 deg hold)")
    ax.plot(hold_pd["t"], hold_pd["theta"], color="#c0392b", lw=2,
            label=f"PD only (Ki=0): droops {droop_pd:.2f} deg (gravity)")
    ax.plot(hold_pid["t"], hold_pid["theta"], color="#1f77b4", lw=2.5,
            label=f"Full PID (Ki={Ki_final:.0f}): droop {droop_pid:.2f} deg")
    ax.set_xlabel("time [s]"); ax.set_ylabel("arm angle [deg]")
    ax.set_title("Why Integral Action Matters - Holding at 45 deg Against Gravity")
    ax.legend(loc="lower right"); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "04_integral_hold.png"), dpi=130)

    print("\n--- INTEGRAL-ACTION HOLD TEST (45 deg, gravity load) ---")
    print(f"  PD  (Ki=0)      steady-state droop = {droop_pd:.3f} deg")
    print(f"  PID (Ki={Ki_final:.0f})     steady-state droop = {droop_pid:.3f} deg")

    print(f"\nFigures written to: {FIGDIR}")
    print("  01_step_response.png, 02_control_effort.png,")
    print("  03_full_cycle.png, 04_integral_hold.png")
    print("\n" + "=" * 66)
    print(" FINAL RECOMMENDED PID GAINS (implement in Arduino / PLC):")
    print(f"   Kp = {Kp:.1f}    Ki = {Ki_final:.1f}    Kd = {Kd:.1f}")
    print("=" * 66)
