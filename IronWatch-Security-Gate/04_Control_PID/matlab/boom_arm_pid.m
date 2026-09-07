%% IronWatch Security Gate - Boom Arm Dynamic Model & PID Design (MATLAB)
%  Equivalent of boom_arm_pid.py for use with MATLAB / PID Tuner.
%  Requires: Control System Toolbox (for pidtune / step). Falls back to a
%  manual gain set if the toolbox is not present.
%
%  Plant: 12 V DC gearmotor + slender-rod boom arm + gravity + friction.
%  theta = 0   -> CLOSED (horizontal, worst-case gravity torque)
%  theta = 90  -> OPEN   (vertical, zero gravity torque)
%
%  Run:  boom_arm_pid   (in MATLAB, from this folder)

clear; clc; close all;

%% 1. Parameters ---------------------------------------------------------
g      = 9.81;              % gravity [m/s^2]
m_arm  = 1.5;              % arm mass [kg]
L_arm  = 1.0;              % arm length [m]
J_arm  = (1/3)*m_arm*L_arm^2;   % rod-about-end inertia [kg m^2]
r_cg   = L_arm/2;          % CG distance [m]

V_sup  = 12;              % supply voltage [V]
Rm     = 2.0;             % armature resistance [ohm]
Kt     = 0.042;           % torque constant [N m/A]
Ke     = 0.042;           % back-emf constant [V s/rad]
Jm     = 1e-5;            % rotor inertia [kg m^2]
bm     = 1e-5;            % rotor damping [N m s/rad]
N      = 70;              % gear ratio
eta    = 0.70;            % gearbox efficiency
b_piv  = 0.30;            % pivot viscous friction [N m s/rad]

Jt = J_arm + N^2*Jm;      % inertia at arm shaft
bt = b_piv + N^2*bm;      % damping at arm shaft

%% 2. Sizing report ------------------------------------------------------
tau_grav_max = m_arm*g*r_cg;                 % worst-case gravity torque
tau_stall_arm = (Kt*V_sup/Rm)*N*eta;         % stall torque at arm
SF = tau_stall_arm/tau_grav_max;
fprintf('Worst-case gravity torque : %.3f N m\n', tau_grav_max);
fprintf('Stall torque at arm       : %.3f N m\n', tau_stall_arm);
fprintf('Holding safety factor     : %.2fx\n\n', SF);

%% 3. Linearised plant about the operating point -------------------------
% With quasi-static armature (L/R negligible), motor torque at arm:
%   tau = (N*eta*Kt/Rm)*V - (N^2*eta*Kt*Ke/Rm)*omega
% Plant:  Jt*ddtheta + (bt + N^2*eta*Kt*Ke/Rm)*dtheta = (N*eta*Kt/Rm)*V
Kv = N*eta*Kt/Rm;                 % voltage->torque gain
be = bt + N^2*eta*Kt*Ke/Rm;       % effective damping incl. back-emf
% Transfer function theta(s)/V(s) = Kv / (Jt s^2 + be s)
num = Kv;
den = [Jt, be, 0];
P = tf(num, den);

%% 4. PID design ---------------------------------------------------------
useToolbox = license('test','Control_Toolbox') && exist('pidtune','file')==2;
if useToolbox
    % Tune for ~4 rad/s bandwidth, phase margin 70 deg (smooth, low overshoot)
    opts = pidtuneOptions('PhaseMargin',70);
    C = pidtune(P, 'PID', 4.0, opts);
    fprintf('pidtune gains: Kp=%.2f Ki=%.2f Kd=%.2f\n', C.Kp, C.Ki, C.Kd);
else
    % Manual gains matching the Python design
    C = pid(70, 60, 4);
    fprintf('Manual gains : Kp=70 Ki=60 Kd=4 (Control Toolbox not found)\n');
end

%% 5. Closed-loop step response -----------------------------------------
sys_cl = feedback(C*P, 1);
figure('Name','Step Response');
step(sys_cl, 4); grid on;
title('IronWatch Boom Arm - Closed-Loop Step Response (linearised)');
S = stepinfo(sys_cl);
fprintf('\nClosed-loop step metrics:\n');
fprintf('  Rise time    : %.3f s\n', S.RiseTime);
fprintf('  Overshoot    : %.2f %%\n', S.Overshoot);
fprintf('  Settling     : %.3f s\n', S.SettlingTime);

%% 6. Notes -------------------------------------------------------------
% - This linearised model ignores gravity (a load disturbance). The
%   nonlinear Python model (boom_arm_pid.py) includes the gravity term
%   tau = m*g*r_cg*cos(theta) and shows why integral action is needed to
%   hold intermediate angles without droop.
% - For a full physical model, build the plant in Simulink/Simscape:
%     Simscape Electrical DC Motor -> Gear -> Revolute Joint (arm) with a
%     gravity-dependent load torque, then use the PID Tuner app on the
%     linearised subsystem and drop in Kp=70, Ki=60, Kd=4 as a start point.
