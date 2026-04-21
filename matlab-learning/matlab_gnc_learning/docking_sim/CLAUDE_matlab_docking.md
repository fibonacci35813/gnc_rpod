# MATLAB AUTONOMOUS DOCKING SIMULATION
## Complete Mission — Point A to Point B to Dock
## CLAUDE.md Sprint File

---

## AUTONOMOUS OPERATION — READ THIS FIRST

You are building a complete end-to-end autonomous spacecraft docking
simulation in MATLAB. A chaser starts 200m behind a target in ISS
orbit and autonomously navigates, controls, and docks. Every physical
force is modelled. The output is a working MATLAB simulation with
full telemetry plots and a 3D animation of the docking.

Execute all phases in order. Do not stop between phases.
When complete, write DONE.md with every file created and every
verification result.

If a phase fails after three attempts, write BLOCKERS.md and
continue to the next phase.

---

## WHAT TO BUILD

```
MISSION: Chaser at [0, 200, 0] m LVLH  ->  Target at [0, 0, 0] m LVLH

PHYSICS (true dynamics):
  - Two-body gravity (mu = 3.986004418e14 m^3/s^2)
  - J2 oblateness perturbation
  - Atmospheric drag (exponential model, rho0=1.225, H=8500m)
  - Solar radiation pressure (simplified, Cr=1.3)
  - Thruster blow-down model (pressure decreases as fuel consumed)

NAVIGATION (onboard — does NOT know J2/drag):
  - Extended Kalman Filter, 6 states: [pos(3), vel(3)] in LVLH
  - LIDAR RAE sensor: sigma = max(0.002, 0.003*range) metres
  - IMU: accelerometer + gyro bias drift at 10 Hz inner loop
  - CW state transition matrix for EKF propagation

GUIDANCE (4 phases, V-bar approach):
  Phase 0: range > 50m    v_max = 1.00 m/s
  Phase 1: range > 10m    v_max = 0.50 m/s
  Phase 2: range > 1m     v_max = 0.08 m/s
  Phase 3: range <= 1m    v_max = 0.04 m/s

CONTROL:
  - PD per axis, gain scheduled: normal vs terminal
  - Normal:   Kp=[0.30,0.20,0.30], Kd=[20,20,20]
  - Terminal: Kp=[1.20,1.20,1.20], Kd=[42,42,42]
  - MIB deadband: 0.1 N.s normal, 0.005 N.s terminal

ATTITUDE:
  - Quaternion kinematics (symplectic Euler, stable at dt=1s)
  - PD torque controller
  - Reaction wheels (3 orthogonal, saturation at 500 rad/s)
  - Terminal: align -y face toward docking port

PROPULSION:
  - 6 thrusters (+/-x, +/-y, +/-z)
  - Blow-down tank: P0=2MPa, prop=5kg, Isp=220s, mass=500kg

FDIR:
  - Stuck-open: |F_actual|>0.1N when |F_cmd|<0.05N
  - Stuck-closed: |F_cmd|>0.5N when |F_actual|<0.05N
  - Sensor dropout: no valid meas for >3 steps
  - Nav diverge: covariance trace > 1e4

MISSION MANAGER:
  - NOMINAL -> HOLD -> RETREAT (y=200m) -> SAFE (y=500m)
  - Stuck-open: immediate RETREAT, latched

DOCKING SUCCESS:
  - pos_error < 0.05m AND vel_error < 0.01 m/s AND att < 1 deg
  - All three held for 30 consecutive steps

OUTPUTS:
  - 8 telemetry plots
  - 3D animation
  - Monte Carlo (50 seeds)
  - DONE.md
```

---

## FILE STRUCTURE

```
docking_sim/
|-- main_sim.m              <- RUN THIS
|-- run_monte_carlo.m
|-- animate_docking.m
|-- run_single_sim.m        <- function wrapper for MC
|
|-- environment/
|   |-- Rx.m
|   |-- Ry.m
|   |-- Rz.m
|   |-- oe2rv.m
|   |-- eci_to_lvlh.m
|   |-- two_body.m
|   |-- eom_perturbed.m
|   |-- j2_accel.m
|   |-- drag_accel.m
|   +-- srp_accel.m
|
|-- sensors/
|   |-- lidar_rae.m
|   +-- imu_model.m
|
|-- navigation/
|   |-- ekf_init.m
|   |-- ekf_predict.m
|   +-- ekf_update.m
|
|-- guidance/
|   +-- guidance_law.m
|
|-- control/
|   +-- pd_controller.m
|
|-- attitude/
|   |-- quat_kinematics.m
|   |-- quat_multiply.m
|   |-- rot2quat.m
|   |-- attitude_control.m
|   +-- rw_model.m
|
|-- propulsion/
|   +-- blowdown_model.m
|
|-- fdir/
|   +-- fdir_check.m
|
|-- mission/
|   +-- mission_manager.m
|
+-- plots/
    +-- plot_all_telemetry.m
```

---

## PHASE 1 - ENVIRONMENT

### Rx.m
```matlab
function R = Rx(angle)
R = [1,0,0; 0,cos(angle),-sin(angle); 0,sin(angle),cos(angle)];
end
```

### Ry.m
```matlab
function R = Ry(angle)
R = [cos(angle),0,sin(angle); 0,1,0; -sin(angle),0,cos(angle)];
end
```

### Rz.m
```matlab
function R = Rz(angle)
R = [cos(angle),-sin(angle),0; sin(angle),cos(angle),0; 0,0,1];
end
```

### oe2rv.m
```matlab
function [r_eci, v_eci] = oe2rv(a, e, inc, RAAN, omega, nu, mu)
p     = a*(1-e^2);
r_mag = p/(1+e*cos(nu));
r_pqw = r_mag*[cos(nu); sin(nu); 0];
v_pqw = sqrt(mu/p)*[-sin(nu); e+cos(nu); 0];
R     = Rz(-RAAN)*Rx(-inc)*Rz(-omega);
r_eci = R*r_pqw;
v_eci = R*v_pqw;
end
```

### eci_to_lvlh.m
```matlab
function [pos_lvlh, vel_lvlh, R_eci2lvlh] = eci_to_lvlh(r_tgt,v_tgt,r_chs,v_chs)
r_hat = r_tgt/norm(r_tgt);
h_vec = cross(r_tgt,v_tgt);
h_hat = h_vec/norm(h_vec);
y_hat = cross(h_hat,r_hat);
R_eci2lvlh = [r_hat,y_hat,h_hat]';
dr = r_chs - r_tgt;
dv = v_chs - v_tgt;
omega_vec = h_hat*norm(h_vec)/norm(r_tgt)^2;
pos_lvlh = R_eci2lvlh*dr;
vel_lvlh = R_eci2lvlh*(dv - cross(omega_vec,dr));
end
```

### j2_accel.m
```matlab
function a = j2_accel(r_vec, mu, Re, J2)
x=r_vec(1); y=r_vec(2); z=r_vec(3); r=norm(r_vec);
c = -(3/2)*J2*(mu/r^2)*(Re/r)^2;
fxy = 1-5*(z/r)^2; fz = 3-5*(z/r)^2;
a = c*[(x/r)*fxy; (y/r)*fxy; (z/r)*fz];
end
```

### drag_accel.m
```matlab
function a = drag_accel(r_vec, v_vec, Cd, A, m, Re)
alt  = norm(r_vec)-Re;
rho  = 1.225*exp(-alt/8500);
vmag = norm(v_vec);
if vmag<1e-10; a=zeros(3,1); return; end
a = -0.5*rho*Cd*(A/m)*vmag*v_vec;
end
```

### srp_accel.m
```matlab
function a = srp_accel(r_vec, A, m, Cr)
P_srp = 4.56e-6;
r_sun = [1;0;0];
if dot(r_vec/norm(r_vec),r_sun)>0
    a = (Cr*P_srp*A/m)*r_sun;
else
    a = zeros(3,1);
end
end
```

### two_body.m
```matlab
function dX = two_body(~,X,mu)
r=X(1:3); v=X(4:6); rm=norm(r);
dX=[v; -(mu/rm^3)*r];
end
```

### eom_perturbed.m
```matlab
function dX = eom_perturbed(~,X,mu,Re,J2,Cd,A,m,Cr)
r=X(1:3); v=X(4:6); rm=norm(r);
ag = -(mu/rm^3)*r;
aj = j2_accel(r,mu,Re,J2);
ad = drag_accel(r,v,Cd,A,m,Re);
as = srp_accel(r,A,m,Cr);
dX = [v; ag+aj+ad+as];
end
```

### Phase 1 Gate
```
[r,v] = oe2rv(6.771e6, 0, deg2rad(51.6), 0, 0, 0, mu)
norm(v) vs 6.771e6*sqrt(mu/6.771e6^3): error < 1e-4%
One orbit closure < 1m
Initial LVLH offset = [0,200,0] +/- 0.1m
PRINT: PHASE 1 PASS
```

---

## PHASE 2 - SENSORS

### lidar_rae.m
```matlab
function meas = lidar_rae(pos_true, rng_obj)
r   = norm(pos_true);
az  = atan2(pos_true(2),pos_true(1));
el  = asin(pos_true(3)/max(r,0.001));
sr  = max(0.002, 0.003*r);
sa  = 0.001; se = 0.001;
n   = rng_obj.randn(3,1);
meas.range = r  + sr*n(1);
meas.az    = az + sa*n(2);
meas.el    = el + se*n(3);
meas.valid = r > 0.04;
meas.sigma = [sr;sa;se];
end
```

### imu_model.m
```matlab
function [am, bias_out] = imu_model(a_true, bias_in, dt, params, rng_obj)
bias_out = bias_in + params.bias_instability*sqrt(dt)*rng_obj.randn(3,1);
am = a_true + bias_out + params.accel_noise*rng_obj.randn(3,1);
end
```

### Phase 2 Gate
```
sigma at 200m range: ~0.60m
sigma at 1m range:   ~0.003m
PRINT: PHASE 2 PASS
```

---

## PHASE 3 - EKF

### ekf_init.m
```matlab
function ekf = ekf_init(pos0, vel0, n)
ekf.x = [pos0;vel0];
ekf.P = diag([9,9,9,0.01,0.01,0.01]);
ekf.Q = diag([1e-4,1e-4,1e-4,1e-6,1e-6,1e-6]);
ekf.n = n;
end
```

### ekf_predict.m
```matlab
function ekf = ekf_predict(ekf, u, dt)
n=ekf.n; c=cos(n*dt); s=sin(n*dt);
Phi=[4-3*c,      0, 0, s/n,        2*(1-c)/n,    0;
     6*(s-n*dt), 1, 0, -2*(1-c)/n, (4*s-3*n*dt)/n,0;
     0,          0, c, 0,           0,             s/n;
     3*n*s,      0, 0, c,           2*s,           0;
     -6*n*(1-c), 0, 0, -2*s,        4*c-3,         0;
     0,          0,-n*s,0,           0,             c];
B=[dt^2/2*eye(3); dt*eye(3)];
ekf.x = Phi*ekf.x + B*u;
ekf.P = Phi*ekf.P*Phi' + ekf.Q;
end
```

### ekf_update.m
```matlab
function ekf = ekf_update(ekf, meas)
if ~meas.valid; return; end
p=ekf.x(1:3); r=norm(p);
if r<0.01; return; end
rxy=max(sqrt(p(1)^2+p(2)^2),1e-6);
h=[r; atan2(p(2),p(1)); asin(p(3)/r)];
H=zeros(3,6);
H(1,1:3)=p'/r;
H(2,1)=-p(2)/rxy^2; H(2,2)=p(1)/rxy^2;
H(3,1)=-p(1)*p(3)/(r^2*rxy);
H(3,2)=-p(2)*p(3)/(r^2*rxy);
H(3,3)=rxy/r^2;
z=[meas.range;meas.az;meas.el];
inn=z-h; inn(2)=mod(inn(2)+pi,2*pi)-pi;
R=diag(meas.sigma.^2);
S=H*ekf.P*H'+R; K=ekf.P*H'/S;
ekf.x=ekf.x+K*inn;
ekf.P=(eye(6)-K*H)*ekf.P;
end
```

### Phase 3 Gate
```
After 200 steps: nav pos error < 2m
Covariance trace decreasing after step 50
PRINT: PHASE 3 PASS
```

---

## PHASE 4 - GUIDANCE

### guidance_law.m
```matlab
function [pos_ref,vel_ref,phase,v_cmd] = guidance_law(pos_est,vel_est,params)
r=norm(pos_est);
if r>50;     phase=0; vmax=params.v_phase_max(1);
elseif r>10; phase=1; vmax=params.v_phase_max(2);
elseif r>1;  phase=2; vmax=params.v_phase_max(3);
else;        phase=3; vmax=params.v_phase_max(4);
end
v_cmd=min(vmax, params.K_V*r);
if r>0.001; vel_ref=-v_cmd*pos_est/r; else; vel_ref=zeros(3,1); end
pos_ref=pos_est;
end
```

### Phase 4 Gate
```
At r=200m: v_cmd = min(1.0, 0.01*200) = 1.0 m/s
At r=10m:  v_cmd = min(0.08, 0.01*10) = 0.08 m/s
At r=0.5m: v_cmd = 0.005 m/s
PRINT: PHASE 4 PASS
```

---

## PHASE 5 - CONTROL AND PROPULSION

### pd_controller.m
```matlab
function F = pd_controller(pos_err, vel_err, phase, params)
if phase==3; Kp=params.Kp_terminal; Kd=params.Kd_terminal; mib=params.mib_terminal;
else;        Kp=params.Kp_normal;   Kd=params.Kd_normal;   mib=params.mib_normal; end
F=Kp.*pos_err + Kd.*vel_err;
F=sign(F).*min(abs(F),50);
F(abs(F)*1.0 < mib)=0;
end
```

### blowdown_model.m
```matlab
function [F_act, tank] = blowdown_model(F_cmd, tank)
V_gas = tank.V_total - tank.prop_remain/tank.rho_prop;
P     = tank.P0*tank.V_gas0/max(V_gas,tank.V_gas0);
P     = max(P, tank.P_min);
F_act = F_cmd*(P/tank.P_nom);
g0    = 9.80665;
dv    = norm(F_act)/tank.mass;
dm    = tank.mass*(1-exp(-dv/(tank.Isp*g0)));
tank.prop_remain = max(0, tank.prop_remain-dm);
tank.mass        = tank.mass-dm;
tank.P_current   = P;
tank.dv_total    = tank.dv_total+dv;
end
```

### Phase 5 Gate
```
MIB deadband working: small commands produce zero force
Pressure decreasing monotonically as prop consumed
PRINT: PHASE 5 PASS
```

---

## PHASE 6 - ATTITUDE

### quat_multiply.m
```matlab
function q=quat_multiply(q1,q2)
s1=q1(1);v1=q1(2:4); s2=q2(1);v2=q2(2:4);
q=[s1*s2-dot(v1,v2); s1*v2+s2*v1+cross(v1,v2)];
q=q/norm(q);
end
```

### rot2quat.m
```matlab
function q=rot2quat(R)
tr=R(1,1)+R(2,2)+R(3,3);
if tr>0; s=0.5/sqrt(tr+1);
    q=[0.25/s;(R(3,2)-R(2,3))*s;(R(1,3)-R(3,1))*s;(R(2,1)-R(1,2))*s];
elseif R(1,1)>R(2,2)&&R(1,1)>R(3,3); s=2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
    q=[(R(3,2)-R(2,3))/s;0.25*s;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
elseif R(2,2)>R(3,3); s=2*sqrt(1+R(2,2)-R(1,1)-R(3,3));
    q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;0.25*s;(R(2,3)+R(3,2))/s];
else; s=2*sqrt(1+R(3,3)-R(1,1)-R(2,2));
    q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;0.25*s];
end
q=q/norm(q);
end
```

### quat_kinematics.m
```matlab
function q=quat_kinematics(q,omega,dt)
q0=q(1);v=q(2:4);
Xi=[-v';q0*eye(3)+skew(v)];
q=q+0.5*dt*[dot(-v,omega);q0*omega+cross(v,omega)];
q=q/norm(q);
end
function S=skew(v)
S=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
end
```

### attitude_control.m
```matlab
function [tau,err_deg]=attitude_control(q,q_ref,omega,params)
q_ref_inv=[q_ref(1);-q_ref(2:4)];
q_err=quat_multiply(q_ref_inv,q);
err_deg=2*acosd(min(1,abs(q_err(1))));
ev=q_err(2:4)*sign(q_err(1));
tau=-params.Kp_att*ev - params.Kd_att*omega;
tau=sign(tau).*min(abs(tau),0.5);
end
```

### rw_model.m
```matlab
function [h_new,tau_act]=rw_model(h_rw,tau_cmd,params,dt)
sat=abs(h_rw/params.I_rw)>params.omega_max;
tau_act=tau_cmd; tau_act(sat)=0;
h_new=h_rw+tau_act*dt;
end
```

### Phase 6 Gate
```
Quaternion norm = 1.000 throughout
Attitude error converges < 1 deg within 200 steps
PRINT: PHASE 6 PASS
```

---

## PHASE 7 - FDIR AND MISSION MANAGER

### fdir_check.m
```matlab
function [code,msg]=fdir_check(F_cmd,F_act,valid,dropout,cov_tr,p)
code=0; msg='NOMINAL';
if norm(F_act)>p.thr_stuck_open && norm(F_cmd)<p.thr_cmd_zero
    code=1; msg='STUCK_OPEN'; return; end
if norm(F_cmd)>p.thr_cmd_min && norm(F_act)<p.thr_stuck_closed
    code=2; msg='STUCK_CLOSED'; return; end
if dropout>p.dropout_limit
    code=3; msg='DROPOUT'; return; end
if cov_tr>p.cov_max
    code=4; msg='NAV_DIVERGE'; return; end
end
```

### mission_manager.m
```matlab
function [mode_out,override]=mission_manager(mode_in,fault,pos,timer,p)
override=[]; mode_out=mode_in;
switch mode_in
    case 0  % NOMINAL
        if fault>0; mode_out=1; end
    case 1  % HOLD
        if fault==1; mode_out=2;            % stuck-open: immediate retreat
        elseif fault==0&&timer>p.hold_timeout; mode_out=0;  % de-escalate
        elseif timer>p.hold_max; mode_out=2; end  % escalate
        override=pos;
    case 2  % RETREAT
        if timer>p.retreat_timeout; mode_out=3; end
        override=[0;200;0];
    case 3  % SAFE
        override=[0;500;0];
end
end
```

### Phase 7 Gate
```
Inject fault_code=1 at step 300
Mode transitions to RETREAT within 10 steps
Vehicle y > 50m at step 380
PRINT: PHASE 7 PASS
```

---

## PHASE 8 - MAIN SIMULATION LOOP (main_sim.m)

Wire all modules. The loop structure must be:

```
FOR each step k = 1:N_MAX:
  1.  Propagate true target ECI (ode45, eom_perturbed)
  2.  Propagate true chaser ECI (ode45, eom_perturbed)
  3.  Compute true LVLH relative state (eci_to_lvlh)
  4.  IMU inner loop x10 (ekf_predict at 10 Hz)
  5.  LIDAR measurement (lidar_rae)
  6.  EKF update if valid (ekf_update)
  7.  CW velocity correction: use ekf.x(1:3) for pos,
      recompute vel from CW model (prevents IMU noise corruption)
  8.  FDIR check (fdir_check)
  9.  Mission manager (mission_manager)
  10. Guidance (guidance_law, override if retreat/safe)
  11. Attitude reference (identity normal, -y toward target terminal)
  12. Attitude control (attitude_control)
  13. Reaction wheel update (rw_model)
  14. Attitude integration (quat_kinematics)
  15. Translational PD control (pd_controller)
  16. Blow-down propulsion (blowdown_model)
  17. Apply force to chaser ECI state
  18. Docking check (30-step hold)
  19. Log all telemetry
  20. Print progress every 100 steps
END
```

Apply force to chaser after ode45 by adding impulse to velocity:
```matlab
force_eci  = R_eci2lvlh' * force_act;
delta_v_eci = force_eci / tank.mass * dt;
X_chs(4:6) = X_chs(4:6) + delta_v_eci;
```

Print every 100 steps:
```
[XXXXX] t=XXXXX s  range=XXX.XXm  phase=X  dv=X.XXX m/s  mode=NOMINAL
```

Print on dock:
```
[XXXXX] *** DOCKED ***
  Range    : X.XXXX m
  Velocity : X.XXXX m/s
  Attitude : X.XX deg
  Delta-V  : X.XXX m/s
  Time     : XXXX s (XX.X min)
```

---

## PHASE 9 - ALL 8 PLOTS (plot_all_telemetry.m)

Generate these 8 figures in a single function call:

```
Figure 1: Range vs Time (semilogy, red dashed 5cm line)
Figure 2: 3D LVLH Trajectory (plot3, black bg, start=green, port=red*)
Figure 3: Navigation Error per axis (3 subplots, true-estimated)
Figure 4: Guidance Phase (stairs plot, labelled yticks)
Figure 5: Delta-V Budget (cumulative)
Figure 6: Tank Blow-Down Pressure (MPa)
Figure 7: Attitude Error vs Time (deg, 1 deg limit line)
Figure 8: FDIR/Mission Mode (2 subplots: mode and fault code)
```

Save all to plots/ directory as PNG files.

---

## PHASE 10 - 3D ANIMATION (animate_docking.m)

Black background. Show:
- Full trajectory path in dim blue
- Docking port as red star at origin
- Start position as green square
- Animated chaser as blue square moving along path
- Text overlay: range, phase name, dv
- Skip every 5 frames for smooth playback
- Final title turns green: "DOCKED"

---

## PHASE 11 - MONTE CARLO (run_monte_carlo.m and run_single_sim.m)

### run_single_sim.m
Wrap main loop as a function:
```matlab
function [docked,steps,dv,pos_err,vel_err] = run_single_sim(dr0,dv0,thr_scale,seed)
```
Accepts dispersed ICs, returns scalar results.
All params same as main_sim except ICs and thr_scale applied to blowdown.

### run_monte_carlo.m
- N_MC = 50 seeds
- Disperse: pos +/-3m Gaussian, vel +/-0.05 m/s, thr_scale +/-5%
- Print per-seed result
- Print summary: P(dock), mean dv, mean pos_err, mean steps
- Generate 3 histogram plots: dv, pos_err, steps

---

## MASTER GATE

Run in MATLAB command window after all phases:

```matlab
% Gate 1: Core sim
main_sim
% Must print: DOCKED YES, pos<0.05m, vel<0.01 m/s, att<1 deg

% Gate 2: Plots
% 8 figures must appear, all physically sensible

% Gate 3: Animation
animate_docking(telem, step)
% 3D playback must show chaser approaching and docking

% Gate 4: Monte Carlo
run_monte_carlo
% P(dock) >= 0.90 across 50 seeds
```

---

## DONE.md - Write when all gates pass

```
## MATLAB Docking Simulation Complete

Master Gate:
  Docked:     YES
  Pos error:  X.XXX m
  Vel error:  X.XXX m/s
  Att error:  X.XX deg
  Delta-V:    X.XXX m/s
  Time:       XXXX s

Monte Carlo (50 seeds):
  P(dock):    X.XXX
  Mean DV:    X.XXX m/s
  Mean pos:   X.X mm

Phase Status:
  Phase 1 Environment:  PASS
  Phase 2 Sensors:      PASS
  Phase 3 EKF:          PASS
  Phase 4 Guidance:     PASS
  Phase 5 Control:      PASS
  Phase 6 Attitude:     PASS
  Phase 7 FDIR:         PASS
  Phase 8 Main Loop:    PASS
  Phase 9 Plots:        PASS
  Phase 10 Animation:   PASS
  Phase 11 Monte Carlo: PASS

Files created: [list all]
```
