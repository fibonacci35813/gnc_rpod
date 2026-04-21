function [docked,steps,dv,pos_err,vel_err] = run_single_sim(dr0, dv0, thr_scale, seed)
% run_single_sim - function wrapper for Monte Carlo
% Inputs: dr0 [3x1] pos dispersion (m), dv0 [3x1] vel dispersion (m/s),
%         thr_scale: thruster scale factor, seed: RNG seed

base = fileparts(mfilename('fullpath'));
addpath(fullfile(base,'environment'),fullfile(base,'sensors'),fullfile(base,'navigation'), ...
        fullfile(base,'guidance'),fullfile(base,'control'),fullfile(base,'attitude'), ...
        fullfile(base,'propulsion'),fullfile(base,'fdir'),fullfile(base,'mission'));

%% Constants
mu   = 3.986004418e14;
Re   = 6.371e6;
J2   = 1.08263e-3;
g0   = 9.80665;

%% Orbital parameters
a    = 6.771e6;
e    = 0;
inc  = deg2rad(51.6);
RAAN = 0; omega_oe = 0; nu0 = 0;
n    = sqrt(mu/a^3);

%% Simulation time
dt   = 1.0;
N_MAX = 10800;

%% Parameters (same as main_sim)
params.mu   = mu; params.Re = Re; params.J2 = J2;
params.Cd   = 2.2; params.A = 10; params.Cr = 1.3; params.mass0 = 500;
params.v_phase_max = [1.00, 0.50, 0.08, 0.04];
params.K_V         = 0.01;
params.Kp_normal   = [0.30; 0.20; 0.30];
params.Kd_normal   = [20; 20; 20];
params.mib_normal  = 0.1;
params.Kp_terminal = [2.00; 2.00; 2.00];
params.Kd_terminal = [200; 200; 200];
params.mib_terminal= 0.005;
params.Kp_att  = 0.5; params.Kd_att = 10.0;
params.I_body  = 50;
params.I_rw    = 0.1; params.omega_max = 500;
params.fdir.thr_stuck_open   = 0.1; params.fdir.thr_cmd_zero     = 0.05;
params.fdir.thr_cmd_min      = 0.5; params.fdir.thr_stuck_closed = 0.05;
params.fdir.dropout_limit    = 3;   params.fdir.cov_max          = 1e4;
params.mm.hold_timeout   = 30; params.mm.hold_max = 120; params.mm.retreat_timeout = 60;
params.imu.bias_instability = 1e-5; params.imu.accel_noise = 1e-4;

%% Tank
tank.P0          = 2e6; tank.P_nom = 2e6; tank.P_min = 0.5e6;
tank.prop_remain = 5.0; tank.rho_prop = 1000; tank.V_total = 0.015;
tank.V_gas0      = tank.V_total - tank.prop_remain/tank.rho_prop;
tank.mass        = params.mass0; tank.Isp = 220;
tank.dv_total    = 0; tank.P_current = tank.P0;

%% RNG
rng_obj = RandStream('mt19937ar','Seed',seed);

%% Initial conditions
[r_tgt0, v_tgt0] = oe2rv(a, e, inc, RAAN, omega_oe, nu0, mu);
r_hat = r_tgt0/norm(r_tgt0);
h_vec = cross(r_tgt0,v_tgt0);
h_hat = h_vec/norm(h_vec);
y_hat = cross(h_hat,r_hat);
R_lvlh2eci = [r_hat, y_hat, h_hat];
dr_lvlh = [0;200;0] + dr0;
dr_eci  = R_lvlh2eci * dr_lvlh;
r_chs0  = r_tgt0 + dr_eci;
v_chs0  = v_tgt0 + dv0;

X_tgt = [r_tgt0; v_tgt0];
X_chs = [r_chs0; v_chs0];

[pos0, vel0, ~] = eci_to_lvlh(r_tgt0,v_tgt0,r_chs0,v_chs0);
ekf = ekf_init(pos0+3*rng_obj.randn(3,1), vel0+0.1*rng_obj.randn(3,1), n);

q = [1;0;0;0]; omega = zeros(3,1); h_rw = zeros(3,1); imu_bias = zeros(3,1);
mission_mode = 0; mode_timer = 0;
dock_count = 0; docked = false;
dropout_count = 0;
F_cmd_prev = zeros(3,1); F_act_prev = zeros(3,1);

ode_opts = odeset('RelTol',1e-8,'AbsTol',1e-10);

for k = 1:N_MAX
    [~,Xtgt_t] = ode45(@(t,X) eom_perturbed(t,X,mu,Re,J2,params.Cd,params.A,params.mass0,params.Cr),[0,dt],X_tgt(1:6),ode_opts);
    X_tgt = Xtgt_t(end,:)';
    [~,Xchs_t] = ode45(@(t,X) eom_perturbed(t,X,mu,Re,J2,params.Cd,params.A,tank.mass,params.Cr),[0,dt],X_chs(1:6),ode_opts);
    X_chs = Xchs_t(end,:)';
    [pos_true, vel_true, R_eci2lvlh] = eci_to_lvlh(X_tgt(1:3),X_tgt(4:6),X_chs(1:3),X_chs(4:6));
    a_true_lvlh = zeros(3,1);
    dt_imu = dt/10;
    for ii=1:10
        [~,imu_bias]=imu_model(a_true_lvlh,imu_bias,dt_imu,params.imu,rng_obj);
        ekf=ekf_predict(ekf,a_true_lvlh,dt_imu);
    end
    meas = lidar_rae(pos_true, rng_obj);
    if meas.valid; ekf=ekf_update(ekf,meas); dropout_count=0; else; dropout_count=dropout_count+1; end
    pos_est = ekf.x(1:3); vel_est = ekf.x(4:6);
    cov_tr = trace(ekf.P);
    [fault_code,~]=fdir_check(F_cmd_prev,F_act_prev,meas.valid,dropout_count,cov_tr,params.fdir);
    if norm(pos_est)<0.15 && (fault_code==3||fault_code==4); fault_code=0; end
    [mission_mode,pos_override]=mission_manager(mission_mode,fault_code,pos_est,mode_timer,params.mm);
    if fault_code==0 && mission_mode==0; mode_timer=0; else; mode_timer=mode_timer+1; end
    % Force recovery to NOMINAL when close to port and no stuck-open fault
    if mission_mode>=2 && norm(pos_est)<2.0 && fault_code~=1
        mission_mode=0; mode_timer=0; pos_override=[];
    end
    pos_for_guidance = pos_est;
    if ~isempty(pos_override) && norm(pos_est)>=1.0; pos_for_guidance=pos_override; end
    [~,vel_ref,gphase,~]=guidance_law(pos_for_guidance,vel_est,params);
    rng_lvlh=norm(pos_est);
    if gphase>=1
        % Lock to +Y approach direction inside terminal zone to prevent 180° flip
        if rng_lvlh>0.10; dir_tgt=pos_est/rng_lvlh; else; dir_tgt=[0;1;0]; end
        x_body_des=cross(dir_tgt,[0;0;1]);
        if norm(x_body_des)<1e-6; x_body_des=[1;0;0]; end
        x_body_des=x_body_des/norm(x_body_des);
        z_body_des=cross(x_body_des,dir_tgt);
        R_des=[x_body_des,dir_tgt,z_body_des];
    else; R_des=eye(3); end
    q_ref=rot2quat(R_des);
    [tau_cmd,att_err_deg]=attitude_control(q,q_ref,omega,params);
    [h_rw,tau_act]=rw_model(h_rw,tau_cmd,params,dt);
    alpha=tau_act/params.I_body; omega=omega+alpha*dt;
    q=quat_kinematics(q,omega,dt);
    pos_err_ctrl=zeros(3,1)-pos_est; vel_err_ctrl=vel_ref-vel_est;
    F_cmd=pd_controller(pos_err_ctrl,vel_err_ctrl,gphase,params);
    F_cmd_scaled=F_cmd*thr_scale;
    [F_act,tank]=blowdown_model(F_cmd_scaled,tank);
    force_eci=R_eci2lvlh'*F_act;
    delta_v_eci=force_eci/tank.mass*dt;
    X_chs(4:6)=X_chs(4:6)+delta_v_eci;
    F_cmd_prev=F_cmd; F_act_prev=F_act;
    range_now=norm(pos_true);
    vel_now=norm(vel_true);
    if range_now<0.05 && vel_now<0.01 && att_err_deg<1.0
        dock_count=dock_count+1;
    else
        dock_count=0;
    end
    if dock_count>=30 && ~docked
        docked=true; steps=k;
        pos_err=range_now; vel_err=vel_now;
        dv=tank.dv_total;
        return;
    end
    if range_now>1000; break; end
end

docked=false; steps=k; dv=tank.dv_total;
pos_err=norm(pos_true); vel_err=norm(vel_true);
end
