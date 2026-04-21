% visualise_docking_multi.m
%
% HIGH-FIDELITY RELATIVE-MOTION SIMULATION — 8 starting positions
%
% Physics included in the simulation
%   • Clohessy-Wiltshire (CW) relative dynamics
%       – central gravity gradient  (dominant perturbation in relative frame)
%       – Coriolis acceleration
%   • J2 oblateness differential perturbation  (ECI gradient → LVLH)
%   • Atmospheric drag  – absolute force in LVLH, corrected multi-layer
%       density model (fixes the broken single-scale-height sea-level model)
%   • Solar radiation pressure  – absolute force in LVLH
%
% All four environment functions (j2_accel, drag_accel, srp_accel,
% eci_to_lvlh) are reused from  docking_sim/environment/.
%
% Visualisation
%   LEFT  : ECI orbital view  – Earth sphere + atmosphere + stars
%           Blue  arrow = total gravity (large)
%           Cyan  arrow = orbital velocity
%           Orange arrow = atmospheric drag (absolute)
%           Red   arrow = solar radiation pressure
%   RIGHT : LVLH close-up  – bigger 3-D blocks
%           Cyan   = relative velocity
%           Yellow = control thrust
%           Blue   = gravity gradient / J2 combined
%           Orange = atmospheric drag (LVLH frame)
%           Red    = solar radiation pressure (LVLH frame)
%   BOTTOM: range (log) + approach speed for all scenarios
%
% Force magnitudes printed in the telemetry overlay (with exaggeration ×).
%
% Run:  >> visualise_docking_multi

clear; clc; close all;

%% ── Paths ────────────────────────────────────────────────────────────────
base = fileparts(mfilename('fullpath'));
addpath(fullfile(base,'environment'));   % j2_accel, drag_accel, srp_accel, eci_to_lvlh

%% ── Constants & orbital parameters ──────────────────────────────────────
mu  = 3.986004418e14;
Re  = 6.371e6;
J2  = 1.08263e-3;
a   = 6.771e6;           % ISS-like orbital radius (m)
n   = sqrt(mu/a^3);      % mean motion ≈ 1.14e-3 rad/s
a_d = a/Re;              % orbital radius in display units (≈ 1.063)

% SRP / drag spacecraft parameters (same as main_sim)
Cd  = 2.2;  A_sc = 10;  Cr = 1.3;

% LVLH-offset exaggeration for orbital view (200 m → visible near orbit ring)
EXAG = 2200;

%% ── Simulation settings ─────────────────────────────────────────────────
dt    = 1.0;  T_max = 3600;  mass = 500;  F_max = 8.0;
v_ph  = [1.0 0.40 0.08 0.02];  r_gt = [50 10 1];
K_V   = 0.01;  K_damp = 28;

%% ── Starting positions ───────────────────────────────────────────────────
starts = [
     0    200    0;
    65    175   30;
   -75    160  -25;
   110    130    0;
     0    200   85;
   -55    195  -80;
    45    215   55;
   -95    145   50;
];
nc = size(starts,1);

cols = [
    0.25 0.60 1.00;  0.20 0.90 0.40;  1.00 0.30 0.20;  1.00 0.75 0.10;
    0.80 0.30 1.00;  0.15 0.90 0.90;  1.00 0.55 0.15;  0.90 0.90 0.25;
];

%% ── Run simulations with full perturbation forces ────────────────────────
fprintf('Running %d simulations (CW + J2 + drag + SRP)...\n', nc);
sims = cell(nc,1);

for sc = 1:nc
    pos = starts(sc,:)';   vel = zeros(3,1);

    % Pre-allocate logs
    P=zeros(3,T_max); V=zeros(3,T_max);
    Fc_log=zeros(3,T_max);  Fg_log=zeros(3,T_max);
    Fj2_log=zeros(3,T_max); Fd_log=zeros(3,T_max); Fsrp_log=zeros(3,T_max);
    T_log=zeros(1,T_max);   kf=T_max;

    for k = 1:T_max
        P(:,k)=pos; V(:,k)=vel; T_log(k)=(k-1)*dt;

        % ─ Target ECI state at this step (simplified circular equatorial) ─
        theta = n*(k-1)*dt;
        ct=cos(theta); st=sin(theta);
        r_tgt = a*[ct;st;0];
        v_tgt = a*n*[-st;ct;0];
        % LVLH→ECI rotation  (X=radial, Y=along-track, Z=cross-track)
        R_l2e = [ct -st 0; st ct 0; 0 0 1];

        % ─ Chaser ECI state (relative→absolute, neglect Coriolis on vel) ─
        r_chs = r_tgt + R_l2e*pos;
        v_chs = v_tgt + R_l2e*vel;    % approximation: |vel| << |v_tgt|

        % ─ CW gravity-gradient force (tidal) ─────────────────────────────
        Fg_k = mass * [3*n^2*pos(1);  0;  -n^2*pos(3)];
        Fg_log(:,k) = Fg_k;

        % ─ J2 differential acceleration (ECI difference → LVLH) ──────────
        da_j2 = j2_accel(r_chs,mu,Re,J2) - j2_accel(r_tgt,mu,Re,J2);
        Fj2_k = mass * (R_l2e' * da_j2);
        Fj2_log(:,k) = Fj2_k;

        % ─ Atmospheric drag — ABSOLUTE force on chaser in LVLH ───────────
        % (differential between two identical spacecraft ≈ 0;
        %  show absolute drag to illustrate the orbit-decay force)
        ad_abs = drag_accel(r_chs, v_chs, Cd, A_sc, mass, Re);
        Fd_k   = mass * (R_l2e' * ad_abs);
        Fd_log(:,k) = Fd_k;

        % ─ SRP — ABSOLUTE force on chaser in LVLH ────────────────────────
        as_abs  = srp_accel(r_chs, A_sc, mass, Cr);
        Fsrp_k  = mass * (R_l2e' * as_abs);
        Fsrp_log(:,k) = Fsrp_k;

        % ─ Guidance (same proportional-nav as guidance_law.m) ─────────────
        r = norm(pos);
        if    r > r_gt(1);  vmax=v_ph(1);
        elseif r > r_gt(2); vmax=v_ph(2);
        elseif r > r_gt(3); vmax=v_ph(3);
        else;                vmax=v_ph(4);
        end
        vel_ref = zeros(3,1);
        if r > 1e-4;  vel_ref = -pos/r * min(vmax,K_V*r);  end
        Fc_k = max(-F_max, min(F_max, K_damp*(vel_ref-vel)));
        Fc_log(:,k) = Fc_k;

        % ─ Total acceleration: CW + J2 diff + drag + SRP + thrust ─────────
        acc = [2*n*vel(2) + 3*n^2*pos(1) + Fj2_k(1)/mass + Fd_k(1)/mass + Fsrp_k(1)/mass + Fc_k(1)/mass;
               -2*n*vel(1)               + Fj2_k(2)/mass + Fd_k(2)/mass + Fsrp_k(2)/mass + Fc_k(2)/mass;
               -n^2*pos(3)               + Fj2_k(3)/mass + Fd_k(3)/mass + Fsrp_k(3)/mass + Fc_k(3)/mass];

        vel = vel + acc*dt;
        pos = pos + vel*dt;

        if norm(pos) < 0.15 && norm(vel) < 0.05
            kf = k;  break;
        end
    end

    sims{sc}.pos  = P(:,1:kf);    sims{sc}.vel   = V(:,1:kf);
    sims{sc}.Fc   = Fc_log(:,1:kf);  sims{sc}.Fg  = Fg_log(:,1:kf);
    sims{sc}.Fj2  = Fj2_log(:,1:kf); sims{sc}.Fd  = Fd_log(:,1:kf);
    sims{sc}.Fsrp = Fsrp_log(:,1:kf); sims{sc}.t  = T_log(1:kf);
    fprintf('  S%d  [%+4.0f %+4.0f %+4.0f] m  →  %4d s\n', ...
        sc, starts(sc,1),starts(sc,2),starts(sc,3), kf);
end

% Print representative force magnitudes at step 1, scenario 1
s1 = sims{1};
fprintf('\nForce magnitudes at step 1 (S1, approx ISS orbit):\n');
fprintf('  Gravity gradient (CW tidal) : %.2e N\n', norm(s1.Fg(:,1)));
fprintf('  J2 differential             : %.2e N\n', norm(s1.Fj2(:,1)));
fprintf('  Atmospheric drag (absolute) : %.2e N\n', norm(s1.Fd(:,1)));
fprintf('  Solar radiation pressure    : %.2e N\n', norm(s1.Fsrp(:,1)));
fprintf('  Control thrust (max)        : %.2e N\n', F_max);
fprintf('Note: J2/drag/SRP differentials are ~0 for identical spacecraft.\n');
fprintf('Absolute drag + SRP shown for orbital-context understanding.\n\n');

%% ── Figure & axes ────────────────────────────────────────────────────────
fig = figure(2);  clf;
set(fig,'Color','k','Name','Full-Physics Docking Simulation','Position',[20 30 1700 920]);

ax_orb  = axes('Position',[0.02  0.07  0.47  0.90]);
ax_dock = axes('Position',[0.52  0.36  0.46  0.58]);
ax_rng  = axes('Position',[0.52  0.07  0.21  0.23]);
ax_spd  = axes('Position',[0.76  0.07  0.21  0.23]);

for ax = [ax_orb ax_dock ax_rng ax_spd]
    set(ax,'Color',[0.03 0.03 0.06],'XColor','w','YColor','w','ZColor','w',...
        'GridColor',[0.20 0.20 0.28],'GridAlpha',0.40);
    grid(ax,'on');  hold(ax,'on');
end

%% ═══ ORBITAL VIEW — static elements ════════════════════════════════════
set(ax_orb,'XLim',[-1.9 1.9],'YLim',[-1.9 1.9],'ZLim',[-0.65 0.65]);
axis(ax_orb,'off');
view(ax_orb, 22, 18);
title(ax_orb, 'Orbital (ECI) View  —  Forces acting on spacecraft at 400 km orbit',...
    'Color','w','FontSize',10,'FontWeight','bold');

% Stars
rng('default');
Nst=700; ph_s=2*pi*rand(1,Nst); th_s=acos(2*rand(1,Nst)-1);
r_s=2.8+0.5*rand(1,Nst);
plot3(ax_orb,r_s.*sin(th_s).*cos(ph_s),r_s.*sin(th_s).*sin(ph_s),r_s.*cos(th_s),...
    '.','Color','w','MarkerSize',1);

% Atmosphere glow
[Xg,Yg,Zg]=sphere(40);
surf(ax_orb,1.10*Xg,1.10*Yg,1.10*Zg,'FaceColor',[0.10 0.45 0.85],'FaceAlpha',0.09,'EdgeColor','none');
surf(ax_orb,1.06*Xg,1.06*Yg,1.06*Zg,'FaceColor',[0.05 0.25 0.70],'FaceAlpha',0.06,'EdgeColor','none');

% Earth sphere with terrain
[Xe,Ye,Ze]=sphere(64);
lat=asin(max(-1,min(1,Ze))); lon=atan2(Ye,Xe);
land = 0.5 + 0.45*sin(2.8*lon+0.8).*cos(3.5*lat+0.3) ...
           + 0.25*sin(6.5*lon+1.9).*cos(1.8*lat+0.9) ...
           + 0.15*cos(4.0*lon-1.1).*sin(5.0*lat+0.6);
land=max(0,min(1,land)); ocean=land<0.52;
R_c=0.05+0.12*(~ocean).*(land-0.52)/(0.48);
G_c=0.14+0.42*(~ocean).*(land-0.52)/(0.48)+0.04*ocean;
B_c=0.48-0.30*(~ocean).*(land-0.52)/(0.48);
surf(ax_orb,Xe,Ye,Ze,cat(3,R_c,G_c,B_c),'FaceColor','texturemap','EdgeColor','none');

% Polar caps
th_cap=linspace(0,2*pi,48); r_cap=linspace(0,0.38,12);
[Rc,Tc]=meshgrid(r_cap,th_cap);
Xp=Rc.*cos(Tc); Yp=Rc.*sin(Tc);
surf(ax_orb,Xp,Yp, sqrt(max(0,1-Xp.^2-Yp.^2)),'FaceColor','w','FaceAlpha',0.65,'EdgeColor','none');
surf(ax_orb,Xp,Yp,-sqrt(max(0,1-Xp.^2-Yp.^2)),'FaceColor','w','FaceAlpha',0.65,'EdgeColor','none');

% Orbital ring
th_ring=linspace(0,2*pi,360);
plot3(ax_orb,a_d*cos(th_ring),a_d*sin(th_ring),zeros(1,360),'-','Color',[0.35 0.55 0.90],'LineWidth',1.8);

% Equatorial circle (faint)
plot3(ax_orb,cos(th_ring),sin(th_ring),zeros(1,360),'--','Color',[0.22 0.22 0.38],'LineWidth',0.6);

% Radial gravity field lines (faint)
for phi_d=0:30:330
    plot3(ax_orb,[1.05 1.75]*cosd(phi_d),[1.05 1.75]*sind(phi_d),[0 0],'-',...
        'Color',[0.12 0.12 0.22],'LineWidth',0.5);
end

% Sun direction indicator
plot3(ax_orb,[1.5 1.9],[0 0],[0 0],'y-','LineWidth',1.2);
text(ax_orb,1.92,0,0,'☀','Color','y','FontSize',10);
text(ax_orb,1.40,0.05,0,'Sun','Color','y','FontSize',7);

% Labels
text(ax_orb,0,0,1.35,'N','Color','w','FontSize',9,'HorizontalAlignment','center');
text(ax_orb,0,0,-1.40,'S','Color','w','FontSize',9,'HorizontalAlignment','center');
text(ax_orb,a_d+0.06,0,0,'ISS orbit','Color',[0.45 0.65 0.95],'FontSize',7);

% Force legend (orbital view)
lx=-1.78; ly=-1.65;
plot3(ax_orb,lx+[0 0.12],ly*ones(1,2),-0.62*ones(1,2),'-','Color',[0.35 0.55 1.0],'LineWidth',2);
text(ax_orb,lx+0.14,ly,-0.62,'Gravity','Color','w','FontSize',7);
plot3(ax_orb,lx+[0.52 0.64],ly*ones(1,2),-0.62*ones(1,2),'-','Color',[0 0.9 0.9],'LineWidth',2);
text(ax_orb,lx+0.66,ly,-0.62,'Velocity','Color','w','FontSize',7);
plot3(ax_orb,lx+[1.08 1.20],ly*ones(1,2),-0.62*ones(1,2),'-','Color',[1.0 0.55 0.1],'LineWidth',2);
text(ax_orb,lx+1.22,ly,-0.62,'Drag','Color','w','FontSize',7);
plot3(ax_orb,lx+[1.52 1.64],ly*ones(1,2),-0.62*ones(1,2),'-','Color',[1.0 0.2 0.2],'LineWidth',2);
text(ax_orb,lx+1.66,ly,-0.62,'SRP','Color','w','FontSize',7);

%% ORBITAL VIEW — animated handles
tgt_dot_h   = plot3(ax_orb,a_d,0,0,'w*','MarkerSize',14,'LineWidth',2.5);
tgt_trail_h = plot3(ax_orb,a_d,0,0,'w-','LineWidth',1.5);

orb_dot  = gobjects(nc,1);
orb_grav = gobjects(nc,1);
orb_vel  = gobjects(nc,1);
orb_drag = gobjects(nc,1);
orb_srp  = gobjects(nc,1);

for sc=1:nc
    p0e=[a_d;0;0]+sims{sc}.pos(:,1)*EXAG/Re;
    orb_dot(sc) = plot3(ax_orb,p0e(1),p0e(2),0,'o','Color',cols(sc,:),...
        'MarkerSize',10,'MarkerFaceColor',cols(sc,:));
    orb_grav(sc)= quiver3(ax_orb,p0e(1),p0e(2),0,0,0,0,'Color',[0.35 0.55 1.0],...
        'LineWidth',2.0,'MaxHeadSize',0.9,'AutoScale','off');
    orb_vel(sc) = quiver3(ax_orb,p0e(1),p0e(2),0,0,0,0,'c',...
        'LineWidth',1.5,'MaxHeadSize',0.9,'AutoScale','off');
    orb_drag(sc)= quiver3(ax_orb,p0e(1),p0e(2),0,0,0,0,'Color',[1.0 0.55 0.1],...
        'LineWidth',1.5,'MaxHeadSize',0.9,'AutoScale','off');
    orb_srp(sc) = quiver3(ax_orb,p0e(1),p0e(2),0,0,0,0,'Color',[1.0 0.2 0.2],...
        'LineWidth',1.5,'MaxHeadSize',0.9,'AutoScale','off');
end

%% ═══ LVLH DOCKING VIEW — static elements ════════════════════════════════
pos_cells=cellfun(@(s) s.pos,sims,'UniformOutput',false);
all_pos=[pos_cells{:}];
pad=20;
xl=[min(all_pos(1,:))-pad  max(all_pos(1,:))+pad];
yl=[-pad                   max(all_pos(2,:))+pad];
zl=[min(all_pos(3,:))-pad  max(all_pos(3,:))+pad];
set(ax_dock,'XLim',xl,'YLim',yl,'ZLim',zl);
view(ax_dock,38,22);

xlabel(ax_dock,'X_{LVLH} (m)','Color','w','FontSize',8);
ylabel(ax_dock,'Y_{LVLH} (m)','Color','w','FontSize',8);
zlabel(ax_dock,'Z_{LVLH} (m)','Color','w','FontSize',8);
title(ax_dock,...
    'LVLH Close-up  |  Cyan=vel  Yellow=thrust  Blue=gravity  Orange=drag  Red=SRP',...
    'Color','w','FontSize',9,'FontWeight','bold');

% Earth hint
ex=xl(1)-diff(xl)*0.28; er=diff(xl)*0.38;
[Xeh,Yeh,Zeh]=sphere(28);
surf(ax_dock,er*Xeh+ex,er*Yeh,er*Zeh,'FaceColor',[0.05 0.12 0.45],'FaceAlpha',0.55,'EdgeColor','none');
surf(ax_dock,er*1.09*Xeh+ex,er*1.09*Yeh,er*1.09*Zeh,'FaceColor',[0.10 0.40 0.80],'FaceAlpha',0.07,'EdgeColor','none');
text(ax_dock,ex,0,-er*0.65,'Earth','Color',[0.55 0.80 1.0],'FontSize',9,'HorizontalAlignment','center');
text(ax_dock,ex,0,-er*0.82,'(not to scale)','Color',[0.40 0.60 0.80],'FontSize',7,'HorizontalAlignment','center');

% LVLH axes
al=max(diff(xl),diff(zl))*0.09; al=max(al,10);
quiver3(ax_dock,0,0,0,al,0,0,'r','LineWidth',1.5,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(ax_dock,0,0,0,0,al,0,'g','LineWidth',1.5,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(ax_dock,0,0,0,0,0,al,'b','LineWidth',1.5,'MaxHeadSize',0.4,'AutoScale','off');
text(ax_dock,al+1,0,0,'X (radial↑)','Color','r','FontSize',7);
text(ax_dock,0,al+1,0,'Y (along-track)','Color','g','FontSize',7);
text(ax_dock,0,0,al+1,'Z (cross-track)','Color','b','FontSize',7);

% Docking port
th2=linspace(0,2*pi,64);
fill3(ax_dock,0.8*cos(th2),zeros(1,64),0.8*sin(th2),'r','FaceAlpha',0.4,'EdgeColor','r','LineWidth',2.5);
plot3(ax_dock,0,0,0,'r*','MarkerSize',18,'LineWidth',3);
text(ax_dock,3,5,3,'TARGET','Color','r','FontSize',11,'FontWeight','bold');

% Dim background trajectories + start markers
for sc=1:nc
    p=sims{sc}.pos;
    plot3(ax_dock,p(1,:),p(2,:),p(3,:),'-','Color',[cols(sc,:) 0.14],'LineWidth',0.8);
    p0=p(:,1);
    plot3(ax_dock,p0(1),p0(2),p0(3),'^','Color',cols(sc,:),'MarkerSize',11,...
        'MarkerFaceColor',cols(sc,:),'LineWidth',1.5);
    text(ax_dock,p0(1)+2,p0(2)+2,p0(3)+2,sprintf('S%d',sc),'Color',cols(sc,:),'FontSize',7,'FontWeight','bold');
end

%% LVLH VIEW — animated handles (5 arrow types per spacecraft)
traj_h   = gobjects(nc,1);
dock_blk = cell(nc,1);
vel_q    = gobjects(nc,1);
Fc_q     = gobjects(nc,1);
Fg_q     = gobjects(nc,1);   % gravity gradient + J2
Fd_q     = gobjects(nc,1);   % atmospheric drag (absolute, LVLH)
Fsrp_q   = gobjects(nc,1);   % SRP (absolute, LVLH)

for sc=1:nc
    p0=sims{sc}.pos(:,1);
    traj_h(sc) = plot3(ax_dock,p0(1),p0(2),p0(3),'-','Color',cols(sc,:),'LineWidth',2.0);
    dock_blk{sc} = {};
    vel_q(sc)  = quiver3(ax_dock,p0(1),p0(2),p0(3),0,0,0,'c','LineWidth',2.0,'MaxHeadSize',0.7,'AutoScale','off');
    Fc_q(sc)   = quiver3(ax_dock,p0(1),p0(2),p0(3),0,0,0,'y','LineWidth',2.0,'MaxHeadSize',0.7,'AutoScale','off');
    Fg_q(sc)   = quiver3(ax_dock,p0(1),p0(2),p0(3),0,0,0,'Color',[0.35 0.55 1.0],'LineWidth',1.6,'MaxHeadSize',0.7,'AutoScale','off');
    Fd_q(sc)   = quiver3(ax_dock,p0(1),p0(2),p0(3),0,0,0,'Color',[1.0 0.55 0.1],'LineWidth',1.6,'MaxHeadSize',0.7,'AutoScale','off');
    Fsrp_q(sc) = quiver3(ax_dock,p0(1),p0(2),p0(3),0,0,0,'Color',[1.0 0.2 0.2],'LineWidth',1.4,'MaxHeadSize',0.7,'AutoScale','off');
end

%% ═══ TIME SERIES ═════════════════════════════════════════════════════════
t_max_all=max(cellfun(@(s) s.t(end),sims));
set(ax_rng,'YScale','log');
xlabel(ax_rng,'t (s)','Color','w','FontSize',8); ylabel(ax_rng,'Range (m)','Color','w','FontSize',8);
title(ax_rng,'Range to Target','Color','w','FontSize',9);
xlim(ax_rng,[0 t_max_all]); ylim(ax_rng,[0.05 max(sqrt(sum(all_pos.^2,1)))*1.4]);
yline(ax_rng,0.15,'r--','LineWidth',1.2);

xlabel(ax_spd,'t (s)','Color','w','FontSize',8); ylabel(ax_spd,'m/s','Color','w','FontSize',8);
title(ax_spd,'Approach Speed','Color','w','FontSize',9);
xlim(ax_spd,[0 t_max_all]); ylim(ax_spd,[0 v_ph(1)*1.6]);

rng_h=gobjects(nc,1); spd_h=gobjects(nc,1);
for sc=1:nc
    rng_h(sc)=plot(ax_rng,0,norm(sims{sc}.pos(:,1)),'-','Color',cols(sc,:),'LineWidth',1.3);
    spd_h(sc)=plot(ax_spd,0,0,'-','Color',cols(sc,:),'LineWidth',1.3);
end

% Telemetry overlay on LVLH view
txt_t    = text(ax_dock,0.02,0.97,'','Units','normalized','Color','w','FontSize',9);
txt_dk   = text(ax_dock,0.02,0.91,'','Units','normalized','Color',[0.2 1.0 0.2],'FontSize',9);
txt_fmag = text(ax_dock,0.02,0.52,'','Units','normalized','Color',[0.8 0.8 0.8],'FontSize',7);
txt_all  = text(ax_dock,0.50,0.50,'','Units','normalized','Color','g','FontSize',20,...
    'FontWeight','bold','HorizontalAlignment','center','Visible','off');

drawnow;

%% ── Arrow scale factors ──────────────────────────────────────────────────
ss    = max(diff(xl),diff(zl));   % LVLH scene size (m)
vscl  = ss * 0.11;    % velocity: 1 m/s → ~11 % of scene
Fc_s  = ss * 0.022;   % thrust:   1 N  → ~2.2 %
Fg_s  = ss * 1.60;    % gravity gradient: ~0.4 N at 200 m → ~14 % of scene
Fd_s  = ss * 40.0;    % drag (absolute): ~2.6e-3 N → visible (~10 % of scene)
Fsrp_s= ss * 1800;    % SRP (absolute): ~5.9e-5 N → visible (~10 % of scene)
Fj2_s = ss * 2.5e6;   % J2 differential: ~4e-7 N → visible (note: exaggerated)

% Orbital view fixed arrow lengths (display units = Earth radii)
orb_grav_len = 0.11;   % gravity toward Earth
orb_vel_len  = 0.10;   % orbital velocity tangent
orb_drag_len = 0.06;   % drag opposing velocity
orb_srp_len  = 0.04;   % SRP in sun direction

% Pre-compute orbital angles
max_steps = max(cellfun(@(s) size(s.pos,2), sims));
orb_theta = n * ((1:max_steps)-1) * dt;

%% ── Animation loop ───────────────────────────────────────────────────────
fskip = max(1, floor(max_steps/500));
ids   = 1:fskip:max_steps;
Nf    = numel(ids);

for fi = 1:Nf
    k     = ids(fi);
    theta = orb_theta(k);
    ct    = cos(theta);   st = sin(theta);
    R_l2e = [ct -st 0;  st ct 0;  0 0 1];

    % Target on orbit (display units)
    tgt_eci = [a_d*ct;  a_d*st;  0];

    % Orbital target trail
    idx_tr = max(1,fi-120):fi;
    trl_th = orb_theta(ids(idx_tr));
    set(tgt_trail_h,'XData',a_d*cos(trl_th),'YData',a_d*sin(trl_th),'ZData',zeros(size(trl_th)));
    set(tgt_dot_h,'XData',tgt_eci(1),'YData',tgt_eci(2),'ZData',0);

    % Orbital velocity direction (tangent) and sun direction in LVLH
    v_orb_hat = [-st; ct; 0];
    sun_lvlh  = R_l2e' * [1;0;0];   % sun direction in LVLH at this orbital phase

    n_done = 0;
    for sc = 1:nc
        ns  = size(sims{sc}.pos,2);
        ki  = min(k,ns);
        p_k = sims{sc}.pos(:,ki);
        v_k = sims{sc}.vel(:,ki);
        Fc_k= sims{sc}.Fc(:,ki);
        Fg_k= sims{sc}.Fg(:,ki) + sims{sc}.Fj2(:,ki);  % gravity gradient + J2 combined
        Fd_k= sims{sc}.Fd(:,ki);
        Fs_k= sims{sc}.Fsrp(:,ki);
        rk  = norm(p_k);

        %── ORBITAL VIEW ──────────────────────────────────────────────────
        p_eci = tgt_eci + R_l2e*p_k*EXAG/Re;

        set(orb_dot(sc),'XData',p_eci(1),'YData',p_eci(2),'ZData',p_eci(3));

        % Gravity: fixed-length toward Earth (origin)
        g_hat = -p_eci/max(norm(p_eci),1e-9);
        set(orb_grav(sc),'XData',p_eci(1),'YData',p_eci(2),'ZData',p_eci(3),...
            'UData',g_hat(1)*orb_grav_len,'VData',g_hat(2)*orb_grav_len,'WData',g_hat(3)*orb_grav_len);

        % Velocity: orbital tangent
        set(orb_vel(sc),'XData',p_eci(1),'YData',p_eci(2),'ZData',p_eci(3),...
            'UData',v_orb_hat(1)*orb_vel_len,'VData',v_orb_hat(2)*orb_vel_len,'WData',v_orb_hat(3)*orb_vel_len);

        % Drag: opposing orbital velocity direction
        set(orb_drag(sc),'XData',p_eci(1),'YData',p_eci(2),'ZData',p_eci(3),...
            'UData',-v_orb_hat(1)*orb_drag_len,'VData',-v_orb_hat(2)*orb_drag_len,'WData',0);

        % SRP: in sun direction (from [1;0;0] ECI)
        sun_eci = [1;0;0];
        set(orb_srp(sc),'XData',p_eci(1),'YData',p_eci(2),'ZData',p_eci(3),...
            'UData',sun_eci(1)*orb_srp_len,'VData',sun_eci(2)*orb_srp_len,'WData',sun_eci(3)*orb_srp_len);

        %── LVLH VIEW ─────────────────────────────────────────────────────
        set(traj_h(sc),'XData',sims{sc}.pos(1,1:ki),...
            'YData',sims{sc}.pos(2,1:ki),'ZData',sims{sc}.pos(3,1:ki));

        for hi=1:numel(dock_blk{sc}); delete(dock_blk{sc}{hi}); end
        bsz = max(4.0, min(10.0, rk*0.048));
        dock_blk{sc} = make_block(ax_dock, p_k, bsz, cols(sc,:));

        set(vel_q(sc) ,'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3),...
            'UData',v_k(1)*vscl, 'VData',v_k(2)*vscl, 'WData',v_k(3)*vscl);
        set(Fc_q(sc)  ,'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3),...
            'UData',Fc_k(1)*Fc_s,'VData',Fc_k(2)*Fc_s,'WData',Fc_k(3)*Fc_s);
        set(Fg_q(sc)  ,'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3),...
            'UData',Fg_k(1)*Fg_s,'VData',Fg_k(2)*Fg_s,'WData',Fg_k(3)*Fg_s);
        set(Fd_q(sc)  ,'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3),...
            'UData',Fd_k(1)*Fd_s,'VData',Fd_k(2)*Fd_s,'WData',Fd_k(3)*Fd_s);
        set(Fsrp_q(sc),'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3),...
            'UData',Fs_k(1)*Fsrp_s,'VData',Fs_k(2)*Fsrp_s,'WData',Fs_k(3)*Fsrp_s);

        %── TIME SERIES ───────────────────────────────────────────────────
        t_k   = sims{sc}.t(1:ki);
        set(rng_h(sc),'XData',t_k,'YData',sqrt(sum(sims{sc}.pos(:,1:ki).^2,1)));
        set(spd_h(sc),'XData',t_k,'YData',sqrt(sum(sims{sc}.vel(:,1:ki).^2,1)));

        if ki>=ns; n_done=n_done+1; end
    end

    % Text overlays
    t_now=round(ids(fi)*dt);
    set(txt_t, 'String',sprintf('t = %d s  (%.1f min)', t_now, t_now/60));
    set(txt_dk,'String',sprintf('Docked: %d / %d', n_done, nc));

    % Force magnitudes for S1 at current step
    sc1=1; ki1=min(k,size(sims{sc1}.Fc,2));
    Fc_mag   = norm(sims{sc1}.Fc(:,ki1));
    Fg_mag   = norm(sims{sc1}.Fg(:,ki1)+sims{sc1}.Fj2(:,ki1));
    Fd_mag   = norm(sims{sc1}.Fd(:,ki1));
    Fsrp_mag = norm(sims{sc1}.Fsrp(:,ki1));
    Fj2_mag  = norm(sims{sc1}.Fj2(:,ki1));
    set(txt_fmag,'String',sprintf(...
        'S1 forces  |\n Thrust: %.2e N\n Tidal:  %.2e N\n J2 diff:%.2e N\n Drag:   %.2e N\n SRP:    %.2e N\n(drag/SRP exaggerated ×%g/×%g)', ...
        Fc_mag, Fg_mag, Fj2_mag, Fd_mag, Fsrp_mag, round(Fd_s/Fc_s), round(Fsrp_s/Fc_s)));

    if n_done==nc && strcmp(get(txt_all,'Visible'),'off')
        set(txt_all,'String','ALL DOCKED  ✓','Visible','on');
        title(ax_dock,'ALL SCENARIOS DOCKED  ✓','Color','g','FontSize',10,'FontWeight','bold');
    end

    view(ax_dock, 38+20*sin(fi/Nf*pi), 22);
    drawnow limitrate;
    pause(0.008);
end
fprintf('\nAll %d scenarios docked.\n', nc);

% ── make_block ────────────────────────────────────────────────────────────
function hlist = make_block(ax, pos, sz, col)
    h=sz*0.5;
    verts=h*[-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1;
             -1 -1  1; 1 -1  1; 1 1  1; -1 1  1]+pos';
    faces=[1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    h1=patch(ax,'Vertices',verts,'Faces',faces,'FaceColor',col,'FaceAlpha',0.88,...
        'EdgeColor','w','EdgeAlpha',0.90,'LineWidth',0.7);
    hlist={h1};
end
