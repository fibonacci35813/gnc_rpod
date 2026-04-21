function animate_docking(telem, step)
% animate_docking  Single-simulation 3-D replay with block + force vectors.
%
% Fixes the empty-axes issue by deriving limits from telem data before
% rendering anything.  Uses subplot (not tiledlayout) for compatibility.
%
% Layout
%   Left panel  (large) — 3-D LVLH view: block, velocity arrow, thrust arrow
%   Top-right           — range vs time (log-scale, phase gates)
%   Bottom-right        — relative speed & cumulative delta-V

if nargin < 2; step = length(telem.time); end

pos  = telem.pos_true(:,1:step);
vel  = telem.vel_true(:,1:step);
t    = telem.time(1:step);
rng  = telem.range(1:step);
ph   = telem.phase(1:step);
dv_v = telem.dv(1:step);
Fact = telem.F_act_log(:,1:step);
velm = sqrt(sum(vel.^2, 1));

pc = [0.20 0.55 1.00;   % Phase 0
      0.20 0.90 0.40;   % Phase 1
      1.00 0.75 0.10;   % Phase 2
      1.00 0.25 0.20];  % Phase 3
pnames = {'Ph0 >50 m','Ph1 >10 m','Ph2 >1 m','Ph3 ≤1 m'};
mnames = {'NOMINAL','HOLD','RETREAT','SAFE'};

%% Figure & subplots
fig = figure(100); clf;
set(fig, 'Color','k', 'Name','Docking Animation', 'Position',[50 50 1400 800]);

ax3  = subplot(2, 3, [1 2 4 5]);
ax_r = subplot(2, 3, 3);
ax_v = subplot(2, 3, 6);

for ax = [ax3 ax_r ax_v]
    set(ax, 'Color',[0.05 0.05 0.05], 'XColor','w', 'YColor','w', 'ZColor','w', ...
        'GridColor',[0.25 0.25 0.25], 'GridAlpha',0.5);
    grid(ax, 'on');  hold(ax, 'on');
end

%% 3-D axis limits derived from data  (critical — prevents black screen)
pad = 15;
xl = [min(pos(1,:))-pad   max(pos(1,:))+pad];
yl = [min(pos(2,:))-pad   max(pos(2,:))+pad];
zl = [min(pos(3,:))-pad   max(pos(3,:))+pad];
set(ax3, 'XLim',xl, 'YLim',yl, 'ZLim',zl);
view(ax3, 38, 22);

xlabel(ax3, 'X_{LVLH} (m)', 'Color','w');
ylabel(ax3, 'Y_{LVLH} (m)', 'Color','w');
zlabel(ax3, 'Z_{LVLH} (m)', 'Color','w');
title(ax3, 'Autonomous Spacecraft Docking', 'Color','w', 'FontSize',12, 'FontWeight','bold');

%% Static elements
% LVLH reference arrows
al = max(diff(xl), diff(zl)) * 0.10;  al = max(al, 8);
quiver3(ax3,0,0,0,al,0,0,'r','LineWidth',2,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(ax3,0,0,0,0,al,0,'g','LineWidth',2,'MaxHeadSize',0.4,'AutoScale','off');
quiver3(ax3,0,0,0,0,0,al,'b','LineWidth',2,'MaxHeadSize',0.4,'AutoScale','off');
text(ax3,al+1,0,0,'X','Color','r','FontSize',8);
text(ax3,0,al+1,0,'Y','Color','g','FontSize',8);
text(ax3,0,0,al+1,'Z','Color','b','FontSize',8);

% Docking port
th = linspace(0,2*pi,64);
fill3(ax3, 0.7*cos(th), zeros(1,64), 0.7*sin(th), 'r', ...
    'FaceAlpha',0.4, 'EdgeColor','r', 'LineWidth',2.5);
plot3(ax3,0,0,0,'r*','MarkerSize',18,'LineWidth',2.5);
text(ax3, 2,4,2, 'TARGET', 'Color','r', 'FontSize',10, 'FontWeight','bold');

% Phase-coloured dim background path
for p = 0:3
    idx = find(ph == p);
    if isempty(idx); continue; end
    segs = get_segs(idx);
    for si = 1:numel(segs)
        ii = segs{si};
        plot3(ax3, pos(1,ii),pos(2,ii),pos(3,ii), '-', ...
            'Color',[pc(p+1,:) 0.18], 'LineWidth',0.7);
    end
end

% Start marker
plot3(ax3,pos(1,1),pos(2,1),pos(3,1),'g^','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','g');
text(ax3,pos(1,1)+2,pos(2,1)+3,pos(3,1)+2,'START','Color','g','FontSize',9,'FontWeight','bold');

% Legend
for p = 0:3
    plot3(ax3,NaN,NaN,NaN,'-','Color',pc(p+1,:),'LineWidth',2,'DisplayName',pnames{p+1});
end
plot3(ax3,NaN,NaN,NaN,'c-','LineWidth',2,'DisplayName','Velocity');
plot3(ax3,NaN,NaN,NaN,'y-','LineWidth',2,'DisplayName','Thrust');
legend(ax3,'TextColor','w','Color',[0.08 0.08 0.08],'FontSize',7,...
    'Location','northeast','NumColumns',2);

%% Animated handles
traj_h = plot3(ax3,pos(1,1),pos(2,1),pos(3,1),'w-','LineWidth',2.0);
sc_h   = {};

% Vector arrows (set to zero initially)
vel_q = quiver3(ax3,pos(1,1),pos(2,1),pos(3,1),0,0,0,'c','LineWidth',2,...
    'MaxHeadSize',0.8,'AutoScale','off');
F_q   = quiver3(ax3,pos(1,1),pos(2,1),pos(3,1),0,0,0,'y','LineWidth',2,...
    'MaxHeadSize',0.8,'AutoScale','off');

%% Telemetry overlays
to_r  = text(ax3,0.02,0.97,'','Units','normalized','Color','w','FontSize',10);
to_p  = text(ax3,0.02,0.91,'','Units','normalized','Color','c','FontSize',10);
to_v  = text(ax3,0.02,0.85,'','Units','normalized','Color',[0.4 1 1],'FontSize',10);
to_d  = text(ax3,0.02,0.79,'','Units','normalized','Color','y','FontSize',10);
to_m  = text(ax3,0.02,0.73,'','Units','normalized','Color',[1 0.5 0],'FontSize',10);
to_dk = text(ax3,0.50,0.50,'','Units','normalized','Color','g','FontSize',22,...
    'FontWeight','bold','HorizontalAlignment','center');

%% Side panels
% Range
rng_h = plot(ax_r, t(1),rng(1), 'w-','LineWidth',1.3);
set(ax_r,'YScale','log');
xlim(ax_r,[0 t(end)]);  ylim(ax_r,[max(0.01,min(rng)*0.4)  max(rng)*1.4]);
xlabel(ax_r,'t (s)','Color','w');  ylabel(ax_r,'Range (m)','Color','w');
title(ax_r,'Range to Port','Color','w','FontSize',9);
yline(ax_r,50,'--','Color',pc(1,:),'LineWidth',1.0);
yline(ax_r,10,'--','Color',pc(2,:),'LineWidth',1.0);
yline(ax_r, 1,'--','Color',pc(3,:),'LineWidth',1.0);

% Speed / delta-V
spd_h = plot(ax_v, t(1),velm(1), 'c-', 'LineWidth',1.3);
dv_h  = plot(ax_v, t(1),dv_v(1), 'y--','LineWidth',1.3);
xlim(ax_v,[0 t(end)]);  ylim(ax_v,[0  max(max(velm),max(dv_v))*1.3+0.01]);
xlabel(ax_v,'t (s)','Color','w');  ylabel(ax_v,'m/s','Color','w');
title(ax_v,'Speed & \DeltaV','Color','w','FontSize',9);
legend(ax_v,{'Speed','\DeltaV'},'TextColor','w','Color',[0.05 0.05 0.05],...
    'FontSize',7,'Location','northwest');

drawnow;   % flush static scene before animation starts

%% Animation
fskip = max(1, round(step / 500));
ids   = 1:fskip:step;
Nf    = numel(ids);

% Arrow scale: 10 % of the larger horizontal span
vscl = max(diff(xl), diff(zl)) * 0.12;
Fscl = max(diff(xl), diff(zl)) * 0.025;

for fi = 1:Nf
    k    = ids(fi);
    p_k  = pos(:,k);
    v_k  = vel(:,k);
    F_k  = Fact(:,k);
    ph_k = ph(k);
    rk   = rng(k);

    % Phase-coloured live trail
    set(traj_h, 'XData',pos(1,1:k),'YData',pos(2,1:k),'ZData',pos(3,1:k), ...
        'Color',[pc(ph_k+1,:) 0.90]);

    % Spacecraft block (delete previous, redraw)
    for hi = 1:numel(sc_h); delete(sc_h{hi}); end
    bsz = max(2.0, min(5.5, rk * 0.040));
    sc_h = make_block(ax3, p_k, bsz, pc(ph_k+1,:));

    % Velocity arrow
    set(vel_q,'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3), ...
        'UData',v_k(1)*vscl,'VData',v_k(2)*vscl,'WData',v_k(3)*vscl);

    % Thrust arrow
    set(F_q,'XData',p_k(1),'YData',p_k(2),'ZData',p_k(3), ...
        'UData',F_k(1)*Fscl,'VData',F_k(2)*Fscl,'WData',F_k(3)*Fscl);

    % Text overlays
    set(to_r,'String',sprintf('Range : %.3f m',    rk));
    set(to_p,'String',sprintf('Phase : %s',pnames{ph_k+1}),'Color',pc(ph_k+1,:));
    set(to_v,'String',sprintf('Speed : %.4f m/s',  norm(v_k)));
    set(to_d,'String',sprintf('\DeltaV : %.3f m/s',dv_v(k)));
    if isfield(telem,'mode')
        set(to_m,'String',sprintf('Mode  : %s',mnames{telem.mode(k)+1}));
    end
    if rk < 0.06
        set(to_dk,'String','DOCKED');
        title(ax3,'Autonomous Spacecraft Docking — DOCKED','Color','g','FontSize',12,'FontWeight','bold');
    end

    % Side panels
    set(rng_h,'XData',t(1:k),'YData',rng(1:k));
    set(spd_h,'XData',t(1:k),'YData',velm(1:k));
    set(dv_h, 'XData',t(1:k),'YData',dv_v(1:k));

    % Camera: gentle azimuth sweep
    view(ax3, 38 + 18*sin(fi/Nf*pi), 22);

    drawnow limitrate;
    pause(0.008);
end
fprintf('Animation complete.\n');
end

% ── Helpers ──────────────────────────────────────────────────────────────

function hlist = make_block(ax, pos, sz, col)
% Draw a coloured cube centred at pos with half-size sz*0.5.
    h = sz * 0.5;
    verts = h * [-1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1;
                 -1 -1  1;  1 -1  1;  1  1  1; -1  1  1] + pos';
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    h1 = patch(ax, 'Vertices',verts, 'Faces',faces, ...
        'FaceColor',col, 'FaceAlpha',0.85, 'EdgeColor','w', ...
        'EdgeAlpha',0.9, 'LineWidth',0.6);
    hlist = {h1};
end

function segs = get_segs(idx)
% Split an index vector into cell array of contiguous runs.
    segs = {};
    if isempty(idx); return; end
    brk = find(diff(idx) > 1);
    s = [1; brk+1];  e = [brk; numel(idx)];
    for i = 1:numel(s)
        segs{end+1} = idx(s(i):e(i));  %#ok<AGROW>
    end
end
