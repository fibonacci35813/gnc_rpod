function plot_all_telemetry(telem, step)
% Plot all 8 telemetry figures and save to plots/ directory

if nargin < 2; step = length(telem.time); end

t = telem.time(1:step);
phase_names = {'Phase 0 (>50m)','Phase 1 (>10m)','Phase 2 (>1m)','Phase 3 (<=1m)'};
mode_names  = {'NOMINAL','HOLD','RETREAT','SAFE'};

%% Figure 1: Range vs Time
figure(1); clf;
semilogy(t, telem.range(1:step), 'b-', 'LineWidth', 1.5);
hold on;
semilogy([t(1), t(end)], [0.05, 0.05], 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Range (m)');
title('Figure 1: Range vs Time');
legend('Range','Docking threshold (5cm)','Location','best');
grid on;
saveas(gcf, 'plots/fig1_range_vs_time.png');

%% Figure 2: 3D LVLH Trajectory
figure(2); clf;
set(gcf,'Color','k');
ax = axes; ax.Color = 'k'; ax.XColor = 'w'; ax.YColor = 'w'; ax.ZColor = 'w';
hold on;
pos = telem.pos_true(:,1:step);
plot3(pos(1,:), pos(2,:), pos(3,:), 'Color',[0.3,0.5,1.0], 'LineWidth', 1.5);
plot3(pos(1,1), pos(2,1), pos(3,1), 'gs', 'MarkerSize', 12, 'LineWidth', 2);
plot3(0, 0, 0, 'r*', 'MarkerSize', 14, 'LineWidth', 2);
xlabel('X_{LVLH} (m)','Color','w');
ylabel('Y_{LVLH} (m)','Color','w');
zlabel('Z_{LVLH} (m)','Color','w');
title('Figure 2: 3D LVLH Trajectory','Color','w');
legend({'Path','Start','Docking Port'},'TextColor','w','Color','k');
grid on; ax.GridColor = [0.4,0.4,0.4];
saveas(gcf, 'plots/fig2_3d_trajectory.png');

%% Figure 3: Navigation Error per axis
figure(3); clf;
nav_err = telem.nav_err(:,1:step);
ax_labels = {'X','Y','Z'};
for i=1:3
    subplot(3,1,i);
    plot(t, nav_err(i,:), 'LineWidth', 1.2);
    ylabel(sprintf('%s error (m)', ax_labels{i}));
    grid on;
    if i==1; title('Figure 3: Navigation Error (True - Estimated)'); end
end
xlabel('Time (s)');
saveas(gcf, 'plots/fig3_nav_error.png');

%% Figure 4: Guidance Phase
figure(4); clf;
stairs(t, telem.phase(1:step), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Guidance Phase');
title('Figure 4: Guidance Phase vs Time');
yticks([0,1,2,3]);
yticklabels(phase_names);
ylim([-0.5, 3.5]);
grid on;
saveas(gcf, 'plots/fig4_guidance_phase.png');

%% Figure 5: Delta-V Budget
figure(5); clf;
plot(t, telem.dv(1:step), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Cumulative \Delta V (m/s)');
title('Figure 5: Delta-V Budget');
grid on;
saveas(gcf, 'plots/fig5_delta_v.png');

%% Figure 6: Tank Blow-Down Pressure
figure(6); clf;
plot(t, telem.pressure(1:step), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Tank Pressure (MPa)');
title('Figure 6: Tank Blow-Down Pressure');
grid on;
saveas(gcf, 'plots/fig6_tank_pressure.png');

%% Figure 7: Attitude Error vs Time
figure(7); clf;
plot(t, telem.att_err(1:step), 'b-', 'LineWidth', 1.2);
hold on;
plot([t(1), t(end)], [1, 1], 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Attitude Error (deg)');
title('Figure 7: Attitude Error vs Time');
legend('Att error','1 deg limit','Location','best');
grid on;
saveas(gcf, 'plots/fig7_attitude_error.png');

%% Figure 8: FDIR / Mission Mode
figure(8); clf;
subplot(2,1,1);
stairs(t, telem.mode(1:step), 'b-', 'LineWidth', 1.5);
ylabel('Mission Mode');
yticks([0,1,2,3]);
yticklabels(mode_names);
ylim([-0.5,3.5]);
title('Figure 8: FDIR / Mission Mode');
grid on;

subplot(2,1,2);
stairs(t, telem.fault_code(1:step), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Fault Code');
yticks([0,1,2,3,4]);
yticklabels({'NOMINAL','STUCK\_OPEN','STUCK\_CLOSED','DROPOUT','NAV\_DIVERGE'});
ylim([-0.5,4.5]);
grid on;
saveas(gcf, 'plots/fig8_fdir_mode.png');

fprintf('All 8 figures saved to plots/\n');
end
