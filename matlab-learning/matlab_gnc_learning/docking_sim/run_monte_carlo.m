% run_monte_carlo.m - Monte Carlo analysis (50 seeds)

clear; clc;
base = fileparts(mfilename('fullpath'));
addpath(fullfile(base,'environment'),fullfile(base,'sensors'),fullfile(base,'navigation'), ...
        fullfile(base,'guidance'),fullfile(base,'control'),fullfile(base,'attitude'), ...
        fullfile(base,'propulsion'),fullfile(base,'fdir'),fullfile(base,'mission'));

N_MC = 50;
rng_mc = RandStream('mt19937ar','Seed',12345);

results.docked   = false(1,N_MC);
results.steps    = zeros(1,N_MC);
results.dv       = zeros(1,N_MC);
results.pos_err  = zeros(1,N_MC);
results.vel_err  = zeros(1,N_MC);

fprintf('=== Monte Carlo Analysis (%d seeds) ===\n', N_MC);
fprintf('%-6s %-8s %-8s %-10s %-10s %-10s\n', 'Seed','Docked','Steps','DV(m/s)','PosErr(m)','VelErr(m/s)');
fprintf('%s\n', repmat('-',1,60));

for i = 1:N_MC
    seed_i = i * 137;  % deterministic but varied seeds
    
    % Dispersions: pos +/- 3m, vel +/- 0.05 m/s, thr +/- 5%
    dr0 = 3.0 * rng_mc.randn(3,1);
    dv0 = 0.05 * rng_mc.randn(3,1);
    thr_scale = 1.0 + 0.05 * rng_mc.randn();
    
    [docked_i, steps_i, dv_i, pos_err_i, vel_err_i] = run_single_sim(dr0, dv0, thr_scale, seed_i);
    
    results.docked(i)  = docked_i;
    results.steps(i)   = steps_i;
    results.dv(i)      = dv_i;
    results.pos_err(i) = pos_err_i;
    results.vel_err(i) = vel_err_i;
    
    if docked_i; dock_str='YES'; else; dock_str='NO '; end
    fprintf('%-6d %-8s %-8d %-10.3f %-10.4f %-10.4f\n', ...
        i, dock_str, steps_i, dv_i, pos_err_i, vel_err_i);
end

%% Summary statistics
n_docked   = sum(results.docked);
p_dock     = n_docked / N_MC;
mean_dv    = mean(results.dv(results.docked));
mean_pos   = mean(results.pos_err(results.docked));
mean_steps = mean(results.steps(results.docked));

fprintf('\n=== Monte Carlo Summary ===\n');
fprintf('P(dock)     : %.3f (%d/%d)\n', p_dock, n_docked, N_MC);
fprintf('Mean DV     : %.3f m/s\n', mean_dv);
fprintf('Mean PosErr : %.4f m (%.1f mm)\n', mean_pos, mean_pos*1000);
fprintf('Mean Steps  : %.0f steps (%.1f min)\n', mean_steps, mean_steps/60);

%% Histogram plots
figure(200); clf;
subplot(1,3,1);
histogram(results.dv(results.docked), 10, 'FaceColor', 'b');
xlabel('Delta-V (m/s)'); ylabel('Count');
title('DV Budget Distribution');
grid on;

subplot(1,3,2);
histogram(results.pos_err(results.docked)*1000, 10, 'FaceColor', 'r');
xlabel('Position Error (mm)'); ylabel('Count');
title('Final Position Error');
grid on;

subplot(1,3,3);
histogram(results.steps(results.docked), 10, 'FaceColor', 'g');
xlabel('Steps to Dock'); ylabel('Count');
title('Time to Dock');
grid on;

sgtitle(sprintf('Monte Carlo Results (N=%d, P(dock)=%.0f%%)', N_MC, p_dock*100));
saveas(gcf, 'plots/monte_carlo_histograms.png');

fprintf('\nMonte Carlo complete. Histograms saved to plots/\n');
