function [x_est, b_est, Ax_f, Ay_f, Az_f, p_f, q_f, r_f] = SID(u_k, y_k, t, dt)
% Inputs:
%   u_k - N×6 IMU input data [Axm Aym Azm pm qm rm]
%   y_k - N×12 GPS output data
%   t   - N×1 time vector
%   dt  - scalar timestep
% Outputs:
%   x_est - N×12 state estimates [x y z u v w phi theta psi VwxE VwyE VwzE]
%   b_est - N×6 estimated IMU biases [bAx bAy bAz bp bq br]
%   *_f   - vector of fault times for each input channel (or 0 if none)
%% Load data
% --- Load optimized parameters from fixed file (weighted method) ---
load('best_per_channel_weighted_results.mat', 'best_per_channel');
params = best_per_channel(:, 1:3);  % [γ δ σ]

%load('best_per_channel_results_pareto.mat', 'best_params');
%params = best_params(:, 1:3);  % [γ δ σ]

%load('best_per_channel_results_epsilon.mat', 'best_params_epsilon')
%params = best_params_epsilon(:, 1:3);  % [γ δ σ]

% --- Load nominal data to compute threshold statistics ---
nom = load('dataTask2.mat');
[x_nom, ~, b_nom] = Task3_2Run_v2(nom.u_k, nom.y_k, nom.t, nom.dt);
imu_nom = nom.u_k - b_nom;
mu = mean(imu_nom(nom.t <= 250, :), 1);
sigma = std(imu_nom(nom.t <= 250, :), 0, 1);


% --- Final run with tuned EKF ---
[x_est, ~, b_est] = Task3_2Run_v2(u_k, y_k, t, dt);
imu_est = u_k - b_est;
N = numel(t);
warmUp = round(5 / dt);
fault_times = cell(1,6);

% --- Run CUSUM per channel (multi-fault support) ---
for ch = 1:6
    g = params(ch,1) * sigma(ch);
    d = params(ch,2) * sigma(ch);
    s = imu_est(:,ch) - mu(ch);
    gPos = 0; gNeg = 0;
    times = [];

    for k = warmUp+1:N
        gPos = max(0, gPos + s(k) - g);
        gNeg = min(0, gNeg + s(k) + g);
        if (gPos > d) || (gNeg < -d)
            times(end+1) = t(k); %#ok<AGROW>
            gPos = 0; gNeg = 0;
        end
    end
    fault_times{ch} = times;
end

% --- Return fault vectors (or 0 if none) ---
Ax_f = returnFault(fault_times{1});
Ay_f = returnFault(fault_times{2});
Az_f = returnFault(fault_times{3});
p_f  = returnFault(fault_times{4});
q_f  = returnFault(fault_times{5});
r_f  = returnFault(fault_times{6});

% --- Plot (optional but helpful for testing) ---
labels = {'A_x','A_y','A_z','p','q','r'};
figure('Name','SID240162810 Fault Detection');
for i = 1:6
    subplot(2,3,i)
    plot(t, imu_est(:,i), 'b'); hold on;
    plot(t, imu_nom(:,i), 'g'); hold on;
    for f = fault_times{i}
        xline(f, 'r--', 'LineWidth', 1.2);
    end
    title(labels{i}); xlabel('Time (s)'); ylabel('IMU'); grid on;
end

end

function out = returnFault(times)
% Return vector of fault times, or 0 if none
if isempty(times)
    out = 0;
else
    out = times(:);
end
end
