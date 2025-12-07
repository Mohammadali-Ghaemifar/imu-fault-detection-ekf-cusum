clc; clear; close all;

% Define bounds
gamma_bounds = [1, 5];
delta_bounds = [3, 10];
sigma_bounds = [0.01, 0.1];

% Sampling settings
N_samples = 300;
sobol = sobolset(3, 'Skip', 1000, 'Leap', 100);
sobol = scramble(sobol, 'MatousekAffineOwen');
samples = net(sobol, N_samples);

% Rescale samples
gammas = gamma_bounds(1) + samples(:,1) * (gamma_bounds(2) - gamma_bounds(1));
deltas = delta_bounds(1) + samples(:,2) * (delta_bounds(2) - delta_bounds(1));
sigmas = sigma_bounds(1) + samples(:,3) * (sigma_bounds(2) - sigma_bounds(1));

% Configuration
channels = 1:6;
labels = {'A_x','A_y','A_z','p','q','r'};
best_params = zeros(6, 6);  % [gamma delta sigma D F V]

figure('Name','Pareto Fronts for All Channels','Position',[100 100 1200 600]);

for ch = channels
    fprintf('\n🔍 Channel %d – Evaluating Sobol Samples...\n', ch);
    results = zeros(N_samples, 6);

    parfor i = 1:N_samples
        g = gammas(i);
        d = deltas(i);
        s = sigmas(i);
        [D, F, V] = evaluateDesign_channel_pareto([g d s], ch);
        results(i,:) = [g, d, s, D, F, V];
    end

    % Pareto front
    obj_vals = results(:, 4:6);  % [D, F, V]
    isFront = isParetoEfficient(obj_vals);
    pareto_results = results(isFront, :);

    % Normalized weighted sum to pick 1 point
    norm_vals = normalize(pareto_results(:, 4:6));
    weights = [0.2, 0.3, 0.5];
    cost = norm_vals * weights';
    [~, idx_best] = min(cost);
    best_params(ch, :) = pareto_results(idx_best, :);

    % Plotting
    subplot(2,3,ch);
    scatter3(obj_vals(:,1), obj_vals(:,2), obj_vals(:,3), 'k.'); hold on;
    scatter3(pareto_results(:,4), pareto_results(:,5), pareto_results(:,6), 'ro', 'filled');
    scatter3(best_params(ch,4), best_params(ch,5), best_params(ch,6), 60, 'b*', 'filled');
    xlabel('Delay'); ylabel('False Alarms'); zlabel('Bias Var');
    title(sprintf('Pareto Front - %s', labels{ch}));
    legend('All samples','Pareto Front','Best Trade-off'); grid on;
end

% Display final best parameters
disp('===== Best Balanced Parameters (Sobol + Pareto + Normalized Scoring) =====');
T = array2table(best_params, 'VariableNames', ...
    {'gamma','delta','sigma_bias','Delay','FalseAlarms','Var'}, ...
    'RowNames', labels);
disp(T);

% Save output
save('best_per_channel_results_pareto.mat', 'best_params');
