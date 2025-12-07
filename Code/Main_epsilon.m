clc; clear; close all;

channels = 1:6;
labels = {'A_x','A_y','A_z','p','q','r'};
best_params_epsilon = zeros(6, 6);  % [γ δ σ D F V]

% ε thresholds
eps_F = 150;     % max false alarms
eps_V = 0.6;     % max variance

% Sobol Sampling configuration
N_samples = 300;
gamma_bounds = [1, 5];
delta_bounds = [3, 10];
sigma_bounds = [0.01, 0.1];

sobol = sobolset(3, 'Skip', 1000, 'Leap', 100);
sobol = scramble(sobol, 'MatousekAffineOwen');
samples = net(sobol, N_samples);

% Rescale to parameter ranges
gammas = gamma_bounds(1) + samples(:,1) * (gamma_bounds(2) - gamma_bounds(1));
deltas = delta_bounds(1) + samples(:,2) * (delta_bounds(2) - delta_bounds(1));
sigmas = sigma_bounds(1) + samples(:,3) * (sigma_bounds(2) - sigma_bounds(1));

for ch = channels
    fprintf('\n🔍 Optimizing channel %d using Sobol + ε-Constraint...\n', ch);
    results = zeros(N_samples, 6);  % [γ δ σ D F V]

    parfor i = 1:N_samples
        g = gammas(i);
        d = deltas(i);
        s = sigmas(i);
        [D, F, V] = evaluateDesign_channel_pareto([g d s], ch);
        results(i, :) = [g, d, s, D, F, V];
    end

    % ε-Constraint filtering
    feasible = results(results(:,5) <= eps_F & results(:,6) <= eps_V, :);

    if isempty(feasible)
        warning('⚠️ No feasible solution for channel %d.', ch);
        best_params_epsilon(ch, :) = NaN;
    else
        [~, idx] = min(feasible(:,4));  % select with min delay
        best_params_epsilon(ch, :) = feasible(idx, :);
    end
end

% Show results
disp('===== ε-Constraint Optimization Results (Sobol + Parallel) =====');
T = array2table(best_params_epsilon, 'VariableNames', ...
    {'gamma','delta','sigma_bias','Delay','FalseAlarms','Variance'}, ...
    'RowNames', labels);
disp(T);

% Save to file
save('best_per_channel_results_epsilon.mat', 'best_params_epsilon');
