clc; clear; close all

channels = 1:6;  % Ax to r
best_per_channel = zeros(6, 4);  % [gamma, delta, sigma_bias, J]

% Define parameter bounds
gamma_bounds = [1, 5];
delta_bounds = [3, 10];
sigma_bounds = [0.01, 0.1];

% Sobol sampling configuration
N_samples = 300;
sobol = sobolset(3, 'Skip', 1000, 'Leap', 100);
sobol = scramble(sobol, 'MatousekAffineOwen');
samples = net(sobol, N_samples);

% Rescale Sobol samples to actual parameter ranges
gammas = gamma_bounds(1) + samples(:,1) * (gamma_bounds(2) - gamma_bounds(1));
deltas = delta_bounds(1) + samples(:,2) * (delta_bounds(2) - delta_bounds(1));
sigmas = sigma_bounds(1) + samples(:,3) * (sigma_bounds(2) - sigma_bounds(1));

for ch = channels
    fprintf("\n🔎 Optimizing for channel %d using Sobol + Parallel\n", ch);
    J_all = zeros(N_samples, 1);

    % Preallocate parameter vectors for results
    g_all = gammas;
    d_all = deltas;
    s_all = sigmas;

    % Parallel evaluation
    parfor i = 1:N_samples
        J_all(i) = evaluateDesign_channel([g_all(i), d_all(i), s_all(i)], ch);
    end

    % Find best
    [best_J, idx] = min(J_all);
    best_per_channel(ch, :) = [g_all(idx), d_all(idx), s_all(idx), best_J];

    fprintf("✅ Best for channel %d: γ=%.2f, δ=%.2f, σ=%.3f, J=%.2f\n", ...
        ch, g_all(idx), d_all(idx), s_all(idx), best_J);
end

% Save final result
save('best_per_channel_weighted_results.mat', 'best_per_channel');
