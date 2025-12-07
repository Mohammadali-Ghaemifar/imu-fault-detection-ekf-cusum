clc; clear; close all;

% Variable bounds
gamma_bounds = [1, 5];
delta_bounds = [2, 10];
sigma_bounds = [0.01, 0.1];

nPoints = 500;  % Total number of sample points for each method

% ==== 1. Grid Sampling ====
n_side = ceil(nPoints^(1/3));
gammas = linspace(gamma_bounds(1), gamma_bounds(2), n_side);
deltas = linspace(delta_bounds(1), delta_bounds(2), n_side);
sigmas = linspace(sigma_bounds(1), sigma_bounds(2), n_side);
[G, D, S] = ndgrid(gammas, deltas, sigmas);
grid_samples = [G(:), D(:), S(:)];
grid_samples = grid_samples(1:nPoints, :);  % Trim to nPoints

% ==== 2. Latin Hypercube Sampling ====
lhs_raw = lhsdesign(nPoints, 3);  % Normalized between 0 and 1
lhs_samples = [...
    gamma_bounds(1) + (gamma_bounds(2)-gamma_bounds(1)) * lhs_raw(:,1), ...
    delta_bounds(1) + (delta_bounds(2)-delta_bounds(1)) * lhs_raw(:,2), ...
    sigma_bounds(1) + (sigma_bounds(2)-sigma_bounds(1)) * lhs_raw(:,3)];

% ==== 3. Sobol Sampling ====
p = sobolset(3);
sobol_raw = net(p, nPoints);
sobol_samples = [...
    gamma_bounds(1) + (gamma_bounds(2)-gamma_bounds(1)) * sobol_raw(:,1), ...
    delta_bounds(1) + (delta_bounds(2)-delta_bounds(1)) * sobol_raw(:,2), ...
    sigma_bounds(1) + (sigma_bounds(2)-sigma_bounds(1)) * sobol_raw(:,3)];

% ==== Plot All ====
figure('Position',[100 100 1400 500])

subplot(1,3,1)
scatter3(grid_samples(:,1), grid_samples(:,2), grid_samples(:,3), 10, 'k', 'filled');
title('Grid Sampling'); xlabel('\gamma'); ylabel('\delta'); zlabel('\sigma_{bias}'); grid on;

subplot(1,3,2)
scatter3(lhs_samples(:,1), lhs_samples(:,2), lhs_samples(:,3), 10, 'b', 'filled');
title('Latin Hypercube Sampling'); xlabel('\gamma'); ylabel('\delta'); zlabel('\sigma_{bias}'); grid on;

subplot(1,3,3)
scatter3(sobol_samples(:,1), sobol_samples(:,2), sobol_samples(:,3), 10, 'r', 'filled');
title('Sobol Sampling'); xlabel('\gamma'); ylabel('\delta'); zlabel('\sigma_{bias}'); grid on;
