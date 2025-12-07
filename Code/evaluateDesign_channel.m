function J = evaluateDesign_channel(params, channel_idx)
    % Inputs:
    %   params = [gamma, delta, sigma_bias]
    %   channel_idx = 1 to 6 (Ax to r)

    gamma = params(1);
    delta = params(2);
    sigma_bias = params(3);

    % Load faulty data
    load('dataTask3.mat');
    global stdw_override_channel
    stdw_override_channel = [channel_idx, sigma_bias];

    % Run EKF
    [~, ~, b_est] = Task3_2Run_v2(u_k, y_k, t, dt);
    imu_est = u_k - b_est;

    % Load nominal baseline once (persistent for performance)
    persistent mu sigma
    if isempty(mu)
        load('dataTask2.mat');
        [~, ~, b_nom] = Task3_2Run_v2(u_k, y_k, t, dt);
        imu_nom = u_k - b_nom;
        idx = (t >= 0) & (t <= 250);
        mu = mean(imu_nom(idx,:), 1);
        sigma = std(imu_nom(idx,:), 0, 1);
    end

    % CUSUM detection
    [faultTime, falseAlarms] = runCUSUM_channel(t, imu_est(:,channel_idx), mu(channel_idx), sigma(channel_idx), gamma, delta, dt);

    % Cost components
    true_fault_time = 140;
    delay = max(0, faultTime - true_fault_time);

    % Use bias variance only in [120s - 220s] window
    var_window = (t >= 120) & (t <= 220);
    var_bias = var(b_est(var_window,channel_idx));

    % Exponential cost on delay, plus weighted terms
    J = exp(0.05 * delay) + 10 * falseAlarms + 50 * var_bias;

    % Debug Print
    fprintf("→ ch=%d | γ=%.1f δ=%.1f σ=%.3f → Delay=%.1f | FA=%d | Var=%.3f | J=%.2f\n", ...
        channel_idx, gamma, delta, sigma_bias, delay, falseAlarms, var_bias, J);
end
