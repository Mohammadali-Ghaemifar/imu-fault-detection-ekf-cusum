function [D, F, V] = evaluateDesign_channel_pareto(params, channel_idx)
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

    % Load nominal stats only once
    persistent mu sigma
    if isempty(mu)
        load('dataTask2.mat');
        [~, ~, b_nom] = Task3_2Run_v2(u_k, y_k, t, dt);
        imu_nom = u_k - b_nom;
        idx = (t >= 0) & (t <= 250);
        mu = mean(imu_nom(idx,:), 1);
        sigma = std(imu_nom(idx,:), 0, 1);
    end

    % Run CUSUM (returns all detection times as a vector)
    [faultTimes, falseAlarms] = runCUSUM_channel(t, imu_est(:,channel_idx), ...
        mu(channel_idx), sigma(channel_idx), gamma, delta, dt);

    % Delay logic: penalize detections before 20s
    min_time = 20;
    if isempty(faultTimes)
        delay = 0;  % no detection → assume no fault, no delay
    elseif all(faultTimes < min_time)
        delay = 999;  % all detections too early → penalize
    else
        delay = min(faultTimes(faultTimes >= min_time));  % earliest valid detection
    end

    % Return vector of raw objectives
    D = delay;
    F = falseAlarms;
    V = var(b_est(:,channel_idx));
end
