clear; clc; close all;

% ============================
% Step 1: Nominal statistics
% ============================
load('dataTask2.mat')
[x_nom ,~,b_nom] = Task3_2Run_v2(u_k, y_k, t, dt);
imu_nom = u_k - b_nom;

% Choose stable interval (e.g. 0s to 250s)
idx_nom = (t >= 0) & (t <= 250);
mu      = mean(imu_nom(idx_nom,:), 1);
sigma   = std( imu_nom(idx_nom,:), 0, 1);

% ============================
% Step 2: Process faulty data
% ============================
load('dataTask3.mat')
[x_est ,~, b_est] = Task3_2Run_v2(u_k, y_k, t, dt);
imu_est = u_k - b_est;

N      = numel(t);
labels = {'A_x','A_y','A_z','p','q','r'};
gamma      = 1.00;
delta      = 3.00;
delta  = delta * sigma;   % Threshold
gamma  = gamma * sigma;   % Leakage
warmUp = round(20 / dt);  % 20s delay

faultTimes_all = cell(1,6);  % Store all detections
faultFlag      = false(N,6); % Mark detected regions

% ============================
% Step 3: CUSUM loop (multi-detection, with reset)
% ============================
for i = 1:6
    gPos = 0; gNeg = 0;
    times_i = [];

    for k = warmUp+1 : N
        s = imu_est(k,i) - mu(i);
        gPos = max(0, gPos + s - gamma(i));
        gNeg = min(0, gNeg + s + gamma(i));

        if (gPos > delta(i)) || (gNeg < -delta(i))
            times_i(end+1) = t(k);          % store fault time
            faultFlag(k:end,i) = true;
            gPos = 0; gNeg = 0;             % reset after detection
        end
    end

    faultTimes_all{i} = times_i;
end

% ============================
% Step 4: Plotting
% ============================
figure('Name','CUSUM Fault Detection (Multi Faults)');
for i = 1:6
    subplot(2,3,i)
    plot(t, imu_est(:,i), 'b'); hold on;
    plot(t, imu_nom(:,i), 'g'); hold on;

    % plot all detections
    if ~isempty(faultTimes_all{i})
        for f = faultTimes_all{i}
            xline(f, 'r--', 'LineWidth', 1.2);
        end
        legend('IMU est.','IMU nom.','Faults');
    else
        legend('IMU est.','IMU nom.');
    end

    title(labels{i});
    xlabel('Time (s)');
    ylabel('IMU');
    grid on;
end

% ============================
% Step 5: Display results
% ============================
disp('────────  Detected Fault Times (s)  ────────')
for i = 1:6
    if isempty(faultTimes_all{i})
        fprintf('%4s :  none detected\n', labels{i});
    else
        fprintf('%4s :  [', labels{i});
        fprintf('%.2f ', faultTimes_all{i});
        fprintf(']\n');
    end
end
