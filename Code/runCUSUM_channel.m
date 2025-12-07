function [faultTimes, falseAlarmCount] = runCUSUM_channel(t, imu_sig, mu, sigma, gamma, delta, dt, resetAfterDetection)
    if nargin < 8
        resetAfterDetection = true;  % default: reset CUSUM after each detection
    end

    N = numel(t);
    warmUp = round(5 / dt);  % 5 seconds delay
    gPos = 0; gNeg = 0;
    faultTimes = [];
    falseAlarmCount = 0;
    lastFaultTime = -Inf;
    min_separation = 10;  % Minimum separation between detections in seconds

    for k = warmUp+1:N
        s = imu_sig(k) - mu;
        gPos = max(0, gPos + s - gamma * sigma);
        gNeg = min(0, gNeg + s + gamma * sigma);

        if (gPos > delta * sigma) || (gNeg < -delta * sigma)
            currentTime = t(k);
            if isempty(faultTimes) || (currentTime - lastFaultTime > min_separation)
                faultTimes(end+1) = currentTime; %#ok<AGROW>
                lastFaultTime = currentTime;

                if resetAfterDetection
                    gPos = 0; gNeg = 0;
                end
            else
                falseAlarmCount = falseAlarmCount + 1;
            end
        end
    end

    if isempty(faultTimes)
        faultTimes = 0;
    end
end
