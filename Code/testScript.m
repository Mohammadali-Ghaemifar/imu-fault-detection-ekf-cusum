clc; clear;

% --- Load test data ---
load('dataTask3.mat');  % Provides u_k, y_k, t, dt

% --- Run your function ---
[x_est, b_est, Ax_f, Ay_f, Az_f, p_f, q_f, r_f] = SID(u_k, y_k, t, dt);

% --- Handle extended x_est (if 18-dimensional) ---
if size(x_est, 2) > 12
    fprintf("x_est has %d columns; truncating to first 12.\n", size(x_est,2));
    x_est = x_est(:, 1:12);
end

% --- Check dimensions ---
assert(isequal(size(x_est, 2), 12), 'x_est must be N×12');
assert(isequal(size(b_est, 2), 6), 'b_est must be N×6');
assert(isvector(Ax_f), 'Ax_f must be a vector');
assert(isvector(Ay_f), 'Ay_f must be a vector');
assert(isvector(Az_f), 'Az_f must be a vector');
assert(isvector(p_f), 'p_f must be a vector');
assert(isvector(q_f), 'q_f must be a vector');
assert(isvector(r_f), 'r_f must be a vector');

disp('✅ All output dimensions are valid.');
