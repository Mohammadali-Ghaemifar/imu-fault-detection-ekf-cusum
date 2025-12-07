function [x_est, innov, bias] = Task3_2Run_v2(u_k,y_k,t,dt)
%%%%%%%%%%%%%%%%%%%%%%%

% My work
% Input Variables
%   u_k: input measurements, a matrix with rows the recordings of different
%   time instances and the columns correspond to different input variables
%   z_k: output measurements, a matrix with rows the recordings of different 
%   time instances and the columns correspond to different output variables
%   t: time vector
%   dt: uniform time step size
%%%%%%%%%%%%%%%%%%%%%%%
% Output Variables
%   x_est: estimated state trajectory
%%%%%%%%%%%%%%%%%%%%%%%

    %% Main script

    % Paste your EKF script (from initialisation to the end of the
    % loop, i.e. just before the plots) here







% 
% Initialization
%g=9.81; %m/s^2
Ts=dt; %time step (already provided by the data)
N=length(y_k); %total number of steps







% %standard deviation of w
% stdw(1,1:3)=0.01; % deviation for A_xyz
% stdw(1,4:6)=deg2rad(0.01); %deviation for p q r 
% stdw(1,7:9)=0.1;stdw(1,10:12)=deg2rad(0.1);
% Q=diag(stdw.^2); % process covariance matrix 
% %standard deviation of v
% stdxy=5; stdz=10; %position measurements
% stdvel=0.1; %velocity parameters u v w Vtas
% stdangles=deg2rad(0.1); % psii theta phi alpha beta 
% stdv(1:2)=stdxy;stdv(3)=stdz;stdv(4:6)=stdvel;stdv(7:9)=stdangles;
% stdv(10)=stdvel;stdv(11:12)=stdangles;
% R=diag(stdv.^2); % covariance matrix
% %initail conditions 
% dk0=d_k(1,:);
% x0=dk0(1);y0=dk0(2);z0=dk0(3);
% Vtas=110;
% u0=Vtas; v0=0; w0=0;
% phi0=dk0(7);theta0=dk0(8); psii0=dk0(9);
% v_wxE0=0; v_wyE0=0; v_wzE0=0;
% %INITIAL BIASES
% % b_A_x0=15;b_A_y0=5;b_A_z0=15;
% % b_p0=5; b_q0=15; b_r0=105;
% b_A_x0=0;b_A_y0=0;b_A_z0=0;
% b_p0=0; b_q0=0; b_r0=0;
% Ex_0=[x0 y0 z0 u0 v0 w0 phi0 theta0 psii0...
% b_A_x0 b_A_y0 b_A_z0 b_p0 b_q0 b_r0 v_wxE0 ...
% v_wyE0 v_wzE0];%expected value of x_0
% stdx_0=[1 1 1 5 5 5 1 1 1 100 100 100 100 100 100  7 7 7];%standard deviation of x_0

    

% Base noise for IMU measurements
% Only for IMU + 6 bias terms
stdw_imu   = [0.01 0.01 0.01 deg2rad(0.01) deg2rad(0.01) deg2rad(0.01)];
%% NEW
global stdw_override_channel
stdw_bias = 0.1 * ones(1,6);  % default bias stds

if ~isempty(stdw_override_channel)
    ch = stdw_override_channel(1);      
    val = stdw_override_channel(2);
    stdw_bias(ch) = val;

end

% Base IMU noise
stdw_imu = [0.01 0.01 0.01 deg2rad(0.01) deg2rad(0.01) deg2rad(0.01)];
% Total stdw = IMU + bias
stdw = [stdw_imu, stdw_bias(1:3), deg2rad(stdw_bias(4:6))];  % length = 12
Q = diag(stdw.^2);  % size = 12x12




%%
stdv = [5 5 10 0.1 0.1 0.1 deg2rad(0.1) deg2rad(0.1) deg2rad(0.1) 0.1 deg2rad(0.1) deg2rad(0.1)];      %standard deviation of v measurement noise (don't change)

stdx_0=[1 1 1 4 4 4 0.9 0.9 0.9 100 100 100 100 100 100  10 10 10]; %standard deviation of x_0

Ex_0 = [y_k(1,1) y_k(1,2) y_k(1,3) 100 0 0 y_k(1,7) y_k(1,8) y_k(1,9) 0 0 0 0 0 0 0 0 0]; % of the expected states












xhat_km1_km1 = Ex_0; % x(0|0) = E{x_0}
P_km1_km1 = diag(stdx_0.^2);  % P(0|0) = P(0)
Q = diag(stdw.^2);
R=diag(stdv.^2);
%% New
% Increase position and velocity noise
stdv_pos = [8 8 15];           % GPS position (was [5 5 10])
stdv_vel = [0.3 0.3 0.3];      % u, v, w (was 0.1)

% Leave angles and others as is (or adjust if needed)
stdv_angles = deg2rad([0.2 0.2 0.2]);    % φ, θ, ψ
stdv_V = 0.2;
stdv_alpha_beta = deg2rad([0.2 0.2]);

% Combine all
stdv = [stdv_pos stdv_vel stdv_angles stdv_V stdv_alpha_beta];
R = diag(stdv.^2);  % Update in your EKF script



%%
n = length(xhat_km1_km1); % n: state dimension
m = size(u_k, 2);     % m: observation dimension
p = size(y_k, 2);     % m: observation dimension
u_km1 = [zeros(1,m); u_k]; % shifted to have the right indices

% Preallocate storage
stdx_cor  = zeros(N, n);  % \sigma(k-1|k-1), standard deviation of state estimation error (hint: diagonal elements of P(k-1|k-1))
x_cor     = zeros(N, n);  % \hat{x}(k-1|k-1), previous estimation
K       = cell(N, 1);   % K(k) Kalman Gain
innov     = zeros(N, p);  % y(k)-y(k|k-1), innovation, with y(k|k-1)=h(\hat{x}(k|k-1),u(k|k-1),k);
    
for k=1:N
    % Step 1: Prediction
    [t_nonlin, x_nonlin] = ode45(@(t,x) funcf(x, u_km1(k,:), t), [0 Ts], xhat_km1_km1);
    xhat_k_km1 = x_nonlin(end,:); % x(k|k-1) (prediction)

    % Step 2: Covariance matrix of state prediction error / Minimum
    % prediction MSE
    [Phi_km1, Gamma_km1] = funcLinDisDyn(xhat_km1_km1, u_km1(k,:), Ts); % Phi(k,k-1), Gamma(k,k-1)
    extraQ = [zeros(1,9), 1e-2*ones(1,6), 5e-1*ones(1,3)];  % Optional soft diffusion
    P_k_km1 = Phi_km1 * P_km1_km1 * Phi_km1' + Gamma_km1 * Q * Gamma_km1' + diag(extraQ);


    % Step 3: Kalman Gain
    H_k = funcLinDisObs(xhat_k_km1, u_km1(k,:), []);
    Ve = (H_k * P_k_km1 * H_k' + R); % Pz(k|k-1) (prediction)
    K_k = P_k_km1 * H_k' / Ve; % K(k) (gain)
    
    % Step 4: Measurement Update (Correction)
    d_k_km1 = funch(xhat_k_km1,u_km1(k,:),[]); % z(k|k-1) (prediction of output)
    xhat_k_k = xhat_k_km1 + (y_k(k,:) - d_k_km1)*K_k'; % x(k|k) (correction)

    % Step 5: Correction for Covariance matrix of state Estimate error /
    % Minimum MSE
    I_KH = eye(n) - K_k * H_k;
    P_k_k = I_KH * P_k_km1 * I_KH' + K_k * R * K_k'; % P(k|k) (correction)

    % Save data: State estimate and std dev
    stdx_cor(k,:) = sqrt(diag(P_km1_km1)); % \sigma(k-1|k-1) Standard deviation of state estimation error
    x_cor(k,:) = xhat_km1_km1; % \hat{x}(k-1|k-1), estimated state
    K{k,1} = K_k; % K(k) (gain)
    innov(k,:)= y_k(k,:) - d_k_km1;
    
    % Recursive step
    xhat_km1_km1 = xhat_k_k; 
    P_km1_km1 = P_k_k;
end








   
    

    % Assign the output variable
    %x_est=x_cor;
    bias = x_cor(:,10:15);
    x_est=x_cor;
%% dynamics in continous time
function F = funcf(xin, uin, t)

%syms x y z u v w phi theta psi v_wxE v_wxE_dot v_wyE v_wyE_dot v_wzE v_wzE_dot u1 A_xm omega_Ax u2 A_ym omega_Ay u3 A_zm omega_Az u4 p_m omega_p u5 q_m omega_q u6 r_m omega_r x_GPS y_GPS z_GPS u_GPS v_GPS w_GPS phi_GPS theta_GPS psi_GPS V b_Ax b_Ax_dot omega_b_Ax b_Ay b_Ay_dot omega_b_Ay b_Az b_Az_dot omega_b_Az b_p b_p_dot omega_b_p b_q b_q_dot omega_b_q b_r b_r_dot omega_b_r
    
    % Extract values
    x = xin(1);
    y = xin(2);
    z = xin(3);
    u = xin(4);
    v = xin(5);
    w = xin(6);
    phi = xin(7);
    theta = xin(8);
    psi = xin(9);
    v_wxE = xin(16);
    v_wyE = xin(17);
    v_wzE = xin(18);

    b_A_x = xin(10);
    b_A_y = xin(11);
    b_A_z = xin(12);
    b_p = xin(13); 
    b_q = xin(14);
    b_r = xin(15);

    A_x = uin(1);
    A_y = uin(2);
    A_z = uin(3);
    p = uin(4);
    q = uin(5);
    r = uin(6);
    
    g = 9.81;

    % original continuous-time nonlinear system equations
    x_dot =(u*cos(theta)+(v*sin(phi)+w*cos(phi))*sin(theta))*cos(psi) - (v*cos(phi)-w*sin(phi))*sin(psi)+v_wxE;
    y_dot =(u*cos(theta)+(v*sin(phi)+w*cos(phi))*sin(theta))*sin(psi) + (v*cos(phi)-w*sin(phi))*cos(psi)+v_wyE;
    z_dot =-u*sin(theta)+(v*sin(phi)+w*cos(phi))*cos(theta)+v_wzE;

    u_dot =(A_x-b_A_x)-g*sin(theta)+(r-b_r)*v-(q-b_q)*w;
    v_dot =(A_y-b_A_y)+g*cos(theta)*sin(phi)+(p-b_p)*w-(r-b_r)*u;
    w_dot =(A_z-b_A_z)+g*cos(theta)*cos(phi)+(q-b_q)*u-(p-b_p)*v;
    
    phi_dot =(p-b_p)+(q-b_q)*sin(phi)*tan(theta)+(r-b_r)*cos(phi)*tan(theta);
    theta_dot =(q-b_q)*cos(phi)-(r-b_r)*sin(phi);
    psi_dot =(q-b_q)*sin(phi)/cos(theta)+(r-b_r)*cos(phi)/cos(theta);
    
      
    b_Ax_dot = 0;
    b_Ay_dot = 0;
    b_Az_dot = 0;
    b_p_dot = 0;
    b_q_dot = 0;
    b_r_dot = 0;

    v_wxE_dot =0;
    v_wyE_dot =0;
    v_wzE_dot =0;  

    % original, continuous-time, noiseless nonlinear dynamics
    F = [x_dot; y_dot; z_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; b_Ax_dot; b_Ay_dot; b_Az_dot; b_p_dot; b_q_dot; b_r_dot; v_wxE_dot; v_wyE_dot; v_wzE_dot];
end

function d = funch(xin,uin,t)
    % Extract values
    x = xin(1);
    y = xin(2);
    z = xin(3);
    u = xin(4);
    v = xin(5);
    w = xin(6);
    phi = xin(7);
    theta = xin(8);
    psi = xin(9);
    v_wxE = xin(16);
    v_wyE = xin(17);
    v_wzE = xin(18);

    b_A_x = xin(10);
    b_A_y = xin(11);
    b_A_z = xin(12);
    b_p = xin(13); 
    b_q = xin(14);
    b_r = xin(15);


    A_x = uin(1);
    A_y = uin(2);
    A_z = uin(3);
    p = uin(4);
    q = uin(5);
    r = uin(6);

    g = 9.81;

    % original continuous-time nonlinear system equations
    x_GPS = x;
    y_GPS = y;
    z_GPS = z;
        
    u_GPS = (u*cos(theta)+(v*sin(phi)+w*cos(phi))*sin(theta))*cos(psi)-(v*cos(phi)-w*sin(phi))*sin(psi)+v_wxE;
    v_GPS = (u*cos(theta)+(v*sin(phi)+w*cos(phi))*sin(theta))*sin(psi)+(v*cos(phi)-w*sin(phi))*cos(psi)+v_wyE;
    w_GPS = -u*sin(theta)+(v*sin(phi)+w*cos(phi))*cos(theta)+v_wzE;
        
    phi_GPS = phi;
    theta_GPS = theta;
    psi_GPS = psi;
    
    V = sqrt(u^2+v^2+w^2);
    alpha = atan(w/u);
    beta = atan(v/sqrt(u^2+w^2));
    
    % Final observation dynamics
    d = [x_GPS, y_GPS, z_GPS, u_GPS, v_GPS, w_GPS, phi_GPS, theta_GPS, psi_GPS, V, alpha, beta];
end

%% lineaized dynamics in discrete time
function [Phi,Gamma] = funcLinDisDyn(x_linpt,u_linpt,Ts)
    % Extract values
    x = x_linpt(1);
    y = x_linpt(2);
    z = x_linpt(3);
    u = x_linpt(4);
    v = x_linpt(5);
    w = x_linpt(6);
    phi = x_linpt(7);
    theta = x_linpt(8);
    psi = x_linpt(9);
    v_wxE = x_linpt(16);
    v_wyE = x_linpt(17);
    v_wzE = x_linpt(18);

    b_A_x = x_linpt(10);
    b_A_y = x_linpt(11);
    b_A_z = x_linpt(12);
    b_p = x_linpt(13); 
    b_q = x_linpt(14);
    b_r = x_linpt(15);


    A_x_m = u_linpt(1);
    A_y_m = u_linpt(2);
    A_z_m = u_linpt(3);
    p_m = u_linpt(4);
    q_m = u_linpt(5);
    r_m = u_linpt(6);

    omega_A_x = 0;
    omega_A_y = 0;
    omega_A_z = 0;
    omega_p = 0;
    omega_q = 0;
    omega_r = 0;

    % F/A Matrix
    A = [[0, 0, 0, cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), sin(psi)*(w*cos(phi) + v*sin(phi)) + cos(psi)*sin(theta)*(v*cos(phi) - w*sin(phi)),                                   -cos(psi)*(u*sin(theta) - cos(theta)*(w*cos(phi) + v*sin(phi))), - sin(psi)*(sin(theta)*(w*cos(phi) + v*sin(phi)) + u*cos(theta)) - cos(psi)*(v*cos(phi) - w*sin(phi)),  0,  0,  0,  0,                    0,                    0, 1, 0, 0]
         [0, 0, 0, cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), sin(psi)*sin(theta)*(v*cos(phi) - w*sin(phi)) - cos(psi)*(w*cos(phi) + v*sin(phi)),                                   -sin(psi)*(u*sin(theta) - cos(theta)*(w*cos(phi) + v*sin(phi))),   cos(psi)*(sin(theta)*(w*cos(phi) + v*sin(phi)) + u*cos(theta)) - sin(psi)*(v*cos(phi) - w*sin(phi)),  0,  0,  0,  0,                    0,                    0, 0, 1, 0]
         [0, 0, 0,         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta),                                               cos(theta)*(v*cos(phi) - w*sin(phi)),                                             - sin(theta)*(w*cos(phi) + v*sin(phi)) - u*cos(theta),                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 1]
         [0, 0, 0,                   0,                                        r_m - b_r,                                        b_q - q_m,                                                                                  0,                                                                             -(981*cos(theta))/100,                                                                                                     0, -1,  0,  0,  0,                    w,                   -v, 0, 0, 0]
         [0, 0, 0,           b_r - r_m,                                                0,                                        p_m - b_p,                                                      (981*cos(phi)*cos(theta))/100,                                                                    -(981*sin(phi)*sin(theta))/100,                                                                                                     0,  0, -1,  0, -w,                    0,                    u, 0, 0, 0]
         [0, 0, 0,           q_m - b_q,                                        b_p - p_m,                                                0,                                                     -(981*cos(theta)*sin(phi))/100,                                                                    -(981*cos(phi)*sin(theta))/100,                                                                                                     0,  0,  0, -1,  v,                   -u,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                  sin(phi)*tan(theta)*(b_r - r_m) - cos(phi)*tan(theta)*(b_q - q_m),               - cos(phi)*(b_r - r_m)*(tan(theta)^2 + 1) - sin(phi)*(b_q - q_m)*(tan(theta)^2 + 1),                                                                                                     0,  0,  0,  0, -1, -sin(phi)*tan(theta), -cos(phi)*tan(theta), 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                        cos(phi)*(b_r - r_m) + sin(phi)*(b_q - q_m),                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,            -cos(phi),             sin(phi), 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,              (sin(phi)*(b_r - r_m))/cos(theta) - (cos(phi)*(b_q - q_m))/cos(theta), - (cos(phi)*sin(theta)*(b_r - r_m))/cos(theta)^2 - (sin(phi)*sin(theta)*(b_q - q_m))/cos(theta)^2,                                                                                                     0,  0,  0,  0,  0, -sin(phi)/cos(theta), -cos(phi)/cos(theta), 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]
         [0, 0, 0,                   0,                                                0,                                                0,                                                                                  0,                                                                                                 0,                                                                                                     0,  0,  0,  0,  0,                    0,                    0, 0, 0, 0]];



    % G Matrix
    G = [[ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0,    0]
        [-1,  0,  0,  0,                    w,                   -v,    0,    0,    0,    0,    0,    0]
        [ 0, -1,  0, -w,                    0,                    u,    0,    0,    0,    0,    0,    0]
        [ 0,  0, -1,  v,                   -u,                    0,    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0, -1, -sin(phi)*tan(theta), -cos(phi)*tan(theta),    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,            -cos(phi),             sin(phi),    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0, -sin(phi)/cos(theta), -cos(phi)/cos(theta),    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0, 1/Ts,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0, 1/Ts,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0, 1/Ts,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0, 1/Ts,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0, 1/Ts,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0, 1/Ts]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0,    0]
        [ 0,  0,  0,  0,                    0,                    0,    0,    0,    0,    0,    0,    0]];













    [Phi,Gamma]=c2d(A,G,Ts);
end

function H = funcLinDisObs(x_linpt,u_linpt,t)
    % Extract values
    x = x_linpt(1);
    y = x_linpt(2);
    z = x_linpt(3);
    u = x_linpt(4);
    v = x_linpt(5);
    w = x_linpt(6);
    phi = x_linpt(7);
    theta = x_linpt(8);
    psi = x_linpt(9);
    v_wxE = x_linpt(16);
    v_wyE = x_linpt(17);
    v_wzE = x_linpt(18);

    b_A_x = x_linpt(10);
    b_A_y = x_linpt(11);
    b_A_z = x_linpt(12);
    b_p = x_linpt(13); 
    b_q = x_linpt(14);
    b_r = x_linpt(15);


    A_x = u_linpt(1);
    A_y = u_linpt(2);
    A_z = u_linpt(3);
    p = u_linpt(4);
    q = u_linpt(5);
    r = u_linpt(6);

    H = [[1, 0, 0,                                                0,                                                0,                                                0,                                                                                  0,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 1, 0,                                                0,                                                0,                                                0,                                                                                  0,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 1,                                                0,                                                0,                                                0,                                                                                  0,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 0,                              cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), sin(psi)*(w*cos(phi) + v*sin(phi)) + cos(psi)*sin(theta)*(v*cos(phi) - w*sin(phi)), -cos(psi)*(u*sin(theta) - cos(theta)*(w*cos(phi) + v*sin(phi))), - sin(psi)*(sin(theta)*(w*cos(phi) + v*sin(phi)) + u*cos(theta)) - cos(psi)*(v*cos(phi) - w*sin(phi)), 0, 0, 0, 0, 0, 0, 1, 0, 0]
         [0, 0, 0,                              cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), sin(psi)*sin(theta)*(v*cos(phi) - w*sin(phi)) - cos(psi)*(w*cos(phi) + v*sin(phi)), -sin(psi)*(u*sin(theta) - cos(theta)*(w*cos(phi) + v*sin(phi))),   cos(psi)*(sin(theta)*(w*cos(phi) + v*sin(phi)) + u*cos(theta)) - sin(psi)*(v*cos(phi) - w*sin(phi)), 0, 0, 0, 0, 0, 0, 0, 1, 0]
         [0, 0, 0,                                      -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta),                                               cos(theta)*(v*cos(phi) - w*sin(phi)),           - sin(theta)*(w*cos(phi) + v*sin(phi)) - u*cos(theta),                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
         [0, 0, 0,                                                0,                                                0,                                                0,                                                                                  1,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 0,                                                0,                                                0,                                                0,                                                                                  0,                                                               1,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 0,                                                0,                                                0,                                                0,                                                                                  0,                                                               0,                                                                                                     1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 0,                        u/(u^2 + v^2 + w^2)^(1/2),                        v/(u^2 + v^2 + w^2)^(1/2),                        w/(u^2 + v^2 + w^2)^(1/2),                                                                                  0,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 0,                           -w/(u^2*(w^2/u^2 + 1)),                                                0,                              1/(u*(w^2/u^2 + 1)),                                                                                  0,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
         [0, 0, 0, -(u*v)/((u^2 + w^2)^(3/2)*(v^2/(u^2 + w^2) + 1)),      1/((u^2 + w^2)^(1/2)*(v^2/(u^2 + w^2) + 1)), -(v*w)/((u^2 + w^2)^(3/2)*(v^2/(u^2 + w^2) + 1)),                                                                                  0,                                                               0,                                                                                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0]];


end

end