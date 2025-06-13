%%

clear; close all; clc;

%% ----- Generate bitmap for Lidar usage -----
generateNursery; % The bitmap is "ground-truth"; robot DO NOT consult it except through laserScannerNoisy.p Needed for simulation.
% x; y;

%% ----- Initialize all parameters -----

% Parameters of the vehicle
L = 3;                      % wheelbase, 3 meters
W = 2.5;                    % width of the vehicle, 2 meters
gamma_limit = deg2rad(55);  % steering limit, 55 deg
gamma_v = 0.1;              % steering lag, 0.1 sec
tau_v = 0.1;                % velocity lag, 0.1 sec
v_ref = 1;                  % placeholder speed, may adjust dependent on the project
steer_tau = 0.1;
vel_tau = 0.1;

% initial state of vehicle [x, y, theta, gamma, v]
q = [0, 0, pi/2, 0, v_ref]; % start from (0,0), facing north, define the v_ref

umin = [-gamma_limit;0];    % steering lower bounds
umax = [gamma_limit;3];     % upper bounds
Qmin = [-inf; -inf; -inf; -gamma_limit; 0];
Qmax = [inf; inf; inf; gamma_limit;3];

%$ ----- Time parameters -----
global dt DT %#ok<GVMIS>
dt = 0.001;
DT  = 0.01;
Tsim = 40;      % total simulation time [s] % quick test
steps = 0:DT:Tsim;
Nsteps = numel(steps);

% EKF initial state estimate [x, y, theta]
q_true = [0 0 pi/2 0 v_ref];    % [x y theta gamma v]
x_est = q_true(1:3)';           % EKF state [x y theta]
P = eye(3);                     % initial covariance

% Parameters of the Lidar
angleSpan = deg2rad(180);   % max angle range, 180 deg
angleStep = deg2rad(0.125); % angular resolution, 0.125 deg
rangeMax = 20;              % max Lidar range, 20 meters

% Grid parameters
[R,C]   = size(bitmap);         % use same resolution as provided map
logOdds = zeros(R,C);           % start with p=0.5 everywhere
p_occ   = 0.7; p_free = 0.3;    % inverse‑sensor model constants
L_max   = 6;                    % log‑odds saturation

%% ----- Data Storage -----
q_hist  = zeros(Nsteps,5);
odo_hist = zeros(Nsteps,2);
gps_hist = zeros(floor(Nsteps/100),3);
x_est_hist = zeros(Nsteps,3);
scan_hist = cell(Nsteps,1);                % each cell will hold LiDAR rays

%% ----- Noise covariances (initial guess) -----
Q_odo = eye(2)*1e-3;                     % will be re‑estimated
R_gps = diag([0.05 0.05 0.01]);          % will be re‑estimated

%% ==================================================================
%                           EKF MAIN LOOP                            
%% ==================================================================

gps_idx = 0;
for k = 1:Nsteps
    t = steps(k);

    % ----- True dynamics
    u_cmd   = [0; v_ref];                       % straight motion demo
    [q_true, odo] = robot_odo(q_true, u_cmd, umin, umax, Qmin, Qmax, L, steer_tau, vel_tau);

    % Log true state & odometry (distance, heading incr returned by .p)
    q_hist(k,:)   = q_true;
    odo_hist(k,:) = odo(:)';

    % ----- EKF prediction
    delta_d     = odo(1);
    delta_theta = odo(2);
    th          = x_est(3);

    x_pred = x_est + [delta_d*cos(th);
                      delta_d*sin(th);
                      delta_theta];
    x_pred(3) = wrapToPi(x_pred(3));

    Fx = eye(3);               % dF/dx
    Fx(1,3) = -delta_d*sin(th);
    Fx(2,3) =  delta_d*cos(th);

    Fu = [cos(th) 0;
          sin(th) 0;
          0       1];          % dF/dw, where w = [d vtheta] noise

    P = Fx*P*Fx' + Fu*Q_odo*Fu';

    % ----- EKF correction (GPS each 1 s)
    if mod(k,100) == 0
        gps_idx = gps_idx + 1;
        [x_gps,y_gps,th_gps] = GPS_CompassNoisy(q_true(1),q_true(2),q_true(3));
        z     = [x_gps; y_gps; th_gps];
        gps_hist(gps_idx,:) = z';

        innov = z - x_pred;
        innov(3) = wrapToPi(innov(3));

        H  = eye(3);
        S  = H*P*H' + R_gps;
        K  = P*H'/S;

        x_est = x_pred + K*innov;
        x_est(3) = wrapToPi(x_est(3));
        P = (eye(3)-K*H)*P;
    else
        x_est = x_pred;
    end

    % ----- Log estimate
    x_est_hist(k,:) = x_est';

    % --- LiDAR scan & occupancy‑grid update
    Tl = [cos(q_true(3)) -sin(q_true(3)) q_true(1);
          sin(q_true(3))  cos(q_true(3)) q_true(2);
          0               0              1       ];        % SE(2) pose
    scan = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl, bitmap, Xmax, Ymax);
    angles = scan(:,1);
    ranges = medfilt1(scan(:,2),5);    % spike removal

    logOdds = updateLaserBeamGrid(angles, ranges, Tl, ...
                              logOdds, R, C, Xmax, Ymax);
    
    % ----- robot trace its path while the sim runs
    if k == 1
    figure(99); clf; hold on; axis equal; grid on;
    hTrue = plot(NaN,NaN,'k--');       % handles saved for speed
    hEst  = plot(NaN,NaN,'b');
    xlim([-5 45]); ylim([-5 45]);      % adjust to your map
    end

    if mod(k,20) == 0                       % refresh 5× per second
    set(hTrue,'XData',q_hist(1:k,1),'YData',q_hist(1:k,2));
    set(hEst ,'XData',x_est_hist(1:k,1),'YData',x_est_hist(1:k,2));
    drawnow limitrate;
    end

    if mod(k,100) == 0                       % once per simulated second (DT=0.01)
    fprintf('t = %4.1f / %4.1f  s  (%5.1f %% done)\\n',...
            steps(k), Tsim, 100*k/Nsteps);
    end

end

%% ----- Compute empirical covariances -----
[emp_Q, emp_R] = compute_sensor_covariances(q_hist, odo_hist, gps_hist);

fprintf('\nEmpirical odometry covariance (Q_odo):\n'); disp(emp_Q);
fprintf('Empirical GPS + compass covariance (R_gps):\n'); disp(emp_R);

%% ----- Trajectory plots -----
figure; subplot(1,2,1);
imagesc(logOdds>0); axis image; title('Occupancy grid (p>0.5)');
colormap gray;
subplot(1,2,2); hold on; grid on; axis equal;
plot(q_hist(:,1), q_hist(:,2),'k--');
plot(x_est_hist(:,1), x_est_hist(:,2),'b');
legend('True','EKF'); xlabel('X [m]'); ylabel('Y [m]');
%% ==================================================================
%                  Local functions (covariance tools)                
%% ==================================================================


%% ----- Implement Laser Scanner -----
% p = laserScannerNoisy(angleSpan, angleStep, rangeMax, q(3), bitmap, Xmax, Ymax);

%% ----- Main function -----

function [Q_odo, R_gps] = compute_sensor_covariances(q_history, odo_history, gps_history)
% COMPUTE_SENSOR_COVARIANCES Compute covariance matrices from recorded data
%
% Inputs:
%   q_history: Nx5 matrix of true robot states [x, y, theta, gamma, v]
%   odo_history: Nx2 matrix of odometry measurements [x, y] 
%   gps_history: Nx3 matrix of GPS/compass measurements [x, y, theta]
%
% Outputs:
%   Q_odo: 2x2 odometry error covariance matrix
%   R_gps: 3x3 GPS/compass error covariance matrix

%% Input validation
if size(q_history, 2) ~= 5
    error('q_history must be Nx5 matrix [x, y, theta, gamma, v]');
end
if size(odo_history, 2) ~= 2
    error('odo_history must be Nx2 matrix [x, y]');
end
if size(gps_history, 2) ~= 3
    error('gps_history must be Nx3 matrix [x, y, theta]');
end

N_odo = size(odo_history, 1);
N_gps = size(gps_history, 1);
N_true = size(q_history, 1);

%% Compute Odometry Error Covariance (2x2)
% Extract true positions for odometry comparison
true_positions_odo = q_history(1:N_odo, 1:2); % [x, y] from q_history

% Compute odometry errors
odo_errors = odo_history - true_positions_odo;

% Compute covariance matrix from scratch
Q_odo = compute_covariance_matrix(odo_errors);

%% Compute GPS/Compass Error Covariance (3x3)
% Extract true pose for GPS comparison
true_pose_gps = q_history(1:N_gps, 1:3); % [x, y, theta] from q_history

% Compute GPS errors with angle wrapping for theta
gps_errors = zeros(N_gps, 3);
gps_errors(:, 1:2) = gps_history(:, 1:2) - true_pose_gps(:, 1:2); % x, y errors

% Handle angle wrapping for theta errors
for i = 1:N_gps
    gps_errors(i, 3) = angle_difference(gps_history(i, 3), true_pose_gps(i, 3));
end

% Compute covariance matrix from scratch
R_gps = compute_covariance_matrix(gps_errors);

%% Validation
validate_covariance_matrix(Q_odo, 'Odometry');
validate_covariance_matrix(R_gps, 'GPS/Compass');

end

%% Helper Functions

function C = compute_covariance_matrix(errors)
% COMPUTE_COVARIANCE_MATRIX Compute covariance matrix from scratch
% Input: errors - Nx2 or Nx3 matrix of error samples
% Output: C - covariance matrix

[N, dim] = size(errors);

% Compute sample means
error_means = mean(errors, 1);

% Center the data (subtract means)
centered_errors = errors - repmat(error_means, N, 1);

% Compute covariance matrix: C = (1/(N-1)) * X^T * X
C = (1/(N-1)) * (centered_errors' * centered_errors);

end

function angle_diff = angle_difference(angle1, angle2)
% ANGLE_DIFFERENCE Compute wrapped angle difference
% Returns angle1 - angle2 wrapped to [-pi, pi]

diff = angle1 - angle2;
angle_diff = mod(diff + pi, 2*pi) - pi;

end

function validate_covariance_matrix(C, name)
% VALIDATE_COVARIANCE_MATRIX Check if covariance matrix is valid

% Check if symmetric
is_symmetric = norm(C - C') < 1e-12;
if ~is_symmetric
    warning('%s covariance matrix is not symmetric', name);
end

% Check if positive definite
eigenvals = eig(C);
is_pos_def = all(eigenvals > 0);
if ~is_pos_def
    warning('%s covariance matrix is not positive definite', name);
end

% Check condition number
cond_num = cond(C);
if cond_num > 1000
    warning('%s covariance matrix has high condition number (%.2f) - may be ill-conditioned', name, cond_num);
end

end