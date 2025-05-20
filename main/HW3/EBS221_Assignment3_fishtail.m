%%

clear; close all; clc;


%% -------------------- Step A: Define Problem Parameters --------------------
N = 10;                     % number of rows
RL = 20;                    % row length
W = 2.5;                    % implement width (row spacing)
L = 3;                      % tractor length
Ld = 2;                   % look-ahead distance
gamma_limit = deg2rad(60);  % steering limit
Rmin = L / tan(gamma_limit);  % minimum turning radius
v_ref = 0.5;  % [m/s], desired reference speed


% Start and end positions
x_start = -3 * W;
y_start = RL / 2;
x_end = x_start;
y_end = y_start;

% Coordinates of headland nodes (lower and upper)
x_lower = W + 2.5 * (0:N-1); % added 2.5 m to correct X coordinates
y_lower = zeros(1, N);

x_upper = x_lower;
y_upper = RL * ones(1, N);

% Assemble full node coordinate list (1=start, 2..N+1=lower, N+2..2N+1=upper, 2N+2=end)
x = [x_start, x_lower, x_upper, x_end];
y = [y_start, y_lower, y_upper, y_end];

% Plot for verification
figure; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2)); axis equal;
title('Node Layout Verification'); xlabel('X [m]'); ylabel('Y [m]');
status = mkdir('results');
saveas(gcf, "results\node_layout.png")

%% -------------------- Step B: Cost Matrix with Reeds-Shepp Fishtail Turns --------------------

HUGE = 1e6;
DMAT_FT = HUGE * ones(2*N+2);  % Cost matrix for fishtail maneuvering

% 1. Free row traversal (lower <-> upper)
for i = 2:N+1
    DMAT_FT(i, i+N) = 0;
    DMAT_FT(i+N, i) = 0;
end

% 2. Headland transitions (same orientation → reverse entry into next row)
for i = 2:N
    for j = i+1:N+1
        % Bottom headland: both facing south
        q0_bot = [x(i), y(i), -pi/2];
        q1_bot = [x(j), y(j), -pi/2];
        poses = reedsSheppPathSample(q0_bot, q1_bot, Rmin, 0.25);
        len = sum(vecnorm(diff(poses(:,1:2)),2,2));
        DMAT_FT(i, j) = len;
        DMAT_FT(j, i) = len;

        % Top headland: both facing north
        q0_top = [x(i+N), y(i+N), pi/2];
        q1_top = [x(j+N), y(j+N), pi/2];
        poses = reedsSheppPathSample(q0_top, q1_top, Rmin, 0.25);
        len = sum(vecnorm(diff(poses(:,1:2)),2,2));
        DMAT_FT(i+N, j+N) = len;
        DMAT_FT(j+N, i+N) = len;
    end
end

% 3. Connections from/to start and end nodes using Manhattan distance
for i = 2:2*N+1
    DMAT_FT(1,i) = abs(x(1) - x(i)) + abs(y(1) - y(i));
    DMAT_FT(i,1) = DMAT_FT(1,i);
    DMAT_FT(end,i) = abs(x(end) - x(i)) + abs(y(end) - y(i));
    DMAT_FT(i,end) = DMAT_FT(end,i);
end

% 4. Prohibit direct jump from start to end
DMAT_FT(1,end) = HUGE;
DMAT_FT(end,1) = HUGE;

% 5. Solve the TSP using provided GA function
XY = [x', y'];
resultFT = tspof_ga('xy', XY, 'DMAT', DMAT_FT, ...
                    'SHOWRESULT', true, ...
                    'SHOWWAITBAR', true, ...
                    'SHOWPROG', true);

% route assignment for downstream Steps C and D
routeFT = [1, resultFT.optRoute, 2*N+2];

%% -------------------- Step C: Generate Waypoints with Reeds-Shepp Fishtail Turns --------------------

waypointsFT = [];

for k = 1:numel(routeFT)-1
    i = routeFT(k);
    j = routeFT(k+1);
    p1 = [x(i), y(i)];
    p2 = [x(j), y(j)];

    % Free traversal within a row
    if ((i>=2 && i<=N+1 && j-i==N) || (j>=2 && j<=N+1 && i-j==N))
        n = ceil(norm(p2 - p1)/0.5);
        t = linspace(0, 1, n).';
        waypointsFT = [waypointsFT; (p1 + (p2 - p1).*t(2:end))];
        continue
    end

    % Fishtail maneuver: reverse into next row (same orientation)
    sameBottom = all([i j] >= 2    & [i j] <= N+1);
    sameTop    = all([i j] >= N+2 & [i j] <= 2*N+1);

    if sameBottom
        theta0 = -pi/2; theta1 = -pi/2;
    elseif sameTop
        theta0 = pi/2; theta1 = pi/2;
    else
        theta0 = 0; theta1 = 0;
    end

    q0 = [x(i), y(i), theta0];
    q1 = [x(j), y(j), theta1];
    poses = reedsSheppPathSample(q0, q1, Rmin, 0.1);
    waypointsFT = [waypointsFT; poses(2:end, 1:2)];
end

figure;
plot(waypointsFT(:,1), waypointsFT(:,2), 'b.-');
axis equal; grid on;
title('Reeds-Shepp Fishtail Waypoints');
saveas(gcf, 'results/waypoints_fishtail.png');

%% -------------------- Step D: Follow Fishtail Path with Pure Pursuit --------------------

global dt DT
dt = 0.001;   % inner integration time step [s]
DT = 0.01;     % control period [s]
T_total = 100;    % total simulation time [s], adjust as needed
time_vec = 0:DT:T_total;

clear q_history cte_history

% Initial state
q = [-3*W; RL/2; 0; 0; v_ref];
q_history = q.';
cte_history = [];

Qmax = [inf; inf; inf; gamma_limit; v_ref];
Qmin = [-inf; -inf; -inf; -gamma_limit; 0];


for k = 1:length(time_vec)
    [steerAngle, cte] = purePursuitController(q, L, Ld, waypointsFT);
    steerAngle = max(min(steerAngle, gamma_limit), -gamma_limit);  % saturate here
    control = [steerAngle; v_ref];
    q = robot_bike_dyn(q, control, [gamma_limit;0], [gamma_limit;v_ref], Qmin, Qmax, L, 0, 0);

    q_history = [q_history; q.'];
    cte_history = [cte_history; cte];

    if norm(q(1:2) - waypointsFT(end,:)') < 0.5
        break;
    end
end

% Plot path and tracking
figure;
plot(waypointsFT(:,1), waypointsFT(:,2), 'r--'); hold on;
plot(q_history(:,1), q_history(:,2), 'b-');
axis equal; grid on;
title('Fishtail Path Tracking');
saveas(gcf, 'results/robot_path_fishtail.png');

% Plot CTE
figure;
plot(0:DT:(length(cte_history)-1)*DT, cte_history, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Cross-Track Error [m]');
title('Cross-Track Error - Fishtail');
grid on;
saveas(gcf, 'results/cte_fishtail.png');

% Save RMS
rms_cte_ft = sqrt(mean(cte_history.^2));
disp(['RMS CTE (Fishtail): ', num2str(rms_cte_ft), ' m']);
fid = fopen('results/RMS_fishtail.txt','w');
fprintf(fid, 'RMS CTE (m) - Fishtail: %.4f\n', rms_cte_ft);
fclose(fid);


%% -------------------- Step F: Reeds-Shepp Fishtail Turns with ±30° --------------------

% Use updated turning radius and limits
gamma_limit_30 = deg2rad(30);
Rmin_FT = L / tan(gamma_limit_30);
gamma_maxFT = gamma_limit_30;
gamma_minFT = -gamma_limit_30;

% Rebuild cost matrix with Reeds-Shepp sampled paths
DMAT_FT = HUGE * ones(2*N+2);

% Free row traversal
for i = 2:N+1
    DMAT_FT(i, i+N) = 0;
    DMAT_FT(i+N, i) = 0;
end

% Headland transitions with fishtail Reeds-Shepp paths
for i = 2:N
    for j = i+1:N+1
        % bottom rows, reverse entry
        q0 = [x(i), y(i), -pi/2];
        q1 = [x(j), y(j), -pi/2];
        path = reedsSheppPathSample(q0, q1, Rmin, 0.25);
        DMAT_FT(i,j) = sum(vecnorm(diff(path(:,1:2)),2,2));
        DMAT_FT(j,i) = DMAT_FT(i,j);

        % top rows, reverse entry
        q0 = [x(i+N), y(i+N), pi/2];
        q1 = [x(j+N), y(j+N), pi/2];
        path = reedsSheppPathSample(q0, q1, Rmin, 0.25);
        DMAT_FT(i+N,j+N) = sum(vecnorm(diff(path(:,1:2)),2,2));
        DMAT_FT(j+N,i+N) = DMAT_FT(i+N,j+N);
    end
end

% Start/end connections
for i = 2:2*N+1
    DMAT_FT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
    DMAT_FT(i,1) = DMAT_FT(1,i);
    DMAT_FT(end,i) = abs(x(end)-x(i)) + abs(y(end)-y(i));
    DMAT_FT(i,end) = DMAT_FT(end,i);
end
DMAT_FT(1,end) = HUGE;
DMAT_FT(end,1) = HUGE;

% Solve optimal route using GA
resultFT = tspof_ga('xy', XY, 'DMAT', DMAT_FT, 'SHOWRESULT', true);
routeFT = [1, resultFT.optRoute, 2*N+2];

% STEP C: Generate Fishtail Path Waypoints with ±30° Steering
waypointsFT = [];

for k = 1:numel(routeFT)-1
    i = routeFT(k); j = routeFT(k+1);
    p1 = [x(i), y(i)];
    p2 = [x(j), y(j)];

    % Row traversal: straight interpolation
    if ((i>=2 && i<=N+1 && j-i==N) || (j>=2 && j<=N+1 && i-j==N))
        n = ceil(norm(p2-p1)/0.5);
        t = linspace(0,1,n).';
        waypointsFT = [waypointsFT; (p1 + (p2-p1).*t(2:end))];
        continue
    end

    % Headland turns: reverse entry orientation
    sameBottom = all([i j] >= 2 & [i j] <= N+1);
    sameTop    = all([i j] >= N+2 & [i j] <= 2*N+1);

    if sameBottom
        theta0 = -pi/2; theta1 = -pi/2;
    elseif sameTop
        theta0 = pi/2; theta1 = pi/2;
    else
        theta0 = 0; theta1 = 0;
    end

    q0 = [x(i), y(i), theta0];
    q1 = [x(j), y(j), theta1];
    path = reedsSheppPathSample(q0, q1, Rmin, 0.25);
    waypointsFT = [waypointsFT; path(2:end,1:2)];
end

figure;
plot(waypointsFT(:,1), waypointsFT(:,2), 'b.-');
axis equal; grid on;
title('Fishtail Path Waypoints (±30° Steering)');
saveas(gcf, 'results/waypoints_fishtail_30deg.png');

% STEP D: Path following using Pure Pursuit with fishtail path
clear q_history cte_history

q = [-3*W; RL/2; 0; 0; v_ref];
q_history = q.';
cte_history = [];

for k = 1:length(time_vec)
    [steerAngle, cte] = purePursuitController(q, L, Ld, waypointsFT);
    steerAngle = max(min(steerAngle, gamma_limit), -gamma_limit);
    control = [steerAngle; v_ref];
    q = robot_bike_dyn(q, control, [gamma_limit;0], [gamma_limit;v_ref], Qmin, Qmax, L, 0, 0);
    q_history = [q_history; q.'];
    cte_history = [cte_history; cte];
    if norm(q(1:2) - waypointsFT(end,:)') < 0.5
        break;
    end
end

% Plot path
figure;
plot(waypointsFT(:,1), waypointsFT(:,2), 'r--'); hold on;
plot(q_history(:,1), q_history(:,2), 'b-');
axis equal; grid on;
title('Fishtail Path Tracking (±30°)');
saveas(gcf, 'results/robot_path_fishtail_30deg.png');

% Plot CTE
figure;
plot(0:DT:(length(cte_history)-1)*DT, cte_history, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Cross-Track Error [m]');
title('CTE - Fishtail (±30° Steering)');
grid on;
saveas(gcf, 'results/cte_fishtail_30deg.png');

% Save RMS
rms_cte_ft = sqrt(mean(cte_history.^2));
disp(['RMS CTE (Fishtail, ±30°): ', num2str(rms_cte_ft), ' m']);
fid = fopen('results/RMS_fishtail_30deg.txt','w');
fprintf(fid, 'RMS CTE (m) - Fishtail ±30°: %.4f\n', rms_cte_ft);
fclose(fid);



