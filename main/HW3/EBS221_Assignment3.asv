%%

clear; close all; clc;

%% -------------------- Step A: Define Problem Parameters --------------------
N = 10;                     % number of rows
RL = 20;                    % row length
W = 2.5;                    % implement width (row spacing)
L = 3;                      % tractor length
Ld = 1.5;                   % look-ahead distance
gamma_limit = deg2rad(60);  % steering limit
Rmin = L / tan(gamma_limit);  % minimum turning radius

% Start and end positions
x_start = -3 * W;
y_start = RL / 2;
x_end = x_start;
y_end = y_start;

% Coordinates of headland nodes (lower and upper)
x_lower = W/2 + W * (0:N-1); % Lower headland nodes X coordinates
y_lower = zeros(1, N);       % Lower headland nodes Y coordinates
x_upper = x_lower;           % Upper headland nodes X coordinates  
y_upper = RL * ones(1, N);   % Upper headland nodes Y coordinates

% Assemble full node coordinate list (1=start, 2..N+1=lower, N+2..2N+1=upper, 2N+2=end)
x = [x_start, x_lower, x_upper, x_end];
y = [y_start, y_lower, y_upper, y_end];

% Plot for verification
figure; hold on; axis equal
plot(x, y, 'ko')
dx = 0.50 * ones(size(x));          % default shift in x
dy = 0.50 * ones(size(y));          % default shift in y
% put the first label above-left, last label below-right
dx(1)   = 0.75;   dy(1)   =  1.0;   % start node
dx(end) =  0.75;   dy(end) = -1.0;   % end   node
text(x + dx, ...
     y + dy, ...
     string(1:numel(x)), "HorizontalAlignment","center");
title('Node Layout Verification'); xlabel('X [m]'); ylabel('Y [m]');
xlim([-8 27])

%% -------------------- Step B: Build Cost Matrix --------------------
HUGE = 1e6;                       % "impossible" cost
DMAT = HUGE * ones(2*N+2);        % start with all moves disabled

% ---- (1) Row-traversal costs: lower ↔ upper in SAME row ---------------
% Moving from any lower headland node to any upper headland node (and vice versa)
% is only possible if both nodes belong to the same field row
for i = 2:N+1 % for each lower node
    for j = N+2:2*N+1 % for each upper node
        if (j-i) == N % if nodes belong to same row, non working cost is zero
            DMAT(i,j) = 0; % Cost to travel within a row is zero (assuming cultivation cost is zero)
            DMAT(j,i) = 0; % Same for reverse direction
        else
            DMAT(i,j) = HUGE; % Not possible to go from lower to upper if not in same row
            DMAT(j,i) = HUGE; % Same for reverse direction
        end
    end
end

% ---- (2) Headland turning costs: Π / Ω turns on south or north edges --
% Compute turning costs between nodes i, j (both are at top or bottom)
for i = 2:N+1 % for each node at the 'bottom'
    for j = i+1:N+1 % for each node to the 'right' of node i
        d = abs(j-i); % Number of rows between the nodes
        dW = d * W;   % Physical distance between the rows
        
        % Check if a PI turn can be made (plus a straight line of length (d-1) row widths)
        if (Rmin <= dW/2)
            % PI turn cost = π * Rmin (for the semi-circle) + (d-1)*W (for the straight part)
            DMAT(i,j) = pi * Rmin + (d-1) * W;
        else
            % Omega turn cost = 2 * π * Rmin (full circle of radius Rmin)
            DMAT(i,j) = 2 * pi * Rmin;
        end
        
        DMAT(j,i) = DMAT(i,j);           % symmetry of cost
        DMAT(i+N, j+N) = DMAT(i,j);      % same cost for pair of top-nodes
        DMAT(j+N, i+N) = DMAT(i,j);      % symmetry for top nodes
    end
end

% ---- (3) Manhattan costs start/end ↔ all field nodes ------------------
% Costs to travel from start node to any other field node (except for end node)
% Costs to travel from end node to any other field node (except for start node)
for i = 2:2*N+1 % all field row nodes
    % Manhattan distance between this node and start node
    DMAT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
    DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
    
    % Manhattan distance between this node and end node
    DMAT(2*N+2,i) = abs(x(2*N+2)-x(i)) + abs(y(2*N+2)-y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i); % cost matrix symmetry
end

% ---- (4) Forbid direct Start ↔ End jump --------------------------------
% Cost between start and end nodes is set to HUGE (not allowed)
DMAT(1,2*N+2) = HUGE;
DMAT(2*N+2,1) = HUGE;


%% -------------------- Solve TSP (Step B Completion) --------------------
XY = [x', y'];
t = cputime;
resultStruct = tspof_ga('xy', XY, 'DMAT', DMAT, ...
                        'POPSIZE', 200, ...   % Doubled population size
                        'NUMITER', 2e4, ...   % Doubled iterations
                        'SHOWRESULT', true, ...
                        'SHOWWAITBAR', true, ...
                        'SHOWPROG', true);
E = cputime - t; % computation time
route = [1, resultStruct.optRoute, 2*N+2]; % full node sequence

% Verify that all rows are visited
rows_visited = zeros(1, N);
for i = 1:length(route)
    node_idx = route(i);
    if node_idx >= 2 && node_idx <= N+1  % Lower nodes
        row_idx = node_idx - 1;
        rows_visited(row_idx) = 1;
    elseif node_idx >= N+2 && node_idx <= 2*N+1  % Upper nodes
        row_idx = node_idx - N - 1;
        rows_visited(row_idx) = 1;
    end
end

% Check if all rows were visited
missing_rows = find(rows_visited == 0);
if ~isempty(missing_rows)
    disp('Warning: The following rows were not visited:');
    disp(missing_rows);
end

% Output result
disp('Computed Optimal Route (Node Indices):');
disp(route);
disp(['Computed Minimum Distance: ', num2str(resultStruct.minDist)]);
disp(['Computation Time: ', num2str(E), ' seconds']);

% Save output
% Open a new file (or overwrite if it exists)
[~, ~] = mkdir("results\B");
fid = fopen('results\B\results.txt','w');

% Write each piece
fprintf(fid, 'Computed Optimal Route (Node Indices):\n');
fprintf(fid, '%d ', route);          % print each index with a space
fprintf(fid, '\n\n');                % blank line

fprintf(fid, 'Computed Minimum Distance: %.4f\n', resultStruct.minDist);
fprintf(fid, 'Computation Time: %.4f seconds\n', E);

% Close the file
fclose(fid);

%% Plot Solution
figure; plot(x(route), y(route), 'r-o', 'LineWidth', 2);
hold on; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2));
title('Optimal Node Sequence Visualization');
xlabel('X [m]'); ylabel('Y [m]'); axis equal;
saveas(gcf, "results\B\optimal_node.png")

%% -------------------- Step C: Build waypoints + segment IDs + guidedPoints --------------------
numLinks        = numel(route)-1;
waypoints       = [];
waypointSegment = [];
isTurnLink      = false(numLinks,1);

% first, mark which links are turns
for k = 1:numLinks
    i = route(k); j = route(k+1);
    if k==1 || k==numLinks
        isTurnLink(k) = true;   % entry or exit
    else
        paired   = ( (i>=2 && i<=N+1 && j-i==N) || (j>=2 && j<=N+1 && i-j==N) );
        sameBot  = all([i j]>=2   & [i j]<=N+1);
        sameTop  = all([i j]>=N+2 & [i j]<=2*N+1);
        if ~paired && (sameBot||sameTop)
            isTurnLink(k) = true;
        end
    end
end

% now build waypoints & segments
for k = 1:numLinks
    i = route(k); j = route(k+1);
    p1 = [x(i), y(i)];  p2 = [x(j), y(j)];
    if isTurnLink(k)
        % compute Dubins turn
        if k==1
            theta0 = 0;
            theta1 = (j<=N+1)*pi/2 + (j>N+1)*(-pi/2);
        elseif k==numLinks
            theta0 = (i<=N+1)*-pi/2 + (i>N+1)*(pi/2);
            theta1 = pi;
        else
            sameBot = all([i j]>=2   & [i j]<=N+1);
            theta0  = sameBot*(-pi/2) + (~sameBot)*( pi/2);
            theta1  = -theta0;
        end
        P      = dubins_core([p1,theta0],[p2,theta1], Rmin);
        pts    = dubins_path_sample_many(P,0.25);
        newPts = pts(2:end,1:2);
    else
        % straight
        nPts   = ceil(norm(p2-p1)/0.5);
        xs     = linspace(p1(1),p2(1),nPts).';
        ys     = linspace(p1(2),p2(2),nPts).';
        newPts = [xs(2:end), ys(2:end)];
    end
    waypoints       = [waypoints;       newPts];
    waypointSegment = [waypointSegment; k*ones(size(newPts,1),1)];
end

% now define guidedPoints: one per link
guidedPoints = zeros(numLinks,2);
for k = 1:numLinks
    idxs    = find(waypointSegment==k);
    segPts  = waypoints(idxs,:);
    if isTurnLink(k)
        % bottom or top by mean y
        if mean(segPts(:,2)) < RL/2
            [~,m] = min(segPts(:,2));   % bottom
        else
            [~,m] = max(segPts(:,2));   % top
        end
        guidedPoints(k,:) = segPts(m,:);
    else
        % midpoint for straight
        guidedPoints(k,:) = 0.5*(segPts(1,:) + segPts(end,:));
    end
end
% Add the end point into guided points
guidedPoints = [guidedPoints; waypoints(end,:)];

[~, ~] = mkdir("results\C");
% Plot the generated waypoints
figure; plot(waypoints(:,1), waypoints(:,2), 'b.-'); axis equal; hold on;
plot(guidedPoints(:,1), guidedPoints(:,2), 'o')
title('Generated Waypoints and Guided Points from Node Sequence');
xlabel('X [m]'); ylabel('Y [m]');
legend("Waypoints", "Guided Points", Location="best")
saveas(gcf, "results\C\waypoints.png")

%% -------------------- Step D: Simulate Path Following with Pure Pursuit Controller --------------------

clear q_history cte_history

% Reset Integration Parameters
global dt DT
dt = 0.001;    % 1 ms integration step
DT = 0.01;     % 10 ms control update period
T = 1000.0;      % run until it reaches the end point
time_vec = 0:DT:T-DT;

% Vehicle and Controller Parameters
L = 2.5;               % wheelbase
Ld_line = 1.0;         % look-ahead distance
Ld_turn = 0.5;
gamma_max = deg2rad(60);
gamma_min = -gamma_max;
v_ref = 1.0;           % constant forward speed
tau_gamma = 0.0;       % instant steering dynamics
tau_v = 0.0;           % instant speed dynamics

% State bounds
Qmax = [inf; inf; inf; gamma_max; v_ref];
Qmin = [-inf; -inf; -inf; gamma_min; 0];

% Initial state [x, y, theta, gamma, v]
q = [-3*W; RL/2; 0; 0; v_ref];

numSteps    = length(time_vec);
q_history   = zeros(numSteps+1,5);    % assuming state q has length 5
cte_history = zeros(numSteps,3);
status_history = zeros(numSteps,1);

% store initial state
q_history(1,:) = q.';

for k = 1:numSteps
    % 1) compute steering and cte
    [delta, ~, status] = purePursuitSegmented( ...
        q, L, Ld_line, Ld_turn, ...
        waypoints, waypointSegment, isTurnLink, guidedPoints, ...
        gamma_max, gamma_min);

    % choose a base blend factor (tune between 0 and 1)
    % cte = α·cte_path + (1−α)·cte_gp
    alpha = 0.9;
    [cte_path, cte_gp, cte] = computeBlendedError( ...
    q, waypoints, waypointSegment, guidedPoints, isTurnLink, alpha, status);

    % 2) step the dynamics
    q = robot_bike_dyn( ...
          q, [delta; v_ref], ...
          [gamma_min;0], [gamma_max;v_ref], ...
          Qmin, Qmax, L, 0, 0 );

    % 3) record state
    q_history(k+1,:) = q.';
    status_history(k) = status;

    % 4) compute and store CTE
    cte_history(k, 1) = cte_path;
    cte_history(k, 2) = cte_gp;
    cte_history(k, 3) = cte;

    % 5) check termination (avoid stuck at start)
    if norm(q(1:2) - waypoints(end,:).') < 0.5 && k >= 1000
        % trim unused entries
        q_history   = q_history(1:k+1,:);
        cte_history = cte_history(1:k, :);
        status_history = status_history(1:k, :);
        break;
    end
end


%% Visualization for Step D

% results
rms_cte = sqrt(mean(cte_history(:,1).^2));
disp(['Segment-bounded RMS CTE: ', num2str(rms_cte), ' m']);

[~,~] = mkdir("results\D");
% save figures
figure;
plot(waypoints(:,1), waypoints(:,2), 'r--', LineWidth=2); hold on;
plot(q_history(:,1), q_history(:,2), 'b-');
axis equal; grid on;
title('Adaptive Ld Path Following');
xlabel('X [m]'); ylabel('Y [m]');
legend("Waypoints", "Robot Path", Location="best")
saveas(gcf, 'results\D\adaptive_robot_path.png');

figure;
t = (0:DT:(numel(cte_history(:,1))-1)*DT).';
plot(t, cte_history(:, 1));
grid on;
title('Segment Bounded Adapative Ld Cross-Track Error');
xlabel('Time [s]'); ylabel('CTE [m]');
saveas(gcf, 'results\D\adaptive_cte.png');

% trim q_history to same length as status_history
q_pos = q_history(1:length(status_history), 1:2);

% time vector for status
t_status = (0:(length(status_history)-1)) * DT;

% 1) plot status over time
figure;
stairs(t_status, status_history, 'LineWidth', 1.5);
ylim([-0.1, 1.1]);
yticks([0 1]);
yticklabels({'off','on'});
xlabel('Time [s]');
ylabel('Guided Override');
title('Guided-Point Override Activation Over Time');
grid on;
saveas(gcf, 'results\D\status_time.png');

% 2) plot trajectory colored by status
idxOn  = status_history == 1;
idxOff = status_history == 0;

figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'LineWidth', 1);  % reference path
scatter(q_pos(idxOff,1), q_pos(idxOff,2), 8,'r', 'filled');
scatter(q_pos(idxOn, 1), q_pos(idxOn, 2), 8,'b', 'filled');
legend('Waypoints','Override Off','Override On','Location','Best');
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Robot Path Colored by Guided-Point Override Status');
saveas(gcf, 'results\D\status_path.png');

% save RMS
fid = fopen('results\D\RMS_adaptive.txt','w');
fprintf(fid, 'RMS CTE (adaptive Ld): %.4f\n', rms_cte);
fclose(fid);

%% -------------------- Step E: Repeat Step D with 30 degree steering limit --------------------

clear q_history cte_history

% Reset Integration Parameters
global dt DT
dt = 0.001;    % 1 ms integration step
DT = 0.01;     % 10 ms control update period
T = 1000.0;    % run until it reaches the end point
time_vec = 0:DT:T-DT;

% Vehicle and Controller Parameters
L = 2.5;               % wheelbase
Ld_line = 1.0;         % look-ahead distance
Ld_turn = 0.2;
gamma_max = deg2rad(30);  % CHANGED: maximum steering angle to 30 degrees
gamma_min = -gamma_max;
v_ref = 1.0;           % constant forward speed
tau_gamma = 0.0;       % instant steering dynamics
tau_v = 0.0;           % instant speed dynamics

% State bounds
Qmax = [inf; inf; inf; gamma_max; v_ref];
Qmin = [-inf; -inf; -inf; gamma_min; 0];

% Initial state [x, y, theta, gamma, v]
q = [-3*W; RL/2; 0; 0; v_ref];

numSteps    = length(time_vec);
q_history   = zeros(numSteps+1,5);    % assuming state q has length 5
cte_history = zeros(numSteps,3);
status_history = zeros(numSteps,1);

% store initial state
q_history(1,:) = q.';

for k = 1:numSteps
    % 1) compute steering and cte
    [delta, ~, status] = purePursuitSegmented( ...
        q, L, Ld_line, Ld_turn, ...
        waypoints, waypointSegment, isTurnLink, guidedPoints, ...
        gamma_max, gamma_min);

    % choose a base blend factor (tune between 0 and 1)
    % cte = α·cte_path + (1−α)·cte_gp
    alpha = 0.9;
    [cte_path, cte_gp, cte] = computeBlendedError( ...
    q, waypoints, waypointSegment, guidedPoints, isTurnLink, alpha, status);

    % 2) step the dynamics
    q = robot_bike_dyn( ...
          q, [delta; v_ref], ...
          [gamma_min;0], [gamma_max;v_ref], ...
          Qmin, Qmax, L, 0, 0 );

    % 3) record state
    q_history(k+1,:) = q.';
    status_history(k) = status;

    % 4) compute and store CTE
    cte_history(k, 1) = cte_path;
    cte_history(k, 2) = cte_gp;
    cte_history(k, 3) = cte;

    % 5) check termination (avoid stuck at start)
    if norm(q(1:2) - waypoints(end,:).') < 0.5 && k >= 1000
        % trim unused entries
        q_history   = q_history(1:k+1,:);
        cte_history = cte_history(1:k, :);
        status_history = status_history(1:k, :);
        break;
    end
end

%% Visualization for Step E

% results
rms_cte = sqrt(mean(cte_history(:,1).^2));
disp(['Segment-bounded RMS CTE (30-deg limit): ', num2str(rms_cte), ' m']);

status = mkdir("results\E");
% save figures
figure;
plot(waypoints(:,1), waypoints(:,2), 'r--'); hold on;
plot(q_history(:,1), q_history(:,2), 'b-');
axis equal; grid on;
title('Adaptive Ld Path Following (30° Steering Limit)');
xlabel('X [m]'); ylabel('Y [m]');
saveas(gcf, 'results\E\adaptive_robot_path_30deg.png');

figure;
t = (0:DT:(numel(cte_history(:,1))-1)*DT).';
plot(t, cte_history(:, 1));
grid on;
title('Segment Bounded Adaptive Ld Cross-Track Error (30° Steering Limit)');
xlabel('Time [s]'); ylabel('CTE [m]');
saveas(gcf, 'results\E\adaptive_cte_30deg.png');

% trim q_history to same length as status_history
q_pos = q_history(1:length(status_history), 1:2);

% time vector for status
t_status = (0:(length(status_history)-1)) * DT;

% 1) plot status over time
figure;
stairs(t_status, status_history, 'LineWidth', 1.5);
ylim([-0.1, 1.1]);
yticks([0 1]);
yticklabels({'off','on'});
xlabel('Time [s]');
ylabel('Guided Override');
title('Guided-Point Override Activation Over Time (30° Steering Limit)');
grid on;
saveas(gcf, 'results\E\status_time_30deg.png');

% 2) plot trajectory colored by status
idxOn  = status_history == 1;
idxOff = status_history == 0;

figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'LineWidth', 1);  % reference path
scatter(q_pos(idxOff,1), q_pos(idxOff,2), 8,'r', 'filled');
scatter(q_pos(idxOn, 1), q_pos(idxOn, 2), 8,'b', 'filled');
legend('Waypoints','Override Off','Override On','Location','Best');
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Robot Path Colored by Guided-Point Override Status (30° Steering Limit)');
saveas(gcf, 'results\E\status_path_30deg.png');

% save RMS
fid = fopen('results\E\RMS_adaptive_30deg.txt','w');
fprintf(fid, 'RMS CTE (adaptive Ld, 30° steering): %.4f\n', rms_cte);
fclose(fid);

% Compare steering angles between 60° and 30° simulations
figure;
plot(q_history(:,4) * 180/pi, 'LineWidth', 1.5);
grid on;
title('Steering Angle vs. Time (30° Steering Limit)');
xlabel('Time Step'); ylabel('Steering Angle [deg]');
yline(30, 'r--', 'Max Angle'); yline(-30, 'r--');
saveas(gcf, 'results\E\steering_angles_30deg.png');

%% -------------------- Step F: Redo B, C, D with 30 degree steering limit --------------------

%% F.1 - Redefine Problem Parameters with 30 degree steering limit
clear; close all; clc;

%% -------------------- Step A: Define Problem Parameters (Reused) --------------------
N = 10;                     % number of rows
RL = 20;                    % row length
W = 2.5;                    % implement width (row spacing)
L = 3;                      % tractor length
Ld = 1.5;                   % look-ahead distance
gamma_limit = deg2rad(30);  % CHANGED: steering limit to 30 degrees
Rmin = L / tan(gamma_limit);  % UPDATED: minimum turning radius (larger now)

% Start and end positions
x_start = -3 * W;
y_start = RL / 2;
x_end = x_start;
y_end = y_start;

% Coordinates of headland nodes (lower and upper)
x_lower = W/2 + W * (0:N-1); % added 2.5 m to correct X coordinates
y_lower = zeros(1, N);

x_upper = x_lower;
y_upper = RL * ones(1, N);

% Assemble full node coordinate list (1=start, 2..N+1=lower, N+2..2N+1=upper, 2N+2=end)
x = [x_start, x_lower, x_upper, x_end];
y = [y_start, y_lower, y_upper, y_end];

% Plot for verification
figure; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2)); axis equal;
title('Node Layout Verification (30° Steering Limit)'); xlabel('X [m]'); ylabel('Y [m]');
[~, ~] = mkdir('results\F');
saveas(gcf, "results\F\node_layout_30deg.png")

%% F.2 - Rebuild Cost Matrix with new minimum turning radius
HUGE = 1e6;                       % "impossible" cost
DMAT = HUGE * ones(2*N+2);        % start with all moves disabled

% ---- (1) Row-traversal costs: lower ↔ upper in SAME row ---------------
% Moving from any lower headland node to any upper headland node (and vice versa)
% is only possible if both nodes belong to the same field row
for i = 2:N+1 % for each lower node
    for j = N+2:2*N+1 % for each upper node
        if (j-i) == N % if nodes belong to same row, non working cost is zero
            DMAT(i,j) = 0; % Cost to travel within a row is zero (assuming cultivation cost is zero)
            DMAT(j,i) = 0; % Same for reverse direction
        else
            DMAT(i,j) = HUGE; % Not possible to go from lower to upper if not in same row
            DMAT(j,i) = HUGE; % Same for reverse direction
        end
    end
end

% ---- (2) Headland turning costs: Π / Ω turns on south or north edges --
% Compute turning costs between nodes i, j (both are at top or bottom)
for i = 2:N+1 % for each node at the 'bottom'
    for j = i+1:N+1 % for each node to the 'right' of node i
        d = abs(j-i); % Number of rows between the nodes
        dW = d * W;   % Physical distance between the rows
        
        % Check if a PI turn can be made (plus a straight line of length (d-1) row widths)
        if (Rmin <= dW/2)
            % PI turn cost = π * Rmin (for the semi-circle) + (d-1)*W (for the straight part)
            DMAT(i,j) = pi * Rmin + (d-1) * W;
        else
            % Omega turn cost = 2 * π * Rmin (full circle of radius Rmin)
            DMAT(i,j) = 2 * pi * Rmin;
        end
        
        DMAT(j,i) = DMAT(i,j);           % symmetry of cost
        DMAT(i+N, j+N) = DMAT(i,j);      % same cost for pair of top-nodes
        DMAT(j+N, i+N) = DMAT(i,j);      % symmetry for top nodes
    end
end

% ---- (3) Manhattan costs start/end ↔ all field nodes ------------------
% Costs to travel from start node to any other field node (except for end node)
% Costs to travel from end node to any other field node (except for start node)
for i = 2:2*N+1 % all field row nodes
    % Manhattan distance between this node and start node
    DMAT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
    DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
    
    % Manhattan distance between this node and end node
    DMAT(2*N+2,i) = abs(x(2*N+2)-x(i)) + abs(y(2*N+2)-y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i); % cost matrix symmetry
end

% ---- (4) Forbid direct Start ↔ End jump --------------------------------
% Cost between start and end nodes is set to HUGE (not allowed)
DMAT(1,2*N+2) = HUGE;
DMAT(2*N+2,1) = HUGE;

%% -------------------- Solve TSP (Step F.2 Completion) --------------------
XY = [x', y'];
t = cputime;
resultStruct = tspof_ga('xy', XY, 'DMAT', DMAT, ...
                        'POPSIZE', 200, ...   % Doubled population size
                        'NUMITER', 2e4, ...   % Doubled iterations
                        'SHOWRESULT', true, ...
                        'SHOWWAITBAR', true, ...
                        'SHOWPROG', true);
E = cputime - t; % computation time
route = [1, resultStruct.optRoute, 2*N+2]; % full node sequence

% Verify that all rows are visited
rows_visited = zeros(1, N);
for i = 1:length(route)
    node_idx = route(i);
    if node_idx >= 2 && node_idx <= N+1  % Lower nodes
        row_idx = node_idx - 1;
        rows_visited(row_idx) = 1;
    elseif node_idx >= N+2 && node_idx <= 2*N+1  % Upper nodes
        row_idx = node_idx - N - 1;
        rows_visited(row_idx) = 1;
    end
end

% Check if all rows were visited
missing_rows = find(rows_visited == 0);
if ~isempty(missing_rows)
    disp('Warning: The following rows were not visited:');
    disp(missing_rows);
end

% Output result
disp('Computed Optimal Route with 30° Steering Limit (Node Indices):');
disp(route);
disp(['Computed Minimum Distance (30° limit): ', num2str(resultStruct.minDist)]);
disp(['Computation Time: ', num2str(E), ' seconds']);

% Save output
fid = fopen('results\F\results_30deg.txt','w');
fprintf(fid, 'Computed Optimal Route with 30° Steering Limit (Node Indices):\n');
fprintf(fid, '%d ', route);          % print each index with a space
fprintf(fid, '\n\n');                % blank line
fprintf(fid, 'Computed Minimum Distance (30° limit): %.4f\n', resultStruct.minDist);
fprintf(fid, 'Computation Time: %.4f seconds\n', E);
fclose(fid);

%% Plot Solution
figure; plot(x(route), y(route), 'r-o', 'LineWidth', 2);
hold on; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2));
title('Optimal Node Sequence Visualization (30° Steering Limit)');
xlabel('X [m]'); ylabel('Y [m]'); axis equal;
saveas(gcf, "results\F\optimal_node_30deg.png")

%% F.3 - Rebuild waypoints with 30 degree turning radius
numLinks        = numel(route)-1;
waypoints       = [];
waypointSegment = [];
isTurnLink      = false(numLinks,1);

% first, mark which links are turns
for k = 1:numLinks
    i = route(k); j = route(k+1);
    if k==1 || k==numLinks
        isTurnLink(k) = true;   % entry or exit
    else
        paired   = ( (i>=2 && i<=N+1 && j-i==N) || (j>=2 && j<=N+1 && i-j==N) );
        sameBot  = all([i j]>=2   & [i j]<=N+1);
        sameTop  = all([i j]>=N+2 & [i j]<=2*N+1);
        if ~paired && (sameBot||sameTop)
            isTurnLink(k) = true;
        end
    end
end

% now build waypoints & segments
for k = 1:numLinks
    i = route(k); j = route(k+1);
    p1 = [x(i), y(i)];  p2 = [x(j), y(j)];
    if isTurnLink(k)
        % compute Dubins turn
        if k==1
            theta0 = 0;
            theta1 = (j<=N+1)*pi/2 + (j>N+1)*(-pi/2);
        elseif k==numLinks
            theta0 = (i<=N+1)*-pi/2 + (i>N+1)*(pi/2);
            theta1 = pi;
        else
            sameBot = all([i j]>=2   & [i j]<=N+1);
            theta0  = sameBot*(-pi/2) + (~sameBot)*( pi/2);
            theta1  = -theta0;
        end
        P      = dubins_core([p1,theta0],[p2,theta1], Rmin);
        pts    = dubins_path_sample_many(P,0.25);
        newPts = pts(2:end,1:2);
    else
        % straight
        nPts   = ceil(norm(p2-p1)/0.5);
        xs     = linspace(p1(1),p2(1),nPts).';
        ys     = linspace(p1(2),p2(2),nPts).';
        newPts = [xs(2:end), ys(2:end)];
    end
    waypoints       = [waypoints;       newPts];
    waypointSegment = [waypointSegment; k*ones(size(newPts,1),1)];
end

% now define guidedPoints: one per link
guidedPoints = zeros(numLinks,2);
for k = 1:numLinks
    idxs    = find(waypointSegment==k);
    segPts  = waypoints(idxs,:);
    if k == numLinks
        % For the last link, use the end point as guided point
        guidedPoints(k,:) = [x_end, y_end];
    elseif isTurnLink(k)
        % bottom or top by mean y
        if mean(segPts(:,2)) < RL/2
            [~,m] = min(segPts(:,2));   % bottom
        else
            [~,m] = max(segPts(:,2));   % top
        end
        guidedPoints(k,:) = segPts(m,:);
    else
        % midpoint for straight
        guidedPoints(k,:) = 0.5*(segPts(1,:) + segPts(end,:));
    end
end

% Plot the generated waypoints
figure; plot(waypoints(:,1), waypoints(:,2), 'b.-'); axis equal; hold on;
plot(guidedPoints(:,1), guidedPoints(:,2), 'o')
title('Generated Waypoints and Guided Points (30° Steering Limit)');
xlabel('X [m]'); ylabel('Y [m]');
legend("Waypoints", "Guided Points", Location="best")
saveas(gcf, "results\F\waypoints_30deg.png")

%% F.4 - Re-simulate path following with 30 degree steering limit
clear q_history cte_history

% Reset Integration Parameters
global dt DT
dt = 0.001;    % 1 ms integration step
DT = 0.01;     % 10 ms control update period
T = 2000.0;    % run until it reaches the end point
time_vec = 0:DT:T-DT;

% Vehicle and Controller Parameters
L = 2.5;               % wheelbase
Ld_line = 2.0;         % look-ahead distance
Ld_turn = 1.0;
gamma_max = deg2rad(45);  % 30 degree steering limit
gamma_min = -gamma_max;
v_ref = 1.0;           % constant forward speed
tau_gamma = 0.0;       % instant steering dynamics
tau_v = 0.0;           % instant speed dynamics

% State bounds
Qmax = [inf; inf; inf; gamma_max; v_ref];
Qmin = [-inf; -inf; -inf; gamma_min; 0];

% Initial state [x, y, theta, gamma, v]
q = [-3*W; RL/2; 0; 0; v_ref];

numSteps    = length(time_vec);
q_history   = zeros(numSteps+1,5);    % assuming state q has length 5
cte_history = zeros(numSteps,3);
status_history = zeros(numSteps,1);

% store initial state
q_history(1,:) = q.';

for k = 1:numSteps
    % 1) compute steering and cte
    [delta, ~, status] = purePursuitSegmented( ...
        q, L, Ld_line, Ld_turn, ...
        waypoints, waypointSegment, isTurnLink, guidedPoints, ...
        gamma_max, gamma_min);

    % choose a base blend factor (tune between 0 and 1)
    % cte = α·cte_path + (1−α)·cte_gp
    alpha = 0.9;
    [cte_path, cte_gp, cte] = computeBlendedError( ...
    q, waypoints, waypointSegment, guidedPoints, isTurnLink, alpha, status);

    % 2) step the dynamics
    q = robot_bike_dyn( ...
          q, [delta; v_ref], ...
          [gamma_min;0], [gamma_max;v_ref], ...
          Qmin, Qmax, L, 0, 0 );

    % 3) record state
    q_history(k+1,:) = q.';
    status_history(k) = status;

    % 4) compute and store CTE
    cte_history(k, 1) = cte_path;
    cte_history(k, 2) = cte_gp;
    cte_history(k, 3) = cte;

    % 5) check termination (avoid stuck at start)
    if norm(q(1:2) - waypoints(end,:).') < 0.5 && k >= 1000
        % trim unused entries
        q_history   = q_history(1:k+1,:);
        cte_history = cte_history(1:k, :);
        status_history = status_history(1:k, :);
        break;
    end

    if k == numSteps
        warning("Run out of time!")
    end
end

%% Visualization for Step F.4

% results
rms_cte = sqrt(mean(cte_history(:,1).^2));
disp(['Segment-bounded RMS CTE (30-deg optimized path): ', num2str(rms_cte), ' m']);

% save figures
figure;
plot(waypoints(:,1), waypoints(:,2), 'r--'); hold on;
plot(q_history(:,1), q_history(:,2), 'b-');
axis equal; grid on;
title('Optimized Path Following (30° Steering Limit)');
xlabel('X [m]'); ylabel('Y [m]');
saveas(gcf, 'results\F\optimized_robot_path_30deg.png');

figure;
t = (0:DT:(numel(cte_history(:,1))-1)*DT).';
plot(t, cte_history(:, 1));
grid on;
title('Segment Bounded Cross-Track Error (30° Optimized Path)');
xlabel('Time [s]'); ylabel('CTE [m]');
saveas(gcf, 'results\F\optimized_cte_30deg.png');

% trim q_history to same length as status_history
q_pos = q_history(1:length(status_history), 1:2);

% time vector for status
t_status = (0:(length(status_history)-1)) * DT;

% 1) plot status over time
figure;
stairs(t_status, status_history, 'LineWidth', 1.5);
ylim([-0.1, 1.1]);
yticks([0 1]);
yticklabels({'off','on'});
xlabel('Time [s]');
ylabel('Guided Override');
title('Guided-Point Override Activation (30° Optimized Path)');
grid on;
saveas(gcf, 'results\F\optimized_status_time_30deg.png');

% 2) plot trajectory colored by status
idxOn  = status_history == 1;
idxOff = status_history == 0;

figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'LineWidth', 1);  % reference path
scatter(q_pos(idxOff,1), q_pos(idxOff,2), 8,'r', 'filled');
scatter(q_pos(idxOn, 1), q_pos(idxOn, 2), 8,'b', 'filled');
legend('Waypoints','Override Off','Override On','Location','Best');
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Robot Path by Guided-Point Override Status (30° Optimized Path)');
saveas(gcf, 'results\F\optimized_status_path_30deg.png');

% save RMS
fid = fopen('results\F\RMS_optimized_30deg.txt','w');
fprintf(fid, 'RMS CTE (30° optimized path): %.4f\n', rms_cte);
fclose(fid);

%% 

%-----------------------Helper Functions----------------------------------

function param = dubins_core(p1, p2, r)
    %%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%
    % Here are some usefuldefine headers for better implementation
    % there are 6 types of dubin's curve, only one will have minimum cost
    % LSL = 1;
	% LSR = 2;
	% RSL = 3;
	% RSR = 4;
	% RLR = 5;
    % LRL = 6;
    
    % The three segment types a path can be made up of
    % L_SEG = 1;
    % S_SEG = 2;
    % R_SEG = 3;

    % The segment types for each of the Path types
    %{
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %}
    %%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%
            
    % the return parameter
    param.p_init = p1;              % the initial configuration
    param.seg_param = [0, 0, 0];    % the lengths of the three segments
    param.r = r;                    % model forward velocity / model angular velocity turning radius
    param.type = -1;                % path type. one of LSL, LSR, ... 
    param.flag = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%% START %%%%%%%%%%%%%%%%%%%%%%%%%
    % First, basic properties and normalization of the problem
    dx = p2(1) - p1(1);
    dy = p2(2) - p1(2);
    D = sqrt( dx^2 + dy^2 );
    d = D / r;                  % distance is shrunk by r, this make lengh calculation very easy
    if( r <= 0 )
        param.flag = -1;
        return;
    end
    theta = mod(atan2( dy, dx ), 2*pi);
    alpha = mod((p1(3) - theta), 2*pi);
    beta  = mod((p2(3) - theta), 2*pi);

    % Second, we find all possible curves
    best_word = -1;
    best_cost = -1;
    test_param(1,:) = dubins_LSL(alpha, beta, d);
    test_param(2,:) = dubins_LSR(alpha, beta, d);
    test_param(3,:) = dubins_RSL(alpha, beta, d);
    test_param(4,:) = dubins_RSR(alpha, beta, d);
    test_param(5,:) = dubins_RLR(alpha, beta, d);
    test_param(6,:) = dubins_LRL(alpha, beta, d);
    
    for i = 1:1:6
        if(test_param(i,1) ~= -1) 
            cost = sum(test_param(i,:));
            if(cost < best_cost) || (best_cost == -1)
                best_word = i;
                best_cost = cost;
                param.seg_param = test_param(i,:);
                param.type = i;
            end
        end
    end

    if(best_word == -1) 
        param.flag = -2;             % NO PATH
        return;
    else
        return;
    end
end

function param = dubins_LSL(alpha, beta, d)
    tmp0 = d + sin(alpha) - sin(beta);
    p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(alpha) - sin(beta)));
    if( p_squared < 0 )
        param = [-1, -1, -1];
        return;
    else
        tmp1 = atan2( (cos(beta)-cos(alpha)), tmp0 );
        t = mod((-alpha + tmp1 ), 2*pi);
        p = sqrt( p_squared );
        q = mod((beta - tmp1 ), 2*pi);
        param(1) = t; 
        param(2) = p; 
        param(3) = q;
        return ;
    end
end
function param = dubins_LSR(alpha, beta, d)
    p_squared = -2 + (d*d) + (2*cos(alpha - beta)) + (2*d*(sin(alpha)+sin(beta)));
    if( p_squared < 0 )
        param = [-1, -1, -1];
        return;
    else
        p    = sqrt( p_squared );
        tmp2 = atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) ) - atan2(-2.0, p);
        t    = mod((-alpha + tmp2), 2*pi);
        q    = mod(( -mod((beta), 2*pi) + tmp2 ), 2*pi);
        param(1) = t; 
        param(2) = p; 
        param(3) = q;
        return ;
    end
end
function param = dubins_RSL(alpha, beta, d)
    p_squared = (d*d) -2 + (2*cos(alpha - beta)) - (2*d*(sin(alpha)+sin(beta)));
    if( p_squared< 0 ) 
        param = [-1, -1, -1];
        return;
    else
        p    = sqrt( p_squared );
        tmp2 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta)) ) - atan2(2.0, p);
        t    = mod((alpha - tmp2), 2*pi);
        q    = mod((beta - tmp2), 2*pi);
        param(1) = t;
        param(2) = p; 
        param(3) = q;
        return ;
    end
end
function param = dubins_RSR(alpha, beta, d)
    tmp0 = d-sin(alpha)+sin(beta);
    p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(beta)-sin(alpha)));
    if( p_squared < 0 )
        param = [-1, -1, -1];
        return;
    else
        tmp1 = atan2( (cos(alpha)-cos(beta)), tmp0 );
        t = mod(( alpha - tmp1 ), 2*pi);
        p = sqrt( p_squared );
        q = mod(( -beta + tmp1 ), 2*pi);
        param(1) = t; 
        param(2) = p; 
        param(3) = q;
        return;
    end
end
function param = dubins_RLR(alpha, beta, d)
    tmp_rlr = (6. - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha)-sin(beta))) / 8.;
    if( abs(tmp_rlr) > 1)
        param = [-1, -1, -1];
        return;
    else
        p = mod(( 2*pi - acos( tmp_rlr ) ), 2*pi);
        t = mod((alpha - atan2( cos(alpha)-cos(beta), d-sin(alpha)+sin(beta) ) + mod(p/2, 2*pi)), 2*pi);
        q = mod((alpha - beta - t + mod(p, 2*pi)), 2*pi);
        param(1) = t;
        param(2) = p;
        param(3) = q;
        
        return;
    end
end
function param = dubins_LRL(alpha, beta, d)
    tmp_lrl = (6. - d*d + 2*cos(alpha - beta) + 2*d*(- sin(alpha) + sin(beta))) / 8.;
    if( abs(tmp_lrl) > 1)
        param = [-1, -1, -1]; return;
    else
        p = mod(( 2*pi - acos( tmp_lrl ) ), 2*pi);
        t = mod((-alpha - atan2( cos(alpha)-cos(beta), d+sin(alpha)-sin(beta) ) + p/2), 2*pi);
        q = mod((mod(beta, 2*pi) - alpha -t + mod(p, 2*pi)), 2*pi);
        param(1) = t;
        param(2) = p;
        param(3) = q;
        return;
    end
end
function path = dubins_curve(p1, p2, r, stepsize, quiet)
    
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % there are 6 types of dubin's curve, only one will have minimum cost
    % LSL = 1;
	% LSR = 2;
	% RSL = 3;
	% RSR = 4;
	% RLR = 5;
    % LRL = 6;
    
    % The three segment types a path can be made up of
    % L_SEG = 1;
    % S_SEG = 2;
    % R_SEG = 3;

    % The segment types for each of the Path types
    %{
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %}
            
    % the return parameter from dubins_core
    % param.p_init = p1;              % the initial configuration
    % param.seg_param = [0, 0, 0];    % the lengths of the three segments
    % param.r = r;                    % model forward velocity / model angular velocity turning radius
    % param.type = -1;                % path type. one of LSL, LSR, ... 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Handle inputs.
    if nargin < 3
        error('Function requires at least two inputs.');
    elseif nargin < 4
        stepsize = 0;
    elseif nargin < 5 
        quiet = 0;  %Default/undefined is not quiet
    end
    
    if ~quiet
        close(findobj('type','figure','name','Dubins curve'));
        tic;
    end
    
    % Check if the destination lies on the circle that can be reach
    % directly from the starting point
    if(p1(3)~=p2(3))
        T = [cos(p1(3)), sin(p1(3));...
             cos(p2(3)), sin(p2(3)) ];
        Y = [p1(1)*cos(p1(3)) + p1(2)*sin(p1(3)); ...
             p2(1)*cos(p2(3)) + p2(2)*sin(p2(3)) ];
        X = T \ Y;
        if( norm(X-reshape(p1(1:2), [2,1]),2) == r ) && ( norm(X-reshape(p2(1:2),[2,1]),2) == r )
            warning('p2 lies on the turning circle from the p2, dubins curve may be suboptimal');
        end
    end
        
    
    % main function
    param = dubins_core(p1, p2, r);
    if stepsize <= 0
        stepsize = dubins_length(param)/1000;
    end
    path = dubins_path_sample_many(param, stepsize);
    
    % plot if not quiet
    if ~quiet
        disp('dubins calculation time'); toc;
        % plotting
        tic;    % most of the time is spent on plotting
        figure('name','Dubins curve');
        plot(path(:,1), path(:,2)); axis equal; hold on
        scatter(p1(1), p1(2), 45, '*','r','LineWidth',1); hold on;
        scatter(p2(1), p2(2), 45, 'square','b','LineWidth',1); hold on;
        text(p1(1), p1(2),'start','HorizontalAlignment','center');
        text(p2(1), p2(2),'end','VerticalAlignment','top');
        disp('plot drawing time'); toc;
    end
end

function path = dubins_path_sample_many(param, stepsize)
    if param.flag < 0
        path = 0;
        return
    end
    length = dubins_length(param);
    path = -1 * ones(floor(length/stepsize), 3);
    x = 0;
    i = 1;
    while x <= length
        path(i, :) = dubins_path_sample( param, x );
        x = x + stepsize;
        i = i + 1;
    end
    return
end

function length = dubins_length(param)
    length = param.seg_param(1);
    length = length + param.seg_param(2);
    length = length + param.seg_param(3);
    length = length * param.r;
end


%{
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
 * @param q    - the configuration result
 * @returns    - -1 if 't' is not in the correct range
%}
function end_pt = dubins_path_sample(param, t)
    if( t < 0 || t >= dubins_length(param) || param.flag < 0)
        end_pt = -1;
        return;
    end

    % tprime is the normalised variant of the parameter t
    tprime = t / param.r;

    % In order to take rho != 1 into account this function needs to be more complex
    % than it would be otherwise. The transformation is done in five stages.
    %
    % 1. translate the components of the initial configuration to the origin
    % 2. generate the target configuration
    % 3. transform the target configuration
    %      scale the target configuration
    %      translate the target configration back to the original starting point
    %      normalise the target configurations angular component

    % The translated initial configuration
    p_init = [0, 0, param.p_init(3) ];
    
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The three segment types a path can be made up of
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;

    % The segment types for each of the Path types
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Generate the target configuration
    types = DIRDATA(param.type, :);
    param1 = param.seg_param(1);
    param2 = param.seg_param(2);
    mid_pt1 = dubins_segment( param1, p_init, types(1) );
    mid_pt2 = dubins_segment( param2, mid_pt1,  types(2) );
    
    % Actual calculation of the position of tprime within the curve
    if( tprime < param1 ) 
        end_pt = dubins_segment( tprime, p_init,  types(1) );
    elseif( tprime < (param1+param2) ) 
        end_pt = dubins_segment( tprime-param1, mid_pt1,  types(2) );
    else 
        end_pt = dubins_segment( tprime-param1-param2, mid_pt2,  types(3) );
    end

    % scale the target configuration, translate back to the original starting point
    end_pt(1) = end_pt(1) * param.r + param.p_init(1);
    end_pt(2) = end_pt(2) * param.r + param.p_init(2);
    end_pt(3) = mod(end_pt(3), 2*pi);
    return;
end

%{
 returns the parameter of certain location according to an inititalpoint,
 segment type, and its corresponding parameter
%}

function seg_end = dubins_segment(seg_param, seg_init, seg_type)
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;
    if( seg_type == L_SEG ) 
        seg_end(1) = seg_init(1) + sin(seg_init(3)+seg_param) - sin(seg_init(3));
        seg_end(2) = seg_init(2) - cos(seg_init(3)+seg_param) + cos(seg_init(3));
        seg_end(3) = seg_init(3) + seg_param;
    elseif( seg_type == R_SEG )
        seg_end(1) = seg_init(1) - sin(seg_init(3)-seg_param) + sin(seg_init(3));
        seg_end(2) = seg_init(2) + cos(seg_init(3)-seg_param) - cos(seg_init(3));
        seg_end(3) = seg_init(3) - seg_param;
    elseif( seg_type == S_SEG ) 
        seg_end(1) = seg_init(1) + cos(seg_init(3)) * seg_param;
        seg_end(2) = seg_init(2) + sin(seg_init(3)) * seg_param;
        seg_end(3) = seg_init(3);
    end
end





