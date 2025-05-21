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


%% -------------------- Step B with Fish-tail turns: Build Cost Matrix --------------------
% Create directories
[~, ~] = mkdir("results\fishtail\B");
[~, ~] = mkdir("results\fishtail\C");
[~, ~] = mkdir("results\fishtail\D");

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

% ---- (2) Headland turning costs: Π / Ω / Fish-tail turns on south or north edges --
% Compute turning costs between nodes i, j (both are at top or bottom)
for i = 2:N+1 % for each node at the 'bottom'
    for j = i+1:N+1 % for each node to the 'right' of node i
        d = abs(j-i); % Number of rows between the nodes
        dW = d * W;   % Physical distance between the rows
        
        % Calculate pi turn cost if possible
        if (Rmin <= dW/2)
            % PI turn cost = π * Rmin (for the semi-circle) + (d-1)*W (for the straight part)
            pi_cost = pi * Rmin + (d-1) * W;
        else
            pi_cost = HUGE; % PI turn not possible
        end
        
        % Calculate omega turn cost 
        omega_cost = 2 * pi * Rmin;
        
        % Calculate fish-tail turn cost
        % Basic fish-tail components:
        % 1. Forward quarter-turn (π/2 * Rmin)
        % 2. Reverse quarter-turn (π/2 * Rmin)
        % 3. Straight reverse (distance between rows)
        % 4. Reverse quarter-turn (π/2 * Rmin)
        % 5. Forward quarter-turn (π/2 * Rmin)
        fishtail_cost = pi * Rmin + pi * Rmin + dW;
        
        % Use the minimum cost between pi, omega and fish-tail turns
        DMAT(i,j) = min([pi_cost, omega_cost, fishtail_cost]);
        
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

% Solve TSP (same as original)
XY = [x', y'];
t = cputime;
resultStruct = tspof_ga('xy', XY, 'DMAT', DMAT, ...
                        'POPSIZE', 200, ...
                        'NUMITER', 2e4, ...
                        'SHOWRESULT', true, ...
                        'SHOWWAITBAR', true, ...
                        'SHOWPROG', true);
E = cputime - t; % computation time
route = [1, resultStruct.optRoute, 2*N+2]; % full node sequence

% Check for missing rows (same as original)
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
disp('Computed Optimal Route (Node Indices) with Fish-tail turns:');
disp(route);
disp(['Computed Minimum Distance: ', num2str(resultStruct.minDist)]);
disp(['Computation Time: ', num2str(E), ' seconds']);

% Save output
fid = fopen('results\fishtail\B\results.txt','w');
fprintf(fid, 'Computed Optimal Route (Node Indices) with Fish-tail turns:\n');
fprintf(fid, '%d ', route);
fprintf(fid, '\n\n');
fprintf(fid, 'Computed Minimum Distance: %.4f\n', resultStruct.minDist);
fprintf(fid, 'Computation Time: %.4f seconds\n', E);
fclose(fid);

% Plot Solution
figure; plot(x(route), y(route), 'r-o', 'LineWidth', 2);
hold on; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2));
title('Optimal Node Sequence Visualization with Fish-tail Turns');
xlabel('X [m]'); ylabel('Y [m]'); axis equal;
saveas(gcf, "results\fishtail\B\optimal_node.png");

%% -------------------- Step C with Fish-tail: Build waypoints --------------------
numLinks = numel(route)-1;
waypoints = [];
waypointSegment = [];
isTurnLink = false(numLinks,1);
useFishtail = false(numLinks,1);  % Track which turns use fish-tail

% Mark which links are turns (same as original)
for k = 1:numLinks
    i = route(k); j = route(k+1);
    if k==1 || k==numLinks
        isTurnLink(k) = true;   % entry or exit
    else
        paired = ((i>=2 && i<=N+1 && j-i==N) || (j>=2 && j<=N+1 && i-j==N));
        sameBot = all([i j]>=2 & [i j]<=N+1);
        sameTop = all([i j]>=N+2 & [i j]<=2*N+1);
        if ~paired && (sameBot||sameTop)
            isTurnLink(k) = true;
            
            % Check if this headland turn should use fish-tail
            if sameBot || sameTop
                d = abs(j-i); % Number of rows between the nodes
                dW = d * W;   % Physical distance between the rows
                
                % Determine whether to use fish-tail based on turning radius
                if Rmin > dW/2
                    useFishtail(k) = true;
                end
            end
        end
    end
end

% Build waypoints & segments
for k = 1:numLinks
    i = route(k); j = route(k+1);
    p1 = [x(i), y(i)];  p2 = [x(j), y(j)];
    
    if isTurnLink(k)
        if useFishtail(k)
            % This is a headland turn that should use fish-tail
            % Generate fish-tail path
            % First, determine start and end orientations
            if k==1
                theta0 = 0;
                theta1 = (j<=N+1)*pi/2 + (j>N+1)*(-pi/2);
            elseif k==numLinks
                theta0 = (i<=N+1)*-pi/2 + (i>N+1)*(pi/2);
                theta1 = pi;
            else
                sameBot = all([i j]>=2 & [i j]<=N+1);
                theta0 = sameBot*(-pi/2) + (~sameBot)*(pi/2);
                theta1 = theta0; % Same orientation for fish-tail turns
            end
            
            % Generate fish-tail path
            pts = generateFishtailPath([p1,theta0], [p2,theta1], Rmin, 0.25);
            newPts = pts(2:end,1:2);
        else
            % Use standard Dubins turn
            if k==1
                theta0 = 0;
                theta1 = (j<=N+1)*pi/2 + (j>N+1)*(-pi/2);
            elseif k==numLinks
                theta0 = (i<=N+1)*-pi/2 + (i>N+1)*(pi/2);
                theta1 = pi;
            else
                sameBot = all([i j]>=2 & [i j]<=N+1);
                theta0 = sameBot*(-pi/2) + (~sameBot)*(pi/2);
                theta1 = -theta0;
            end
            
            P = dubins_core([p1,theta0], [p2,theta1], Rmin);
            pts = dubins_path_sample_many(P, 0.25);
            newPts = pts(2:end,1:2);
        end
    else
        % straight path (same as original)
        nPts = ceil(norm(p2-p1)/0.5);
        xs = linspace(p1(1),p2(1),nPts).';
        ys = linspace(p1(2),p2(2),nPts).';
        newPts = [xs(2:end), ys(2:end)];
    end
    
    waypoints = [waypoints; newPts];
    waypointSegment = [waypointSegment; k*ones(size(newPts,1),1)];
end

% Define guidedPoints (same as original)
guidedPoints = zeros(numLinks,2);
for k = 1:numLinks
    idxs = find(waypointSegment==k);
    segPts = waypoints(idxs,:);
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

% Plot the generated waypoints
figure; plot(waypoints(:,1), waypoints(:,2), 'b.-'); axis equal; hold on;
plot(guidedPoints(:,1), guidedPoints(:,2), 'o');
title('Generated Waypoints and Guided Points with Fish-tail Turns');
xlabel('X [m]'); ylabel('Y [m]');
legend("Waypoints", "Guided Points", "Location", "best");
saveas(gcf, "results\fishtail\C\waypoints.png");

% Create a visualization to highlight fish-tail segments
figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'k-', 'LineWidth', 1);

% Highlight fish-tail segments
for k = 1:numLinks
    if useFishtail(k)
        idxs = find(waypointSegment == k);
        plot(waypoints(idxs,1), waypoints(idxs,2), 'r-', 'LineWidth', 2);
    end
end

plot(guidedPoints(:,1), guidedPoints(:,2), 'bo', 'MarkerSize', 6);
axis equal; grid on;
title('Path Visualization with Fish-tail Turns Highlighted');
xlabel('X [m]'); ylabel('Y [m]');
legend('Standard Path', 'Fish-tail Turns', 'Guided Points', 'Location', 'best');
saveas(gcf, "results\fishtail\C\fishtail_highlighted.png");

%% -------------------- Step D with Fish-tail: Path Following --------------------
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

numSteps = length(time_vec);
q_history = zeros(numSteps+1,5);
cte_history = zeros(numSteps,3);
gear_history = zeros(numSteps,1);    % 0 for forward, 1 for reverse

% Store initial state
q_history(1,:) = q.';

for k = 1:numSteps
    % 1) compute steering, cte, and gear
    [delta, ~, reverse_gear] = purePursuitSegmentedWithFishtail(q, L, Ld_line, Ld_turn, ...
        waypoints, waypointSegment, isTurnLink, useFishtail, guidedPoints, ...
        gamma_max, gamma_min);
    
    % Record gear
    gear_history(k) = reverse_gear;
    
    % Choose a base blend factor (tune between 0 and 1)
    alpha = 0.9;
    [cte_path, cte_gp, cte] = computeBlendedError(q, waypoints, waypointSegment, ...
        guidedPoints, isTurnLink, alpha, 0);  % Pass 0 for status since we're not using it
    
    % 2) step the dynamics with reverse gear support
    if reverse_gear
        % For reverse
        q = robot_bike_dyn_reverse(q, [delta; v_ref], ...
            [gamma_min;0], [gamma_max;v_ref], ...
            Qmin, Qmax, L, tau_gamma, tau_v);
    else
        % For forward
        q = robot_bike_dyn(q, [delta; v_ref], ...
            [gamma_min;0], [gamma_max;v_ref], ...
            Qmin, Qmax, L, tau_gamma, tau_v);
    end
    
    % 3) record state
    q_history(k+1,:) = q.';
    
    % 4) compute and store CTE
    cte_history(k, 1) = cte_path;
    cte_history(k, 2) = cte_gp;
    cte_history(k, 3) = cte;
    
    % 5) check termination (avoid stuck at start)
    if norm(q(1:2) - waypoints(end,:).') < 0.5 && k >= 1000
        % trim unused entries
        q_history = q_history(1:k+1,:);
        cte_history = cte_history(1:k, :);
        gear_history = gear_history(1:k, :);
        break;
    end
end

%% Visualization for Step D with Fish-tail
% Calculate RMS CTE
rms_cte = sqrt(mean(cte_history(:,1).^2));
disp(['Segment-bounded RMS CTE with Fish-tail: ', num2str(rms_cte), ' m']);

% Plot robot path
figure;
plot(waypoints(:,1), waypoints(:,2), 'r--', 'LineWidth', 2); hold on;
plot(q_history(:,1), q_history(:,2), 'b-');
axis equal; grid on;
title('Path Following with Fish-tail Turns');
xlabel('X [m]'); ylabel('Y [m]');
legend("Waypoints", "Robot Path", "Location", "best");
saveas(gcf, 'results\fishtail\D\robot_path.png');

% Plot cross-track error
figure;
t = (0:DT:(numel(cte_history(:,1))-1)*DT).';
plot(t, cte_history(:, 1));
grid on;
title('Cross-Track Error with Fish-tail Turns');
xlabel('Time [s]'); ylabel('CTE [m]');
saveas(gcf, 'results\fishtail\D\cte.png');

% Trim q_history to match gear_history length
q_pos = q_history(1:length(gear_history), 1:2);

% Plot trajectory colored by gear
idxForward = gear_history == 0;
idxReverse = gear_history == 1;
figure; hold on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'LineWidth', 1);
scatter(q_pos(idxForward,1), q_pos(idxForward,2), 8, 'g', 'filled');
scatter(q_pos(idxReverse,1), q_pos(idxReverse,2), 8, 'r', 'filled');
legend('Waypoints', 'Forward', 'Reverse', 'Location', 'Best');
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Robot Path Colored by Gear Status (Fish-tail)');
saveas(gcf, 'results\fishtail\D\gear_path.png');

% Calculate separate RMS errors for forward and reverse
if any(idxForward)
    cte_forward = cte_history(idxForward, 1);
    rms_cte_forward = sqrt(mean(cte_forward.^2));
else
    rms_cte_forward = NaN;
end

if any(idxReverse)
    cte_reverse = cte_history(idxReverse, 1);
    rms_cte_reverse = sqrt(mean(cte_reverse.^2));
else
    rms_cte_reverse = NaN;
end

% Calculate separate errors for straight segments and turns
idxTurn = false(size(gear_history));
for i = 1:length(gear_history)
    if i <= length(waypointSegment)
        segIdx = waypointSegment(i);
        if segIdx <= length(isTurnLink)
            idxTurn(i) = isTurnLink(segIdx);
        end
    end
end

idxTurn = idxTurn(1:length(cte_history));
cte_turn_segments = cte_history(idxTurn, 1);
cte_straight_segments = cte_history(~idxTurn, 1);

rms_cte_turn = sqrt(mean(cte_turn_segments.^2));
rms_cte_straight = sqrt(mean(cte_straight_segments.^2));

% Save detailed RMS CTE statistics
fid = fopen('results\fishtail\D\RMS_detailed.txt','w');
fprintf(fid, 'RMS CTE (overall with Fish-tail): %.4f m\n', rms_cte);
fprintf(fid, 'RMS CTE (forward motion): %.4f m\n', rms_cte_forward);
fprintf(fid, 'RMS CTE (reverse motion): %.4f m\n', rms_cte_reverse);
fprintf(fid, 'RMS CTE (turn segments): %.4f m\n', rms_cte_turn);
fprintf(fid, 'RMS CTE (straight segments): %.4f m\n', rms_cte_straight);
fprintf(fid, '\nFish-tail turns were used for %d out of %d turn segments\n', ...
    sum(useFishtail), sum(isTurnLink));
fclose(fid);

% Plot gear status over time
t_gear = (0:(length(gear_history)-1)) * DT;
figure;
stairs(t_gear, gear_history, 'LineWidth', 1.5);
ylim([-0.1, 1.1]);
yticks([0 1]);
yticklabels({'Forward','Reverse'});
xlabel('Time [s]');
ylabel('Gear Status');
title('Forward/Reverse Gear Status During Path Following');
grid on;
saveas(gcf, 'results\fishtail\D\gear_status.png');

% Save overall RMS
fid = fopen('results\fishtail\D\RMS_adaptive.txt','w');
fprintf(fid, 'RMS CTE (adaptive with Fish-tail): %.4f\n', rms_cte);
fclose(fid);
%% -------------------- Helper Functions --------------------

function pts = generateFishtailPath(start, goal, Rmin, step)
    % Generates a fish-tail path between start and goal configurations
    % start = [x, y, theta] in radians
    % goal = [x, y, theta] in radians
    % Rmin = minimum turning radius
    % step = distance between adjacent points in the path
    
    x1 = start(1); y1 = start(2); th1 = start(3);
    x2 = goal(1); y2 = goal(2); th2 = goal(3);
    
    % Calculate vector from start to goal
    dx = x2 - x1;
    dy = y2 - y1;
    dist = sqrt(dx^2 + dy^2);
    heading = atan2(dy, dx);
    
    % Determine fish-tail parameters
    % For a typical fish-tail maneuver:
    % 1. Forward curve (quarter circle)
    % 2. Reverse curve (quarter circle)
    % 3. Straight reverse
    % 4. Reverse curve (quarter circle)
    % 5. Forward curve (quarter circle)
    
    % Calculate number of segments needed
    % Each segment is arc length of Rmin * angle
    quarter_circle = Rmin * pi/2;
    straight_length = max(0, dist - 4*Rmin);
    total_length = 2*quarter_circle + 2*quarter_circle + straight_length;
    
    % Number of points to generate
    num_points = ceil(total_length / step) + 1;
    pts = zeros(num_points, 3);
    
    % Initial position and orientation
    x = x1;
    y = y1;
    theta = th1;
    
    % Determine turn direction based on relative positions
    clockwise = false;
    
    % For headland turns, we need to determine direction based on field layout
    % Bottom headland turns right, top headland turns left
    if abs(th1 - pi/2) < 0.1 || abs(th1 + pi/2) < 0.1
        if th1 < 0
            clockwise = true;  % Bottom headland, turning right
        else
            clockwise = false; % Top headland, turning left
        end
    else
        % For other turns, use cross product to determine direction
        cross_prod = dx * sin(th1) - dy * cos(th1);
        clockwise = cross_prod < 0;
    end
    
    % Ternary operator replacement
    if clockwise
        turn_dir = 1;
    else
        turn_dir = -1;
    end
    
    % Generate points along the path
    idx = 1;
    pts(idx, :) = [x, y, theta];
    
    % 1. First forward curve (quarter circle)
    segment_length = quarter_circle;
    segment_steps = ceil(segment_length / step);
    for i = 1:segment_steps
        delta = step / Rmin;
        theta = theta + turn_dir * delta;
        x = x + step * cos(theta - turn_dir * delta/2);
        y = y + step * sin(theta - turn_dir * delta/2);
        
        idx = idx + 1;
        if idx <= num_points
            pts(idx, :) = [x, y, theta];
        end
    end
    
    % 2. First reverse curve (quarter circle)
    segment_length = quarter_circle;
    segment_steps = ceil(segment_length / step);
    for i = 1:segment_steps
        delta = step / Rmin;
        theta = theta - turn_dir * delta;  % Opposite direction for reverse
        x = x - step * cos(theta + turn_dir * delta/2);  % Negative for reverse
        y = y - step * sin(theta + turn_dir * delta/2);  % Negative for reverse
        
        idx = idx + 1;
        if idx <= num_points
            pts(idx, :) = [x, y, theta];
        end
    end
    
    % 3. Straight reverse segment
    segment_length = straight_length;
    segment_steps = ceil(segment_length / step);
    for i = 1:segment_steps
        x = x - step * cos(theta);  % Negative for reverse
        y = y - step * sin(theta);  % Negative for reverse
        
        idx = idx + 1;
        if idx <= num_points
            pts(idx, :) = [x, y, theta];
        end
    end
    
    % 4. Second reverse curve (quarter circle)
    segment_length = quarter_circle;
    segment_steps = ceil(segment_length / step);
    for i = 1:segment_steps
        delta = step / Rmin;
        theta = theta + turn_dir * delta;  
        x = x - step * cos(theta - turn_dir * delta/2);  % Negative for reverse
        y = y - step * sin(theta - turn_dir * delta/2);  % Negative for reverse
        
        idx = idx + 1;
        if idx <= num_points
            pts(idx, :) = [x, y, theta];
        end
    end
    
    % 5. Final forward curve (quarter circle)
    segment_length = quarter_circle;
    segment_steps = ceil(segment_length / step);
    for i = 1:segment_steps
        delta = step / Rmin;
        theta = theta - turn_dir * delta;  % Back to original direction
        x = x + step * cos(theta + turn_dir * delta/2);
        y = y + step * sin(theta + turn_dir * delta/2);
        
        idx = idx + 1;
        if idx <= num_points
            pts(idx, :) = [x, y, theta];
        end
    end
    
    % Trim extra points if needed
    pts = pts(1:min(idx, num_points), :);
    
    % Ensure final point is close to goal
    pts(end, 1:2) = [x2, y2];
    pts(end, 3) = th2;
end

function [steer, cte, reverse_gear] = purePursuitSegmentedWithFishtail(q, L, Ld_line, Ld_turn, ...
        path, segID, isTurnLink, useFishtail, guidedPoints, gamma_max, gamma_min)
    % SEGMENT-BOUNDED PURE PURSUIT WITH FISH-TAIL SUPPORT
    % Additional output: reverse_gear (true for reverse, false for forward)
    
    x = q(1); y = q(2); th = q(3);
    reverse_gear = false; % Default is forward
    
    % Persistent variables for segment tracking
    persistent curSeg prevDist nxSeg nxprevDist fishtailPhase fishtailProgress
    if isempty(curSeg)
        curSeg = segID(1);
        prevDist = Inf;
        nxSeg = min(curSeg+1, max(segID));
        nxprevDist = Inf;
        fishtailPhase = 0;  % 0=not in fishtail, 1-5=phases of fishtail turn
        fishtailProgress = 0; % Progress through current segment (0-1)
    end
    
    % 1) closest waypoint & cross-track error
    diffs = path - [x y];
    d2 = diffs(:,1).^2 + diffs(:,2).^2;
    [~, idx] = min(d2);
    cte = sqrt(d2(idx));
    
    % 2) detect segment change
    newSeg = segID(idx);
    if newSeg ~= curSeg
        curSeg = newSeg;
        prevDist = Inf;
        nxSeg = min(curSeg+1, max(segID));
        nxprevDist = Inf;
        
        % Reset fishtail phase when entering a new segment
        fishtailPhase = 0;
        fishtailProgress = 0;
        
        % If entering a fishtail turn segment, initialize phase
        if isTurnLink(curSeg) && useFishtail(curSeg)
            fishtailPhase = 1; % Start in phase 1 (forward)
        end
    end
    
    % 3) distances to current & next guided points
    gp = guidedPoints(curSeg, :);   % current guided point
    nxgp = guidedPoints(nxSeg, :);  % next segment's guided point
    gpDist = norm([x y] - gp);
    nxgpDist = norm([x y] - nxgp);
    
    % 4) Fish-tail logic
    if fishtailPhase > 0
        % We're in a fish-tail turn
        
        % Get segment waypoints
        segIdxs = find(segID == curSeg);
        segPts = path(segIdxs, :);
        startPt = segPts(1, :);
        endPt = segPts(end, :);
        
        % Calculate progress through the segment for phase transitions
        % Use projection of current position onto start-end line
        v = endPt - startPt;
        w = [x y] - startPt;
        proj = startPt + (dot(v, w) / dot(v, v)) * v;
        
        % Calculate normalized progress (0 to 1)
        fishtailProgress = norm(proj - startPt) / norm(endPt - startPt);
        
        % Phase transitions based on progress
        if fishtailPhase == 1 && fishtailProgress > 0.2
            % Forward quarter-turn -> Reverse quarter-turn
            fishtailPhase = 2;
            reverse_gear = true;
        elseif fishtailPhase == 2 && fishtailProgress > 0.35
            % Reverse quarter-turn -> Reverse straight
            fishtailPhase = 3;
            reverse_gear = true;
        elseif fishtailPhase == 3 && fishtailProgress > 0.65
            % Reverse straight -> Reverse quarter-turn (opposite direction)
            fishtailPhase = 4;
            reverse_gear = true;
        elseif fishtailPhase == 4 && fishtailProgress > 0.8
            % Reverse quarter-turn -> Forward quarter-turn
            fishtailPhase = 5;
            reverse_gear = false;
        elseif fishtailPhase == 5 && fishtailProgress > 0.95
            % End of fish-tail
            fishtailPhase = 0;
            reverse_gear = false;
        end
        
        % Steering based on current phase
        if fishtailPhase == 1
            % Forward turn to first curve
            targetIdx = min(segIdxs(1) + round(0.2 * length(segIdxs)), segIdxs(end));
            targetPt = path(targetIdx, :);
            vecAng = atan2(targetPt(2)-y, targetPt(1)-x);
            steer = wrapToPi(vecAng - th);
        elseif fishtailPhase == 2
            % First reverse turn (full steering)
            % Check if we're at bottom or top headland to determine steering direction
            if mean(segPts(:,2)) < L/2
                % Bottom headland - reverse right turn
                steer = gamma_max;
            else
                % Top headland - reverse left turn
                steer = gamma_min;
            end
            if reverse_gear
                steer = -steer; % Reverse steering logic
            end
        elseif fishtailPhase == 3
            % Straight reverse
            targetIdx = min(segIdxs(1) + round(0.5 * length(segIdxs)), segIdxs(end));
            targetPt = path(targetIdx, :);
            vecAng = atan2(targetPt(2)-y, targetPt(1)-x);
            steer = wrapToPi(vecAng - th);
            if reverse_gear
                steer = -steer; % Reverse steering logic
            end
        elseif fishtailPhase == 4
            % Second reverse turn (opposite direction of first turn)
            if mean(segPts(:,2)) < L/2
                % Bottom headland - reverse left turn
                steer = gamma_min;
            else
                % Top headland - reverse right turn
                steer = gamma_max;
            end
            if reverse_gear
                steer = -steer; % Reverse steering logic
            end
        elseif fishtailPhase == 5
            % Final forward turn
            targetPt = endPt;
            vecAng = atan2(targetPt(2)-y, targetPt(1)-x);
            steer = wrapToPi(vecAng - th);
        else
            % Fallback to standard lookahead
            % Use regular segment-bounded pure pursuit logic
            if isTurnLink(curSeg) && curSeg ~= max(segID)
                Ld = Ld_turn;
            else
                Ld = Ld_line;
            end
            
            firstIdx = idx;
            while firstIdx > 1 && segID(firstIdx-1) == curSeg
                firstIdx = firstIdx - 1;
            end
            lastIdx = idx;
            while lastIdx < size(segID,1) && segID(lastIdx+1) == curSeg
                lastIdx = lastIdx + 1;
            end
            
            s = 0;
            goalIdx = idx;
            while goalIdx < lastIdx && s < Ld
                s = s + norm(path(goalIdx+1,:) - path(goalIdx,:));
                goalIdx = goalIdx + 1;
            end
            goal = path(goalIdx,:);
            
            dx = goal(1) - x;
            dy = goal(2) - y;
            local_y = -sin(th) * dx + cos(th) * dy;
            curvature = 2 * local_y / (Ld^2);
            steer = atan(L * curvature);
        end
        
        steer = max(min(steer, gamma_max), gamma_min);
    else
        % Regular segment-bounded pure pursuit
        % choose Ld
        if isTurnLink(curSeg) && curSeg ~= max(segID)
            Ld = Ld_turn;
        else
            Ld = Ld_line;
        end
        
        % find segment bounds
        firstIdx = idx;
        while firstIdx > 1 && segID(firstIdx-1) == curSeg
            firstIdx = firstIdx - 1;
        end
        lastIdx = idx;
        while lastIdx < size(segID,1) && segID(lastIdx+1) == curSeg
            lastIdx = lastIdx + 1;
        end
        
        % arc-length lookahead
        s = 0;
        goalIdx = idx;
        while goalIdx < lastIdx && s < Ld
            s = s + norm(path(goalIdx+1,:) - path(goalIdx,:));
            goalIdx = goalIdx + 1;
        end
        goal = path(goalIdx,:);
        
        % compute steering
        dx = goal(1) - x;
        dy = goal(2) - y;
        local_y = -sin(th) * dx + cos(th) * dy;
        curvature = 2 * local_y / (Ld^2);
        steer = atan(L * curvature);
        steer = max(min(steer, gamma_max), gamma_min);
    end
    
    % Update previous distances
    prevDist = gpDist;
    nxprevDist = nxgpDist;
end
function q_next = robot_bike_dyn_reverse(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v)
    % Modified version of robot_bike_dyn for reverse motion
    % The key difference is that velocity-related components are negated
    
    % globals 
    global dt
    global DT
    
    % unpack current state 
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);
    
    % saturate desired inputs
    gamma_d = min(max(u(1), umin(1)), umax(1));
    v_d     = min(max(u(2), umin(2)), umax(2));
    
    eps = 1e-12;                  % numerical zero for the time constants
    
    % fine-step Euler integration over [0, DT) 
    for t = 0 : dt : DT-dt
        % -- steering first-order system --
        if tau_gamma <= eps
            gamma = gamma_d;
        else
            gamma = gamma + dt * (gamma_d - gamma) / tau_gamma;
        end
        
        % -- speed first-order system --
        if tau_v <= eps
            v = v_d;
        else
            v = v + dt * (v_d - v) / tau_v;
        end
        
        % -- kinematic bicycle model with REVERSE motion --
        x     = x - dt * v * cos(theta);  % Negative for reverse
        y     = y - dt * v * sin(theta);  % Negative for reverse
        theta = theta - dt * v * tan(gamma) / L;  % Negative for reverse
        
        % keep γ and v inside physical limits each sub-step
        gamma = min(max(gamma, Qmin(4)), Qmax(4));
        v     = min(max(v,     Qmin(5)), Qmax(5));
    end
    
    % clip final state to workspace limits 
    x     = min(max(x,     Qmin(1)), Qmax(1));
    y     = min(max(y,     Qmin(2)), Qmax(2));
    theta = min(max(theta, Qmin(3)), Qmax(3));
    gamma = min(max(gamma, Qmin(4)), Qmax(4));
    v     = min(max(v,     Qmin(5)), Qmax(5));
    
    % pack output
    q_next = [x; y; theta; gamma; v];
end

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