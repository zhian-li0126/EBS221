
% Start and end positions
x_start = -3 * W;
y_start = RL / 2;
x_end = x_start;
y_end = y_start;

% Coordinates of headland nodes (lower and upper)
x_lower = W + 2.5 * (0:N-1);
y_lower = zeros(1, N);

x_upper = x_lower;
y_upper = RL * ones(1, N);



%% 

HUGE = 1e6; % Large penalty value
DMAT = HUGE * ones(2*N+2); % Initialize with large values

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

%% -------------------- Step B: Build Cost Matrix --------------------
HUGE = 1e6; % Large penalty value
DMAT = HUGE * ones(2*N+2); % Initialize with large values

% 1. Row traversal cost (zero cost between lower-upper of the same row)
for i = 2:N+1 % lower nodes
    for j = N+2:2*N+1 % upper nodes
        if (j - i) == N
            DMAT(i,j) = 0; % row traversal is "free"
            DMAT(j,i) = 0;
        end
    end
end

% 2. Headland turning costs (Π or Ω turns)
for i = 2:N % lower nodes only
    for j = i+1:N+1 % only look ahead to avoid duplicates
        d = abs(j - i);
        spacing = d * W;
        if spacing > 2 * Rmin
            % Π turn cost: half-circle + spacing
            DMAT(i,j) = pi * Rmin + spacing;
        else
            % Ω turn cost: approx. twice a half-circle
            DMAT(i,j) = 2 * pi * Rmin;
        end
        DMAT(j,i) = DMAT(i,j); % Symmetric cost
        DMAT(i+N, j+N) = DMAT(i,j); % Same for top nodes
        DMAT(j+N, i+N) = DMAT(i,j);
    end
end

% 3. Manhattan distances from start and end nodes to all field nodes
for i = 2:2*N+1
    DMAT(1,i) = abs(x(1) - x(i)) + abs(y(1) - y(i));
    DMAT(i,1) = DMAT(1,i); % symmetry

    DMAT(2*N+2,i) = abs(x(2*N+2) - x(i)) + abs(y(2*N+2) - y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i); % symmetry
end

% 4. Prohibit direct start-to-end transitions
DMAT(1,2*N+2) = HUGE;
DMAT(2*N+2,1) = HUGE;

% % Add penalties for crossing the field diagonally (disable jumps not to neighbor rows)
% for i = 2:N+1  % lower nodes
%     for j = 2:N+1  % other lower nodes
%         if abs(i - j) > 1
%             DMAT(i, j) = HUGE;
%             DMAT(i + N, j + N) = HUGE;
%         end
%     end
% end


%% -------------------- Solve TSP (Step B Completion) --------------------
XY = [x', y'];
t = cputime;
resultStruct = tspof_ga('xy', XY, 'DMAT', DMAT, ...
                        'SHOWRESULT', true, ...
                        'SHOWWAITBAR', true, ...
                        'SHOWPROG', true);
E = cputime - t; % computation time

route = [1, resultStruct.optRoute, 2*N+2]; % full node sequence

% Output result
disp('Computed Optimal Route (Node Indices):');
disp(route);
disp(['Computed Minimum Distance: ', num2str(resultStruct.minDist)]);
disp(['Computation Time: ', num2str(E), ' seconds']);

%% Plot Solution
figure; plot(x(route), y(route), 'r-o', 'LineWidth', 2);
hold on; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2));
title('Optimal Node Sequence Visualization');
xlabel('X [m]'); ylabel('Y [m]'); axis equal;

%% -------------------- Step C: Generate Waypoints Along the Node Sequence --------------------
waypoints = [];
for k = 1:length(route)-1
    pt1 = [x(route(k)), y(route(k))];
    pt2 = [x(route(k+1)), y(route(k+1))];

    % Number of interpolation points (adjust spacing for smoothness)
    n_points = ceil(norm(pt2 - pt1) / 0.5);  % every 0.5m

    % Interpolate linearly
    xs = linspace(pt1(1), pt2(1), n_points)';
    ys = linspace(pt1(2), pt2(2), n_points)';
    waypoints = [waypoints; [xs, ys]];
end

% Plot the generated waypoints
figure; plot(waypoints(:,1), waypoints(:,2), 'b.-'); axis equal;
title('Generated Waypoints from Node Sequence');
xlabel('X [m]'); ylabel('Y [m]');

%% -------------------- Step D: Simulate Path Following with Pure Pursuit Controller --------------------

% NOT WORKING YET
global dt DT
dt = 0.01;
DT = 1000;

% Reset initial state
q = [-3*2.5; 20/2; 0; 0; 0]; % [x, y, theta, gamma, v]
v_ref = 1.0;  % desired constant speed

% Declare controller limits
gamma_limit = deg2rad(60);
umin = [-gamma_limit; 0];     % [steering min; velocity min]
umax = [ gamma_limit; 2];     % [steering max; velocity max]
Qmin = [-Inf; -Inf; -Inf; -gamma_limit; 0];
Qmax = [ Inf;  Inf;  Inf;  gamma_limit; 2];
tau_gamma = 0.5; % steering actuator time constant
tau_v = 0.5;     % speed actuator time constant


% Storage for analysis
q_history = q';
cte_history = [];

for t_sim = 0:DT:300  % max sim time (adjust if needed)
    % Call pure pursuit controller to compute steering reference
    [gamma_ref, cross_track_error] = purePursuitController(q, L, Ld, waypoints);

    % Record cross-track error
    cte_history(end+1) = cross_track_error;

    % Apply the control input
    u = [gamma_ref; v_ref];
    q = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v);

    % Log state
    q_history = [q_history; q'];

    % Terminate if reached the end
    if norm(q(1:2) - waypoints(end,:)') < 0.5
        break;
    end
end

% Plot the followed trajectory vs the desired path
figure;
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(q_history(:,1), q_history(:,2), 'r-', 'LineWidth', 2);
legend('Desired Waypoints', 'Robot Path');
xlabel('X [m]'); ylabel('Y [m]');
title('Path Following Result');
axis equal; grid on;

% Report RMS cross-track error
rms_cte = sqrt(mean(cte_history.^2));
disp(['RMS Cross-Track Error: ', num2str(rms_cte), ' meters']);


%%
% 
% %initial state [x; y; theta; gamma; v]
% q= [-3*2.5;20/2;0;0;0];
% 
% % Controller and dynamics parameters
% umin = [-gamma_limit; 0];         % [steering; velocity]
% umax = [ gamma_limit; 2];         % [steering; velocity]
% Qmin = [-Inf; -Inf; -Inf; -gamma_limit; 0]; % state min limits
% Qmax = [ Inf;  Inf;  Inf;  gamma_limit; 2]; % state max limits
% tau_gamma = 0.5; % steering actuator time constant
% tau_v = 0.5;     % speed actuator time constant
% 
% global dt DT
% dt = 0.01;
% DT = 0.1;
% 
% %moving from any lower headland node to any upper
% % headland node (and vice versa) is only possible if both nodes
% % belong to the same field row
% for i=2:N+1 % for each lower node
%     for j=N+2:2*N+1 % for each upper node
%     if (j-i) == N % if nodes belong to same row, non working cost is
%         % zero because the row must be cultivated
%         DMAT(i,j) = 0; DMAT(j,i) = 0;
%         else
%             DMAT(i,j) = HUGE;
%             DMAT(j,i) = HUGE;
%         end
%     end
% end
% 
% %costs to go from start and end node to other nodes
% %approximate with Manhattan distance
% for i=2:2*N+1 %all field row nodes
%  %manhattan distance between this node and start node
%  DMAT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
%  DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
%  DMAT(2*N+2,i) = abs(x(2*N+2)-x(i)) + abs(y(2*N+2)-y(i));
%  DMAT(i,2*N+2) = DMAT(2*N+2,i); % cost matrix symmetry
% end
% 
% %cost between start and end nodes
% DMAT(1,2*N+2) = HUGE;
% DMAT(2*N+2, 1) = HUGE; % cost matrix symmetry
% 
% %now set up headland turning costs between pairs of bottom-nodes
% % and pairs of top-nodes
% for i=2:N % for each node at the ‘bottom’
%  for j=i+1:N+1 % for each node to the ‘right’ of the node
%  d = abs(i-j);
%  % a PI turn can be made (plus a straight line of length (d-1) row widths)
%      if (Rmin <=d*W/2) DMAT(i,j) = ………
%          else % make an omega turn
%          DMAT(i,j) = ……..
%      end
%      DMAT(j,i)=DMAT(i,j); % symmetry of cost
%      DMAT(i+N, j+N) = DMAT(i,j); % same cost for pair of top-nodes
%      DMAT(j+N, i+N)= DMAT(i,j);
%  end
% end
% 
% %costs to go from start and end node to other nodes
% %approximate with Manhattan distance
% for i=2:2*N+1 %all field row nodes
%  %manhattan distance between this node and start node
%  DMAT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
%  DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
%  DMAT(2*N+2,i) = abs(x(2*N+2)-x(i)) + abs(y(2*N+2)-y(i));
%  DMAT(i,2*N+2) = DMAT(2*N+2,i); % cost matrix
% symmetry
% end
% 
% %cost between start and end nodes
% DMAT(1,2*N+2) = HUGE;
% DMAT(2*N+2, 1) = HUGE; % cost matrix symmetry
% 
% 
% %TSPOF_GA Fixed Open Traveling Salesman Problem (TSP) Genetic Algorithm (GA)
% % Finds a (near) optimal solution to a variation of the TSP by setting up a GA to search for the shortest
% % route (least distance for the salesman to travel from a FIXED START to a FIXED END while visiting the other
% % cities exactly once)
% %
% % Summary:
% % 1. A single salesman starts at the first point, ends at the last point, and travels to each of the
% % remaining cities in between, but does not close the loop by returning to the city he started from
% % 2. Each city is visited by the salesman exactly once
% %
% % Note: The Fixed Start is taken to be the first XY point, and the Fixed End is taken to be the last XY point
% % Input:
% % USERCONFIG (structure) with zero or more of the following fields:
% % - XY (float) is an Nx2 matrix of city locations, where N is the number of cities
% % - DMAT (float) is an NxN matrix of point to point distances/costs
% % - POPSIZE (scalar integer) is the size of the population (should be divisible by 4)
% % - NUMITER (scalar integer) is the number of desired iterations for the algorithm to run
% % - SHOWPROG (scalar logical) shows the GA progress if true
% % - SHOWRESULT (scalar logical) shows the GA results if true
% % - SHOWWAITBAR (scalar logical) shows a waitbar if true
% 
% XY = [x' y']; t = cputime;
%  resultStruct = tspof_ga('xy', XY , 'DMAT', DMAT, 'SHOWRESULT', false, 'SHOWWAITBAR', false, 'SHOWPROG', false);
%  E = cputime - t; %print time required to compute it.
%  route = [1 resultStruct.optRoute 2*N+2]; % extract node sequence
%  resultStruct.minDist %print computed minimum dist