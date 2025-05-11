
% initializing Part A
N = 10;                     %number of rows
RL = 20;                    % row length
W = 2.5;                    % implement width
L = 3;                      % tractor length
Ld = 1.5;                   % look-ahead distance
gamma_limit = deg2rad(60);  % steering limit
Rmin = L/tan(gamma_limit);  % minimum turning radius

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

% Assemble full node coordinate list (1=start, 2..N+1=lower, N+2..2N+1=upper, 2N+2=end)
x = [x_start, x_lower, x_upper, x_end];
y = [y_start, y_lower, y_upper, y_end];

% Plot for verification
% figure; plot(x, y, 'ko'); text(x + 0.2, y + 0.2, string(1:2*N+2)); axis equal;
% title('Node Layout Verification'); xlabel('X [m]'); ylabel('Y [m]');

%% 

HUGE = 1e6; % Large penalty value
DMAT = HUGE * ones(2*N+2); % Initialize with large values

%%

%initial state [x; y; theta; gamma; v]
q= [-3*2.5;20/2;0;0;0];

% Controller and dynamics parameters
umin = [-gamma_limit; 0];         % [steering; velocity]
umax = [ gamma_limit; 2];         % [steering; velocity]
Qmin = [-Inf; -Inf; -Inf; -gamma_limit; 0]; % state min limits
Qmax = [ Inf;  Inf;  Inf;  gamma_limit; 2]; % state max limits
tau_gamma = 0.5; % steering actuator time constant
tau_v = 0.5;     % speed actuator time constant

global dt DT
dt = 0.01;
DT = 0.1;

%moving from any lower headland node to any upper
% headland node (and vice versa) is only possible if both nodes
% belong to the same field row
for i=2:N+1 % for each lower node
    for j=N+2:2*N+1 % for each upper node
    if (j-i) == N % if nodes belong to same row, non working cost is
        % zero because the row must be cultivated
        DMAT(i,j) = 0; DMAT(j,i) = 0;
        else
            DMAT(i,j) = HUGE;
            DMAT(j,i) = HUGE;
        end
    end
end

%costs to go from start and end node to other nodes
%approximate with Manhattan distance
for i=2:2*N+1 %all field row nodes
 %manhattan distance between this node and start node
 DMAT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
 DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
 DMAT(2*N+2,i) = abs(x(2*N+2)-x(i)) + abs(y(2*N+2)-y(i));
 DMAT(i,2*N+2) = DMAT(2*N+2,i); % cost matrix symmetry
end

%cost between start and end nodes
DMAT(1,2*N+2) = HUGE;
DMAT(2*N+2, 1) = HUGE; % cost matrix symmetry

%now set up headland turning costs between pairs of bottom-nodes
% and pairs of top-nodes
for i=2:N % for each node at the ‘bottom’
 for j=i+1:N+1 % for each node to the ‘right’ of the node
 d = abs(i-j);
 % a PI turn can be made (plus a straight line of length (d-1) row widths)
     if (Rmin <=d*W/2) DMAT(i,j) = ………
         else % make an omega turn
         DMAT(i,j) = ……..
     end
     DMAT(j,i)=DMAT(i,j); % symmetry of cost
     DMAT(i+N, j+N) = DMAT(i,j); % same cost for pair of top-nodes
     DMAT(j+N, i+N)= DMAT(i,j);
 end
end

%costs to go from start and end node to other nodes
%approximate with Manhattan distance
for i=2:2*N+1 %all field row nodes
 %manhattan distance between this node and start node
 DMAT(1,i) = abs(x(1)-x(i)) + abs(y(1)-y(i));
 DMAT(i,1) = DMAT(1,i); % cost matrix symmetry
 DMAT(2*N+2,i) = abs(x(2*N+2)-x(i)) + abs(y(2*N+2)-y(i));
 DMAT(i,2*N+2) = DMAT(2*N+2,i); % cost matrix
symmetry
end

%cost between start and end nodes
DMAT(1,2*N+2) = HUGE;
DMAT(2*N+2, 1) = HUGE; % cost matrix symmetry


%TSPOF_GA Fixed Open Traveling Salesman Problem (TSP) Genetic Algorithm (GA)
% Finds a (near) optimal solution to a variation of the TSP by setting up a GA to search for the shortest
% route (least distance for the salesman to travel from a FIXED START to a FIXED END while visiting the other
% cities exactly once)
%
% Summary:
% 1. A single salesman starts at the first point, ends at the last point, and travels to each of the
% remaining cities in between, but does not close the loop by returning to the city he started from
% 2. Each city is visited by the salesman exactly once
%
% Note: The Fixed Start is taken to be the first XY point, and the Fixed End is taken to be the last XY point
% Input:
% USERCONFIG (structure) with zero or more of the following fields:
% - XY (float) is an Nx2 matrix of city locations, where N is the number of cities
% - DMAT (float) is an NxN matrix of point to point distances/costs
% - POPSIZE (scalar integer) is the size of the population (should be divisible by 4)
% - NUMITER (scalar integer) is the number of desired iterations for the algorithm to run
% - SHOWPROG (scalar logical) shows the GA progress if true
% - SHOWRESULT (scalar logical) shows the GA results if true
% - SHOWWAITBAR (scalar logical) shows a waitbar if true

XY = [x' y']; t = cputime;
 resultStruct = tspof_ga('xy', XY , 'DMAT', DMAT, 'SHOWRESULT', false, 'SHOWWAITBAR', false, 'SHOWPROG', false);
 E = cputime - t; %print time required to compute it.
 route = [1 resultStruct.optRoute 2*N+2]; % extract node sequence
 resultStruct.minDist %print computed minimum dist