% buildCostMatrix  Generate DMAT for orchard‑row TSP (Pi/Omega model)
%
%   DMAT = buildCostMatrix(XY,rowW,gamma_max,L)
%
%   XY       – (2*N+2)×2 list of node coordinates produced by
%              makeFieldNodes() or similar.  Node #1 is START, the last
%              node is END,   nodes 2…N+1 are LOWER headland points,
%              nodes N+2…2N+1 are UPPER headland points (1‑based index).
%   rowW     – row spacing [m].
%   gamma_max – maximum steering angle [rad].
%   L        – wheel‑base [m].
%
%   The cost model follows Lecture‑6 (headland manoeuvres):
%      • In‑row traversal cost  = 0.
%      • Π‑turn if  d*rowW >= 2*Rmin      cost = π*Rmin + (d-1)*rowW
%      • Ω‑turn otherwise                  cost = (3π - 2γ)*Rmin  (see slides)
%      • Manhattan metrics for start/end ↔ field nodes.
%      • Direct start ↔ end  move is forbidden (HUGE).
%
%   ------------------------------------------------------------------
function DMAT = buildCostMatrix(XY,rowW,gamma_max,L)

HUGE = 1e6;
N     = (size(XY,1)-2)/2;   % number of rows
if abs(N-round(N))>eps, error('XY size does not match 2N+2 pattern'); end
N     = round(N);

Rmin  = L / tan(gamma_max);
RL    = XY(N+2,2) - XY(2,2);   % row length from lower->upper

DMAT  = HUGE*ones(2*N+2);

%% 1)   lower to upper   (free in‑row traversal)
for i=2:N+1
    j = i+N;           % paired upper index
    DMAT(i ,j ) = 0;   % lower→upper (cultivation)
    DMAT(j ,i ) = 0;   % upper→lower
end

%% 2)   headland turns (bottom & top)
for i=2:N+1                % iterate over bottom nodes
    for j=i+1:N+1          % only look "right" to avoid duplicates
        d   = j-i;         % row separation (integer)
        dx  = d*rowW;
        if dx >= 2*Rmin           % Pi‑turn possible
            cost = pi*Rmin + (d-1)*rowW;
        else                      % Omega‑turn
            gamma = acos(1 - (2*Rmin + dx)^2/(8*Rmin^2));
            cost  = 3*pi*Rmin - 2*gamma*Rmin;  %#ok<NASGU>
            % Ω analytic length simplified by lecture slide (always > Pi)
            cost  = 2*pi*Rmin;   % practical approximation
        end
        % bottom↔bottom
        DMAT(i ,j ) = cost;  DMAT(j ,i ) = cost;
        % top   ↔top   (offset by N)
        DMAT(i+N,j+N) = cost;  DMAT(j+N,i+N) = cost;
    end
end

%% 3)   Manhattan start/end to field nodes
start = 1;  finish = 2*N+2;
for i=2:2*N+1
    DMAT(start,i)  = abs(XY(start,1)-XY(i,1))  + abs(XY(start,2)-XY(i,2));
    DMAT(i,start)  = DMAT(start,i);
    DMAT(finish,i) = abs(XY(finish,1)-XY(i,1)) + abs(XY(finish,2)-XY(i,2));
    DMAT(i,finish) = DMAT(finish,i);
end

%% 4)   Forbid start to end direct jump
DMAT(start,finish)  = HUGE;
DMAT(finish,start)  = HUGE;

end % buildCostMatrix
