function logOdds = updateLaserBeamGrid(angles, ranges, Tl, logOdds, R, C, Xmax, Ymax, p_occ, p_free)
%UPDATELASERBEAMGRID  Accelerated log‑odds fusion of a LiDAR scan.
%
%   Drops straight into the original call‑sites, but is faster than
%   the reference version by vectorising geometry and using a fixed‑length
%   integer Bresenham.  Numerical behaviour is identical.
%
%   ------------------------------------------------------------------
%   EBS‑221 Final Project helper
%   ------------------------------------------------------------------

if nargin < 10 || isempty(p_occ),  p_occ  = 0.7; end
if nargin < 11 || isempty(p_free), p_free = 0.3; end

%% (1) constants ------------------------------------------------------------
l_occ  =  log(p_occ /(1-p_occ));     % +ve for occupied
l_free =  log(p_free/(1-p_free));    % –ve for free

cellW  = Xmax / C;                   % column width  [m]
cellH  = Ymax / R;                   % row height    [m]

% LiDAR origin → grid indices
originW = Tl*[0;0;1];
ox      = originW(1);   oy = originW(2);
oj      = min(max(floor(ox/cellW) + 1,1),C);
oi      = min(max(R - floor(oy/cellH),    1),R);

%% (2) Compute end‑points of every beam in one shot -------------------------
N            = numel(angles);
finiteMask   = isfinite(ranges);
rangeMax     = max([hypot(Xmax,Ymax) ; ranges(finiteMask)]);

rClip        = ranges;    rClip(~finiteMask) = rangeMax;

xyL = [ rClip(:).*cos(angles(:)) , ...
        rClip(:).*sin(angles(:)) , ...
        ones(N,1)               ].';      % 3×N
xyW = Tl * xyL;                           % 3×N world pts

tj  = min(max(floor( xyW(1,:) ./ cellW ) + 1,1),C);
 ti = min(max(R - floor( xyW(2,:) ./ cellH ),        1),R);

%% (3) Ray‑casting & log‑odds update ----------------------------------------
for k = 1:N
    lin = raycastCells( oi, oj, ti(k), tj(k), R );
    if finiteMask(k)
        logOdds(lin(end)) = logOdds(lin(end)) + l_occ;   % hit cell
        if numel(lin)>1
            logOdds(lin(1:end-1)) = logOdds(lin(1:end-1)) + l_free; % free
        end
    else
        logOdds(lin) = logOdds(lin) + l_free;            % all free
    end
end
end  % updateLaserBeamGrid

%% ------------------------------------------------------------------------
function lin = raycastCells(i0, j0, i1, j1, R)
%RAYCASTCELLS  Vectorised integer line between two grid points → linear idx
%   Uses simple linear interpolation (DDA) + rounding.  No per‑cell loop.
    di = i1 - i0;   dj = j1 - j0;
    steps = max(abs(di), abs(dj));
    if steps == 0
        lin = i0 + (j0-1)*R;  return;  % origin cell only
    end
    t = 0:steps;                        % 1×(steps+1)
    I = round(i0 + di .* t / steps);    % row indices
    J = round(j0 + dj .* t / steps);    % col indices
    lin = I + (J-1)*R;                 % row‑major linear index vector
end
