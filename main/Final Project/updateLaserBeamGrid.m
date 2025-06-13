function logOdds = updateLaserBeamGrid(angles, ranges, Tl, logOdds, R, C, Xmax, Ymax, p_occ, p_free)
%UPDATELASERBEAMGRID  Integrate one LiDAR scan into a 2‑D occupancy grid.
%
%   logOdds = updateLaserBeamGrid(angles, ranges, Tl, logOdds, R, C, ...
%                                 Xmax, Ymax, p_occ, p_free)
%
%   The grid spans the rectangle   x ∈ [0, Xmax],   y ∈ [0, Ymax]
%   and uses row‑major indexing:   logOdds(i,j)  i = 1..R (rows – Y),
%                                              j = 1..C (cols – X).
%
%   INPUTS
%     angles  – Nx1 vector [rad]   (LiDAR frame, CCW, 0 = +X)
%     ranges  – Nx1 vector [m]     (Inf ⇒ no hit within rangeMax)
%     Tl      – 3×3 SE(2) hom. transform from LiDAR to WORLD
%     logOdds – R×C   current log‑odds grid (will be updated in place)
%     R,C     – grid size
%     Xmax,   – physical size of map (origin at 0,0)
%     Ymax
%     p_occ   – (optional) probability for an occupied cell   (default 0.7)
%     p_free  – (optional) probability for a free cell         (default 0.3)
%
%   OUTPUT
%     logOdds – updated log‑odds grid
%
%   The function implements the log‑odds update
%       l_k+1 = l_k + log( p / (1-p) )
%   with   p = p_occ   for the end‑cell of a hit;   p = p_free for
%   each traversed free cell.
%
%   Uses a fast, integer Bresenham to enumerate the cells between the
%   LiDAR origin and the end‑point of every beam.
%
%   ------------------------------------------------------------------
%   EBS‑221 Final Project helper • 2025
%   ------------------------------------------------------------------

if nargin < 10 || isempty(p_occ),  p_occ  = 0.7; end
if nargin < 11 || isempty(p_free), p_free = 0.3; end

% ---------- derive a fallback rangeMax (used for Inf beams) ---------
finiteRanges = ranges(isfinite(ranges));
if ~isempty(finiteRanges)
    rangeMax = max(finiteRanges);
else
    rangeMax = hypot(Xmax, Ymax);   % no finite returns: use map diagonal
end

% Pre‑compute log‑odds increments
l_occ  =  log(p_occ /(1-p_occ));    % positive  (≈ +0.847 for 0.7)
l_free =  log(p_free/(1-p_free));   % negative  (≈ −0.847 for 0.3)

% Grid resolution
cellW = Xmax / C;   % width  of a column [m]
cellH = Ymax / R;   % height of a row    [m]

% LiDAR origin in world coords & grid indices
originW = Tl*[0;0;1];
ox = originW(1);     oy = originW(2);
[oi, oj] = worldToGrid(ox, oy, cellW, cellH, R, C);

N = numel(angles);
for k = 1:N
    ang = angles(k);
    r   = ranges(k);

    % Compute hit point or max‑range point in WORLD frame
    if isfinite(r)
        ptL  = [ r*cos(ang);  r*sin(ang); 1 ];
    else   % treat as miss – project a ray out to rangeMax (same as max(ranges))
        ptL  = [ rangeMax*cos(ang); rangeMax*sin(ang); 1 ];
    end
    ptW = Tl*ptL;

    [ti, tj] = worldToGrid(ptW(1), ptW(2), cellW, cellH, R, C);

    % Discretise the ray with Bresenham from (oi,oj) → (ti,tj)
    [rayI, rayJ] = bresenhamLine(oi, oj, ti, tj);
    if isempty(rayI), continue; end   % completely outside map

    % All points except the last are free space
    freeIdx   = 1:max(1,length(rayI)-1);
    occIdxEnd = length(rayI);      % last index

    % Update log‑odds for free cells
    linIndFree = sub2ind([R C], rayI(freeIdx), rayJ(freeIdx));
    logOdds(linIndFree) = logOdds(linIndFree) + l_free;

    % Update occupied cell (only if this was a real hit)
    if isfinite(r)
        linOcc = sub2ind([R C], rayI(occIdxEnd), rayJ(occIdxEnd));
        logOdds(linOcc) = logOdds(linOcc) + l_occ;
    end
end

end % updateLaserBeamGrid

%% ------------------------------------------------------------------
function [i,j] = worldToGrid(x,y,cellW,cellH,R,C)
% Convert world‑coords to grid indices (row=i, col=j)
    j = floor(x/cellW) + 1;
    i = R - floor(y/cellH);   % y up → row downwards
    % clamp to [1..R/C] to avoid index errors
    i = min(max(i,1),R);
    j = min(max(j,1),C);
end

%% ------------------------------------------------------------------
function [I,J] = bresenhamLine(i0,j0,i1,j1)
% Pure‑integer Bresenham between two grid points (clip at map border)
    di = abs(i1-i0);    dj = abs(j1-j0);
    si = sign(i1-i0);   sj = sign(j1-j0);
    I = []; J = [];
    if di==0 && dj==0, return; end
    if di > dj
        err = di/2;
        j = j0;
        for i = i0:si:i1
            I(end+1)=i; J(end+1)=j; %#ok<AGROW>
            err = err - dj;
            if err < 0
                j   = j + sj;
                err = err + di;
            end
        end
    else
        err = dj/2;
        i = i0;
        for j = j0:sj:j1
            I(end+1)=i; J(end+1)=j; %#ok<AGROW>
            err = err - di;
            if err < 0
                i   = i + si;
                err = err + dj;
            end
        end
    end
end
