% purePursuitSegmented  PP controller aware of row vs headland links
%
%   [delta,cte,status] = purePursuitSegmented(q,L,Ld_line,Ld_turn,
%                                   wps,segId,isTurn,guides,
%                                   gamma_max,gamma_min,doPlot)
%
%   The function selects an appropriate look‑ahead distance depending on
%   whether the current segment is a straight row (Ld_line) or a headland
%   manoeuvre (Ld_turn). It also supports an optional guided‑point override
%   that keeps the tractor centred when passing through U‑turn apices.
%
%   INPUTS
%     q         – 5×1 state [x y θ γ v]
%     L         – wheel‑base [m]
%     Ld_line   – look‑ahead in straight lines
%     Ld_turn   – look‑ahead during turns
%     wps       – P×2 full waypoint list
%     segId     – P×1 segment id for each wp
%     isTurn    – logical(K) flags (K = num links)
%     guides    – K×2 guided points (apex / mid‑rows)
%     gamma_max – steering saturation [rad]
%     gamma_min – steering saturation [rad]
%     doPlot    – (optional) debug plot flag
%
%   OUTPUTS
%     delta     – desired steering angle [rad]
%     cte       – cross‑track error [m]
%     status    – 1 = guided‑point override active, 0 = off
%
%   ------------------------------------------------------------------


function [delta,cte,status] = purePursuitSegmented(q,L,Ld_line,Ld_turn,...
                                   wps,segId,isTurn,guides,...
                                   gamma_max,gamma_min,doPlot)
if nargin<11, doPlot = false; end

pos   = q(1:2).';   % row vector
theta = q(3);

% 1) find closest waypoint & its segment
[d2,idx] = min(sum((wps - pos).^2,2));
seg      = segId(idx);

% 2) choose look‑ahead and target point
if isTurn(seg)
    Ld = Ld_turn;
else
    Ld = Ld_line;
end

% travel forward along wps until cum distance ≥ Ld
j = idx;
acc = 0;
while j<size(wps,1)
    ds = norm(wps(j+1,:) - wps(j,:));
    acc = acc + ds;
    j = j + 1;
    if acc>=Ld, break; end
end

if j>=size(wps,1)
    tgt = wps(end,:);
else
    tgt = wps(j,:);
end

% optional guided‑point override in apex
status = 0;
if isTurn(seg)
    gp = guides(seg,:);
    if norm(pos-gp)<Ld_turn      % close to apex
        tgt   = gp;
        status = 1;
    end
end

% 3) classic PP geometry in vehicle frame
R = [cos(theta) sin(theta); -sin(theta) cos(theta)];
pt = R*(tgt - pos).';  % 2×1 in body frame
ex = pt(1); ey = pt(2);
if ex < 1e-3    % guard against numeric trouble
    curvature = 0;
else
    curvature = 2*ey/(Ld^2);
end

delta = atan(L*curvature);

% saturate
delta = min(max(delta,gamma_min),gamma_max);

% simple CTE (signed lateral error to last closest wp)
lastWP = wps(idx,:);
vec = R*(lastWP-pos).';
cte = vec(2);

% debug plot
if doPlot
    plot(tgt(1),tgt(2),'gx');
end
end
