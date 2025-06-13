% nodes2Waypoints  Convert optimal node route → dense way‑points
%
%   [wps,segId,isTurn] = nodes2Waypoints(route,XY,Rmin,rowW,varargin)
%
%   route  – 1×M list of node indices from TSP (including start & end).
%   XY     – (2*N+2)×2 node coordinates.
%   Rmin   – minimum turning radius [m] (from vehicle).
%   rowW   – row spacing [m].
%
%   Name‑value pairs:
%     'StepStraight'  spacing for straight segments   (def 0.5 m)
%     'StepArc'       spacing for arc samples         (def 0.25 m)
%
%   OUTPUTS
%     wps      – Px2 list of [x y] way‑points.
%     segId    – Px1 integer: which original link each wp belongs to.
%     isTurn   – M‑1 logical: true when link is a headland turn.
%
%   ------------------------------------------------------------------
%   The turning primitive is generated with Dubins curves (LSL / RSR)
%   depending on TOP/BOTTOM headland.  For π‑turn the path is a half‑
%   circle joined with straight bridging segment; for Ω‑turn we fall
%   back to Dubins RLR / LRL.
%   ------------------------------------------------------------------
function [wps,segId,isTurn] = nodes2Waypoints(route,XY,Rmin,rowW,varargin)

p = inputParser; p.addParameter('StepStraight',0.5); p.addParameter('StepArc',0.25);
p.parse(varargin{:}); sStep = p.Results.StepStraight; aStep=p.Results.StepArc;

numLinks   = numel(route)-1;
wps        = [];
segId      = [];
isTurn     = false(numLinks,1);

N = (size(XY,1)-2)/2;     % number of rows
for k = 1:numLinks
    i = route(k); j = route(k+1);
    p1 = XY(i,:);  p2 = XY(j,:);
    % classify link type -------------------------------------------------
    if k==1 || k==numLinks
        isTurn(k)=true;                % start or finish arc
    else
        paired  = ( (i>=2 && i<=N+1 && j-i==N) || (j>=2 && j<=N+1 && i-j==N) );
        sameBot = all([i j]>=2   & [i j]<=N+1);
        sameTop = all([i j]>=N+2 & [i j]<=2*N+1);
        if ~paired && (sameBot||sameTop)
            isTurn(k)=true;            % headland connection
        end
    end

    % generate samples ---------------------------------------------------
    if ~isTurn(k)   % straight row traversal
        nPts = max(2,ceil(norm(p2-p1)/sStep));
        xs   = linspace(p1(1),p2(1),nPts)';
        ys   = linspace(p1(2),p2(2),nPts)';
        pts  = [xs,ys];
    else            % turn – use Dubins primitive
        % heading conventions: bottom rows run +X, top rows run −X
        if i<=N+1, th0 = 0; else, th0 = pi; end
        if j<=N+1, th1 = 0; else, th1 = pi; end
        dub = dubins_core([p1,th0],[p2,th1],Rmin);
        pts = dubins_path_sample_many(dub,aStep);
        pts = pts(:,1:2); pts(1,:)=[]; % drop duplicate first point
    end
    wps   = [wps;   pts];
    segId = [segId; k*ones(size(pts,1),1)];
end
end
