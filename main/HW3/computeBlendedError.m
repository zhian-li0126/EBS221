function [cte_path, cte_gp, cte_blend] = computeBlendedError( ...
        q, path, segID, guidedPoints, isTurnLink, alpha, status)
% COMPUTEBLENDEDError  Combined cross‐error for pure pursuit + guided override
%   q             : [x; y; theta]
%   path          : M×2 waypoints
%   segID         : M×1 segment index per waypoint
%   guidedPoints  : K×2 guided point per segment
%   isTurnLink(k) : true if segment k is a turn
%   alpha         : blending factor in [0,1]

    x = q(1);  y = q(2);

    % 1) find current segment from nearest waypoint
    d2 = sum((path - [x y]).^2,2);
    [~, idx] = min(d2);
    curSeg = segID(idx);

    % 2) extract only the waypoints of this segment
    segIdxs = find(segID==curSeg);
    segPts  = path(segIdxs,:);

    % 3) cte_path = minimum distance from (x,y) to any segment‐polyline
    cte_path = Inf;
    for i = 1:size(segPts,1)-1
        A = segPts(i,:);
        B = segPts(i+1,:);
        v = B - A;
        w = [x y] - A;
        t = max(0,min(1, dot(w,v)/dot(v,v)));
        proj = A + t*v;
        cte_path = min(cte_path, norm([x y]-proj));
    end

    % 4) cte_gp = distance to guided point of this segment
    if status
        gp      = guidedPoints(curSeg,:);
        cte_gp  = norm([x y] - gp);
    else
        cte_gp  = 0;
    end

    % 6) blended error
    cte_blend = alpha * cte_path + (1 - alpha) * cte_gp;
end
