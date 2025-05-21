function [steer, cte, status] = purePursuitSegmented( ...
        q, L, Ld_line, Ld_turn, ...
        path, segID, isTurnLink, guidedPoints, gamma_max, gamma_min)
    % SEGMENT-BOUNDED + DUAL-GUIDED-POINT OVERRIDE PURE PURSUIT
    x  = q(1);  y  = q(2);  th = q(3);
    status = 0;

    % persist current & next segment and their previous guided-point distances
    persistent curSeg prevDist nxSeg nxprevDist
    if isempty(curSeg)
        curSeg     = segID(1);
        prevDist   = Inf;
        nxSeg      = min(curSeg+1, max(segID));
        nxprevDist = Inf;
    end

    % 1) closest waypoint & cte
    diffs = path - [x y];
    d2    = diffs(:,1).^2 + diffs(:,2).^2;
    [~, idx] = min(d2);
    cte      = sqrt(d2(idx));

    % 2) detect segment change
    newSeg = segID(idx);
    if newSeg ~= curSeg
        curSeg     = newSeg;
        prevDist   = Inf;
        nxSeg      = min(curSeg+1, max(segID));
        nxprevDist = Inf;
    end

    % 3) distances to current & next guided points
    gp      = guidedPoints(curSeg, :);   % current guided point
    nxgp    = guidedPoints(nxSeg,  :);   % next segment's guided point
    gpDist  = norm([x y] - gp);
    nxgpDist= norm([x y] - nxgp);

    % 4) override if moving away from both guided points
    lastSeg = max(segID);
    end_point = [guidedPoints(end,1), guidedPoints(end,2)]; % The end point coordinates [x, y]
    
    if gpDist > prevDist && gpDist > 5 && nxgpDist > nxprevDist && curSeg < lastSeg
        % steer directly at current guided point
        vecAng = atan2(gp(2)-y, gp(1)-x);
        steer  = wrapToPi(vecAng - th);
        steer  = max(min(steer, gamma_max), gamma_min);
        status = 1;      
    
    elseif curSeg == lastSeg && gpDist > prevDist && nxgpDist < nxprevDist %prevent the robot get lost in the last segment
        % If in the last segment AND:
        % 1) Moving away from the last guided point, PLUS,
        % 2) Getting closer to the end point
        % Then steer directly to the end point
        vecAng = atan2(end_point(2)-y, end_point(1)-x);
        steer  = wrapToPi(vecAng - th);
        steer  = max(min(steer, gamma_max), gamma_min);
        status = 1;
    else
        % 5) regular segment‐bounded pure pursuit
        % choose Ld
        if isTurnLink(curSeg) && curSeg ~= lastSeg
            Ld = Ld_turn;
        else
            Ld = Ld_line;
        end

        % find segment bounds
        firstIdx = idx;
        while firstIdx>1 && segID(firstIdx-1)==curSeg
            firstIdx = firstIdx-1;
        end
        lastIdx = idx;
        while lastIdx<size(segID,1) && segID(lastIdx+1)==curSeg
            lastIdx = lastIdx+1;
        end

        % arc‐length lookahead
        s = 0; goalIdx = idx;
        while goalIdx<lastIdx && s < Ld
            s = s + norm(path(goalIdx+1,:) - path(goalIdx,:));
            goalIdx = goalIdx + 1;
        end
        goal = path(goalIdx,:);

        % compute steering
        dx = goal(1)-x; dy = goal(2)-y;
        local_y  = -sin(th)*dx + cos(th)*dy;
        curvature= 2 * local_y / (Ld^2);
        steer    = atan(L * curvature);
        steer    = max(min(steer, gamma_max), gamma_min);
    end  % Added the missing 'end' statement here

    % 6) update previous distances
    prevDist   = gpDist;
    nxprevDist = nxgpDist;
end