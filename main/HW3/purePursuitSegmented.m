function [steer, cte, status] = purePursuitSegmented( ...
        q, L, Ld_line, Ld_turn, ...
        path, segID, isTurnLink, guidedPoints, gamma_max, gamma_min, ~)
    % SEGMENT-BOUNDED + DUAL-GUIDED-POINT OVERRIDE PURE PURSUIT
    % Enhanced to ensure robot moves towards end point in final segments
    
    x  = q(1);  y  = q(2);  th = q(3);
    status = 0;
    segahead = 3; %default lookahaed segment
    
    % persist current & next segment and their previous guided-point distances
    persistent curSeg prevDist nxSeg nxprevDist curPathIdx
    if isempty(curSeg)
        curSeg     = segID(1);
        prevDist   = Inf;
        nxSeg      = min(curSeg+1, max(segID));
        nxprevDist = Inf;
        curPathIdx = 1;  % Initialize current path index
    end
    
    % 1) Find closest waypoint that is ahead and within allowed segments
    min_dist = inf;
    idx = curPathIdx;  % Start from current path index
    valid_found = false;
    
    % Define segment range to consider
    min_seg_to_consider = curSeg;
    max_seg_to_consider = min(curSeg + segahead, max(segID)); % Look ahead up to 3 segments
    
    % First, find all valid waypoints in the allowed segment range
    valid_indices = find(segID >= min_seg_to_consider & segID <= max_seg_to_consider);
    
    if ~isempty(valid_indices)
        % Calculate distances to all valid waypoints
        valid_dists = zeros(length(valid_indices), 1);
        for i = 1:length(valid_indices)
            wp_idx = valid_indices(i);
            valid_dists(i) = norm([path(wp_idx,1) - x, path(wp_idx,2) - y]);
        end
        
        % Find the closest valid waypoint
        [min_dist, min_idx] = min(valid_dists);
        idx = valid_indices(min_idx);
        valid_found = true;
        
        % Update current path index to ensure forward progress
        % Only move forward along the path, never backward
        if idx >= curPathIdx
            curPathIdx = idx;
        else
            % If closest point is behind, find the next closest that's ahead
            ahead_indices = valid_indices(valid_indices >= curPathIdx);
            if ~isempty(ahead_indices)
                ahead_dists = zeros(length(ahead_indices), 1);
                for i = 1:length(ahead_indices)
                    wp_idx = ahead_indices(i);
                    ahead_dists(i) = norm([path(wp_idx,1) - x, path(wp_idx,2) - y]);
                end
                [~, min_ahead_idx] = min(ahead_dists);
                idx = ahead_indices(min_ahead_idx);
                curPathIdx = idx;
            else
                % Use current path index if no ahead points found
                idx = curPathIdx;
            end
        end
    end
    
    % Fallback: if no valid waypoint found, use global closest within segment range
    if ~valid_found
        diffs = path - [x y];
        d2 = diffs(:,1).^2 + diffs(:,2).^2;
        
        % Filter by segment range
        valid_segments = (segID >= min_seg_to_consider & segID <= max_seg_to_consider);
        valid_indices = find(valid_segments);
        
        if ~isempty(valid_indices)
            [~, min_idx] = min(d2(valid_indices));
            idx = valid_indices(min_idx);
            curPathIdx = max(curPathIdx, idx); % Ensure forward progress
        else
            % Ultimate fallback
            [~, idx] = min(d2);
            curPathIdx = max(curPathIdx, idx);
        end
    end

    cte = norm([x y] - path(idx,:));

    
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
    
    % 4) prepare end point and last segment info
    lastSeg = max(segID);
    end_point = [guidedPoints(end,1), guidedPoints(end,2)]; % The end point coordinates [x, y]
    
    
    % 5) Steering logic with enhanced end-point convergence
    
    if gpDist > prevDist && nxgpDist > nxprevDist && gpDist > 10 && curSeg < lastSeg
        % steer directly at current guided point
        vecAng = atan2(gp(2)-y, gp(1)-x);
        steer  = wrapToPi(vecAng - th);
        steer  = max(min(steer, gamma_max), gamma_min);
        status = 1;
    elseif curSeg == lastSeg && nxgpDist > 15 && gpDist > prevDist && (q(4) < gamma_max && q(4) > 2*pi - gamma_max)
        % steer directly to end point (prevent the robot get lost in the
        % last seg and loop over)
        vecAng = atan2(end_point(2)-y, end_point(1)-x);
        steer  = wrapToPi(vecAng - th);
        steer  = max(min(steer, gamma_max), gamma_min);
        status = 1;
    else
        % 6) regular segment‐bounded pure pursuit
        % choose Ld
        if isTurnLink(curSeg) && curSeg ~= lastSeg && curSeg ~= segID(1)
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
    end
    
    % 7) update previous distances
    prevDist   = gpDist;
    nxprevDist = nxgpDist;
end