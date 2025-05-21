function [steer, cte, status] = purePursuitSegmented_diagnostic( ...
        q, L, Ld_line, Ld_turn, ...
        path, segID, isTurnLink, guidedPoints, gamma_max, gamma_min)
    % DIAGNOSTIC VERSION OF PURSUIT FUNCTION
    x = q(1); y = q(2); th = q(3);
    status = 0;
    
    % persist variables
    persistent curSeg prevDist nxSeg nxprevDist cycleDetectionCounter cycleDetectionPos cycleLockSwitch
    if isempty(curSeg)
        curSeg = segID(1);
        prevDist = Inf;
        nxSeg = min(curSeg+1, max(segID));
        nxprevDist = Inf;
        cycleDetectionCounter = 0;
        cycleDetectionPos = [x y];
        cycleLockSwitch = false;
    end
    
    % 1) closest waypoint & cte
    diffs = path - [x y];
    d2 = diffs(:,1).^2 + diffs(:,2).^2;
    [~, idx] = min(d2);
    cte = sqrt(d2(idx));
    
    % Log current segment and position for debugging
    disp(['Current Position: [', num2str(x), ', ', num2str(y), '], Segment: ', num2str(curSeg)]);
    disp(['Current Guided Point: [', num2str(guidedPoints(curSeg,1)), ', ', num2str(guidedPoints(curSeg,2)), ']']);
    disp(['Distance to Guided Point: ', num2str(norm([x y] - guidedPoints(curSeg,:)))]);
    
    % CYCLE DETECTION: Check if robot is looping around a small area
    if norm([x y] - cycleDetectionPos) < 0.5  % Robot is near a previously visited position
        cycleDetectionCounter = cycleDetectionCounter + 1;
        if cycleDetectionCounter > 50  % If this happens many times in a row
            disp('*** CYCLE DETECTED! Robot is stuck in a loop ***');
            disp(['Stuck between segment ', num2str(curSeg), ' and next segment']);
            cycleLockSwitch = true;  % Emergency mode to escape cycle
            % Force progress to next segment
            if curSeg < max(segID)
                disp('*** EMERGENCY: Forcing progress to next segment ***');
                curSeg = curSeg + 1;
                prevDist = Inf;
                nxSeg = min(curSeg+1, max(segID));
                nxprevDist = Inf;
            end
            cycleDetectionCounter = 0;
        end
    else
        % Update position for cycle detection
        cycleDetectionPos = [x y];
        cycleDetectionCounter = 0;
    end
    
    % 2) segment detection with detailed logging
    newSeg = segID(idx);
    if newSeg ~= curSeg
        disp(['*** SEGMENT CHANGE DETECTED: ', num2str(curSeg), ' -> ', num2str(newSeg)]);
        % Check if we're jumping backward
        if newSeg < curSeg
            disp('WARNING: Moving BACKWARD in segments!');
        end
        curSeg = newSeg;
        prevDist = Inf;
        nxSeg = min(curSeg+1, max(segID));
        nxprevDist = Inf;
    end
    
    % 3) distances to current & next guided points with detailed logging
    gp = guidedPoints(curSeg, :);
    nxgp = guidedPoints(nxSeg, :);
    gpDist = norm([x y] - gp);
    nxgpDist = norm([x y] - nxgp);
    
    disp(['Distance to current guided point: ', num2str(gpDist)]);
    disp(['Distance to next guided point: ', num2str(nxgpDist)]);
    disp(['Previous distance to current guided point: ', num2str(prevDist)]);
    
    if gpDist > prevDist
        disp('MOVING AWAY from current guided point');
    else
        disp('Moving TOWARD current guided point');
    end
    
    % 4) override decisions with diagnostic output
    lastSeg = max(segID);
    end_point = [guidedPoints(end,1), guidedPoints(end,2)];
    
    % Special handling if we detected a cycle
    if cycleLockSwitch
        % CYCLE ESCAPE MODE: Force direct movement to next guided point
        nextSegPoint = guidedPoints(min(curSeg+1, lastSeg), :);
        vecAng = atan2(nextSegPoint(2)-y, nextSegPoint(1)-x);
        steer = wrapToPi(vecAng - th);
        steer = max(min(steer, gamma_max), gamma_min);
        status = 2;  % Special status for cycle escape
        disp('*** CYCLE ESCAPE MODE: Direct steering to next guided point ***');
        
        % Reset cycle lock after some time
        if norm([x y] - nextSegPoint) < 2.0
            cycleLockSwitch = false;
            disp('*** CYCLE ESCAPE MODE DEACTIVATED ***');
        end
    % Regular logic with detailed logging
    elseif gpDist > prevDist && gpDist > 10 && nxgpDist > nxprevDist && curSeg < lastSeg
        % Original override condition 
        vecAng = atan2(gp(2)-y, gp(1)-x);
        steer = wrapToPi(vecAng - th);
        steer = max(min(steer, gamma_max), gamma_min);
        status = 1;
        disp('OVERRIDE MODE: Steering directly to guided point');
    elseif curSeg == lastSeg && gpDist > prevDist
        % Last segment condition
        vecAng = atan2(end_point(2)-y, end_point(1)-x);
        steer = wrapToPi(vecAng - th);
        steer = max(min(steer, gamma_max), gamma_min);
        status = 1;
        disp('LAST SEGMENT: Steering directly to end point');
    else
        % Regular pure pursuit
        if isTurnLink(curSeg) && curSeg ~= lastSeg
            Ld = Ld_turn;
            disp('Using TURN lookahead distance');
        else
            Ld = Ld_line;
            disp('Using LINE lookahead distance');
        end
        
        % find segment bounds with diagnostic output
        firstIdx = idx;
        while firstIdx>1 && segID(firstIdx-1)==curSeg
            firstIdx = firstIdx-1;
        end
        lastIdx = idx;
        while lastIdx<size(segID,1) && segID(lastIdx+1)==curSeg
            lastIdx = lastIdx+1;
        end
        
        disp(['Segment bounds: ', num2str(firstIdx), ' to ', num2str(lastIdx)]);
        
        % arc‐length lookahead
        s = 0; goalIdx = idx;
        while goalIdx<lastIdx && s < Ld
            s = s + norm(path(goalIdx+1,:) - path(goalIdx,:));
            goalIdx = goalIdx + 1;
        end
        goal = path(goalIdx,:);
        
        disp(['Pure pursuit target: [', num2str(goal(1)), ', ', num2str(goal(2)), ']']);
        
        % compute steering
        dx = goal(1)-x; dy = goal(2)-y;
        local_y = -sin(th)*dx + cos(th)*dy;
        curvature = 2 * local_y / (Ld^2);
        steer = atan(L * curvature);
        steer = max(min(steer, gamma_max), gamma_min);
        
        disp(['Pure pursuit steering: ', num2str(rad2deg(steer)), ' degrees']);
    end
    
    % 6) update previous distances
    prevDist = gpDist;
    nxprevDist = nxgpDist;
    
    % Final steering output
    disp(['Final steering command: ', num2str(rad2deg(steer)), ' degrees']);
    disp('------------------------');
end