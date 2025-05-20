function [steer, cte] = purePursuitSegmented(q, L, Ld, path, segID)
% q = [x; y; theta; ...]   path = Nx2 [x y]   segID = Nx1 segment index
    x = q(1);  y = q(2);  th = q(3);

    % closest point
    d2 = (path(:,1)-x).^2 + (path(:,2)-y).^2;
    [~, idx] = min(d2);
    curSeg   = segID(idx);

    % segment bounds
    firstIdx = idx;
    while firstIdx>1     && segID(firstIdx-1)==curSeg, firstIdx=firstIdx-1; end
    lastIdx  = idx;
    while lastIdx<size(segID,1) && segID(lastIdx+1)==curSeg, lastIdx=lastIdx+1; end

    % bounded look-ahead
    s = 0; goalIdx = idx;
    while goalIdx<lastIdx && s<Ld
        s = s + norm(path(goalIdx+1,:)-path(goalIdx,:));
        goalIdx = goalIdx + 1;
    end
    goal = path(goalIdx,:);

    % vehicle-frame coordinates
    dx = goal(1)-x;  dy = goal(2)-y;
    local_x =  cos(th)*dx + sin(th)*dy;
    local_y = -sin(th)*dx + cos(th)*dy;

    % curvature & steering
    curvature = 2*local_y / (Ld^2);
    steer     = atan(L*curvature);
    steer     = max(min(steer, deg2rad(45)), -deg2rad(45));

    % cross-track error to closest point
    cte = sqrt(d2(idx));
end
