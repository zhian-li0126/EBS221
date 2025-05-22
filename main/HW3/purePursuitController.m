function [steer_angle, cross_track_error, status] = purePursuitController( ...
        q, L, Ld, path, waypointSegment, guidedPoints, gamma_max, gamma_min)
    % PUREPURSUITCONTROLLER with guided-point override
    %  q               : [x; y; theta; ...]
    %  L               : wheelbase
    %  Ld              : nominal lookahead
    %  path            : M×2 [x y] waypoints
    %  waypointSegment : M×1 segment index per waypoint
    %  guidedPoints    : K×2 guided [x y] per segment

    persistent prevDist prevSeg
    if isempty(prevDist)
        prevDist = Inf;
        prevSeg  = -1;
    end

    % 1) extract state
    x     = q(1);
    y     = q(2);
    theta = q(3);
    status = 0;

    % 2) find closest waypoint & segment
    diffs = path - [x y];
    d2    = diffs(:,1).^2 + diffs(:,2).^2;
    [~, idx] = min(d2);
    seg = waypointSegment(idx);

    % if we entered a new segment, reset the guided‐point memory
    if seg ~= prevSeg
        prevSeg  = seg;
        prevDist = Inf;
    end

    % 3) distance to this segment’s guided point
    gp      = guidedPoints(seg, :);        % [gx gy]
    currDist = norm([x y] - gp);

    % 4) override if we are moving away from the guided point
    if currDist > prevDist
        % direct‐at‐guided‐point steering
        angToGP   = atan2(gp(2)-y, gp(1)-x);
        steer_angle = wrapToPi(angToGP - theta);
        steer_angle = max(min(steer_angle, gamma_max, gamma_min));
    else
        % 5) regular pure pursuit
        %    (1) find lookahead goal
        goal_idx = idx;
        while goal_idx < size(path,1) && ...
              norm([x y] - path(goal_idx,:)) < Ld
            goal_idx = goal_idx + 1;
        end
        if goal_idx > size(path,1)
            goal_idx = size(path,1);
        end
        goal = path(goal_idx, :);

        %    (2) compute steering to goal
        dx      = goal(1) - x;
        dy      = goal(2) - y;
        local_x =  cos(theta)*dx + sin(theta)*dy;
        local_y = -sin(theta)*dx + cos(theta)*dy;
        curvature   = 2 * local_y / (Ld^2);
        steer_angle = atan(L * curvature);
        steer_angle = max(min(steer_angle, deg2rad(45)), -deg2rad(45));
    end

    % 6) cross-track error as before
    cross_track_error = sqrt(d2(idx));

    % 7) update memory
    prevDist = currDist;
end
