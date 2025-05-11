<<<<<<< HEAD
function [steer_angle, cross_track_error] = purePursuitController(q, L, Ld, path)
    % Extract current state
    x = q(1);
    y = q(2);
    theta = q(3);

    % Find the closest point on the path
    distances = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
    [~, closest_idx] = min(distances);

    % Lookahead goal point: find the first point that is at least Ld away
    goal_idx = closest_idx;
    while goal_idx < size(path,1) && norm([x y] - path(goal_idx,:)) < Ld
        goal_idx = goal_idx + 1;
    end
    
    % If end of path is reached, just use the last point
    if goal_idx >= size(path,1)
        goal_idx = size(path,1);
    end

    goal_point = path(goal_idx, :);

    % Transform goal point to vehicle coordinate frame
    dx = goal_point(1) - x;
    dy = goal_point(2) - y;
    local_x =  cos(theta)*dx + sin(theta)*dy;
    local_y = -sin(theta)*dx + cos(theta)*dy;

    % Compute curvature and steering angle
    curvature = 2 * local_y / (Ld^2);
    steer_angle = atan(L * curvature);

    % Cross-track error: distance to the closest path point
    cross_track_error = sqrt((x - path(closest_idx,1))^2 + (y - path(closest_idx,2))^2);

    % Enforce steering limits (±45 deg in radians)
    steer_angle = max(min(steer_angle, deg2rad(45)), -deg2rad(45));
end
=======
function [steer_angle, cross_track_error] = purePursuitController(q, L, Ld, path)
    % Extract current state
    x = q(1);
    y = q(2);
    theta = q(3);

    % Find the closest point on the path
    distances = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
    [~, closest_idx] = min(distances);

    % Lookahead goal point: find the first point that is at least Ld away
    goal_idx = closest_idx;
    while goal_idx < size(path,1) && norm([x y] - path(goal_idx,:)) < Ld
        goal_idx = goal_idx + 1;
    end
    
    % If end of path is reached, just use the last point
    if goal_idx >= size(path,1)
        goal_idx = size(path,1);
    end

    goal_point = path(goal_idx, :);

    % Transform goal point to vehicle coordinate frame
    dx = goal_point(1) - x;
    dy = goal_point(2) - y;
    local_x =  cos(theta)*dx + sin(theta)*dy;
    local_y = -sin(theta)*dx + cos(theta)*dy;

    % Compute curvature and steering angle
    curvature = 2 * local_y / (Ld^2);
    steer_angle = atan(L * curvature);

    % Cross-track error: distance to the closest path point
    cross_track_error = sqrt((x - path(closest_idx,1))^2 + (y - path(closest_idx,2))^2);

    % Enforce steering limits (±45 deg in radians)
    steer_angle = max(min(steer_angle, deg2rad(45)), -deg2rad(45));
end
>>>>>>> a408ab02c216c843a09b2e49d0ba7420fb00696d
