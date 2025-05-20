function poses = reedsSheppPathSample(q0, q1, Rmin, step)
    % Transform to local frame
    dx = q1(1) - q0(1);
    dy = q1(2) - q0(2);
    dtheta = q1(3) - q0(3);

    ct = cos(q0(3));
    st = sin(q0(3));

    x = (ct * dx + st * dy) / Rmin;
    y = (-st * dx + ct * dy) / Rmin;
    phi = mod2pi(dtheta);

    % Find optimal path
    [path, type] = bestReedsSheppPath(x, y, phi);

    % Path sampling
    totalLength = sum(abs(path));
    nPts = ceil(totalLength / (step / Rmin));
    poses = zeros(nPts, 3);

    q = [0, 0, 0];
    idx = 1;

    for s = linspace(0, totalLength, nPts)
        poses(idx,:) = integrateSegment(q, path, type, s);
        idx = idx + 1;
    end

    % Transform to world frame
    T = @(pt) [ct -st; st ct] * pt(1:2)' + q0(1:2)';
    for i = 1:size(poses,1)
        p = poses(i,1:2);
        poses(i,1:2) = T([p(1), p(2)]);
        poses(i,3) = mod2pi(poses(i,3) + q0(3));
    end
end

function [bestPath, bestType] = bestReedsSheppPath(x, y, phi)
    types = {@LSLrev, @RSLrev, @LSRrev, @RSRrev, ...
         @LSL, @LSR, @RSL, @RSR, ...
         @LRL, @RLR};
    bestLength = Inf;
    bestPath = [];
    bestType = [];
    for k = 1:length(types)
        p = types{k}(x, y, phi);
        if ~isempty(p)
            len = sum(abs(p));
            if len < bestLength
                bestLength = len;
                bestPath = p;
                bestType = k;
            end
        end
    end
end

function pose = integrateSegment(q0, path, type, s)
    len = 0;
    q = q0;
    for i = 1:length(path)
        L = abs(path(i));
        dir = sign(path(i));
        d = min(s - len, L);
        if d < 0, break; end
        switch typeSegment(type,i)
            case 1  % left
                delta = dir * d;
                q(1) = q(1) + sin(q(3) + delta) - sin(q(3));
                q(2) = q(2) - cos(q(3) + delta) + cos(q(3));
                q(3) = q(3) + delta;
            case 2  % right
                delta = dir * d;
                q(1) = q(1) - sin(q(3) - delta) + sin(q(3));
                q(2) = q(2) + cos(q(3) - delta) - cos(q(3));
                q(3) = q(3) - delta;
            case 3  % straight
                delta = dir * d;
                q(1) = q(1) + delta * cos(q(3));
                q(2) = q(2) + delta * sin(q(3));
        end
        len = len + L;
    end
    pose = q;
end

function code = typeSegment(t, i)
    names = { [1 3 1], [1 3 2], [2 3 1], [2 3 2], ...
              [2 1 2], [1 2 1], [1 3 1], [1 3 2], [2 3 1], [2 3 2] };
    code = names{t}(i);
end

function a = mod2pi(theta)
    a = mod(theta + pi, 2*pi) - pi;
end

% Segment generators
function path = LSL(x, y, phi)
    [rho, theta] = cart2pol(x - sin(phi), y - 1 + cos(phi));
    u = rho; t = theta; v = mod2pi(phi - t);
    path = [t, u, v];
end

function path = LSR(x, y, phi)
    [rho, theta] = cart2pol(x + sin(phi), y - 1 - cos(phi));
    if rho^2 < 4, path = []; return; end
    u = sqrt(rho^2 - 4);
    t = mod2pi(theta + atan2(2, u));
    v = mod2pi(t - phi);
    path = [t, u, v];
end

function path = RSL(x, y, phi)
    path = LSR(-x, y, -phi);
end

function path = RSR(x, y, phi)
    path = LSL(-x, y, -phi);
end

function path = LRL(x, y, phi)
    xi = x - sin(phi); eta = y - 1 + cos(phi);
    [rho, theta] = cart2pol(xi, eta);
    if rho > 4, path = []; return; end
    A = acos(rho / 4);
    t = mod2pi(theta + pi/2 + A);
    u = mod2pi(pi - 2*A);
    v = mod2pi(phi - t - u);
    path = [t, u, v];
end

function path = RLR(x, y, phi)
    path = LRL(-x, y, -phi);
end

function path = LSLrev(x, y, phi)
    p = LSL(-x, -y, -phi);
    path = -fliplr(p);
end

function path = LSRrev(x, y, phi)
    p = LSR(-x, -y, -phi);
    path = -fliplr(p);
end

function path = RSLrev(x, y, phi)
    p = RSL(-x, -y, -phi);
    path = -fliplr(p);
end

function path = RSRrev(x, y, phi)
    p = RSR(-x, -y, -phi);
    path = -fliplr(p);
end
