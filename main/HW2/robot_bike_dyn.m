<<<<<<< HEAD
function q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v)
% robot_bike_dyn  Integrates a 5-state bicycle model for DT seconds
%
%   q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, ...
%                           tau_gamma, tau_v)
%
%   The function advances the state from q(k) to q(k+1) by integrating
%   the dynamics with a small time-step dt inside the sampling interval DT.
%
%   GLOBALS (set once in main script):
%       dt  – inner Euler step (e.g. 1 ms)
%       DT  – controller period (e.g. 100 ms)

    % globals 
    global dt
    global DT

    % unpack current state 
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);

    % saturate desired inputs
    gamma_d = min(max(u(1), umin(1)), umax(1));
    v_d     = min(max(u(2), umin(2)), umax(2));

    eps = 1e-12;                  % numerical zero for the time constants

    % fine-step Euler integration over [0, DT) 
    for t = 0 : dt : DT-dt
        % -- steering first-order system --
        if tau_gamma <= eps
            gamma = gamma_d;
        else
            gamma = gamma + dt * (gamma_d - gamma) / tau_gamma;
        end

        % -- speed first-order system --
        if tau_v <= eps
            v = v_d;
        else
            v = v + dt * (v_d - v) / tau_v;
        end

        % -- kinematic bicycle model --
        x     = x + dt * v * cos(theta);
        y     = y + dt * v * sin(theta);
        theta = theta + dt * v * tan(gamma) / L;

        % keep γ and v inside physical limits each sub-step
        gamma = min(max(gamma, Qmin(4)), Qmax(4));
        v     = min(max(v,     Qmin(5)), Qmax(5));
    end

    % clip final state to workspace limits 
    x     = min(max(x,     Qmin(1)), Qmax(1));
    y     = min(max(y,     Qmin(2)), Qmax(2));
    theta = min(max(theta, Qmin(3)), Qmax(3));
    gamma = min(max(gamma, Qmin(4)), Qmax(4));
    v     = min(max(v,     Qmin(5)), Qmax(5));

    % pack output
    q_next = [x; y; theta; gamma; v];
end
=======
function q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v)
% robot_bike_dyn  Integrates a 5-state bicycle model for DT seconds
%
%   q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, ...
%                           tau_gamma, tau_v)
%
%   The function advances the state from q(k) to q(k+1) by integrating
%   the dynamics with a small time-step dt inside the sampling interval DT.
%
%   GLOBALS (set once in main script):
%       dt  – inner Euler step (e.g. 1 ms)
%       DT  – controller period (e.g. 100 ms)

    % globals 
    global dt
    global DT

    % unpack current state 
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);

    % saturate desired inputs
    gamma_d = min(max(u(1), umin(1)), umax(1));
    v_d     = min(max(u(2), umin(2)), umax(2));

    eps = 1e-12;                  % numerical zero for the time constants

    % fine-step Euler integration over [0, DT) 
    for t = 0 : dt : DT-dt
        % -- steering first-order system --
        if tau_gamma <= eps
            gamma = gamma_d;
        else
            gamma = gamma + dt * (gamma_d - gamma) / tau_gamma;
        end

        % -- speed first-order system --
        if tau_v <= eps
            v = v_d;
        else
            v = v + dt * (v_d - v) / tau_v;
        end

        % -- kinematic bicycle model --
        x     = x + dt * v * cos(theta);
        y     = y + dt * v * sin(theta);
        theta = theta + dt * v * tan(gamma) / L;

        % keep γ and v inside physical limits each sub-step
        gamma = min(max(gamma, Qmin(4)), Qmax(4));
        v     = min(max(v,     Qmin(5)), Qmax(5));
    end

    % clip final state to workspace limits 
    x     = min(max(x,     Qmin(1)), Qmax(1));
    y     = min(max(y,     Qmin(2)), Qmax(2));
    theta = min(max(theta, Qmin(3)), Qmax(3));
    gamma = min(max(gamma, Qmin(4)), Qmax(4));
    v     = min(max(v,     Qmin(5)), Qmax(5));

    % pack output
    q_next = [x; y; theta; gamma; v];
end
>>>>>>> a408ab02c216c843a09b2e49d0ba7420fb00696d
