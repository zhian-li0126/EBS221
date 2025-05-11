<<<<<<< HEAD
%{

This version now computes DT
corrected v and gamma integration
tau_gamma corrected, added tolerance for tau_gamma and tau_v

%}


function q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v)
    global dt; 
    global DT;

    % Clip desired inputs
    gamma_d = max(min(u(1), umax(1)), umin(1));
    v_d     = max(min(u(2), umax(2)), umin(2));
    
    % Unpack initial state
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);
    epsilon = 10e-12; % tolerance of error

    % Number of integration steps
    N_steps = DT - dt;
    
    for i = 0:N_steps
        % Update steering
        if tau_gamma <= epsilon
            gamma = gamma_d;
        else
            gamma = gamma + dt * (gamma_d - gamma) / tau_gamma;
        end
    
        % Update velocity
        if tau_v <= epsilon
            v = v_d;
        else
            v = v + dt * (v_d - v) / tau_v;
        end
    
        % Bicycle model kinematics
        dx     = v * cos(theta);
        dy     = v * sin(theta);
        dtheta = v * tan(gamma) / L;
    
        % Euler integration
        x     = x + dt * dx;
        y     = y + dt * dy;
        theta = theta + dt * dtheta;
    
        % Optional: clamp gamma and v after each update if needed
    end

    % Wrap theta to [-pi, pi]
    theta = wrapToPi(theta);
    
    % Clamp final state
    x     = max(min(x, Qmax(1)), Qmin(1));
    y     = max(min(y, Qmax(2)), Qmin(2));
    theta = max(min(theta, Qmax(3)), Qmin(3));
    gamma = max(min(gamma, Qmax(4)), Qmin(4));
    v     = max(min(v, Qmax(5)), Qmin(5));
    
    q_next = [x; y; theta; gamma; v];


    % Extract current state
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);

    % Desired inputs
    gamma_d = u(1);  % desired steering angle
    v_d     = u(2);  % desired velocity

    % Saturate desired inputs
    gamma_d = max(min(gamma_d, umax(1)), umin(1));
    v_d     = max(min(v_d, umax(2)), umin(2));

    % First-order dynamics for steering and velocity
    dgamma = (gamma_d - gamma) / tau_gamma;
    dv     = (v_d - v) / tau_v;

    % Bicycle model kinematics
    dx     = v * cos(theta);
    dy     = v * sin(theta);
    dtheta = v * tan(gamma) / L;

    % Euler integration to update pose
    x     = x + dt * dx;
    y     = y + dt * dy;
    theta = theta + dt * dtheta;
    gamma = gamma + dt * dgamma;
    v     = v + dt * dv;

    % Saturate state vector
    x     = max(min(x, Qmax(1)), Qmin(1));
    y     = max(min(y, Qmax(2)), Qmin(2));
    theta = max(min(theta, Qmax(3)), Qmin(3));
    gamma = max(min(gamma, Qmax(4)), Qmin(4));
    v     = max(min(v, Qmax(5)), Qmin(5));

    % Pack next state
    q_next = [x; y; theta; gamma; v];
=======
%{

This version now computes DT
corrected v and gamma integration
tau_gamma corrected, added tolerance for tau_gamma and tau_v

%}


function q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v)
    global dt; 
    global DT;

    % Clip desired inputs
    gamma_d = max(min(u(1), umax(1)), umin(1));
    v_d     = max(min(u(2), umax(2)), umin(2));
    
    % Unpack initial state
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);
    epsilon = 10e-12; % tolerance of error

    % Number of integration steps
    N_steps = DT - dt;
    
    for i = 0:N_steps
        % Update steering
        if tau_gamma <= epsilon
            gamma = gamma_d;
        else
            gamma = gamma + dt * (gamma_d - gamma) / tau_gamma;
        end
    
        % Update velocity
        if tau_v <= epsilon
            v = v_d;
        else
            v = v + dt * (v_d - v) / tau_v;
        end
    
        % Bicycle model kinematics
        dx     = v * cos(theta);
        dy     = v * sin(theta);
        dtheta = v * tan(gamma) / L;
    
        % Euler integration
        x     = x + dt * dx;
        y     = y + dt * dy;
        theta = theta + dt * dtheta;
    
        % Optional: clamp gamma and v after each update if needed
    end

    % Wrap theta to [-pi, pi]
    theta = wrapToPi(theta);
    
    % Clamp final state
    x     = max(min(x, Qmax(1)), Qmin(1));
    y     = max(min(y, Qmax(2)), Qmin(2));
    theta = max(min(theta, Qmax(3)), Qmin(3));
    gamma = max(min(gamma, Qmax(4)), Qmin(4));
    v     = max(min(v, Qmax(5)), Qmin(5));
    
    q_next = [x; y; theta; gamma; v];


    % Extract current state
    x     = q(1);
    y     = q(2);
    theta = q(3);
    gamma = q(4);
    v     = q(5);

    % Desired inputs
    gamma_d = u(1);  % desired steering angle
    v_d     = u(2);  % desired velocity

    % Saturate desired inputs
    gamma_d = max(min(gamma_d, umax(1)), umin(1));
    v_d     = max(min(v_d, umax(2)), umin(2));

    % First-order dynamics for steering and velocity
    dgamma = (gamma_d - gamma) / tau_gamma;
    dv     = (v_d - v) / tau_v;

    % Bicycle model kinematics
    dx     = v * cos(theta);
    dy     = v * sin(theta);
    dtheta = v * tan(gamma) / L;

    % Euler integration to update pose
    x     = x + dt * dx;
    y     = y + dt * dy;
    theta = theta + dt * dtheta;
    gamma = gamma + dt * dgamma;
    v     = v + dt * dv;

    % Saturate state vector
    x     = max(min(x, Qmax(1)), Qmin(1));
    y     = max(min(y, Qmax(2)), Qmin(2));
    theta = max(min(theta, Qmax(3)), Qmin(3));
    gamma = max(min(gamma, Qmax(4)), Qmin(4));
    v     = max(min(v, Qmax(5)), Qmin(5));

    % Pack next state
    q_next = [x; y; theta; gamma; v];
>>>>>>> a408ab02c216c843a09b2e49d0ba7420fb00696d
end