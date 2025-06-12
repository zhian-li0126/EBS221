%%

clear; close all; clc;

%% ----- Initialize all parameters -----

% Parameters of the vehicle
L = 3;                      % wheelbase, 3 meters
W = 2.5;                    % width of the vehicle, 2 meters
gamma_limit = deg2rad(55);  % steering limit, 55 deg
gamma_v = 0.1;              % steering lag, 0.1 sec
tau_v = 0.1;                % velocity lag, 0.1 sec
v_ref = 1;                  % placeholder speed, may adjust dependent on the project

% initial state of vehicle [x, y, theta, gamma, v]
q = [0, 0, pi/2, 0, v_ref]; % start from (0,0), facing north, define the v_ref


% Parameters of the Lidar
angleSpan = deg2rad(180);   % max angle range, 180 deg
angleStep = deg2rad(0.125); % angular resolution, 0.125 deg
rangeMax = 20;              % max Lidar range, 20 meters


%% ----- Generate bitmap for Lidar usage -----
generateNursery
x; y;



%% ----- helper function -----

function [] = compute_covar()



end