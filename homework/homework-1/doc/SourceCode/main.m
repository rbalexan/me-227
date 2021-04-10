%% ME227 Assignment #1
% Code Template
% Spring 2020
% Prof. Chris Gerdes & CAs Peter Schleede and John Talbot

clear; close all;

%% Parameters
v = 10;                         % [m/s] Ux, constant vehicle speed

% Path information; uncomment s&k to follow different paths
% straight  path
s = [0     150];                % [m]   path segment length intervals
k = [0     0];                  % [1/m] path curvature for intervals

% constant radius  path
% s = [0      150];
% k = [1/18   1/18];

% "undulating" path
% s = [0      20      40   150];
% k = [1/18  -1/18     0     0];

% Integrate s & k with given initial conditions [Psi0, E0, N0] to get path
path = integrate_path(s,k,[0;0;0]);
% Returns Path structure containing:
                     % Path.s:    [m]   Distance along the path
                     % Path.k:    [1/m] Curvature at each point along path
                     % Path.psi:  [rad] Path heading
                     % Path.posE: [m]   Position East of the ref. pt
                     % Path.posN: [m]   Position North of the ref. pt

% Vehicle parameters
setup_niki;

% Simulation time
t_final = 10;                   % [s]   Final simulation time
dT      = 0.001;                % [s]   Discretization timestep
t       = 0:dT:t_final;         % [s]   Simulation Time array measured
N       = length(t);            %       Number of time steps in simulation

% Allocate space for simulation data
dpsi    = zeros(N,1);           % [rad] Heading error
s       = zeros(N,1);           % [m]   Distance along the path
e       = zeros(N,1);           % [m]   Lateral error
delta   = zeros(N,1);           % [rad] Steering angle
e_la    = zeros(N,1);           % [m]   Lookahead error

% Set initial conditions
dpsi(1) = 0;
s(1)    = 0;
e(1)    = 0.5;

%% simulation loop
for i = 1:N
    % Look up K
    K = interp1(path.s, path.k, s(i));
    
    % compute delta
    %%%%% STUDENT CODE HERE %%%%%
    [delta(i), ~, ~, e_la(i)] = 0;
    %%%%% END STUDENT CODE %%%%%

    % Use equations of motion to compute state derivatives
    %%%%% STUDENT CODE HERE %%%%%
    [ s_dot, e_dot, dpsi_dot ] = 0;
    %%%%% END STUDENT CODE %%%%%

    if i < N % only update next state if not at end of simulation
        % euler integration to compute discrete next states
        %%%%% STUDENT CODE HERE %%%%%
        dpsi(i+1) = 0;
        s(i+1)    = 0;
        e(i+1)    = 0;
        %%%%% END STUDENT CODE %%%%%
    end
end

%% Plot
animate(path, veh, dpsi, s, e, delta);

%%%%% STUDENT CODE HERE %%%%%
figure('Name', 'ME227 HW1');
subplot(3, 1, 1)
    ylabel('Lateral Error [m]')

subplot(3, 1, 2)
    ylabel('Heading Error [rad]')

subplot(3, 1, 3)
    xlabel('Time [s]')
    ylabel('Lookahead Error [m]')
%%%%% END STUDENT CODE %%%%%
    

