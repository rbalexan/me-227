%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Nonlinear Vehicle Simulation

% clear; clc; close all

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
%%% STUDENT CODE HERE
% Load Niki params
setup_niki;

% Create time vector (!!Do not change these variable names - store
% your results in these variables!!)
dt = 0.001;
t_ = ??:dt:??;
lenT = length(t_);

% Set control gains
gains.K_la = ??;
gains.x_la = ??;
gains.K_long = ??;

% Select lookahead controller mode
% control_mode = 1; % Feedback only
% control_mode = 2; % Feedforward + Feedback

% Select path (Uncomment one of these paths and comment the others)
% straight  path
s = [0     750];
k = [0     0];

% constant radius  path
% s = [0      750];
% k = [1/980   1/980];

% "undulating" path
% s = [0      250      500   750];
% k = [1/980   -1/980   0    0];
%%% END STUDENT CODE

%--------------------------------------------------------------------------
%% GENERATE PATH AND ALLOCATE MEMORY
%--------------------------------------------------------------------------
% path information
init = [0;0;0];

% integrate s and k to get path
path = integrate_path(s, k, init);

% Allocate space for results (!!Do not change these variable names - store
% your results in these variables!!)
s_ = zeros(1,lenT);
e_ = zeros(1,lenT);
dpsi_ = zeros(1,lenT);
r_ = zeros(1,lenT);
Ux_ = zeros(1,lenT);
Uy_ = zeros(1,lenT);
delta_ = zeros(1,lenT);
Fx_ = zeros(1,lenT);

%--------------------------------------------------------------------------
%% SET SPEED PROFILE
%--------------------------------------------------------------------------
% Set speed profile here (!!Do not change these variable names - store
% your results in these variables!!)

%%% STUDENT CODE HERE
path.UxDes = ??*ones(size(path.s));
%%% END STUDENT CODE

%--------------------------------------------------------------------------
%% SET INITIAL CONDITIONS
%--------------------------------------------------------------------------
%%% STUDENT CODE HERE
s_(1) = 0;
e_(1) = 0;
dpsi_(1) = 0;
r_(1) = 0;
Ux_(1) = 0;
Uy_(1) = 0;
%%% END STUDENT CODE

%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
% Loop through every time step
for i = 1:lenT-1

    % look up kappa
    kappa = interp1(path.s, path.k, s_(i));
    
    %%% STUDENT CODE HERE

    % Calculate actuator commands
    
    % Take simulation step
    
    %%% END STUDENT CODE
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
% Plot all of the vehicle states
figure

% Plot Ux
subplot(2,3,1)
plot(t_, Ux_)
grid on
xlabel('Time [s]')
ylabel('Ux [m/s]')

% Plot Uy
subplot(2,3,2)
plot(t_, Uy_)
grid on
title('Vehicle States')
xlabel('Time [s]')
ylabel('Uy [m/s]')

% Plot r
subplot(2,3,3)
plot(t_, rad2deg(r_))
grid on
xlabel('Time [s]')
ylabel('r [deg/s]')

% Plot s
subplot(2,3,4)
plot(t_, s_)
grid on
xlabel('Time [s]')
ylabel('s [m]')

% Plot e
subplot(2,3,5)
plot(t_, e_)
grid on
xlabel('Time [s]')
ylabel('e [m]')

% Plot dpsi
subplot(2,3,6)
plot(t_, rad2deg(dpsi_))
grid on
xlabel('Time [s]')
ylabel('dpsi [deg]')


% Plot the actuator commands
figure

subplot(2,1,1)
plot(t_, rad2deg(delta_))
grid on
xlabel('Time [s]')
ylabel('Steer Angle [deg]')
title('Actuator Commands')

subplot(2,1,2)
plot(t_, Fx_)
grid on
xlabel('Time [s]')
ylabel('Total Longitudinal Force [Fx]')

%--------------------------------------------------------------------------
%% ANIMATE VEHICLE
%--------------------------------------------------------------------------
% animate(path, veh, dpsi_, s_, e_, delta_)
