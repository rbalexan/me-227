%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Homework 2

clear; clc; close all


%--------------------------------------------------------------------------
%% CONSTANTS AND SIM PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Simulation time
t_f = 2;             % Time to end sim [s]
dt = 0.001;          % Time step [s]
t_   = 0:dt:t_f;     % Create time vector [s]
N  = length(t_);     % Number of steps in sim

% Simulation params
%%% STUDENT CODE HERE
ux = 0;
%%% END STUDENT CODE

% Create vector of steer angle
delta_ = deg2rad(5)*ones(N,1);   % [rad] step steer angle

% Allocate array space
uy_     = cell(2,1);
r_      = cell(2,1);

ay_     = cell(2,1);


%--------------------------------------------------------------------------
%% INITIAL CONDITIONS
%--------------------------------------------------------------------------
% Set first value in every cell to corresponding initial condition
for i = 1:2
    uy_{i}(1) = 0;
    r_{i}(1) = 0;
    ay_{i}(1) = 0;
end

%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
% Loop through each speed creating one full simulation
for i = 1:2
    % Pick a tire model
    if i == 1
        %%% STUDENT CODE HERE
        %tire_mode = 'linear';
        %tire_mode = 'fiala';
        %%% END STUDENT CODE
    else
        %%% STUDENT CODE HERE
        %tire_mode = 'linear';
        %tire_mode = 'fiala';
        %%% END STUDENT CODE
    end

    
    % Loop through each simulation step
    for j = 1:N
        %%% STUDENT CODE HERE
        [uy_update, r_update, ay] = 0;
        %%% END STUDENT CODE
        
        if j < N
            % Update vehicle states
            %%% STUDENT CODE HERE
            uy_{i}(j+1) = 0;
            r_{i}(j+1) = 0;
            %%% END STUDENT CODE
        end
        
        % Save lateral acceleration
        %%% STUDENT CODE HERE
        ay_{i}(j) = 0;
        %%% END STUDENT CODE
    end
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
% YAW RATE
fig1 = figure(1);
    % Plot yaw rates for each speed
    for i = 1:2
        %%% STUDENT CODE HERE
        
        %%% END STUDENT CODE
        hold on
    end
    
    % Annotations
    grid on
    title('Yaw Rate for Different Tire Models')
    xlabel('Time [s]')
    ylabel('Yaw Rate [rad/s]')
    legend('Linear Tire Model','Fiala Tire Model');
    
% LATERAL VELOCITY
fig2 = figure(2);
    % Plot yaw rates for each speed
    for i = 1:2
        %%% STUDENT CODE HERE
        
        %%% END STUDENT CODE
        hold on
    end
    
    % Annotations
    grid on
    title('Lateral Velocity for Different Tire Models')
    xlabel('Time [s]')
    ylabel('Lateral Velocity [m/s]')
    legend('Linear Tire Model','Fiala Tire Model');
    
% LATERAL ACCELERATION
fig3 = figure(3);
    % Plot yaw rates for each speed
    for i = 1:2
        %%% STUDENT CODE HERE
        
        %%% END STUDENT CODE
        hold on
    end
    
    % Annotations
    grid on
    title('Lateral Acceleration for Different Tire Models')
    xlabel('Time [s]')
    ylabel('Lateral Acceleration [m/s]')
    legend('Linear Tire Model','Fiala Tire Model');
