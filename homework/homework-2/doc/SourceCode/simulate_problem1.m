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
ux_ = {     };
%%% END STUDENT CODE
lenV = length(ux_);

% Create vector of steer angle
delta_ = deg2rad(5)*ones(N,1);   % [rad] step steer angle

% Allocate array space
uy_     = cell(lenV,1);
r_      = cell(lenV,1);

ay_     = cell(lenV,1);


%--------------------------------------------------------------------------
%% INITIAL CONDITIONS
%--------------------------------------------------------------------------
% Set first value in every cell to corresponding initial condition
for i = 1:lenV
    uy_{i}(1) = 0;
    r_{i}(1) = 0;
    ay_{i}(1) = 0;
end

%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
% Loop through each speed creating one full simulation
for i = 1:lenV
    % Pick a tire model
    %%% STUDENT CODE HERE
    %tire_mode = 'linear';
    %tire_mode = 'fiala';
    %%% END STUDENT CODE

    
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
    for i = 1:lenV
        %%% STUDENT CODE HERE
        
        %%% END STUDENT CODE
        hold on
    end
    
    % Annotations
    grid on
    title('Yaw Rate for Several Speeds')
    xlabel('Time [s]')
    ylabel('Yaw Rate [rad/s]')
    legend('10 m/s','20 m/s','30 m/s');
    
% LATERAL VELOCITY
fig2 = figure(2);
    % Plot yaw rates for each speed
    for i = 1:lenV
        %%% STUDENT CODE HERE
        
        %%% END STUDENT CODE
        hold on
    end
    
    % Annotations
    grid on
    title('Lateral Velocity for Several Speeds')
    xlabel('Time [s]')
    ylabel('Lateral Velocity [m/s]')
    legend('10 m/s','20 m/s','30 m/s');
    
% LATERAL ACCELERATION
fig3 = figure(3);
    % Plot yaw rates for each speed
    for i = 1:lenV
        %%% STUDENT CODE HERE
        
        %%% END STUDENT CODE
        hold on
    end
    
    % Annotations
    grid on
    title('Lateral Acceleration for Several Speeds')
    xlabel('Time [s]')
    ylabel('Lateral Acceleration [m/s]')
    legend('10 m/s','20 m/s','30 m/s');
