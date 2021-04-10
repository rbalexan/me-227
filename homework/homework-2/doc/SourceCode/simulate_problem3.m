%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Homework 2

clear; clc; close all

%--------------------------------------------------------------------------
%% PARSE EXPERIMENTAL DATA
%--------------------------------------------------------------------------
% Load data file
load('doubleLaneChangeDataNiki.mat');


%--------------------------------------------------------------------------
%% CONSTANTS AND SIM PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Simulation time
dt   = 0.001;

for i = 1:3
    t_s_{i} = t_{i}(1):dt:t_{i}(end);
    N{i}    = length(t_s_{i});
end

% Select tire model
%%% STUDENT CODE HERE
%tire_mode = 'linear';
%tire_mode = 'fiala';
%%% STUDENT CODE HERE

% Allocate array space
r_sim_           = cell(3,1);
uy_sim_          = cell(3,1);
ay_sim_          = cell(3,1);

alphaF_rad_      = cell(3,1);
alphaR_rad_      = cell(3,1);

fyf_N_           = cell(3,1);
fyr_N_           = cell(3,1);


%--------------------------------------------------------------------------
%% FIX DATA/SIM TIME MISMATCH
%--------------------------------------------------------------------------
for i = 1:3
    intrp = interp1(t_{i}', [ux_mps_{i}', delta_rad_{i}'], t_s_{i}');

    ux_sim_{i}    = intrp(:,1);
    delta_sim_{i} = intrp(:,2);

    ux_avg{i} = mean(ux_sim_{i}); % Average longitudinal speed of run
end


%--------------------------------------------------------------------------
%% INITIAL CONDITIONS
%--------------------------------------------------------------------------
for i = 1:3
    % set initial conditions
    r_sim_{i}(1)   = r_radps_{i}(1);
    uy_sim_{i}(1)  = uy_mps_{i}(1);
end

%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
for i = 1:3

    % Loop through full sim time vector
    for j = 1:N{i}
        % Set longitudinal speed
        %%% STUDENT CODE HERE
        ux = 0;
        %%% END STUDENT CODE

        % Take single simulation step
        %%% STUDENT CODE HERE
        [uy_update, r_update, ay] = 0;
        %%% END STUDENT CODE

        % Update vehicle states
        if j < N{i}
        %%% STUDENT CODE HERE
            uy_sim_{i}(j+1) = 0;
            r_sim_{i}(j+1) = 0;
        %%% END STUDENT CODE
        end

        % Save acceleration data
        %%% STUDENT CODE HERE
        ay_sim_{i}(j) = 0;
        %%% END STUDENT CODE
    end
end


%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
figure;

spRows = 4;
spCols = 3;

for i = 1:3
    plotIdx = i;

    sp(1,i) = subplot(spRows,spCols,plotIdx); hold on;
    plot(t_{i}, ux_mps_{i}, 'b', 'LineWidth', 2);
    ylim([0 12]);
    grid on; box on;
    ylabel('Longitudinal speed, m/s');
    xlabel('Time, s');

    switch i
        case 1
            title('Low speed');
        case 2
            title('Medium speed');
        case 3
            title('High speed');
    end

    plotIdx = plotIdx + spCols;

    sp(2,i) = subplot(spRows,spCols,plotIdx); hold on;
    plot(t_{i}, ay_mps2_{i}, 'b', 'LineWidth', 2);
    plot(t_s_{i}, ay_sim_{i}, 'r', 'LineWidth', 2);
    ylim([-12 12]);
    grid on; box on;
    ylabel('Lateral acceleration, m/s2');
    xlabel('Time, s');
    legend('Measured', 'Simulated','Location','North');

    plotIdx = plotIdx + spCols;

    sp(3,i) = subplot(spRows,spCols,plotIdx); hold on;
    plot(t_{i}, r_radps_{i}, 'b', 'LineWidth', 2);
    plot(t_s_{i}, r_sim_{i}, 'r', 'LineWidth', 2);
    ylim([-1.5 1.5]);
    grid on; box on;
    ylabel('Yaw rate, rad/s');
    xlabel('Time, s');

    plotIdx = plotIdx + spCols;

    sp(4,i) = subplot(spRows,spCols,plotIdx); hold on;
    plot(t_{i}, uy_mps_{i}, 'b', 'LineWidth', 2);
    plot(t_s_{i}, uy_sim_{i}, 'r', 'LineWidth', 2);
    ylim([-2 2]);
    grid on; box on;
    ylabel('Lateral velocity, m/s');
    xlabel('Time, s');

    linkaxes(sp(:,i), 'x');
    xlim([t_{i}(1) t_{i}(end)]);

end
