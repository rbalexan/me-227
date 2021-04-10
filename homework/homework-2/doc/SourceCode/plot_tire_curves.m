%% QUESTION 2.B -- PLOT TIRE FORCE CURVES

%--------------------------------------------------------------------------
%% CONSTANTS
%--------------------------------------------------------------------------
% Load vehicle params
setup_niki;


%--------------------------------------------------------------------------
%% CREATE VECTOR OF SLIP ANGLES
%--------------------------------------------------------------------------
step = 0.1;

alpha_fiala_ = deg2rad(0:step:12);
alpha_linear_ = deg2rad(0:step:4);

N_fiala = length(alpha_fiala_);
N_linear = length(alpha_linear_);

% Allocate space for tire forces
Fyf_fiala = zeros(N_fiala,1);
Fyr_fiala = zeros(N_fiala,1);
Fyf_linear = zeros(N_linear,1);
Fyr_linear = zeros(N_linear,1);

%--------------------------------------------------------------------------
%% CALCULATE TIRE FORCE CURVES
%--------------------------------------------------------------------------
for i = 1:N_fiala
    %%% STUDENT CODE HERE
    Fyf_fiala(i) = 0;
    Fyr_fiala(i) = 0;
    %%% END STUDENT CODE
end

for j= 1:N_linear
    %%% STUDENT CODE HERE
    Fyf_linear(j) = 0;
    Fyr_linear(j) = 0;
    %%% END STUDENT CODE
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
fig1 = figure(1);

    % Plot fiala tire curves
    %%% STUDENT CODE HERE

    %%% END STUDENT CODE
    hold on
    %%% STUDENT CODE HERE

    %%% END STUDENT CODE

    % Plot linear tire curves
    %%% STUDENT CODE HERE
    

    %%% END STUDENT CODE

    % Annotations
    grid on
    title('Tire Curves for Linear and Fiala Models')
    xlabel('Alpha [rad]')
    ylabel('Lateral Force [N]')
    legend('Front Fiala','Rear Fiala','Front Linear','Rear Linear')
