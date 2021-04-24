%% Set Vehicle Parameters
% declare parameters for a VW GTI with 4 passengers
% to the MATLAB workspace

% Authors: John Alsterda & Nathan Speilberg for Stanford ME227

%--------------------------------------------------------------------------
%% Constants
%--------------------------------------------------------------------------
g = 9.81;                       % [m/s^2]  gravity

%--------------------------------------------------------------------------
%% Vehicle Parameters
%--------------------------------------------------------------------------
veh.m  = 1926.2;                % [kg]     mass
veh.Iz = 2763.49;               % [kg-m^2] rotational inertia
veh.wDist = [0.3 0.7];          % [perc]   weight distribution
veh.L  = 2.6310;                % [m]      wheelbase
veh.a  = veh.L*veh.wDist(2);    % [m]      distance from CoM to front axle
veh.b  = veh.L*veh.wDist(1);    % [m]      distance from C0M to rear axle
veh.Wf = veh.m*g*veh.wDist(1);  % [N]      static front axle weight
veh.Wr = veh.m*g*veh.wDist(2);  % [N]      static rear axle weight
veh.rW = 0.318;                 % [m]      tire radius

%--------------------------------------------------------------------------
%% Tire Parameters
%--------------------------------------------------------------------------
% Front tires
f_tire.Ca_lin = 80000;          % [N/rad]  linear model cornering stiffness
f_tire.Cy     = 110000;         % [N/rad]  fiala model cornering stiffness
f_tire.mu_s   = 0.90;           %          sliding friction coefficient
f_tire.mu     = 0.90;           %          peak friction coefficient

% Rear tires
r_tire.Ca_lin = 120000;
r_tire.Cy     = 180000;
r_tire.mu_s   = 0.94;
r_tire.mu     = 0.94;

%--------------------------------------------------------------------------
%% Helper Functions
%--------------------------------------------------------------------------
deg2rad = @(deg) (pi/180)*deg;
rad2deg = @(rad) (180/pi)*rad;
