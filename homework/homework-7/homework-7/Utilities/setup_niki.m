%% Set Vehicle Parameters
% declare parameters for a VW GTI with 4 passengers
% to the MATLAB workspace

% Authors: John Alsterda & Nathan Speilberg for Stanford ME227

%--------------------------------------------------------------------------
%% Constants
%--------------------------------------------------------------------------
g = 9.81;

veh.g = 9.81;           % [m/s^2]  gravity
veh.rho = 1.225;        % [kg/m^3] Density of air at sea level

%--------------------------------------------------------------------------
%% Vehicle Parameters
%--------------------------------------------------------------------------
veh.m  = 1926;                      % [kg]     mass
veh.Iz = 2763;                      % [kg-m^2] rotational inertia
veh.a  = 1.2629;                    % [m]      distance from CoM to front axle
veh.b  = 1.3681;                    % [m]      distance from C0M to rear axle
veh.L  = veh.a + veh.b;             % [m]      wheelbase
veh.Wf = veh.m*veh.g*(veh.b/veh.L); % [N]      static front axle weight
veh.Wr = veh.m*veh.g*(veh.a/veh.L); % [N]      static rear axle weight
veh.rW = 0.318;                     % [m]      tire radius
veh.m_tire = 14;                    % [kg]     mass of tire (VERIFY)
veh.w_tire = .225;                  % [m]      tire width (VERIFY)
veh.Iz_wheel = veh.m_tire*veh.rW^2/4 ...
    + veh.m_tire*veh.w_tire^2/12;   % [kgm^2] rotational inertia of wheel
veh.hcg = 0.564;                    % [m] Distance between "CG" and ground
veh.driveDistro = [1; 0];           % [frac] Fraction front, rear wheel drive, sum = 1, currently used
veh.cdA = 0.594;                    % [m^2] Coefficient of drag
veh.frr = 0.015;                    % [unitless] Coefficient of rolling resistance
veh.tf = 1.549;                     % [m] front track width
veh.tr = 1.519;                     % [m] rear track width
veh.hf = 0.11;                      % [m] front roll center height
veh.hr = 0.15;                      % [m] rear roll center height
veh.hrc = (veh.hf*veh.b + veh.hr*veh.a)/(veh.L);   % [m] roll center height at CG
veh.h1 = veh.hcg - veh.hrc;         % [m] distance from roll center to cg

%--------------------------------------------------------------------------
%% Actuator Dynamics
%--------------------------------------------------------------------------
veh.maxPower_W = 131 * 1e3;         % [W]   Max engine power
veh.maxFxEngine_N = 5500;           % [N]   Max force from engine

veh.steer_limit = deg2rad(30);      % [rad] Steering limit

%--------------------------------------------------------------------------
%% Tire Parameters
%--------------------------------------------------------------------------
% Front tires
f_tire.Ca_lin = 80000;          % [N/rad]  linear model cornering stiffness
f_tire.Cy = f_tire.Ca_lin;

% Rear tires
r_tire.Ca_lin = 120000;
r_tire.Cy = r_tire.Ca_lin;

%--------------------------------------------------------------------------
%% Aerodynamic Parameters
%--------------------------------------------------------------------------
veh.Af = 0.1*1.8;       % [m^2] area of front wing
veh.Ar = 0.35*1.8;      % [m^2] area of rear wing
veh.hFwing = 0;    % [m] height of front wing over front wheel
veh.hRwing = 1.23; % [m] height of rear wing over rear wheel
veh.distFwing = veh.a;    % [m] distance of front wing to CG
veh.distRwing = veh.b;  % [m] distance of rear wing to CG
veh.cdAf = 0.1*veh.Af;  % [m^2] Coefficient of drag of the front wing
veh.cdAr = 0.05*veh.Ar; % [m^2] Coefficient of drag of the rear wing
veh.clAf = 0.6*veh.Af;  % [m^2] Coefficient of lift of the front wing
veh.clAr = 0.37*veh.Ar; % [m^2] Coefficient of lift of the rear wing

%--------------------------------------------------------------------------
%% Helper Functions
%--------------------------------------------------------------------------
deg2rad = @(deg) (pi/180)*deg;
rad2deg = @(rad) (180/pi)*rad;
