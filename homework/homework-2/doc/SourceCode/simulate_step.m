function [Uy_1, r_1, ay] = simulate_step(Ux_0, Uy_0, r_0, delta_0, dt, veh, f_tire, r_tire, tire_mode)
%SIMULATE_STEP
%   Take a single time step in simulation updating vehicle states.

%   Inputs
%       Ux_0:       Current longitudinal velocity [m/s]
%       Uy_0:       Current lateral velocity [m/s]
%       r_0:        Current yaw rate [rad/s]
%       delta_0:    Current steer angle [rad]
%       dt:         Length of time step [s]
%       veh:        Vehicle parameters struct
%       f_tire:     Front tire parameters struct
%       r_tire:     Rear tire parameters struct
%       tire_mode:  String to select linear or nonlinear tire model
%
%   Output:
%       Uy_1:       Updated lateral velocity [m/s]
%       r_1:        Updated yaw rate [rad/s]
%       ay:         Lateral Acceleration at Current Time Step [m/s^2]


%--------------------------------------------------------------------------
%% CALCULATE SLIP ANGLES
%--------------------------------------------------------------------------
[alpha_f, alpha_r] = 0;

%--------------------------------------------------------------------------
%% CALCULATE TIRE FORCES
%--------------------------------------------------------------------------
if strcmp(tire_mode,'linear')
    F_yf = 0;
    F_yr = 0;
elseif strcmp(tire_mode,'fiala')
    F_yf = 0;
    F_yr = 0;
else
    error('Invalid mode selection');
end
    
%--------------------------------------------------------------------------
%% CALCULATE STATE DERIVATIVES
%--------------------------------------------------------------------------
[Uy_dot, r_dot] = 0;

%--------------------------------------------------------------------------
%% UPDATE VEHICLE STATES
%--------------------------------------------------------------------------
Uy_1 = 0;
r_1  = 0;

%--------------------------------------------------------------------------
%% CALCULATE LATERAL ACCELERATION (FOR PLOTTING LATER)
%--------------------------------------------------------------------------
ay = 0;

end
