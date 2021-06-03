function [Ux_1, Uy_1, r_1, s_1, e_1, dpsi_1] =...
    simulate_step(Ux_0, Uy_0, r_0, s_0, e_0, dpsi_0,...
                  delta_0, Fx_0, kappa, dt, veh, f_tire, r_tire)
%SIMULATE_STEP
%   Take a single time step in simulation updating vehicle states.

%   Inputs
%       Ux_0:       Current longitudinal velocity [m/s]
%       Uy_0:       Current lateral velocity [m/s]
%       r_0:        Current yaw rate [rad/s]
%       s_0:        Current distance along path [m]
%       e_0:        Current error from path [m]
%       dpsi_0:     Current heading error [rad]
%       delta_0:    Current steer angle [rad]
%       Fx_0:       Current longitudinal force [N]
%       kappa:      Road curvature at current position [1/m]
%       dt:         Length of time step [s]
%       veh:        Vehicle parameters struct
%       f_tire:     Front tire parameters struct
%       r_tire:     Rear tire parameters struct
%
%   Output:
%       Ux_1:       Updated longitudinal velocity [m/s]
%       Uy_1:       Updated lateral velocity [m/s]
%       r_1:        Updated yaw rate [rad/s]
%       s_1:        Updated distance along path [m]
%       e_1:        Updated error from path [m]
%       dpsi_1:     Update heading error [rad]


%--------------------------------------------------------------------------
%% CALCULATE SLIP ANGLES
%--------------------------------------------------------------------------
%%% STUDENT CODE HERE
[alpha_f, alpha_r] = slip_angles(Ux_0, Uy_0, r_0, delta_0, veh);
%%% END STUDENT CODE

%--------------------------------------------------------------------------
%% CALCULATE TIRE FORCES
%--------------------------------------------------------------------------
%%% STUDENT CODE HERE
F_yf_0 = Fy_fiala(alpha_f, veh.Wf, f_tire);
F_yr_0 = Fy_fiala(alpha_r, veh.Wr, r_tire);
%%% END STUDENT CODE
    
%--------------------------------------------------------------------------
%% CALCULATE STATE DERIVATIVES
%--------------------------------------------------------------------------
%%% STUDENT CODE HERE
[Ux_dot, Uy_dot, r_dot, s_dot, e_dot, dpsi_dot] = ...
    nonlinear_bicycle_model(F_yf_0, F_yr_0, Fx_0, Ux_0, Uy_0, r_0, e_0, dpsi_0, delta_0, kappa, veh);
%%% END STUDENT CODE

%--------------------------------------------------------------------------
%% UPDATE VEHICLE STATES
%--------------------------------------------------------------------------
%%% STUDENT CODE HERE
Ux_1   = integrate_euler(Ux_0,   Ux_dot,   dt);
Uy_1   = integrate_euler(Uy_0,   Uy_dot,   dt);
r_1    = integrate_euler(r_0,    r_dot,    dt);
s_1    = integrate_euler(s_0,    s_dot,    dt);
e_1    = integrate_euler(e_0,    e_dot,    dt);
dpsi_1 = integrate_euler(dpsi_0, dpsi_dot, dt);
%%% END STUDENT CODE

end
