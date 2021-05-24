function [ r_dot, uy_dot, ux_dot, e_dot, s_dot, dpsi_dot, alpha_f, alpha_r ] = ...
    nonlinear_bicycle_model( r, uy, ux, e, s, dpsi, Fx, delta, kappa, veh, tire_f, tire_r )
%NONLINEAR_BICYCLE_MODEL
%   Calculates the vehicle state derivatives using the bicycle model
%
%   Inputs
%       r:          Yaw rate [rad/s]
%       Ux:         Longitudinal velocity [m/s]
%       Uy:         Lateral velocity [m/s]
%       Fx:         Longitudinal force [N]
%       delta:      Steer angle [rad]
%       veh:        Vehicle parameters struct
%       tire_f:     Front tire parameter struct
%       tire_r:     Rear tire parameter struct
%
%   Output:
%       r_dot:      Derivative of yaw rate [rad/s^2]
%       Uy_dot:     Derivative of lateral velocity [m/s^2]
%       Ux_dot:     Derivative of longitudinal velocity [m/s^2]
%       alpha_f:    Front tire slip angle [rad]
%       alpha_r:    Rear tire slip angle [rad]

%Split longitudinal force based on drive and brakedistribution
fxf = 0;
fxr = Fx;

% slip angles
[alpha_f, alpha_r] = slip_angles(ux, uy, r, delta, veh);

% lateral tire forces
Fyf = fiala_model(alpha_f, tire_f, fxf);
Fyr = fiala_model(alpha_r, tire_r, fxr);

% dynamics
[ux_dot, uy_dot, r_dot] = ...
            state_derivatives(fxr, Fyf, Fyr, ux, uy, r, delta, veh);

s_dot = (1/(1 - e*kappa))*(ux*cos(dpsi) - uy*sin(dpsi)); % [m/s]
e_dot = uy*cos(dpsi) + ux*sin(dpsi); % [m/s]
dpsi_dot = r - kappa*s_dot; % [rad/s]
end
