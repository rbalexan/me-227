function [Uy_dot, r_dot] = lateral_bicycle_model(F_yf, F_yr, Ux, r, veh)
%LATERAL_BICYCLE_MODEL
%   Calculates the vehicle state derivatives using the bicycle model
%
%   Inputs
%       F_yf:       Front lateral tire force [N]
%       F_yr:       Rear lateral tire force [N]
%       Ux:         Longitudinal velocity [m/s]
%       r:          Yaw rate [rad/s]
%       veh:        Vehicle parameters struct
%
%   Output:
%       Uy_dot:     Derivative of lateral velocity [m/s^2]
%       r_dot:      Derivative of yaw rate [rad/s^2]

% Calculate Uy_dot
Uy_dot = 0;

% Calculate Uy_dot
r_dot  = 0;

end

