function [s_dot, e_dot, dpsi_dot] = kinematic_model(v_mps, dpsi_rad, e_m, K_1pm, delta, veh)
%KINEMATIC_MODEL 
%   This function calculates the equations of motion for a vehicle using the kinematic bicycle model
%
%   Inputs
%       v_mps:      Velocity [m/s]
%       dpsi_rad:   Difference in vehicle and path heading [rad]
%       e_m:        Error in path tracking [m]
%       K_1pm:      Curvature of the road [1/m]
%       delta:      Steering angle of the vehicle roadwheels [rad]
%       veh:        Vehicle paramter structure
%
%   Output:
%       s_dot:      Rate of change of distance along path [m/s]
%       e_dot:      Rate of change of path tracking error [m/s]
%       dpsi_dot:   Rate of change of heading error [rad/s]

%%%%% STUDENT CODE HERE %%%%%
% Calculate state derivatives

s_dot = 0; % [m/s]
e_dot = 0; % [m/s]
dpsi_dot = 0; % [rad/s]

%%%%% END STUDENT CODE %%%%%
end
