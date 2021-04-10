function [delta, K_la, d_la, e_la] = lookahead_lateral_control_curvature(e_m, dpsi_rad, K_1pm, veh)
%LOOKAHEAD_LATERAL_CONTROL_CURVATURE
%   Calculates the desired steering angle for the vehicle to track a path
%   including compensation for non-zero curvature
%
%   Inputs
%       e_m:        Error in path tracking [m]
%       dpsi_rad:   Difference in vehicle and path heading [rad]
%       K_1pm:      Curvature of the road [1/m]
%       veh:        Vehicle paramter structure
%
%   Output:
%       delta:      Steering angle [rad]
%       K_la:       Lookahead gain [rad/m]
%       d_la:       Lookahead distance [m]
%       e_la:       Lookahead error [m]

%%%%% STUDENT CODE HERE %%%%%
% Set lookahead gain and lookahead distance
K_la = 0; % [rad/m]
d_la = 0; % [m]

% Calculate lookahead error
e_la = 0; % [m]

% Calculate desired steering angle with feedforward compensation
delta = 0; % [rad]

%%%%% END STUDENT CODE %%%%%

end

