function [Fx_f, Fz_f, Fx_r, Fz_r] = aero_effects(Ux, veh)
%DOWNFORCE
%   Calculates the additional load as a result of downforce on the front
%   and rear axles
%
%   Inputs
%       Ux:  Longitudinal velocity of the vehicle [m/s]
%      veh:  Vehicle parameters structure
%
%   Output:
%       Fx_f:    Drag force caused by the front wing [N]
%       Fz_f:    Normal load on the front axle as a result of downforce [N]
%       Fx_r:    Drag force caused by the rear wing [N]
%       Fz_r:    Normal load on the rear axle as a result of downforce [N]

%%%% STUDENT CODE HERE %%%%
Fx_f = 0;   % [N] Front aero drag
Fz_f = 0;   % [N] Front aero downforce
Fx_r = 0;   % [N] Rear aero drag
Fz_r = 0;   % [N] Rear aero downforce
%%%% END STUDENT CODE %%%%