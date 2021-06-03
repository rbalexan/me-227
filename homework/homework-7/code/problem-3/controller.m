function [ delta, Fx ] = controller( x_, veh, f_tire, r_tire, path)
%ME227 Controller:
% Spring 2019
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
% 
% Here you should use the inputs from above to calculate the inputs to the
% vehicle. These will be delta and Fx and will be determined based upon
% your control laws below. 
%
% For the project you wil use this same input output structure and in this
% homework you will use this control structure for defining the control
% inputs in your simulation.

%--------------------------------------------------------------------------
%% Constants
%--------------------------------------------------------------------------
g = 9.81;                       % [m/s^2]  gravity

%--------------------------------------------------------------------------
%% Unpack states
%--------------------------------------------------------------------------
s = x_(1);
e = x_(2);
dpsi = x_(3);
Ux = x_(4);
Uy = x_(5);
r = x_(6);

%--------------------------------------------------------------------------
%% Understeer Gradient
%--------------------------------------------------------------------------
% Calculate the understeer gradient for Niki
K_radpmps2 = (veh.Wf/f_tire.Ca_lin - veh.Wr/r_tire.Ca_lin)/g; % [rad/m/s^2]

%--------------------------------------------------------------------------
%% Control Parameters
%--------------------------------------------------------------------------
K_la = 20e3;    % [N/m]
x_la = 20;      % [m]
K_long = 5e3;   % [N/m/s]

%--------------------------------------------------------------------------
%% Find Path Dependent Parameters
%--------------------------------------------------------------------------
% Find desired speed and acceleration
UxDes = interp1(path.s, path.UxDes, s);
AxDes = interp1(path.s, path.AxDes, s);

% Find Curvature for the current distance along the path via interpolation
kappa = interp1(path.s, path.k, s);

%--------------------------------------------------------------------------
%% Lateral Control Law
%--------------------------------------------------------------------------
%Use the Lateral Control Law to Caclulate Delta
dpsi_ss = kappa*(veh.m*veh.a*Ux^2/(r_tire.Ca_lin*veh.L)-veh.b);
delta_ff = kappa*(veh.L + K_radpmps2*Ux^2) + K_la*x_la/f_tire.Ca_lin*dpsi_ss;
delta = -K_la/f_tire.Ca_lin*(e + x_la*dpsi) + delta_ff;

%--------------------------------------------------------------------------
%% Longitudinal Control Law
%--------------------------------------------------------------------------
%Use the Longitudinal Control Law to Cacluate Fx
Fx = K_long*(UxDes - Ux) + veh.m*AxDes;

end
