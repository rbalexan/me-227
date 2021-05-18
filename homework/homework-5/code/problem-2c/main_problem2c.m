% ME227 Vehicle Dynamics & Control
% Homework 5: Getting Sideways
% % code template
% Spring 2020
% Prof. Chris Gerdes & CAs Peter Schleede, John Talbot, previous CAs


clear all;
close all;
clc;

addpath('Utils')

% Vehicle parameters (Marty)
veh.Caf = 60000;      % [N/rad]
veh.Car = 160000;     % [N/rad]
veh.m  = 1450;        % [kg]
veh.Iz = 2300;        % [m^2]
veh.L  = 2.4;         % [m]
veh.a  = 0.67*veh.L;  % [m]
veh.b  = veh.L-veh.a; % [m]
veh.mu = 1.1;         % [-]
tire_f.Ca = 60000;
tire_f.mu = 1.1;
tire_r.Ca = 160000;
tire_r.mu = 1.1;

% Constants
g   = 9.81;       % [m/s^2]

t_final = 4;

% simulation time
dT      = 0.001;
t_s     = 0:dT:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
delta_rad   = zeros(N,1);
alphaF_rad  = zeros(N,1);
alphaR_rad  = zeros(N,1);
Fxf_N       = zeros(N,1);
Fxr_N       = zeros(N,1);
Fyf_N       = zeros(N,1);
Fyr_N       = zeros(N,1);

% set initial conditions
%%%% STUDENT CODE HERE %%%%
r_radps(1)  = 0;
uy_mps(1)   = 0;
ux_mps(1)   = 0;
%%%% END STUDENT CODE %%%%

% Compute the static normal load
tire_f.Fz = veh.m * g * .33;
tire_r.Fz = veh.m * g * .67;


for idx = 1:N
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    
    %%%% STUDENT CODE HERE %%%%
    Fx = 0;
    delta = 0;
    %%%% END STUDENT CODE %%%%
    
    [ r_dot, uy_dot, ux_dot, alpha_f, alpha_r ] = ...
        nonlinear_bicycle_model( r, uy, ux, Fx, delta, veh, tire_f, tire_r );
    
    if idx < N
        r_radps(idx+1) = integrate_euler(r, r_dot, dT);
        uy_mps(idx+1) = integrate_euler(uy, uy_dot, dT);
        ux_mps(idx+1) = integrate_euler(ux, ux_dot, dT);
    end
    delta_rad(idx) = delta;
    Fxr_N(idx) = Fx;
end

animateDrift(ux_mps, uy_mps, r_radps, delta_rad, veh, dT)

figure; hold on;
plot(t_s, ux_mps, 'r', 'LineWidth', 2);
plot(t_s, uy_mps, 'b', 'LineWidth', 2);
plot(t_s, r_radps, 'Color', [0 0.6 0], 'LineWidth', 2);
ylim([-8 10]);
grid on; box on;
title('Vehicle States over Time')
xlabel('Time, s');
ylabel('Vehicle states');
legend('Ux, m/s', 'Uy, m/s', 'r, rad/s', 'Location', 'SouthEast');

figure(2)
subplot(2,1,1)
plot(t_s, rad2deg(delta_rad))
grid on
ylabel('delta [deg]')
title('Actuator Commands over Time')

subplot(2,1,2)
plot(t_s, Fxr_N)
grid on
ylabel('Fxr [N]')
xlabel('Time [s]')