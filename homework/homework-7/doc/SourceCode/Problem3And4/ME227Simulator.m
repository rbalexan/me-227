%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Nonlinear Vehicle Simulation

clear; clc; close all

addpath('../Utilities')

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Create time vector
dt = 0.001;
t_ = 0:dt:14;
lenT = length(t_);

% Load path and speed profile
load('hairpin_corner.mat')

%--------------------------------------------------------------------------
%% SET ROLL STIFFNESS PARAMS
%--------------------------------------------------------------------------
% COMBINATION A
% veh.Kphi_f = 40e3;   % [Nm/rad]
% veh.Kphi_r = 80e3;   % [Nm/rad]

% COMBINATION B
% veh.Kphi_f = 90e3;   % [Nm/rad]
% veh.Kphi_r = 30e3;   % [Nm/rad]

% COMBINATION C
veh.Kphi_f = 70e3;   % [Nm/rad]
veh.Kphi_r = 50e3;   % [Nm/rad]

%--------------------------------------------------------------------------
%% SET BRAKE PROPORTIONING PARAMS
%--------------------------------------------------------------------------
% COMBINATION A
veh.brakeDistro = [0.64 0.36];   % (front/rear) [frac]

% COMBINATION B
% veh.brakeDistro = [0.36 0.64];   % (front/rear) [frac]

%--------------------------------------------------------------------------
%% ALLOCATE MEMORY
%--------------------------------------------------------------------------
% Allocate space for results (!!Do not change these variable names - store
% your results in these variables!!)
x_ = zeros(6,lenT);     % [s e dpsi ux uy r]
ax_ = zeros(1,lenT);
ay_ = zeros(1,lenT);
delta_cmd_ = zeros(1,lenT-1);
delta_ = zeros(1,lenT-1);
Fx_cmd_ = zeros(1,lenT-1);
Fx_ = zeros(2,lenT-1);
Fy_ = zeros(2,lenT-1);
Fz_ = zeros(4,lenT-1);
phi_rad_ = zeros(1,lenT-1);
mu_eff_ = zeros(2,lenT-1);

%--------------------------------------------------------------------------
%% SET INITIAL CONDITIONS
%--------------------------------------------------------------------------
x_(1,1) = 0;
x_(2,1) = 0;
x_(3,1) = 0;
x_(4,1) = path.UxDes(1);
x_(5,1) = 0;
x_(6,1) = 0;
ay_(1) = 0;
ax_(1) = 0;


%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
% Loop through every time step
for i = 1:lenT-1

    % look up kappa
    kappa = interp1(path.s, path.k, x_(1,i));

    % Calculate actuator commands
    [delta_cmd_(i), Fx_cmd_(i)] = controller(x_(:,i), veh, f_tire, r_tire, path);
    
    % Take simulation step
    [x_(:,i+1), ax_(i+1), ay_(i+1), phi_rad_(i), delta_(i), Fx_(:,i), Fy_(:,i),...
        Fz_(:,i), mu_eff_(:,i)] =...
        simulate_step(x_(:,i), ay_(i), delta_cmd_(i), Fx_cmd_(i), kappa, dt, veh, f_tire, r_tire);
end


%--------------------------------------------------------------------------
%% UNPACK STATES
%--------------------------------------------------------------------------
s_ = x_(1,:);
e_ = x_(2,:);
dpsi_ = x_(3,:);
Ux_ = x_(4,:);
Uy_ = x_(5,:);
r_ = x_(6,:);

FfTot_ = mu_eff_(1,:).*(Fz_(1,:) + Fz_(2,:));
FrTot_ = mu_eff_(2,:).*(Fz_(3,:) + Fz_(4,:));

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
% Plot all of the vehicle states
figure

% Plot Ux
subplot(2,3,1)
plot(t_, Ux_)
hold on
plot(t_, interp1(path.s, path.UxDes, s_))
grid on
xlabel('Time [s]')
ylabel('Ux [m/s]')
legend('Actual','Desired')

% Plot Uy
subplot(2,3,2)
plot(t_, Uy_)
grid on
title('Vehicle States')
xlabel('Time [s]')
ylabel('Uy [m/s]')

% Plot r
subplot(2,3,3)
plot(t_, rad2deg(r_))
grid on
xlabel('Time [s]')
ylabel('r [deg/s]')

% Plot s
subplot(2,3,4)
plot(t_, s_)
grid on
xlabel('Time [s]')
ylabel('s [m]')

% Plot e
subplot(2,3,5)
plot(t_, e_)
grid on
xlabel('Time [s]')
ylabel('e [m]')

% Plot dpsi
subplot(2,3,6)
plot(t_, rad2deg(dpsi_))
grid on
xlabel('Time [s]')
ylabel('dpsi [deg]')


% Plot the actuator commands
figure

subplot(2,1,1)
plot(t_(1:end-1), rad2deg(delta_cmd_))
hold on
plot(t_(1:end-1), rad2deg(delta_), '--')
grid on
ylabel('Steer Angle [deg]')
title('Actuator Commands')
legend('Commanded','Actual')

subplot(2,1,2)
plot(t_(1:end-1), Fx_(1,:))
hold on
plot(t_(1:end-1), Fx_(2,:))
plot(t_(1:end-1), Fx_(1,:) + Fx_(2,:), '--')
plot(t_(1:end-1), Fx_cmd_, '-.')
grid on
xlabel('Time [s]')
ylabel('Longitudinal Force [N]')
legend('Front','Rear','Total','Total Commanded')

% Plot normal load and roll angle
figure

subplot(2,1,1)
plot(t_(1:end-1), Fz_(1,:))
hold on
plot(t_(1:end-1), Fz_(2,:))
plot(t_(1:end-1), Fz_(3,:))
plot(t_(1:end-1), Fz_(4,:))
grid on
ylabel('Normal Load [N]')
title('Weight Transfer')
legend('Front Left','Front Right','Rear Left','Rear Right')

subplot(2,1,2)
plot(t_(1:end-1), rad2deg(phi_rad_))
grid on
xlabel('Time [s]')
ylabel('Roll Angle [deg]')


% Plot effective mu
figure
plot(t_(1:end-1), mu_eff_(1,:))
hold on
plot(t_(1:end-1), mu_eff_(2,:))
grid on
xlabel('Time [s]')
ylabel('Friction Coefficient')
title('Effective Friction Coefficient per Axle')
legend('Front Axle','Rear Axle')

% Plot Forces
figure
subplot(2,1,1)

hold on
plot(t_(1:end-1), Fx_(1,:))
plot(t_(1:end-1), Fy_(1,:))
plot(t_(1:end-1), sqrt(Fx_(1,:).^2 + Fy_(1,:).^2), '--', 'Color', '#7E2F8E')
plot(t_(1:end-1), FfTot_, 'k--')
plot(t_(1:end-1), -FfTot_, 'k--')
grid on
ylabel('Front Tire Forces [N]')
title('Tire Forces on Each Axle')
legend('Fx','Fy','Ftotal','Available Force')

subplot(2,1,2)
hold on
plot(t_(1:end-1), Fx_(2,:))
plot(t_(1:end-1), Fy_(2,:))
plot(t_(1:end-1), sqrt(Fx_(2,:).^2 + Fy_(2,:).^2), '--', 'Color', '#7E2F8E')
plot(t_(1:end-1), FrTot_, 'k--')
plot(t_(1:end-1), -FrTot_, 'k--')
grid on
ylabel('Rear Tire Forces [N]')
legend('Fx','Fy','Ftotal','Available Force')

%--------------------------------------------------------------------------
%% ANIMATE VEHICLE
%--------------------------------------------------------------------------
animate(path, veh, dpsi_, s_, e_, delta_, Fx_, Fy_, Fz_, mu_eff_)


%--------------------------------------------------------------------------
%% PRINT RESULTS
%--------------------------------------------------------------------------
fprintf('%s\n',repmat('-',60,1))
fprintf('RESULTS:\n')
fprintf('%s\n',repmat('-',60,1))

fprintf('Niki made it %5.1f meters through the hairpin in %4.1f seconds.\n',...
    max(s_), max(t_))
