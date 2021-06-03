%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Plotting Script for Hairpin Turn

clear; clc; close all

addpath('../Utilities')

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load path and speed profile
load('hairpin_corner.mat')

% Acceleration due to gravity
g = 9.81;

%--------------------------------------------------------------------------
%% PLOT PATH AND SPEED PROFILE
%--------------------------------------------------------------------------
% Plot path
figure(1)

z = zeros(length(path.s),1);
color = path.s;
surface([path.posE path.posE], [path.posN path.posN], [z z], [color color],...
    'facecol','no',...
    'edgecol','flat',...
    'linew', 2)
grid on
axis equal
xlabel('East Position [m]')
ylabel('North Position [m]')
cbar = colorbar;
cbar.Label.String = 'Distance Along Path (s) [m]';
title('Hairpin Corner')


% Plot speed profile
figure(2)

a(1) = subplot(2,1,1); 
plot( path.s, path.UxDes, 'linew', 1.5); 
ylabel('Ux, m/s', 'Interpreter', 'tex');
grid on
title('Speed Profile for the Hairpin')

% Calculate AyDes
path.AyDes = (path.UxDes.^2).*path.k;

a(2) = subplot(2,1,2); 
plot(path.s, path.AxDes/g, path.s, path.AyDes/g, path.s, sqrt(path.AxDes.^2 + path.AyDes.^2)/g, 'linew', 1.5); 
linkaxes(a, 'x');  
ylabel('Acceleration [g]', 'Interpreter', 'tex'); 
xlabel('s, m');
grid on;
xlim([0 max(path.s)])
legend('ax', 'ay', 'Location', 'best', 'Interpreter', 'tex')