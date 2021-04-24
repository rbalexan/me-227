%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Homework 4 - Question 1.C

clear; clc; close all

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Gains and conditions
K_la = 0;   % [N/m]
x_la = 0;           % [m]    
Ux_ = 0:0:0;       % [m/s]
lenUx = length(Ux_);

% Allocate space for poles (We know there are 4)
poles_ = zeros(4,lenUx);
cbarStr = cell(lenUx,1);

%--------------------------------------------------------------------------
%% CREATE SYSTEM MATRIX
%--------------------------------------------------------------------------
for idx = 1:lenUx
    % Select speed
    Ux = Ux_(idx);
    
    %{
    To make the state matrix easier to input, create each term separately
    here according to this template - we'll complile these into the matrix
    at the end. We recommend you keep this general and let MATLAB fill in
    each of the values as you set them up above. Then you can copy and
    paste this section into later problems.
    
        A = [aM,  bM,  cM,  dM]
            [eM,  fM,  gM,  hM]
            [iM,  jM,  kM,  lM]
            [mM,  nM,  oM,  pM]
    %}
    
    aM = ;
    bM = ;
    cM = ;
    dM = ;
    eM = ;
    fM = ;
    gM = ;
    hM = ;
    iM = ;
    jM = ;
    kM = ;
    lM = ;
    mM = ;
    nM = ;
    oM = ;
    pM = ;

    A = [[aM,  bM,  cM,  dM];
         [eM,  fM,  gM,  hM];
         [iM,  jM,  kM,  lM];
         [mM,  nM,  oM,  pM]];
    
   % Calculate pole positions
   poles_(:,idx) = 0;
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
figure
cmap = colormap(winter(lenUx));
for idx = 1:lenUx
    plot(real(poles_(:,idx)), imag(poles_(:,idx)), 'x', 'Color', cmap(idx,:))
    hold on
    cbarStr{idx} = sprintf('%2i',Ux_(idx));
end
xline(0,'--');
grid on
xlabel('Real Axis')
ylabel('Imaginary Axis')
cbar = colorbar('Ticks', Ux_, 'TickLabels', cbarStr);
caxis([Ux_(1) Ux_(end)])
cbar.Label.String = 'Ux [m/s]';