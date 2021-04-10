function F_y = Fy_fiala(alpha, Fz, tire)
%FY_FIALA
%   Calculates the tire force at a single tire using a fiala tire model
%
%   Inputs
%       alpha:  Slip angle [rad]
%       Fz:     Normal load on tire [N]
%       tire:   Tire parameters struct
%
%   Output:
%       F_y:    Lateral tire force [N]

% Calculate alpha sliding
alpha_sl = 0;

% Calculate tire force if not sliding 
% NOTE: We recommend making each of the 3 terms its own variable and then combining to find the tire force
a1 = 0;
a2 = 0;
a3 = 0;

% Check if tire is sliding or not
if abs(alpha) < alpha_sl
    F_y = 0;
else
    F_y = 0;
end

end

