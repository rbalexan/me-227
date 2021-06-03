function [Fy, Fy_max, Fx] = MF_tire(alpha, mu_eff, Fx, Fz, Fz_0, C_alpha)
%   MF_tire
%   Calculates the tire force at a single tire using a simplified Pacejka
%   model.  The lateral force model is derated based on the commanded
%   longitudinal force.
%   Inputs
%       alpha:  Slip angle [rad]
%       mu_eff: Effective friction for the axle
%       Fx:     Commanded longitudinal force [N]
%       Fz:     Normal load on tire [N]
%       Fz_0:   Nominal normal load on tire [N]
%       C_alpha:Tire cornering stiffness
%
%   Output:
%       F_y:    Lateral tire force [N]

% Force value check
if Fx^2 >= (mu_eff*.999*Fz)^2
    warning('Fx higher than allowable, trying to command %f, but only %f is available\n', Fx, sign(Fx)*.999*mu_eff*Fz);
    Fx = sign(Fx)*.999*mu_eff*Fz;
end

% calculate derating parameter
inside = (mu_eff*Fz)^2 - Fx^2;
if inside < 0
    inside = 0;
end

Fy_max = sqrt(inside);


% Constants
mu_0 = 1.25;
Cy = 1.3;
Ey = -.1;

Dy_0 = mu_0 * Fz_0;
By_0 = C_alpha / (Cy*Dy_0);
phi_x = sqrt(mu_eff^2 * Fz^2 - Fx^2) / (mu_eff * Fz);
alpha_eq = mu_0*Fz_0*alpha/ (mu_eff * Fz * phi_x);
Fy_0 = Dy_0*sin(Cy*atan(By_0*alpha_eq-Ey*(By_0*alpha_eq-atan(By_0*alpha_eq))));
Fy = -(mu_eff * Fz * Fy_0 * phi_x / (mu_0 * Fz_0));

end
