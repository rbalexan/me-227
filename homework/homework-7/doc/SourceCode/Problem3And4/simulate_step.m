function [x_1_, ax_1, ay_1, phi_rad_0, delta_0, Fx_0, Fy_0, Fz_0, mu_eff_0] =...
    simulate_step(x_0_, ay_0,...
                  delta_cmd_0, Fx_cmd_0, kappa, dt, veh, f_tire, r_tire)
%SIMULATE_STEP
%   Take a single time step in simulation updating vehicle states.

%   Inputs
%       x_0:        Current state vector
%       ay_0:       Current lateral acceleration [m/s2]
%       delta_0:    Current steer angle [rad]
%       Fx_0:       Current longitudinal force [N]
%       kappa:      Road curvature at current position [1/m]
%       dt:         Length of time step [s]
%       veh:        Vehicle parameters struct
%       f_tire:     Front tire parameters struct
%       r_tire:     Rear tire parameters struct
%
%   Output:
%       x_1:        Updated state vector
%       ax_1:       Updated longitudinal acceleration [m/s2]
%       ay_1:       Updated lateral acceleration [m/s2]
%       phi_rad_0:  Current roll angle [rad]
%       Fx_0:       Current front longitudinal force [N]
%       Fy_0:       Current front lateral force [N]
%       Fz_0:       Current normal force [N]
%       mu_eff_0:   Current effective friction front axle

% NOTE: There is a single step offset in acceleration values. To break the
% algebraic loop introduced when accounting for roll effects, we're
% assuming the acceleration value doesn't change much from a single time
% step.

%--------------------------------------------------------------------------
%% UNPACK STATES
%--------------------------------------------------------------------------
s_0 = x_0_(1);
e_0 = x_0_(2);
dpsi_0 = x_0_(3);
Ux_0 = x_0_(4);
Uy_0 = x_0_(5);
r_0 = x_0_(6);


%--------------------------------------------------------------------------
%% ENFORCE ACTUATOR LIMITS
%--------------------------------------------------------------------------
delta_0 = min(veh.steer_limit, delta_cmd_0);
delta_0 = max(-veh.steer_limit, delta_0);

if (Fx_cmd_0 > 0)
    Fx_cmd_0 = min(veh.maxFxEngine_N, Fx_cmd_0);
    Fx_cmd_0 = min(veh.maxPower_W/Ux_0, Fx_cmd_0);
end


%--------------------------------------------------------------------------
%% SPLIT LONGITUDINAL FORCE
%--------------------------------------------------------------------------
% Split longitudinal force between axles
if (Fx_cmd_0 > 0)
    Fxf = veh.driveDistro(1)*Fx_cmd_0;
    Fxr = veh.driveDistro(2)*Fx_cmd_0;
else
    Fxf = veh.brakeDistro(1)*Fx_cmd_0;
    Fxr = veh.brakeDistro(2)*Fx_cmd_0;
end


%--------------------------------------------------------------------------
%% CALCULATE LONGITUDINAL ACCELERATION
%--------------------------------------------------------------------------
ax_1 = Fx_cmd_0/veh.m;    % [m/s2]


%--------------------------------------------------------------------------
%% CALCULATE TIRE LOADING
%--------------------------------------------------------------------------
[Fz_fl, Fz_fr, Fz_rl, Fz_rr, phi_rad_0] = normal_load(Ux_0, ax_1, ay_0, veh);

% Calculate normal load on axle
Fzf = Fz_fl + Fz_fr;
Fzr = Fz_rl + Fz_rr;

%--------------------------------------------------------------------------
%% CALCULATE EFFECTIVE FRICTION
%--------------------------------------------------------------------------
[mu_eff_f, ~, ~] = calc_effective_mu(Fz_fl, Fz_fr);
[mu_eff_r, ~, ~] = calc_effective_mu(Fz_rl, Fz_rr);


%--------------------------------------------------------------------------
%% CALCULATE SLIP ANGLES
%--------------------------------------------------------------------------
[alpha_f, alpha_r] = slip_angles(Ux_0, Uy_0, r_0, delta_0, veh);


%--------------------------------------------------------------------------
%% CALCULATE TIRE FORCES
%--------------------------------------------------------------------------
[Fyf, ~, Fxf] = MF_tire(alpha_f, mu_eff_f, Fxf, Fzf, veh.Wf, f_tire.Cy);
[Fyr, ~, Fxr] = MF_tire(alpha_r, mu_eff_r, Fxr, Fzr, veh.Wr, r_tire.Cy);

%--------------------------------------------------------------------------
%% CALCULATE STATE DERIVATIVES
%--------------------------------------------------------------------------
[Ux_dot, Uy_dot, r_dot, s_dot, e_dot, dpsi_dot] =...
    nonlinear_bicycle_model(Fyf, Fyr, Fxf, Fxr, Ux_0, Uy_0, r_0, e_0,...
                            dpsi_0, delta_0, kappa, veh);

                        
%--------------------------------------------------------------------------
%% UPDATE VEHICLE STATES
%--------------------------------------------------------------------------
Ux_1 = integrate_euler( Ux_0, Ux_dot, dt);
Uy_1 = integrate_euler( Uy_0, Uy_dot, dt);
r_1  = integrate_euler( r_0,  r_dot,  dt);
e_1  = integrate_euler( e_0,  e_dot,  dt);
s_1  = integrate_euler( s_0,  s_dot,  dt);
dpsi_1  = integrate_euler( dpsi_0,  dpsi_dot,  dt);
ay_1 = Uy_dot + r_0*Ux_0;


%--------------------------------------------------------------------------
%% PACK UP OUTPUTS
%--------------------------------------------------------------------------
x_1_ = [s_1 e_1 dpsi_1 Ux_1 Uy_1 r_1];
Fx_0 = [Fxf Fxr];
Fy_0 = [Fyf Fyr];
Fz_0 = [Fz_fl, Fz_fr, Fz_rl, Fz_rr];
mu_eff_0 = [mu_eff_f mu_eff_r];

end
