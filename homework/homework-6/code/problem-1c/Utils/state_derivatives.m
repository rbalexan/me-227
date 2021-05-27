function [ux_dot, uy_dot, r_dot] = ...
            state_derivatives(Fxr, Fyf, Fyr, ux, uy, r, delta, veh)
            
ux_dot = (1/veh.m) * (Fxr-Fyf*sin(delta)) + r*uy;
uy_dot = (1/veh.m) * (Fyf*cos(delta)+Fyr) - r*ux;
r_dot = (1/veh.Iz) * (veh.a*Fyf*cos(delta) - veh.b*Fyr);

end