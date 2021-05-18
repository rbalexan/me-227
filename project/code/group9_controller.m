function [ delta, Fx ] = group9_controller( s, e, dpsi, Ux, Uy, r, control_mode, path)
    %ME227 Controller:
    % Spring 2021
    % Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda, Alaisha
    % Alexander, Will Harvey, Lucio Mondavi, John Talbot, Trey Weber
    
    %Team Information
%     Ross Alexander:  rbalexander@stanford.edu
%     Matthew Hunter:  mhunter8@stanford.edu
%     Adyasha Mohanty: madyasha@stanford.edu
%     Roshan Nair:     rpnair@stanford.edu

    %--------------------------------------------------------------------------
    %% Constants
    %--------------------------------------------------------------------------
    g = 9.81;                       % [m/s^2]  gravity

    %--------------------------------------------------------------------------
    %% Vehicle Parameters
    %--------------------------------------------------------------------------
    m  = 1776;                  % [kg]     mass with 2 occupants
    Iz = 2763.49;               % [kg-m^2] rotational inertia
    a  = 1.264;                 % [m]      distance from CoM to front axle
    b  = 1.367;                 % [m]      distance from C0M to rear axle
    L  = a + b;         % [m]      wheelbase
    Wf = m*g*(b/L); % [N]      static front axle weight
    Wr = m*g*(a/L); % [N]      static rear axle weight
    
    
    %new 
    frr = 0.015;
    cda = 0.594;
    rho = 1.225;

    %--------------------------------------------------------------------------
    %% Tire Parameters
    %--------------------------------------------------------------------------
    % Front tires
    f_tire.Ca_lin = 80000;          % [N/rad]  linear model cornering stiffness
    f_tire.Cy     = 110000;         % [N/rad]  fiala model cornering stiffness
    f_tire.mu_s   = 0.90;           %          sliding friction coefficient
    f_tire.mu     = 0.90;           %          peak friction coefficient

    % Rear tires
    r_tire.Ca_lin = 120000;
    r_tire.Cy     = 180000;
    r_tire.mu_s   = 0.94;
    r_tire.mu     = 0.94;

    %--------------------------------------------------------------------------
    %% Find Path Dependent Parameters
    %--------------------------------------------------------------------------

    Ux_des     = interp1(path.s_m, path.UxDes, s);
    Ux_des_dot = interp1(path.s_m, path.axDes, s);
    kappa      = interp1(path.s_m, path.k_1pm, s);

    %--------------------------------------------------------------------------
    %% Control Parameters
    %--------------------------------------------------------------------------
    x_la = 8;
    K_la = 3000;
    
    K_p = 1;
    K_i = 1.1;
    K_d = .3;
    max_int_control = deg2rad(5);

    K_long = 1.8;

    %--------------------------------------------------------------------------
    %% Lateral Control Law
    %--------------------------------------------------------------------------
    %Use the Lateral Control Law to Calculate Delta

    Caf = f_tire.Ca_lin;
    Car = r_tire.Ca_lin;

    K_radpmps2 = (m./L) .* ((b .*Car - a .*Caf)./ (Caf .* Car)); % [rad/m/s^2]

    if control_mode == 1 %Lookahead Controler

        dpsi_ss  = kappa*(m*a*Ux^2/(L*Car) - b);
        delta_ff = kappa*(L + K_radpmps2*Ux^2) + K_la*x_la/Caf*dpsi_ss;
        delta_fb = -K_la/Caf*(e + x_la*dpsi);
        
        delta    = delta_ff +  delta_fb;

    else % PID control
        persistent e_int; persistent e_last; %persistent s_last; persistent Ux_last;
        
        if isempty(e_int) e_int = 0;
        else
            %dt = (s-s_last)*2/(Ux+Ux_last);
            dt = 0.005;
            e_int = e_int+e_last*dt;
        end
        if K_i*e_int>max_int_control
            e_int = max_int_control/K_i;
        end
        
        e_dot = Ux*tan(dpsi)+Uy;
        
        delta = -(K_p*e + K_i*e_int + K_d*e_dot);
        
        e_last = e;
        %s_last = s;
        %Ux_last = Ux;
    end

    %--------------------------------------------------------------------------
    %% Longitudinal Control Law
    %--------------------------------------------------------------------------
    %Use the Longitudinal Control Law to Calculate Fx
    
    Frr     = frr*m*g;
    Fd      = 1/2*rho*cda*Ux^2;
    
    Fx_ff   = m*Ux_des_dot + Frr + Fd;
    Fx_fb   = K_long*m* (Ux_des - Ux);
    
    Fx      = Fx_ff + Fx_fb;

end