%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AAE 439 Drop Code
% Contributors: Nashe Mucharambeyi, Ethan Labianca-Campbell
% Assumptions:               
% Rocket will be upright while falling
% Frontal Area will be assumed to be the make up of a cone and cylinder w/
% fins                                                                  
% Cds used are rough values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dXdt = parachute_dynamics(t, state, params)
    % unpack parameters
    m = params.mass_em + params.m_motor_em;
    vx = state(3);
    vz = state(4);

    % Area of X components
    A_cone = 49.276* 0.00064516; % in m2
    A_booster = 331.124 * 0.00064516; % in m2
    A_parachute = (2) * pi * 18^2 * 0.00064516; % in m2
    A_vert = A_cone + A_booster + A_parachute;
    % Area of Y components
    A_side = pi * 1.55^2 * 0.00064516; % in m2
    
    % Cd values
    Cd_hemisphere = 0.4;
    Cd_parachute = 0.8425;
    Cd_cylinder = 1;
    Cdx = Cd_hemisphere + Cd_cylinder;
    Cdz = Cd_parachute;
    
    % Other stuffs
    rho = 1.225; % in kg/m3
    g = 9.81; % in m/s2
    
    % Relative velocity between rocket and wind 
    v_rel_x = vx - params.wind;
    v_rel_z = vz;
    v_rel = sqrt(v_rel_x^2 + v_rel_z^2);
    
    % Gravity force
    F_gravity = -m*g;
    
    % Drag force
    F_drag_x = -0.5 * Cdx * rho * A_side * v_rel * v_rel_x;
    F_drag_z = -0.5 * Cdz * rho * A_vert * v_rel * v_rel_z;

    % Drag estimates
    %v_wind = params.wind;
    %F_drag_x = -0.5*rho*(vx+v_wind)^2*(Cd_cylinder + Cd_parachute + Cd_hemisphere)*A_side;
    %F_drag_z = -0.5*rho*vz^2*(Cd_cylinder + Cd_parachute + Cd_hemisphere)*A_vert;
    %F_drag = [F_drag_x,F_drag_z];

    % Acceleration 
    ax = F_drag_x / m;
    az = (F_drag_z + F_gravity) / m;

    % Pack dstate vector
    dXdt = [vx,vz,ax,az,0];

end