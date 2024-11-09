%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computes composition of forces on rocket to generate equations of motion. 
% Contributors: Ethan Labianca-Campbell, Ben Abrams
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% WIP: pitch & flight path angle not really accounted for yet. 
%% FORCE DUE TO WIND ALSO NOT REALLY ACCOUNTED FOR 

%% Computes rates of change of state vector (velocity & accl)
function dXdt = rocket_dynamics(t, X, params)

    % Read relevant data
    cd_aoa_data = readmatrix("AAE412_Team8_CDvsAoA_PRELIM.csv");
    AoA = cd_aoa_data(:,2);
    Cd = cd_aoa_data(:,3);
    [AoA,Cd] = filter_columns(AoA,Cd);
    % Extract from parameters structure 
    g = params.g(X(3));
    rho = params.rho;
    A = params.ref_area; % reference area
    if t < params.tb
        thrust = params.thrust(t); % Thrust as a function of time
        m = params.mass_em + params.m_motor_full + params.m_chute;
    else % after burnout, before chute charge
        thrust = 0;
        m = params.mass_em + params.m_chute + params.m_motor_em;
    end
    v_wind = [params.wind,]; % Wind velocity (assuming -x direction only)
    
    % Extract position and velocity from state vector
    x = X(1); z = X(2); 
    vx = X(3); vz = X(4);
    theta = X(5);
    v = [vx, vz]; % Velocity vector
    
    % get velocity magnitude and relative velocity w.r.t wind 
    vel_mag = norm(v);
    vrel = v - v_wind;
    %vrel = (v_wind + vel_mag*cosd(theta))/cosd(psi);
    vrel_mag = norm(vrel);

    % calculate flight path angle
    %psi = atand((vel_mag*sind(theta))./(v_wind+vel_mag*cosd(theta)));
    psi = atan2d(vrel(2),vrel(1));

    % calculate relative velocity 
    %vrel = sqrt((vel_mag*sind(theta)).^2 + (v_wind+vel_mag*cosd(theta)).^2);
    %vrel = vel_mag*sin(theta)/sin(psi);
    
    %% FORCES
    % Gravity force
    F_gravity = [0, -m * g];
    
    % Thrust force
    F_thrust = thrust * [cosd(theta), sind(theta)]; % <- Thrust direction is along velocity vector, will need to change later

    % Drag force (motion)
    
    % calculate drag
    cd = interp1(AoA,Cd,theta-psi,'linear','extrap');
    F_drag = 0.5 * rho *vrel_mag^2 * A * cd * (-vrel / vrel_mag);
    
    % Total force
    F_total = F_gravity + F_drag + F_thrust;
    
    %% MOMENTS 
    % based on aerodynamic moments?
    % Dr. Heister's notes yield this as our dtheta term: 
    % calculate dtheta/dt
    dtheta = ((norm(F_thrust)-norm(F_drag))*sind(psi-theta) - m*g*cosd(theta))/(m*vel_mag);
    
    %% EOMS
    % Equations of motion (F = ma)
    dxdt = vx;
    dzdt = vz;
    dvxdt = F_total(1) / m;
    dvzdt = F_total(2) / m;
    
    dthetadt = dtheta;

    % Return the derivative of the state vector
    dXdt = [dxdt, dzdt, dvxdt, dvzdt, dthetadt];

end