%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 ROCKET LAUNCH TRAJECTORY 3DoF MODEL                     %
%                           AAE 439 TEAM 8                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Description
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Contributors: Ethan Labianca-Campbell, Ben Abrams, Mihir Buty, 
%               Nashe Mucharambeyi 
% Class: AAE 439, Fall 2024
% 
% State info is parsed as: [x,z,vx,vz,theta]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear

%% Main File %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% User options

run_options = ["single_run","optimize"];
run = run_options(2);

% make mass function and cg function based on prop burn

% Define mass and dynamics parameters (WIP - NEED TO CHANGE TO REAL VALUES)
params.mass_em = .834;        % Rocket mass (kg)
params.m_chute = .089;
params.m_motor_full = (.0875+0.0872)/2; % average of full motor mass (kg)
params.m_motor_em = (0.0442+0.0435)/2; % average of post-burn motor mass (kg)

% lies propagated by Big Dynamics 
params.Iyy = .2;       % Moment of inertia about y-axis (kg*m^2)
params.CoG = 0.8255;    % Center of Gravity of rocket (m) from cone tip <- cg needed 
params.CoP = 1.1;       % Center of Pressure of rocket (m) from cone tip

% Define area parameters
diameter = 0.0762; % [m]
params.nose_area = pi*(diameter/2)^2;
params.fin_area = (1/8*0.0254)*(0.0254*6.25*cosd(60)); %<- need length dim for fin 
params.ref_area = params.nose_area + params.fin_area;

% Define motor parameters 
Ttdata = fig2mat; %get thrust curve
thrust_lbf = Ttdata(:,2); %[lbf]
thrust = thrust_lbf * 4.44822; % Thrust vector [N]
time_burn = Ttdata(:,1)-Ttdata(1,1); %Time vector during burn [s]
%params.m_prop = 39.2; %[g]
params.tb = time_burn(end)-time_burn(1); 
params.thrust = @(t) (t<params.tb).*interp1(time_burn,thrust,t); % Thrust in N, stops after tb seconds

% Define flight condition parameters
params.g = @(z) 9.81;  % Gravitational acceleration (m/s^2)
params.rho = 1.225;      % Air density (kg/m^3) - assuming roughly constant (change later?)
params.wind =[-10]; % Constant wind in the +x direction (m/s)

% Define chute parameters
params.t_charge = 6; % time from burnout until chute charge blows

% Launch Angle <- the main user input 
theta0 = 83;

% Openrocket Rod assumption:
vx0 = 11.9 * cosd(theta0);
vz0 = 11.9 * sind(theta0);

% Initial state [x0, z0, vx0, vz0, theta0] - directly off the launch rod
X0 = [0, 0, vx0, vz0, theta0]; 

% Desired final state (as called in optimizer)
x_desired = 0; % want it to land right back at x = 0. 

% Time parameters
t0 = 0;
tf = params.t_charge+params.tb;
dt = 0.1;

%% RUN METHODS

% SINGLE RUN TRAJ
if strcmp(run,"single_run")
    % Run the RK4 propagator
    [t, state] = rk4_rocket(t0, tf, dt, X0, params);


    % Run parachute drop code

    % Post-process results 
    %[t,state] = postprocessing(t,state);


% OPTIMIZER LOOP ON ANGLE OF LAUNCH
elseif strcmp(run,"optimize") %% WIP - WANT TO MINIMIZE LANDING DIST TO START 
    % objective function
    [theta0_opt,x_offset,t,state] = optimize_angle(t0,tf,dt,X0,params,x_desired);
    fprintf('The optimal launch angle is = %.2f [deg]\n',theta0_opt)
end

%% PLOTTING OF RESULTS
% Plot Altitude Time history
subplot(2,1,1)
plot(t, state(:, 2))
hold on
xlabel('Time (s)')
ylabel('z (Altitude) (m)')
title('Altitude History')
grid on
hold off

% Plot the rocket's trajectory (x-z)
subplot(2,1,2)
plot(state(:, 1), state(:, 2))
hold on
xlabel('x (Forward Dir) (m)')
ylabel('z (Altitude) (m)')
title('Rocket Trajectory')
plot(state(end,1),state(end, 2),'r*')
grid on
hold off


%% TODO:
% 1. Figure out why velocity direciton is being goofy
% 2. Optimizer that gets landing to be minimized and spits out initial
% flight path angle

%% NOTES FROM DR. HEISTER: 

% literatlly just use the funcitons shown in class 
% probably just set lift = 0?
% openrocket for drag computation? 
