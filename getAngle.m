%% this function is not used; contents have been used instead in rocket_dynamics

function[theta_new] = getAngle(a,b,front_area,altitude,mass,thrust,v_wind,theta_old,vel_mag,lift,g,delta_t)
% assumes wind is in the positive x direction
% angles should be in degrees
% contributors: Ben Abrams

% inputs
% a = 'filename' of openrocket cd vs AoA chart
% b = 'filename' of std atm table (I used the one from prop website)
% front_area = area of rocket
% altitude = current height of rocket
% mass = mass of rocket at time t
% thrust = thrust of motor at time t
% v_wind = wind speed
% theta_old = previous theta
% vel_mag = magnitue of velocity
% lift = lift
% g = grav accell
% delta_t = time step

%data = readmatrix("AAE439_Team8_CpvsAoA.csv");
%drag = readmatrix;
alt = readmatrix(b);

rho = 1.226*(interp1(alt(:,2),alt(:,4),altitude));
Cd = drag(:,3);
AoA = drag(:,2);

% calculate flight path angle
psi = atand((vel_mag*sind(theta))/(v_wind+vel_mag*cosd(theta)));

% calculate relative velocity (might change)
vrel = sqrt((vel_mag*sind(theta))^2 + (v_wind+vel_mag*cosd(theta))^2);

% calculate drag
cd = interp1(AoA,Cd,theta-psi);
D = 0.5 * rho *vrel^2 * front_area * cd;

% calculate forces
force = thrust-D;

% calculate dtheta/dt
dtheta = (force*sind(psi-theta_old) + lift*cosd(psi-theta_old)-mass*g*cosd(theta_old))/(mass*vel_mag);

% update theta
theta_new = theta_old + dtheta*delta_t;


end