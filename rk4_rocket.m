%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Runge-Kutta based Propogator that takes rocket dynamics EoMs to generate 
% new state vectors. 
% Contributors: Ethan Labianca-Campbell
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Function for Runge-Kutta Propagation
function [t, state] = rk4_rocket(t0, tf, dt, X0, params)
    % rk4_rocket: RK4 Propagator for Rocket Dynamics
    % Inputs:
    % t0: Initial time
    % tf: Final time
    % dt: Time step
    % X0: Initial state vector [x0, z0, vx0, vz0]
    % params: Struct containing rocket parameters
    
    % Initialize time and state arrays
    t = t0:dt:tf;
    n_steps = length(t);
    state = zeros(n_steps, 5); % [x, z, vx, vz, theta]
    state(1, :) = X0;
    i = 1;
    % Loop through time steps
    for n = 1:length(t) % rocket ascent 
        % RK4 steps
        k1 = dt * rocket_dynamics(t(i), state(i, :), params);
        k2 = dt * rocket_dynamics(t(i) + dt/2, state(i, :) + k1/2, params);
        k3 = dt * rocket_dynamics(t(i) + dt/2, state(i, :) + k2/2, params);
        k4 = dt * rocket_dynamics(t(i) + dt, state(i, :) + k3, params);
        
        % Update state
        state(i + 1, :) = state(i, :) + (k1 + 2*k2 + 2*k3 + k4) / 6;
        i = i+1;
    end
    state = state(1:end-1,:);
    % assume immediate decelerration when charge explodes (vx=vz=0)
    state(i-1,:) = [state(end,1),state(end,2),0,0,state(end,5)];
    i = i-1;
    while state(i,2) > 0 % drop code 
        k1 = dt * parachute_dynamics(t(i), state(i, :), params);
        k2 = dt * parachute_dynamics(t(i) + dt/2, state(i, :) + k1/2, params);
        k3 = dt * parachute_dynamics(t(i) + dt/2, state(i, :) + k2/2, params);
        k4 = dt * parachute_dynamics(t(i) + dt, state(i, :) + k3, params);

        % Append to state array
        % (Or store descent states separately if preferred)
        state(i+1,:) = state(i, :) + (k1 + 2*k2 + 2*k3 + k4) / 6;
        t(i+1) = t(i)+dt;
        i = i+1;
    end

    
end