%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optimization Loop to find best launch angle (theta0) that produces
% x-value as user has input. 
% Contributors: Ethan Labianca-Campbell
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta0_opt,x_offset,t_opt,state_opt] = optimize_angle(t0,tf,dt,X0,params,x_desired)

    % Define range of launch angles (space to search in)
    theta_min = 35; % lower bound of angle in deg
    theta_max = 90; % upper bound of angle in deg
    
    % Initialize arrays to hold best-run t and state vecs
    t_opt = [];
    state_opt = [];
    x_offset_opt = 5;
    % use fminbnd to find optimal angle minimizing distance between final state
    % and desired x distance
    options = optimset('TolX', 1e-2);
    [theta0_opt, x_offset] = fminbnd(@objective_function, theta_min, theta_max, options);
    
    % Objective function
    function [x_offset] = objective_function(theta0)
        % update states given off-rod velocity and new theta
        vx0 = 11.9 * cosd(theta0); 
        vz0 = 11.9 * sind(theta0);
        % Initial state [x0, z0, vx0, vz0, theta0] - directly off the launch rod
        X0 = [0, 0, vx0, vz0, theta0]; 
    
        [t, state] = rk4_rocket(t0, tf, dt, X0, params);
        x_final = state(end,1);
        x_offset = abs(x_final - x_desired);

        % Store t and state for the optimal angle
        if x_offset < x_offset_opt
            t_opt = t;
            state_opt = state;
            x_offset_opt = x_offset;
        end
    end
end