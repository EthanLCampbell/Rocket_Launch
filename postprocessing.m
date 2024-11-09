% Trims state vecs based on when it smacks the ground for graphing 
% Contributors: Ethan Labianca-Campbell
% Hopefully this function never gets used, literally pointless if the rk4
% does what i tell it to do 

function [t_trimmed,state_trimmed] = postprocessing(t,state)
    ground_index = find(state(5:end, 3) <= 0, 1); % z-coordinate is the 3rd column
    
    % If the rocket never hits the ground, return the original data
    if isempty(ground_index)
        t_trimmed = t;
        state_trimmed = state;
    else
        % Trim the time and state vectors
        t_trimmed = t(1:ground_index);
        state_trimmed = state(1:ground_index, :);
    end
end