% filters, cleans, and sorts columns of data in order s.t. they can be used
% for interpolation calls (more typically for AoA vs Cd calculations)
% Contributors: Ethan Labianca-Campbell, Ben Abrams

function [col1, col2] = filter_columns(col1,col2)
    
    % Filter out NaN values and non-real numbers in column 2
    valid_rows = ~isnan(col2) & isreal(col2);
    
    % Apply the filter to both columns
    col1_filtered = col1(valid_rows);
    col2_filtered = col2(valid_rows);
    
    data = [col1_filtered, col2_filtered]; 
    
    % Remove duplicate rows based on both columns
    data = unique(data, 'rows', 'stable');
    
    % Extract cleaned x and y
    x_clean = data(:, 1);
    y_clean = data(:, 2);
    
    % Find unique values in both x and y
    [x_unique, x_unique_idx] = unique(x_clean,'stable'); % Keep first occurrences of unique x
    [y_unique, y_unique_idx] = unique(y_clean,'stable'); % Keep first occurrences of unique y
    
    % Find the intersection of unique x and unique y
    unique_indices = intersect(x_unique_idx, y_unique_idx);
    
    % Apply the unique index filter to both x and y
    x_filtered = x_clean(unique_indices);
    y_filtered = y_clean(unique_indices);
    
    % Sort based on the x-values
    [col1, sortIdx] = sort(x_filtered);
    col2 = y_filtered(sortIdx);
end