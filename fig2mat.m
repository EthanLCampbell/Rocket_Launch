% Turns figure data (from rocket launch) into data file to be used for
% thrust-time curve data. 
% Contributors: Ethan Labianca-Campbell, Mihir Buty

function [data_cut2] = fig2mat() 
    %% Save data from figure to matlab file
    fig = openfig('ThrustvsTimePlot.fig'); % Open figure and assign it to fig object
    dataObjs = findobj(fig,'-property','YData'); % Find all graphic objects with YData, in our case line values
    xval        = dataObjs(1).XData; % Find the X-axis value
    Ymat = [xval(:)]; % Create a matrix with first column of x values
    for i=1:length(dataObjs)
        legend_name{i,1} = dataObjs(i).DisplayName;
        yval        = dataObjs(i).YData;
        Ymat = [Ymat yval(:)]; % Keep appending column vectors
    end
    close(fig); % close the figure
    data.names = ['X';legend_name]; 
    data.Y = Ymat;
    
    %% Cut the data at the front off
    x = Ymat(:,1);
    y = Ymat(:,2);
    
    % Find the first index where y > 1
    startIdx = find(y > 1, 1, 'first');
    
    % Find the last index where y < 1 after y has been greater than 1
    endIdx = find(y < 1, 1, 'last');
    
    % If such indices exist, cut the data; otherwise, return an empty matrix
    if ~isempty(startIdx) && ~isempty(endIdx) && startIdx < endIdx
        data_cut1 = data.Y(startIdx:endIdx, :);
    else
        data_cut1 = [];
    end
    
    %% Cut the data at the end off
    x = data_cut1(:,1);
    y = data_cut1(:,2);
    
    % Find the first index where y > 1
    startIdx = find(y > 1, 1, 'first');
    
    % Find the first index after startIdx where y becomes < 1 again
    endIdx = find(y < 1 & (1:length(y))' > startIdx, 1, 'first') - 1;
    
    % If endIdx is not empty, use it; otherwise, take all data from startIdx to the end
    if ~isempty(startIdx)
        if isempty(endIdx)  % If y never goes back to < 1
            data_cut2 = data_cut1(startIdx:end, :);
        else
            data_cut2 = data_cut1(startIdx:endIdx, :);
        end
    else
        data_cut2 = [];
    end
end