function handle = drawGrid( fig, map )
%DRAWGRID Setup the grid environment for the robot
%   inputs:
%       fig: figure number to draw on
%       map: logical square map of the environment. 1's are obstacles

    % Flip the map across the horizontal axis. This is necessary since the
    % map is a matrix with the origin at the top-left. The grid that is
    % plotted has its origin at the bottom-left, however. Therefore
    % flipping here is easiest. That way range_finder isn't too crazy.
    map = flip(map);

    figure(fig), clf;
    
    N = size(map,1); % num of rows
    M = size(map,2); % num of cols
    
    % Using meshgrid puts center of cell as (x, y)
    [X, Y] = meshgrid(-0.5:1:((N-1)+0.5));
    
    % Expand the map slightly so it all shows up
    m = [map zeros(N,1); zeros(1,M+1)];
    C = double(~m);
    
    % Draw the pseudocolor (checkerboard) plot in B&W
    h = pcolor(X,Y,C);
    colormap(gray(2));
    set(h, 'EdgeColor', 'k');
    set(h, 'LineStyle', ':');
    axis square % make the aspect ratio square
    
    % Set the tick values so it is clearer to read
    set(gca, 'XTick', 0:1:M);
    set(gca, 'YTick', 0:1:N);
    
    % So that you don't have to do this elsewhere
    hold on;

    % return the handle just in case
    handle = h;
end

