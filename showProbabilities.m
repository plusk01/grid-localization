function showProbabilities( fig, probs, xt)
%SHOWPROBABILITIES Show the grid of probabilities, i.e., the 2D PDF
%   Detailed explanation goes here

    probs = flip(probs);

    figure(fig), clf;
    
    N = size(probs,1); % num of rows
    M = size(probs,2); % num of cols
    
    % Using meshgrid puts center of cell as (x, y)
    [X, Y] = meshgrid(-0.5:1:((N-1)+0.5));
    
    % Expand the map slightly so it all shows up
    C = [probs zeros(N,1); zeros(1,M+1)];

    % Draw the pseudocolor (checkerboard) plot
    h = pcolor(X,Y,C);
    colormap(flipud(gray));
    colorbar;
    
    % Make visually easy to see
    set(h, 'EdgeColor', 'k');
    set(h, 'LineStyle', ':');
    axis square % make the aspect ratio square
    
    % Add true robot position (*)
    if (nargin == 3)
        hold on;
        h = plot(xt(1),xt(2),'w*');
        set(h, 'MarkerSize', 20);
        set(h, 'LineWidth', 4);
    end
end

