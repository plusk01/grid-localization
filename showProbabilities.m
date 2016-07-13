function showProbabilities( fig, probs )
%SHOWPROBABILITIES Summary of this function goes here
%   Detailed explanation goes here

    figure(fig), clf;
    
    N = size(probs,1); % num of rows
    M = size(probs,2); % num of cols
    
    % Using meshgrid puts center of cell as (x, y)
    [X, Y] = meshgrid(-0.5:1:((N-1)+0.5));
    
    % Expand the map slightly so it all shows up
    C = [probs zeros(N,1); zeros(1,M+1)];

    % Draw the pseudocolor (checkerboard) plot
    h = pcolor(X,Y,C);
    colormap(jet);
    colorbar;
    
    set(h, 'EdgeColor', 'k');
    set(h, 'LineStyle', ':');
    axis square % make the aspect ratio square
    
end

