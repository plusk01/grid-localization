function [ i, j, k ] = state2sub( xt, m, depth )
%STATE2SUB Takes a pose and turns into MATLAB matrix subscripts

    if (nargin < 3)
        depth = 4;
    end
    
    depth2deg = @(index)(index-1)*(360/depth);
    deg2depth = @(deg)(deg/(360/depth))+1;

    % Where am I in the map (row, col)? --> note index swap below
    i = xt(2)+1; % Silly MATLAB indexing
    j = xt(1)+1; % for indexing the map
    
    % flip the row index (i) to match how the grid looks
    i = (size(m,1)+1) - i; % size+1 deals with MATLAB indexing
    
    % get depth of discretization (the angle)
    if length(xt) == 3
        k = deg2depth(xt(3));
    end
end

