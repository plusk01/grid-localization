function zt = range_finder( xt, map, params )
%RANGE_FINDER Use a zero-mean Gaussian noise model to return a range 
%measurement based on the current location, the map, and the intrinsic
%range finder parameters
%   inputs:
%       xt: the robot's real current state (x, y, theta (deg))
%       map: sqaure map of the environment. 1's are obstacles
%       params: the intrinsic range finder params, a vector with:
%           range_max:  the maximum range of the sensor
%           sigma:      the std dev of the noise
%   outputs:
%       zt: the noisy range measurement

    if (nargin == 2)
        % If `params` wasn't passed in, that means we want to do
        % 'ray tracing', i.e., we want the actual distance with no noise
        % and no max range, Rmax
        params = [ size(map,1) 0 ];
    end

    % Unpack the params
    Rmax = params(1);
    sigma = params(2);
    
    % Where am I in the map (row, col)? --> note index swap below
    i = xt(2)+1; % Silly MATLAB indexing
    j = xt(1)+1; % for indexing the map
    
    % flip the row index (i) to match how the grid looks
    i = (size(map,1)+1) - i; % size+1 deals with MATLAB indexing
    
    % Which way should I look for obstacles?
    c = exp(1j*xt(3)*pi/180);
    dir = [-imag(c) real(c)]; % (row, col) with robot as origin
    % Again, notice that imag(c) is negated to match the fmap
    
    % Force dir to have integer components
    dir(find(abs(dir)<2*eps)) = 0;
    
    % Based on where I am in the map, is there an obstacle in front of me?
    looker = [i j]; % which index is being looked at right now?
    zt = -1;
    
    % If not walled in, don't allow the looker to go beyond matrix indices
    if any(dir<(-eps))
        % Special treatment: if the looker is going in a negative direction
        % then the max_look needs to be flipped again because of the fmap
        max_look = looker(find(abs(dir)>eps)) - 1;
    else
        max_look = size(map,1) - looker(find(abs(dir)>eps));
    end
    
    % to see where the robot is (uncomment below)
    tmp = map;
    tmp(looker(1),looker(2)) = 2;
    
    for k = 1:min((Rmax+1),max_look)
        % move looker along line of sight
        looker = looker + dir;
        
        % Uncomment to see step-by-step visualization
%         tmp(looker(1), looker(2)) = 99
%         pause;
        
        if map(looker(1), looker(2)) == 1
            % an obstacle was hit!
            zt = k-1;
            break;
        end
    end
    
    if max_look == 0
        % We were on the edge of the matrix looking out, therefore there is
        % nothing else to see!
        zt = 0;
    end
    
    if zt == -1
        if min((Rmax+1),max_look) == max_look
            % We were constrained by the matrix size, not the range of the
            % sensor, so zt should be max_look
            zt = max_look;
        else
            % there was no obstacle detected, so use the Rmax
            zt = Rmax;
        end
    end
    
    % Apply Gaussian noise to the measurement
    zt = normrnd(zt, sigma);
end

