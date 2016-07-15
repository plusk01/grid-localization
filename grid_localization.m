function [ pkt ] = grid_localization( pkt_d1, ut, zt, m, motion_params )
%GRID_LOCALIZATION A localizer using the discrete Bayes filter
%   Using stochastic motion and measurement models, localize the Thrunbot
%   based on a discrete Bayes filter. Here we use the Odometry Motion Model
%   (p. 134) and the Range Finder Measurement Model (p. 158).
%
%   Remember that:
%       motion_model == p(xt | ut, xt_d1)
%       measurement_model = p(zt | xt, m)
%
%   Probabilistic Robotics, Thrun et al., p 238, Table 8.1

    % What is the size of this map?
    N = size(m,1); % num of rows and columns (it's square)
    depth = 4; % theta = 0, 90, 180, 270
    count = N*N*depth; % how many cells are there?
    
    % Initialize variables
    pbarkt = zeros([size(m) depth]);
    pkt = zeros([size(m) depth]);
    
    % Convert from MATLAB depth index into one of the 4 states.
    depth2deg = @(index)(index-1)*(360/depth);
    
    for k = 1:count % line 2 of Table 8.1
        
        % pick a new hypothesis of were the successor pose will be
        % (i.e., what's the probability of being at xt given that I
        % was initially at xt_d1 and my encoders said I went ut
        [rowk, colk, depthk] = ind2sub([N N depth], k);
        xt = [colk-1, rowk-1, depth2deg(depthk)]; % Silly MATLAB!
        
%         if (is_occupied())
%            continue; 
%         end
        
        for i = 1:count % line 3 of Table 8.1
            
            % pick a new initial pose to test
            [rowi, coli, depthi] = ind2sub([N N depth], i);
            xt_d1 = [coli-1, rowi-1, depth2deg(depthi)]; % Silly MATLAB!
            
            pbarkt(rowk,colk,depthk) = pbarkt(rowk,colk,depthk) + ...
                pkt_d1(rowi,coli,depthi)*...
                    motion_model_odometry(xt,ut,xt_d1,motion_params);
        end
        
        
        pkt(rowk,colk,depthk) = pbarkt(rowk,colk,depthk)*...
                    beam_range_finder_model(zt,xt,m); % line 4
        % note that there is no eta -- that normalization is done at the
        % end of this function
    end
    
%     pkt = pkt/sum(pkt(:));
    pkt = pbarkt/sum(pbarkt(:));
end

function flag = is_occupied(xt, xt_d1, m)

end