function [ pkt ] = grid_localization( pkt_d1, ut, zt, m )
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
    N = size(map,1); % num of rows and columns (it's square)
    count = N*N; % how many cells are there?
    
    % Initialize variables
    pbarkt = zeros(1,count);
    
    
    for k = 1:count
        for i = 1:count
            pbarkt(k) = pbarkt(k) + ...
                        pkt_d1(i)*motion_model_odometry(xt, ut, xt_d1);
        end
        pkt = eta*pbarkt * beam_range_finder_model();
    end

end

