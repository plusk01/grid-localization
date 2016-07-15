function q = beam_range_finder_model( zt, xt, m, Theta )
%BEAM_RANGE_FINDER_MODEL Measurement model for the range finder on Thrunbot
%   Probabilistic Robotics, Thrun et al., p 158, Table 6.1
%   Returns the probability of getting the noisy measurement zt given that
%   Thrunbot is currently at the hypothesized successor pose xt and the map
    
    % TODO: For some reason, probabilities are being returned that are
    % greater than 1. Is there a normalizing factor missing in this code,
    % or is it in the grid_localization routine?

    % -- Intrinsic Parameters ---------------------------------------------
    % These should be found using Table 6.2, p. 160:
    % They were empirically chosen.
    % Algorithm learn_intrinsic_parameters(Z,X,m)
    
    if (nargin < 4)
        Theta = [0.85  0.05  0.05  0.05  0.1  4  1];
    end
    
    % mixing weights
    zhit   = Theta(1);
    zshort = Theta(2);
    zmax   = Theta(3);
    zrand  = Theta(4);
    
    % std dev of noise of sensor and sensor's max range
    sigma_hit = Theta(5);
    zmax_range = Theta(6);
    
    % parameter (1/mean) of exponential distribution for short measurements
    lambda_short = Theta(7);
    % ---------------------------------------------------------------------

    K = 1; % there is only one scan associated with a measurement because 
           % we are just using an ultrasonic range finder (i.e., what's 
           % right in front of me...)

    % initialize the probability to 1
    q = 1; % line 2, Table 6.1
    
    for k = 1:K
        ztk = zt; % change this for K > 1
        
        % -- Ray Casting --------------------------------------------------
        % compute ztk* for the measurement ztk using ray casting
        ztk_star = range_finder(xt,m); % use exact range_finder by not
                                       % passing in noise params
                                       
        % I don't really understand this. If we know the exact distance,
        % why are we doing any of this anyways?
        % I suppose 'ray casting' could just mean, use the hypothesized
        % pose and the map and decide what the scans should be.
        % -----------------------------------------------------------------
        
        p = zhit   * phit(ztk,xt,m, sigma_hit,ztk_star,zmax_range)  +...
            zshort * pshort(ztk,xt,m, lambda_short, ztk_star)       +...
            zmax   * pmax(ztk,xt,m, zmax_range)                     +...
            zrand  * prand(ztk,xt,m, zmax_range);
        
        q = q*p; % accrue the probability from each scan in zt
    end;

end

function p = phit(ztk, xt, m, sigma_hit, ztk_star, zmax)
%PHIT Find the probability of being hit
%
    if (0 <= ztk && ztk <= zmax) % eq (6.4)
        eta = normcdf([0 zmax], ztk_star, sigma_hit); % eq (6.6)
        eta = eta(2)-eta(1);
        p = eta*normpdf(ztk, ztk_star, sigma_hit);
    else
        p = 0;
    end;
end

function p = pshort(ztk, xt, m, lambda_short, ztk_star)
%PSHORT Find the probability
%
    if (0 <= ztk && ztk <= ztk_star) % eq (6.7)
        eta = 1/(1-exp(-lambda_short*ztk_star)); % eq (6.9)
        mu = (1/lambda_short);
        p = eta*exppdf(ztk,mu);
    else
        p = 0;
    end;
end

function p = pmax(ztk, xt, m, zmax)
%PMAX Find the probability
%
    if (ztk == zmax) % eq (6.10)
        p = 1;
    else
        p = 0;
    end;
end

function p = prand(ztk, xt, m, zmax)
%PRAND Find the probability
%
    if (0 <= ztk && ztk < zmax) % eq (6.11)
        p = 1/zmax;
    else
        p = 0;
    end;
end