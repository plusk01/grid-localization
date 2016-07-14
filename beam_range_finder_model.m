function q = beam_range_finder_model( zt, xt, m )
%BEAM_RANGE_FINDER_MODEL Measurement model for the range finder on Thrunbot
%   Probabilistic Robotics, Thrun et al., p 158, Table 6.1

    zhit   = 0.85;
    zshort = 0.05;
    pzmax   = 0.05;
    zrand  = 0.05;
    
    sigma_hit = 0.005;
    zmax = 4;
    
    lambda_short = 1;

    K = 1; % there is only one scan associated with a measurement because 
           % we are just using an ultrasonic range finder (i.e., what's 
           % right in front of me...)

    % initialize the probability to 1
    q = 1; % line 2, Table 6.1
    
    for k = 1:K
        ztk = zt; % change this for K > 1
        
        % -- Ray Casting --------------------------------------------------
        % compute ztk* for the measurement ztk using ray casting
        try
            ztk_star = range_finder(xt,m); % use exact range_finder by not
                                       % passing in noise params
        catch exc
            disp(exc)
%             disp([zt xt]);
            ztk_star = 0;
        end;
        
        % I don't really understand this. If we know the exact distance,
        % why are we doing any of this anyways?
        % -----------------------------------------------------------------
        
        p = zhit   * phit(ztk,xt,m, sigma_hit,ztk_star,zmax)   +...
            zshort * pshort(ztk,xt,m, lambda_short, ztk_star) +...
            pzmax   * pmax(ztk,xt,m, zmax)   +...
            zrand  * prand(ztk,xt,m, zmax);
        
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