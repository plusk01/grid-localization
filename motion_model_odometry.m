function p = motion_model_odometry( xt, ut, xt_d1, params )
%MOTION_MODEL Motion Model using Odometry for p(xt | ut, xt_d1)
%   Probabilistic Robotics, Thrun et al., p 134
%   inputs:
%       xt = (x_prime y_prime theta_prime): hypothesized successor pose
%       ut = (xbart_d1 xbart): motion information based on odometry
%       xt_d1 = (x y theta): initial robot pose
%       params = (a1 a2 a3 a4): noise parameters
%   output:
%       p: the probability of being at the hypothesized pose xt
%
%   Information about ut = (xbart_d1 xbart):
%       this is a relative advance from xbart_d1 to xbart. Basically a
%       difference. (p. 133)
%       xbart_d1 = (xbar ybar thetabar)
%       xbart = (xbar_prime ybar_prime thetabar_prime)

    % -- Unpack values ---------------
    % hypothesized pose (xt)
    x_prime = xt(1);
    y_prime = xt(2);
    theta_prime = xt(3);
    % odometry motion information (ut)
    xbar = ut(1);
    ybar = ut(2);
    thetabar = ut(3);
    xbar_prime = ut(4);
    ybar_prime = ut(5);
    thetabar_prime = ut(6);
    % initial pose (xt_d1)
    x = xt_d1(1);
    y = xt_d1(2);
    theta = xt_d1(3);
    % --------------------------------
    

    % Calculate the relative motion based on what our odometry told us
    delta_rot1 = atan2(ybar_prime - ybar, xbar_prime - xbar);
    delta_trans = sqrt((xbar - xbar_prime)^2 + (ybar - ybar_prime)^2);
    delta_rot2 = thetabar_prime - thetabar - delta_rot1;
    
    % Calculate what the relative motion would be if we were to end up at
    % the hypothesized successor pose, xt
    deltahat_rot1 = atan2(y_prime - y, x_prime - x);
    deltahat_trans = sqrt((x - x_prime)^2 + (y - y_prime)^2);
    deltahat_rot2 = theta_prime - theta - deltahat_rot1;
    
    % Use the rotational and translational deltas to sample a PDF and get
    % the correct posterior probability. This computes p(xt | ut, xt_d1).
    mu = delta_rot1 - deltahat_rot1;
    sigma_sq = a1*(deltahat_rot1)^2 + a2*(deltahat_trans)^2;
    p1 = prob(mu, sigma_sq);
    
    mu = delta_trans - deltahat_trans;
    sigma_sq = a3*(deltahat_trans)^2 + ...
                        a4*(deltahat_rot1)^2 + a4*(deltahat_rot2)^2;
    p2 = prob(mu, sigma_sq);
    
    mu = delta_rot2 - deltahat_rot2;
    sigma_sq = a1*(deltahat_rot2)^2 + a2*(deltahat_trans)^2;
    p3 = prob(mu, sigma_sq);
    
    p = p1*p2*p3;
end

function p = prob(mu, sigma_sq)
    
end

