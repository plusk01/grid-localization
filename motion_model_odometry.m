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
%
%   Information about params = (a1 a2 a3 a4):
%       "robot-specific paramerters that specify the noise in robot motion"
%       (p. 135 or p. 139). See Figure 5.8 in book for visualization of
%       different params.
    
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
    % noise params
    a1 = params(1);
    a2 = params(2);
    a3 = params(3);
    a4 = params(4);
    % --------------------------------
    
    % Convert degrees to radians
    theta_prime = deg2rad(theta_prime);
    thetabar = deg2rad(thetabar);
    thetabar_prime = deg2rad(thetabar_prime);
    theta = deg2rad(theta);

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
    % prob(a, b^2) is defined on p. 135, 122. It is a zero-mean gaussian
    % with variance b^2, evaluated at a.
    a = constrain(delta_rot1 - deltahat_rot1);
    check(a);
    bb = a1*(deltahat_rot1)^2 + a2*(deltahat_trans)^2;
    p1 = prob(a, bb);
    
    a = delta_trans - deltahat_trans;
    bb = a3*(deltahat_trans)^2 + ...
                        a4*(deltahat_rot1)^2 + a4*(deltahat_rot2)^2;
    p2 = prob(a, bb);
    
    a = constrain(delta_rot2 - deltahat_rot2);
    check(a);
    bb = a1*(deltahat_rot2)^2 + a2*(deltahat_trans)^2;
    p3 = prob(a, bb);
    
    p = p1*p2*p3;
    
    % If you only want to do a 2D example...
%     p = p2;
    
    if isnan(p)
        % Because of the architecture of this MATLAB script, if we get a
        % NaN it is because we are hypothesizing that the robot is in the
        % same place as when it started.
        % We could say that this probability will be zero because in this 
        % setup, `grid_localization` is only run after a robot movement,
        % but to be a little more robust, we give it a 1% chance of not
        % moving even if there was a command. There has to be a better way
        p = .01;
        
        % The probability that I end up in the same spot as I started when
        % the commanded ut is non-zero is low.
        % the probability that I end up in the same spot as I started when
        % the commanded ut is zero is high
        
        dut = ut(4:6) - ut(1:3);
        if any(dut)
            % commanded ut was non-zero
            p = 0.01;
        else
            p = 0.99;
        end
        
    end;
end

function p = prob(a, bb)
    % It would probably be more efficient to redo the math (p. 137) & pass
    % in only the std dev, b instead of the variance bb... but I didn't :)
    
    % zero-mean, Gaussian distribution with variance bb
    p = normpdf(a, 0, sqrt(bb));
end

function angle = constrain(angle)
%CONSTRAIN make the angle be between [-pi, pi].
%   See Probabilistic Robotics, p. 135:
%   "the implementer must observe that all angular differences must lie in
%   [-pi, pi]."
%
%   Code taken from: http://stackoverflow.com/a/2323034/2392520
%   Note that MATLAB's (and Python's) mod() function automatically makes
%   the angle positive, therefore, we comment this line out here.
%
%   `angle` is expected in radians

    % reduce the angle
    angle = mod(angle, 2*pi);
    
    % force it to be the positive remainder, so that 0 <= angle < 360
    %angle = mod((angle + 2*pi), 2*pi);
    
    if (angle > pi)
        angle = angle - 2*pi;
    end;
end

function check(a)
    if (abs(a) > pi)
        disp(a);
        error('outside of pi');
    end
end
