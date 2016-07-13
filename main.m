%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Grid Localization using a Discrete Bayes Filter for the Thrunbot.
% 
% Assumptions:
%   the robot is like a taxicab in Manhattan (i.e., no diagonals)
%   Thrunbot's encoders are perfect...?
%
% Parker Lusk, BYU
% 12 July 2016
%
% Probabilistic Robotics, Thrun et al.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;
% === Robot Parameters ====================================================
range_max = 4; % how many unobstructed cells can the range finder see ahead
range_sigma = 0.01; % std dev of range finder
ranger_params = [range_max range_sigma];
% =========================================================================

% Load the 'map' variable
load('map.mat');

% What is the size of this map?
N = size(map,1); % num of rows and columns (it's square)

% Initializes the robot starting point (x, y, theta (deg))
x0 = [floor(N/2) floor(N/2) 90];

% Create the grid
grid_fig = 1; % Which figure to draw on
grid = drawGrid(grid_fig, map);

% Create the robot on the grid
bot = drawRobot(x0, []);

% Initialize the probability map to be equally likely
probs = ones(N)/(N*N);

% Create the surface plot for probabilities
prob_fig = 2; % Which figure to draw on
showProbabilities(prob_fig, probs);

% Wait for the user to get windows positioned, etc
pause;

% -------------------------------------------------------------------------
% --- Robot Motion and Localization ---------------------------------------
% -------------------------------------------------------------------------

% A list of moves to make
moves = [... %  x y thetad 
                5 5 90;...
                5 6 90;...
                5 7 90;...
                4 7 90;...
];

% initialize xt
xt = x0;

for i = 1:size(moves,1)

    % Move the robot
    xt_d1 = xt;
    xt = moves(i,:);
    bot = moveRobot(xt, bot);

    % Get the odometry (the commanded ut vector), see p. 134
    ut = [xt_d1 xt];
    % dt = ut(4:6) - ut(1:3);

    % Get the range measurements
    zt = range_finder(xt, map, ranger_params);

    % Do grid localization
    probs_d1 = probs; % for safekeeping
    probs = grid_localization(probs, ut, zt, map);
    
    % Wait for the user to press a key
    pause;

end

% -------------------------------------------------------------------------