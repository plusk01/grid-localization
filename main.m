%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Grid Localization using a Discrete Bayes Filter for the Thrunbot.
% 
% Assumptions:
%   the robot is like a taxicab in Manhattan (i.e., no diagonals)
%   Thrunbot's encoders are perfect...?
%
% See https://www.cs.princeton.edu/courses/archive/fall11/
%                           cos495/COS495-Lecture14-MarkovLocalization.pdf
% for more information about stepping through the algorithm.
%
% Parker Lusk, BYU
% 12 July 2016
%
% Probabilistic Robotics, Thrun et al.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;
% === Robot Parameters ====================================================
range_max = 10; % how many empty cells can the range finder see ahead
range_sigma = 0.01; % std dev of range finder
ranger_params = [range_max range_sigma];
% Odometry motion model error params (p. 139, 135, or Figure 5.8)
motion_params = [.18 .4 .18 .0025]; % tuned by hand in `motion_model_test`
% =========================================================================

% Load the 'map' variable
load('map.mat');

% What is the size of this map?
N = size(map,1); % num of rows and columns (it's square)

% Create the grid
grid_fig = 1; % Which figure to draw on
grid = drawGrid(grid_fig, map);

% Initialize the probability map to be equally likely
depth = 4; % The depth of discretization. How many different headings can
           % the robot have? 0, 90, 180, 270
probs = ones([N N depth])/(N*N*depth);

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
                3 7 90;...
                3 8 90;...
];

moves = [... %  x y thetad 
                1 1 0;...
                2 1 0;...
                3 1 0;...
                4 1 0;...
                5 1 0;...
                6 1 0;...
                7 1 0;...
                8 1 0;...
];

moves = [... %  x y thetad 
                1 1 90;...
                1 2 90;...
                1 3 90;...
                1 4 90;...
                1 5 90;...
                1 6 90;...
                1 7 90;...
                1 8 90;...
];

moves = [... %  x y thetad 
                0 0 0;...
                0 0 90;...
                0 1 90;...
                0 1 0;...
                1 1 0;...
                2 1 0;...
];

moves = [... %  x y thetad 
                0 0 0;...
                1 0 0;...
                2 0 0;...
                2 0 90;...
                2 1 90;...
];

% initialize xt
xt = moves(1,:);

% Initializes the robot starting point (x, y, theta (deg))
figure(grid_fig); % select the grid figure
bot = drawRobot(xt, []);

% Show the initial global belief of where the robot is
showProbabilities(prob_fig, probs, xt);

pause;

for i = 2:size(moves,1)

    % Move the robot
    xt_d1 = xt;
    xt = moves(i,:);
    bot = moveRobot(xt, bot);

    % Get the odometry (the commanded ut vector), see p. 134
    ut = [xt_d1 xt];
    % dt = ut(4:6) - ut(1:3);

    % Get the range measurements (abs() since it shouldn't be negative)
    zt = abs(range_finder(xt, map, ranger_params));

    % Do grid localization
    probs_d1 = probs; % for safekeeping
    probs = grid_localization(probs, ut, zt, map, motion_params);
    
    % Update probability map
    showProbabilities(prob_fig, probs, xt);
       
    % Wait for the user to press a key
    pause;

end

% -------------------------------------------------------------------------