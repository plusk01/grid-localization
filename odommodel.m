
m = [0 0 0;...
     0 0 0;...
     0 0 0;];
 

% What is the size of this map?
N = size(m,1); % num of rows and columns (it's square)
count = N*N*4; % how many cells are there?

% Initialize variables
pbarkt = zeros([size(m) 4]);
pkt = zeros([size(m) 4]);

% identify the odometry motion (where I was --> where I'm going)
ut = [ [1 1 0] [2 1 0] ];

pkt_d1 = zeros(N,N,4);

pkt_d1(:,:,1) = [0 0 0;... % 0 deg
                 0 1 0;...
                 0 0 0;];
             
pkt_d1(:,:,2) = [0 0 0;... % 90 deg
                 0 0 0;...
                 0 0 0;];
             
pkt_d1(:,:,3) = [0 0 0;... % 180 deg
                 0 0 0;...
                 0 0 0;];
             
pkt_d1(:,:,4) = [0 0 0;... % 270 deg
                 0 0 0;...
                 0 0 0;];
  
  % motion params
  p = [.18 .4 .18 .0025];
  

for k = 1:count
    
    % pick a new hypothesis of were the successor pose will be
    % (i.e., what's the probability of being at xt given that I
    % was initially at xt_d1 and my encoders said I went ut
    [rowk, colk, depthk] = ind2sub([N N 4], k); 
    xt = [colk-1, rowk-1, (depthk-1)*90]; % Silly MATLAB!
    rowk = N+1 - rowk;
  
    for i = 1:count % line 3 of Table 8.1

        % pick a new initial pose to test
        [rowi, coli, depthi] = ind2sub([N N 4], i);
        xt_d1 = [coli-1, rowi-1, (depthi-1)*90]; % Silly MATLAB!
        rowi = N+1 - rowi;

        pbarkt(rowk,colk,depthk) = pbarkt(rowk,colk,depthk) + ...
            pkt_d1(rowi,coli,depthi)*...
                motion_model_odometry(xt,ut,xt_d1,p);
    end
end

pbarkt = pbarkt/sum(pbarkt(:));

flat = sum(pkt_d1,3);
showProbabilities(3,flat,ut(1:3))

flat = sum(pbarkt,3);
showProbabilities(4,flat,ut(4:6))