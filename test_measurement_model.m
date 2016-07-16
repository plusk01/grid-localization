% Set defaults
set(0,'DefaultAxesFontSize',18);
set(0,'DefaultLineLineWidth',2);

% This will give us ztk = 2 in the range finder model
m = zeros(3); xt = [0 0 90];
ztkstar = 2;

% Measurement paramters
% [zhit  zshort  zmax  zrand  sigma_hit  zmax_range  lambda_short]
params = [0.40  0.20  0.25  0.15  0.1  4  .5];
% params = [0.85  0.05  0.05  0.05  0.1  4  1];
% params = [0.15  0.20  0.225  0.125  0.2  4  .5];

% Span of PDF is from 0 <= z <= zmax
zmax = params(6);

count = 1000; % samples
p = zeros(1,count);

ztk = linspace(0,zmax+.5,count);


for k = 1:count
    zt = ztk(k);
    p(k) = beam_range_finder_model(zt,xt,m,params);
end

figure(5), clf;
plot(ztk,p/sum(p(:)))
title(['p(zt | xt, m) with z_{max} = ' num2str(zmax) ' and z_t^{k*} = '...
                                num2str(ztkstar)], 'FontSize', 20);