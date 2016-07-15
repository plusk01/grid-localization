% This will give us ztk = 1 in the range finder model
m = zeros(3); xt = [1 1 0];

% Measurement paramters
% [zhit  zshort  zmax  zrand  sigma_hit  zmax_range  lambda_short]
params = [0.45  0.10  0.30  0.15  0.1  4  1];

% Span of PDF is from 0 <= z <= zmax
zmax = 4;

count = 1000; % samples
p = zeros(1,count);

ztk = linspace(0,zmax+.5,count);


for k = 1:count
    zt = ztk(k);
    p(k) = beam_range_finder_model(zt,xt,m,params);
end

figure(5), clf;
plot(ztk,p/sum(p(:)))
title(['p(zt | xt, m) with z_{max} = ' num2str(zmax)]);