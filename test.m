[X,Y] = meshgrid(0:1:5, 0:1:5);
X = X(:);
Y = Y(:);
plot(X,Y,'.');
xlabel('X'); % // Label the X and Y axes
ylabel('Y');
grid on;

drawGrid(1, map);
handle = [];
handle = drawRobot([1,6,270], handle);