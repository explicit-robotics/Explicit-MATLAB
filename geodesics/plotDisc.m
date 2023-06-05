function [figure] = plotDisc(figure, radius, nrPoints, nrCircles)
%PLOTDISC Summary of this function goes here
%   Detailed explanation goes here
% First circles
r = linspace(0, radius, nrPoints);
Phi = linspace(0, 2*pi, nrPoints);
x = zeros(nrPoints);
y = zeros(nrPoints);
for nrC = 1:nrCircles
    for i = 1:nrPoints
        x(i) = nrC * r(nrPoints) / 4 * sin(Phi(i));
        y(i) = nrC * r(nrPoints) / 4 * cos(Phi(i));
    end
    hold on;
    plot(x, y, 'LineWidth', 1, 'Color', [0.5, 0.5, 0.5]);
end
hold on;
plot(x, y, 'LineWidth', 2, 'Color', 'k');
axis equal;
% Now lines
for ang = 0 : pi/4 : 2*pi
    x_lin = r(nrPoints) * sin(ang);
    y_lin = r(nrPoints) * cos(ang);
    hold on;
    plot([0,x_lin], [0,y_lin], 'Color', 'b');
end

end

