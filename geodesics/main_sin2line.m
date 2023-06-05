% [Project]        Finding geodesics via strings -- Discretization of PDE
%   Author                       Email
% Johannes Lachner        jlachner@mit.edu

%% Cleaning up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 7;                % Total simulation time
t  = 0;                     % The current time of simulation
dt = 0.005;                 % Time-step of simulation

% Initial string
f = @(x) sin(x);            % Initial string is a sinusoid
h = pi/30;                  % Sample size of trajectory
x = 0:h:2*pi;               % Number of points on string
y = f(x);
y_next = y;

% Plot initial string
figure;
plot(x, y, '-o', 'MarkerFaceColor', [0.6350 0.0780 0.1840], 'LineWidth', 2, 'Color', [0.6350 0.0780 0.1840]);
xlabel('\boldmath$s [rad]$','Interpreter','latex', 'FontSize', 14);
ylabel('\boldmath$\gamma$ [   ]','Interpreter','latex', 'FontSize', 14);
xlim( [-0.2, (2*pi + 0.2) ] );
ylim( [-1.1, 1.1] );

% Initialize arrays
n = length(x);
ddy = zeros(n, 1);
step = 1;

%% Running the main-loop of simulation

while t <= simTime

    % String update for each point on string
    % For Euclidean metric, Christoffel symbols are zero;
    % Hence, only ddy based on central difference is needed
    for k = 2 : n-1

        ddy(k) = ( y(k+1) - 2 * y(k) + y(k-1) ) / h^2;
        y_next(k) = y(k) + dt * ddy(k);

    end

    % Plot update every 10th iteration
    if mod(step, 10) == 0
        hold on;
        plot(x, y_next, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 0.5);
    end

    % Update trajectory and time
    y = y_next;
    t = t + dt;
    step = step + 1;

end

% Plot final geodesic for Euclidean metric
hold on;
plot(x, y_next, '-o', 'MarkerFaceColor', [0.9290 0.6940 0.1250], 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);

% Finally overlay the start and end point
hold on;
plot( 0, 0, 'pentagram', 'MarkerFaceColor', 'k', 'MarkerSize', 14)
plot( 2*pi, 0, 'pentagram', 'MarkerFaceColor', 'k', 'MarkerSize', 14)
text(-0.18, 0.08, 'A', 'FontSize', 14);
text(2*pi + 0.05, 0.08, 'B', 'FontSize', 14);

