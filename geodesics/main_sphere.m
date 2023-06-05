% [Project]        Finding geodesics via strings -- Discretization of PDE
%   Author                       Email
% Johannes Lachner        jlachner@mit.edu

%% Cleaning up + Environment Setup
clear; clc;
% clear; close all; clc;

% Simulation settings
simTime = 7;                % Total simulation time
t  = 0;                     % The current time of simulation
dt = 0.005;                 % Time-step of simulation
h = pi/30;                  % Arc-length steps: delta s

%% Plot sphere
nrP = 30;
fig = figure;
r_earth = 1;                        % Radius of the earth
theta = linspace(0, pi, nrP);         % Latitude
phi = linspace(0, 2*pi, nrP);     % Longitude
[phi, theta] = meshgrid(phi, theta);
x_m = sin(theta) .* cos(phi);
y_m = sin(theta) .* sin(phi);
z_m = cos(theta);
me = mesh(r_earth * x_m, r_earth* y_m, r_earth* z_m);
set(me, 'EdgeColor', [0.5, 0.5, 0.5], 'EdgeAlpha', 0.3);
set(me, 'FaceAlpha', 0.1);
axis equal
axis off

%% Test curve between points A and B
theta_AB = linspace(0.2, pi/2+0.2, nrP);
theta_AB = sin(theta_AB);
Phi_AB =  linspace(0, -0.9*pi, nrP);
[x_AB, y_AB, z_AB] = sph2cart( Phi_AB, theta_AB, r_earth);           % Disc has zero elevation
hold on;
view(3)
plot3(x_AB(1,:), y_AB(1,:), z_AB(1,:), 'LineWidth', 4, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', ':');
view(-90, 32);

%% Test curve: a great circle (already a geodesic)
theta_CD = linspace(0+pi/2, 3*pi/4+pi/2, nrP);
Phi_CD =  linspace(-3*pi/4, -3*pi/4, nrP);
[x_CD, y_CD, z_CD] = sph2cart( Phi_CD, theta_CD, r_earth);
hold on;
plot3(x_CD(1,:), y_CD(1,:), z_CD(1,:), 'LineWidth', 4, 'Color', 'm', 'LineStyle', ':');

%% Test curve between points E and F
theta_EF = linspace(0.9*pi+pi/2, 0.3*pi+pi/2, nrP);  
Phi_EF =  0.5 .* sin( linspace(0, -0.9*pi, nrP) );
[x_EF, y_EF, z_EF] = sph2cart( Phi_EF, theta_EF, r_earth);
hold on;
plot3(x_EF(1,:), y_EF(1,:), z_EF(1,:), 'LineWidth', 4, 'Color', [0.4940 0.1840 0.5560], 'LineStyle', ':');

%% Initialize arrays
ddr_AB = zeros(nrP, 1);
ddPhi_AB = zeros(nrP, 1);
theta_AB_next = theta_AB;
Phi_AB_next = Phi_AB;
c_theta_AB = zeros(nrP, 1);
c_Phi_AB = zeros(nrP, 1);

ddTheta_CD = zeros(nrP, 1);
ddPhi_CD = zeros(nrP, 1);
theta_CD_next = theta_CD;
Phi_CD_next = Phi_CD;
c_theta_CD = zeros(nrP, 1);
c_Phi_CD = zeros(nrP, 1);

ddTheta_EF = zeros(nrP, 1);
ddPhi_EF = zeros(nrP, 1);
theta_EF_next = theta_EF;
Phi_EF_next = Phi_EF;
c_theta_EF = zeros(nrP, 1);
c_Phi_EF = zeros(nrP, 1);

step = 1;

%% Running the main-loop of simulation

while t <= simTime

    % String update for each point on unit sphere
    for k = 2 : nrP-1

        %% Geodesic from A to B

        % Christoffel Symbols (CS)
        C_1_22_AB = sin(theta_AB(k)) * cos(theta_AB(k));
        C_2_21_AB = cot(theta_AB(k));

        % Update terms, including CS
        c_theta_AB(k) = C_1_22_AB * ( ( Phi_AB(k+1) - Phi_AB(k-1) ) / 2*h )^2;
        c_Phi_AB(k) = ( C_2_21_AB * ( Phi_AB(k+1) - Phi_AB(k-1) ) / 2*h  * ( theta_AB(k+1) - theta_AB(k-1) ) / 2*h )^2;

        % Acceleration terms
        ddr_AB(k) = ( theta_AB(k+1) - 2 * theta_AB(k) + theta_AB(k-1) ) / h^2;
        ddPhi_AB(k) = ( Phi_AB(k+1) - 2 * Phi_AB(k) + Phi_AB(k-1) ) / h^2;

        % Update
        theta_AB_next(k) = theta_AB(k) + dt * ddr_AB(k) + dt * c_theta_AB(k);
        Phi_AB_next(k) = Phi_AB(k) + dt * ddPhi_AB(k) + dt * c_Phi_AB(k);

        %% Geodesic from C to D

        % Christoffel Symbols (CS)
        C_1_22_CD = sin(theta_CD(k)) * cos(theta_CD(k));
        C_2_21_CD = cot(theta_CD(k));

        % Update terms, including CS
        c_theta_CD(k) = C_1_22_CD * ( Phi_CD(k+1) - Phi_CD(k-1) )^2 / ( 2*h );
        c_Phi_CD(k) = ( C_2_21_CD * ( Phi_CD(k+1) - Phi_CD(k-1) )^2 / ( 2*h ) * ( theta_CD(k+1) - theta_CD(k-1) )^2 / ( 2*h ) )^2;

        % Acceleration terms
        ddTheta_CD(k) = ( theta_CD(k+1) - 2 * theta_CD(k) + theta_CD(k-1) ) / h^2;
        ddPhi_CD(k) = ( Phi_CD(k+1) - 2 * Phi_CD(k) + Phi_CD(k-1) ) / h^2;

        % Update
        theta_CD_next(k) = theta_CD(k) + dt * ddTheta_CD(k) + dt * c_theta_CD(k);
        Phi_CD_next(k) = Phi_CD(k) + dt * ddPhi_CD(k) + dt * c_Phi_CD(k);

        %% Geodesic from E to F

        % Christoffel Symbols (CS)
        C_1_22_EF = sin(theta_EF(k)) * cos(theta_EF(k));
        C_2_21_EF = cot(theta_EF(k));

        % Update terms, including CS
        c_theta_EF(k) = C_1_22_EF * ( Phi_EF(k+1) - Phi_EF(k-1) )^2 / ( 2*h );
        c_Phi_EF(k) = ( C_2_21_EF * ( Phi_EF(k+1) - Phi_EF(k-1) )^2 / ( 2*h ) * ( theta_EF(k+1) - theta_EF(k-1) )^2 / ( 2*h ) )^2;

        % Acceleration terms
        ddTheta_EF(k) = ( theta_EF(k+1) - 2 * theta_EF(k) + theta_EF(k-1) ) / h^2;
        ddPhi_EF(k) = ( Phi_EF(k+1) - 2 * Phi_EF(k) + Phi_EF(k-1) ) / h^2;

        % Update
        theta_EF_next(k) = theta_EF(k) + dt * ddTheta_EF(k) + dt * c_theta_EF(k);
        Phi_EF_next(k) = Phi_EF(k) + dt * ddPhi_EF(k) + dt * c_Phi_EF(k);

    end

    %% Plot

    % Transform to Cartesian coordinates
    [x_AB, y_AB, z_AB] = sph2cart( Phi_AB_next, theta_AB_next, r_earth);
    [x_CD, y_CD, z_CD] = sph2cart( Phi_CD_next, theta_CD_next, r_earth);
    [x_EF, y_EF, z_EF] = sph2cart( Phi_EF_next, theta_EF_next, r_earth);

    % Plot update every 10th iteration
    % Radius of earth is taken into account in plot!
    if mod(step, 10) == 0

        hold on;
        plot3(x_AB(1,:), y_AB(1,:), z_AB(1,:), 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);
        plot3(x_CD(1,:), y_CD(1,:), z_CD(1,:), 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);
        plot3(x_EF(1,:), y_EF(1,:), z_EF(1,:), 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);

    end

    % Update trajectory, time, and step
    theta_AB = theta_AB_next;
    Phi_AB = Phi_AB_next;
    theta_CD = theta_CD_next;
    Phi_CD = Phi_CD_next;
    theta_EF = theta_EF_next;
    Phi_EF = Phi_EF_next;
    t = t + dt;
    step = step + 1;

end

% Plot final geodesic for Euclidean metric
hold on;
plot3(x_AB(1,:), y_AB(1,:), z_AB(1,:), 'LineWidth', 4, 'Color', [0.9290 0.6940 0.1250]);
plot3(x_CD(1,:), y_CD(1,:), z_CD(1,:), 'LineWidth', 4, 'Color', 'm');
plot3(x_EF(1,:), y_EF(1,:), z_EF(1,:), 'LineWidth', 4, 'Color', [0.4940 0.1840 0.5560]);

