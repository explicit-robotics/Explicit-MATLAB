% [Project]        Finding geodesics via strings -- Discretization of PDE
%   Author                       Email
% Johannes Lachner        jlachner@mit.edu

%% Cleaning up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 7;                % Total simulation time
t  = 0;                     % The current time of simulation
dt = 0.005;                 % Time-step of simulation
h = pi/30;                  % Arc-length steps: delta s

%% Disc with radius r and angle Phi
nrP = 30;
r_earth = 1;            % Adapt later
r = linspace(0, r_earth, nrP);
Phi = linspace(0, 2*pi, nrP);

%% Plot disc
fig = figure;
fig = plotDisc(fig, r_earth, 200, 4);

%% Test curve between points A and B
r_AB = linspace(0.2, pi/2 + 0.2, nrP);
r_AB = sin(r_AB);
Phi_AB =  linspace(0, 0.9*pi, nrP);
[x_AB, y_AB, z_AB] = sph2cart( Phi_AB, zeros(nrP), r_AB);           % Disc has zero elevation
hold on;
plot(x_AB(1,:), y_AB(1,:), 'LineWidth', 3, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', ':');
axis off;

%% Test curve: a great circle (already a geodesic)
r_CD = linspace(0, 3*r(nrP)/4, nrP);
Phi_CD =  linspace(-3*pi/4, -3*pi/4, nrP);
[x_CD, y_CD, z_CD] = sph2cart( Phi_CD, zeros(nrP), r_CD);
hold on;
plot(x_CD(1,:), y_CD(1,:), 'LineWidth', 3, 'Color', 'm', 'LineStyle', ':');

%% Test curve between points E and F
r_EF = linspace(0.9*r(nrP), 0.3*r(nrP), nrP);  
Phi_EF =  0.5 .* sin( linspace(0, -0.9*pi, nrP) );
[x_EF, y_EF, z_EF] = sph2cart( Phi_EF, zeros(nrP), r_EF);
hold on;
plot(x_EF(1,:), y_EF(1,:), 'LineWidth', 3, 'Color', [0.4940 0.1840 0.5560], 'LineStyle', ':');

%% Initialize arrays
ddr_AB = zeros(nrP, 1);
ddPhi_AB = zeros(nrP, 1);
r_AB_next = r_AB;
Phi_AB_next = Phi_AB;
c_r_AB = zeros(nrP, 1);
c_Phi_AB = zeros(nrP, 1);

ddr_CD = zeros(nrP, 1);
ddPhi_CD = zeros(nrP, 1);
r_CD_next = r_CD;
Phi_CD_next = Phi_CD;
c_r_CD = zeros(nrP, 1);
c_Phi_CD = zeros(nrP, 1);

ddr_EF = zeros(nrP, 1);
ddPhi_EF = zeros(nrP, 1);
r_EF_next = r_EF;
Phi_EF_next = Phi_EF;
c_r_EF = zeros(nrP, 1);
c_Phi_EF = zeros(nrP, 1);

step = 1;

%% Running the main-loop of simulation

while t <= simTime

    % String update for each point on unit sphere
    for k = 2 : nrP-1

        %% Geodesic from A to B

        % Christoffel Symbols (CS)
        C_1_22_AB = sin(r_AB(k)) * cos(r_AB(k));
        C_2_21_AB = cot(r_AB(k));

        % Update terms, including CS
        c_r_AB(k) = C_1_22_AB * ( ( Phi_AB(k+1) - Phi_AB(k-1) ) / 2*h )^2;
        c_Phi_AB(k) = ( C_2_21_AB * ( Phi_AB(k+1) - Phi_AB(k-1) ) / 2*h  * ( r_AB(k+1) - r_AB(k-1) ) / 2*h )^2;

        % Acceleration terms
        ddr_AB(k) = ( r_AB(k+1) - 2 * r_AB(k) + r_AB(k-1) ) / h^2;
        ddPhi_AB(k) = ( Phi_AB(k+1) - 2 * Phi_AB(k) + Phi_AB(k-1) ) / h^2;

        % Update
        r_AB_next(k) = r_AB(k) + dt * ddr_AB(k) + dt * c_r_AB(k);
        Phi_AB_next(k) = Phi_AB(k) + dt * ddPhi_AB(k) + dt * c_Phi_AB(k);

        %% Geodesic from C to D

        % Christoffel Symbols (CS)
        C_1_22_CD = sin(r_CD(k)) * cos(r_CD(k));
        C_2_21_CD = cot(r_CD(k));

        % Update terms, including CS
        c_r_CD(k) = C_1_22_CD * ( Phi_CD(k+1) - Phi_CD(k-1) )^2 / ( 2*h );
        c_Phi_CD(k) = ( C_2_21_CD * ( Phi_CD(k+1) - Phi_CD(k-1) )^2 / ( 2*h ) * ( r_CD(k+1) - r_CD(k-1) )^2 / ( 2*h ) )^2;

        % Acceleration terms
        ddr_CD(k) = ( r_CD(k+1) - 2 * r_CD(k) + r_CD(k-1) ) / h^2;
        ddPhi_CD(k) = ( Phi_CD(k+1) - 2 * Phi_CD(k) + Phi_CD(k-1) ) / h^2;

        % Update
        r_CD_next(k) = r_CD(k) + dt * ddr_CD(k) + dt * c_r_CD(k);
        Phi_CD_next(k) = Phi_CD(k) + dt * ddPhi_CD(k) + dt * c_Phi_CD(k);

        %% Geodesic from E to F

        % Christoffel Symbols (CS)
        C_1_22_EF = sin(r_EF(k)) * cos(r_EF(k));
        C_2_21_EF = cot(r_EF(k));

        % Update terms, including CS
        c_r_EF(k) = C_1_22_EF * ( Phi_EF(k+1) - Phi_EF(k-1) )^2 / ( 2*h );
        c_Phi_EF(k) = ( C_2_21_EF * ( Phi_EF(k+1) - Phi_EF(k-1) )^2 / ( 2*h ) * ( r_EF(k+1) - r_EF(k-1) )^2 / ( 2*h ) )^2;

        % Acceleration terms
        ddr_EF(k) = ( r_EF(k+1) - 2 * r_EF(k) + r_EF(k-1) ) / h^2;
        ddPhi_EF(k) = ( Phi_EF(k+1) - 2 * Phi_EF(k) + Phi_EF(k-1) ) / h^2;

        % Update
        r_EF_next(k) = r_EF(k) + dt * ddr_EF(k) + dt * c_r_EF(k);
        Phi_EF_next(k) = Phi_EF(k) + dt * ddPhi_EF(k) + dt * c_Phi_EF(k);

    end

    %% Plot

    % Transform to Cartesian coordinates
    [x_AB, y_AB, z_AB] = sph2cart( Phi_AB_next, zeros(nrP), r_AB_next);
    [x_CD, y_CD, z_CD] = sph2cart( Phi_CD_next, zeros(nrP), r_CD_next);
    [x_EF, y_EF, z_EF] = sph2cart( Phi_EF_next, zeros(nrP), r_EF_next);

    % Plot update every 10th iteration
    % Radius of earth is taken into account in plot!
    if mod(step, 10) == 0

        hold on;
        plot(x_AB(1,:), y_AB(1,:), 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);
        plot(x_CD(1,:), y_CD(1,:), 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);
        plot(x_EF(1,:), y_EF(1,:), 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);

    end

    % Update trajectory, time, and step
    r_AB = r_AB_next;
    Phi_AB = Phi_AB_next;
    r_CD = r_CD_next;
    Phi_CD = Phi_CD_next;
    r_EF = r_EF_next;
    Phi_EF = Phi_EF_next;
    t = t + dt;
    step = step + 1;

end

% Plot final geodesic for Euclidean metric
hold on;
plot(x_AB(1,:), y_AB(1,:), 'LineWidth', 3, 'Color', [0.9290 0.6940 0.1250]);
plot(x_CD(1,:), y_CD(1,:), 'LineWidth', 3, 'Color', 'm');
plot(x_EF(1,:), y_EF(1,:), 'LineWidth', 3, 'Color', [0.4940 0.1840 0.5560]);

% Plot description of figure
text(-0.05, 1.05, '$\pi/2$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(-0.1, -1.08, '$-\pi/2$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(1.05, 0, '$0$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(-1.1, 0, '$\pi$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(-0.92, 0.75, '$3\pi/4$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(-1.0, -0.75, '$-3\pi/4$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(0.75, 0.75, '$\pi/4$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(0.75, -0.75, '$-\pi/4$', 'interpreter', 'latex', 'FontSize', 14, 'Color', 'b');
text(-0.1, 0.03, '$0$', 'interpreter', 'latex', 'FontSize', 14);
text(-0.62, 0.2, '$\pi/2$', 'interpreter', 'latex', 'FontSize', 14);
text(-1.02, 0.4, '$\pi$', 'interpreter', 'latex', 'FontSize', 14);

