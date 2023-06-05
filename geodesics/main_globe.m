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

%% Plot sphere
nrP = 30;
fig = figure('Name', 'The journey of Rika, Moses, and Federico', 'NumberTitle', 'off', ...
    'Units', 'normalized', 'Position', [0 0 1 1]);
r_earth = 1;                        % Radius of the earth
theta = linspace(0, pi, nrP);         % Latitude
phi = linspace(-pi, pi, nrP);     % Longitude
[phi, theta] = meshgrid(phi, theta);
x_m = sin(theta) .* cos(phi);
y_m = sin(theta) .* sin(phi);
z_m = cos(theta);
me = mesh(r_earth * x_m, r_earth* y_m, r_earth* z_m);
set(me, 'EdgeColor', [0.5, 0.5, 0.5], 'EdgeAlpha', 0.2);
set(me, 'FaceAlpha', 0.7);
axis equal
axis off

% Plot borderdata
lat = 180 * rand(500, 1) - 90;
lon = 360 * rand(500, 1);
C = load('borderdata.mat');
for k = 1:246
    [xtmp, ytmp, ztmp] = sph2cart(deg2rad(C.lon{k}), deg2rad(C.lat{k}), r_earth);
    hold on;
    plot3(xtmp, ytmp, ztmp, 'Color', [75/255, 75/255, 75/255], 'LineWidth', 2)
end

%% Test curve between Vancouver and Boston 

% Start Vancouver: 49.2827° N (latitude, theta), 123.1207° W (longitude, phi)
[x_V, y_V, z_V] = sph2cart(deg2rad(-123.1207), deg2rad(49.2827), r_earth);
hold on;
scatter3(x_V, y_V, z_V, 120, 'filled', 'MarkerFaceColor', [0.9290 0.6940 0.1250]);

% End Boston: 42.3601° N, 71.0589° W
[x_B, y_B, z_B] = sph2cart(deg2rad(-71.0589), deg2rad(42.3601), r_earth);
hold on;
scatter3(x_B, y_B, z_B, 120, 'filled', 'MarkerFaceColor', [0.4940 0.1840 0.5560]);

% Linear interpolation of spherical coordinates
Phi_VB = linspace(deg2rad(-71.0589), deg2rad(-123.1207), nrP);          
theta_VB = linspace(deg2rad(42.3601), deg2rad(49.2827), nrP);       

% Define the sinusoidal parameters
amp = 0.15;             % Adjust the amplitude as needed
freq = 2;               % Adjust the frequency as needed

% Apply sinusoidal variation to theta_RB
theta_VB = theta_VB + amp * sin(freq * linspace(0, 1, nrP) * 2*pi);

[x_VB, y_VB, z_VB] = sph2cart( Phi_VB, theta_VB, r_earth);        
hold on;
view(3)
plot3(x_VB, y_VB, z_VB, 'LineWidth', 4, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', ':');
view(31, 34);

%% Test curve between Rome and Boston

% Start Rome: 41.9028° N (latitude, theta), 12.4964° E W (longitude, phi)
[x_R, y_R, z_R] = sph2cart(deg2rad(12.4964), deg2rad(41.9028), r_earth);
hold on;
scatter3(x_R, y_R, z_R, 120, 'filled', 'MarkerFaceColor', 'm');

% Linear interpolation of spherical coordinates
Phi_RB = linspace(deg2rad(-71.0589), deg2rad(12.4964), nrP);          
theta_RB = linspace(deg2rad(42.3601), deg2rad(41.9028), nrP); 

% Find the index where theta_RB is closest to the North Pole (90 degrees)
[~, idx] = min(abs(theta_RB - pi/2));

% Shift theta_RB values to make middle point at the North Pole
for k = 2:nrP-1
    theta_RB(k) = theta_RB(k) - theta_RB(idx) + pi/2;
end

% Calculate the round curve through the North Pole
curve_points = nrP/4; % Adjust this value to control the curvature of curve
theta_RB(1:curve_points) = linspace(theta_RB(1), pi/2, curve_points);
theta_RB(end-curve_points+1:end) = linspace(pi/2, theta_RB(end), curve_points);

% Plot curve from R to B
[x_RB, y_RB, z_RB] = sph2cart( Phi_RB, theta_RB, r_earth);
hold on;
plot3(x_RB(1,:), y_RB(1,:), z_RB(1,:), 'LineWidth', 4, 'Color', 'm', 'LineStyle', ':');

%% Test curve between Seoul and Boston

% Start Seoul: 37.5519° N (latitude, theta), 126.9918° E (longitude, phi)
[x_S, y_S, z_S] = sph2cart(deg2rad(126.9918), deg2rad(37.5519), r_earth);
hold on;
scatter3(x_S, y_S, z_S, 120, 'filled', 'MarkerFaceColor', [0 0.4470 0.7410]);

% Linear interpolation of spherical coordinates
phi_seoul = deg2rad(126.9918);
phi_boston = deg2rad(-71.0589);
theta_seoul = deg2rad(37.5519);
theta_boston = deg2rad(42.3601);

% Calculate the shortest distance along the globe
delta_phi = phi_boston - phi_seoul;
if delta_phi > pi
    delta_phi = delta_phi - 2*pi;
elseif delta_phi < -pi
    delta_phi = delta_phi + 2*pi;
end

% Generate the longitude values with the correct wrapping
Phi_SB = linspace(phi_seoul, phi_seoul + delta_phi, nrP);
Phi_SB(Phi_SB > pi) = Phi_SB(Phi_SB > pi) - 2*pi;
Phi_SB(Phi_SB < -pi) = Phi_SB(Phi_SB < -pi) + 2*pi;

% Interpolate the lattitude values
theta_SB = linspace(theta_seoul, theta_boston, nrP);

% Define the sinusoidal parameters
amp_2 = 0.05;             % Adjust the amplitude as needed
freq_2 = 2;               % Adjust the frequency as needed

% Apply sinusoidal variation to theta_RB
theta_SB = theta_SB + amp_2 * sin(freq_2 * linspace(0, 1, nrP) * 2*pi);

% Plot curve from R to B
[x_SB, y_SB, z_SB] = sph2cart( Phi_SB, theta_SB, r_earth);
hold on;
plot3(x_SB(1,:), y_SB(1,:), z_SB(1,:), 'LineWidth', 4, 'Color', [0 0.4470 0.7410], 'LineStyle', ':');

%% Initialize arrays
ddr_VB = zeros(nrP, 1);
ddPhi_VB = zeros(nrP, 1);
theta_VB_next = theta_VB;
Phi_VB_next = Phi_VB;
c_theta_VB = zeros(nrP, 1);
c_Phi_VB = zeros(nrP, 1);

ddr_RB = zeros(nrP, 1);
ddPhi_RB = zeros(nrP, 1);
theta_RB_next = theta_RB;
Phi_RB_next = Phi_RB;
c_theta_RB = zeros(nrP, 1);
c_Phi_RB = zeros(nrP, 1);

ddr_SB = zeros(nrP, 1);
ddPhi_SB = zeros(nrP, 1);
theta_SB_next = theta_SB;
Phi_SB_next = Phi_SB;
c_theta_SB = zeros(nrP, 1);
c_Phi_SB = zeros(nrP, 1);

step = 1;

%% Running the main-loop of simulation

while t <= simTime

    % String update for each point on unit sphere
    for k = 2 : nrP-1

        %% Geodesic from V to B

        % Christoffel Symbols (CS)
        C_1_22_VB = sin(theta_VB(k)) * cos(theta_VB(k));
        C_2_21_VB = cot(theta_VB(k));

        % Update terms, including CS
        c_theta_VB(k) = C_1_22_VB * ( ( Phi_VB(k+1) - Phi_VB(k-1) ) / 2*h )^2;
        c_Phi_VB(k) = ( C_2_21_VB * ( Phi_VB(k+1) - Phi_VB(k-1) ) / 2*h  * ...
            ( theta_VB(k+1) - theta_VB(k-1) ) / 2*h )^2;

        % Acceleration terms
        ddr_VB(k) = ( theta_VB(k+1) - 2 * theta_VB(k) + theta_VB(k-1) ) / h^2;
        ddPhi_VB(k) = ( Phi_VB(k+1) - 2 * Phi_VB(k) + Phi_VB(k-1) ) / h^2;

        % Update
        theta_VB_next(k) = theta_VB(k) + dt * ddr_VB(k) + dt * c_theta_VB(k);
        Phi_VB_next(k) = Phi_VB(k) + dt * ddPhi_VB(k) + dt * c_Phi_VB(k);

        %% Geodesic from R to B

        % Christoffel Symbols (CS)
        C_1_22_RB = sin(theta_RB(k)) * cos(theta_RB(k));
        C_2_21_RB = cot(theta_RB(k));

        % Update terms, including CS
        c_theta_RB(k) = C_1_22_RB * ( ( Phi_RB(k+1) - Phi_RB(k-1) ) / 2*h )^2;
        c_Phi_RB(k) = ( C_2_21_RB * ( Phi_RB(k+1) - Phi_RB(k-1) ) / 2*h  * ...
            ( theta_RB(k+1) - theta_RB(k-1) ) / 2*h )^2;

        % Acceleration terms
        ddr_RB(k) = ( theta_RB(k+1) - 2 * theta_RB(k) + theta_RB(k-1) ) / h^2;
        ddPhi_RB(k) = ( Phi_RB(k+1) - 2 * Phi_RB(k) + Phi_RB(k-1) ) / h^2;

        % Update
        theta_RB_next(k) = theta_RB(k) + dt * ddr_RB(k) + dt * c_theta_RB(k);
        Phi_RB_next(k) = Phi_RB(k) + dt * ddPhi_RB(k) + dt * c_Phi_RB(k);

        %% Geodesic from S to B

        % Christoffel Symbols (CS)
        C_1_22_SB = sin(theta_SB(k)) * cos(theta_SB(k));
        C_2_21_SB = cot(theta_SB(k));

        % Update terms, including CS
        c_theta_SB(k) = C_1_22_SB * ( ( Phi_SB(k+1) - Phi_SB(k-1) ) / 2*h )^2;
        c_Phi_SB(k) = ( C_2_21_SB * ( Phi_SB(k+1) - Phi_SB(k-1) ) / 2*h  * ...
            ( theta_SB(k+1) - theta_SB(k-1) ) / 2*h )^2;

        % Acceleration terms
        ddr_SB(k) = ( theta_SB(k+1) - 2 * theta_SB(k) + theta_SB(k-1) ) / h^2;
        ddPhi_SB(k) = ( Phi_SB(k+1) - 2 * Phi_SB(k) + Phi_SB(k-1) ) / h^2;

        % Update
        theta_SB_next(k) = theta_SB(k) + dt * ddr_SB(k) + dt * c_theta_SB(k);
        Phi_SB_next(k) = Phi_SB(k) + dt * ddPhi_SB(k) + dt * c_Phi_SB(k);

    end

    %% Plot

    % Transform to Cartesian coordinates
    [x_VB, y_VB, z_VB] = sph2cart( Phi_VB_next, theta_VB_next, r_earth);
    [x_RB, y_RB, z_RB] = sph2cart( Phi_RB_next, theta_RB_next, r_earth);
    [x_SB, y_SB, z_SB] = sph2cart( Phi_SB_next, theta_SB_next, r_earth);

    % Plot update every 10th iteration
    % Radius of earth is taken into account in plot!
    if mod(step, 10) == 0

        hold on;
        plot3(x_VB, y_VB, z_VB, 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);
        plot3(x_RB, y_RB, z_RB, 'LineWidth', 0.3, 'Color', [0.4, 0.4, 0.4]);
        plot3(x_SB, y_SB, z_SB, 'LineWidth', 0.2, 'Color', [0.4, 0.4, 0.4]);

    end

    % Update trajectory, time, and step
    theta_VB = theta_VB_next;
    Phi_VB = Phi_VB_next;
    theta_RB = theta_RB_next;
    Phi_RB = Phi_RB_next;
    theta_SB = theta_SB_next;
    Phi_SB = Phi_SB_next;
    t = t + dt;
    step = step + 1;

end

% Plot final geodesic for Euclidean metric
hold on;
plot3(x_VB, y_VB, z_VB, 'LineWidth', 4, 'Color', [0.9290 0.6940 0.1250]);
plot3(x_RB, y_RB, z_RB, 'LineWidth', 4, 'Color', 'm');
plot3(x_SB, y_SB, z_SB, 'LineWidth', 4, 'Color', [0 0.4470 0.7410]);

