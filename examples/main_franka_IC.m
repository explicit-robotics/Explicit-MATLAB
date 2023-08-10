% [Project]        Robot Simulator - Snake
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
%
% The code is heavily commented. A famous quote says:
% "Code is read more often than it is written"
%           - Guido Van Rossum, the Creator of Python

%% Cleaning up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 10;        % Total simulation time
t  = 0;             % The current time of simulation
dt = 0.01;          % Time-step of simulation
cnt = 1;

% Set figure size and attach robot to simulation
robot = franka( );
robot.init( );

% Initialize robot parameters
q = robot.q_init;
dq = zeros( robot.nq, 1 );

%% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.5,1], 'yLim', [-0.5,0.5], 'zLim', [0,1] );
anim.init( );
anim.attachRobot( robot )

% Get traceplot to plot robot trajectory
tracePlot = findobj( 'tag', 'tracePlot' );

%% Initialization

% Get initial robot transformation matrix
H_ee_ini = robot.getForwardKinematics( q );
p_ee_ini = H_ee_ini( 1:3, 4 );

% Get initial point on elbow
H_eb_ini = robot.getForwardKinematics( q, 'bodyID', 4  );
p_eb_ini = H_ee_ini( 1:3, 4 );

% Amplitute of circular trajectory
A = 0.07;

%% Cyclic code starts here
while t <= simTime
    tic

    % ========================== %
    % ====== Get robot data ==== %
    % ========================== %

    % Get current robot transformation matrix of end-effector
    H_ee = robot.getForwardKinematics( q );
    p_ee = H_ee( 1:3, 4 );

    % Get current robot transformation matrix of point on elbow
    H_eb = robot.getForwardKinematics( q, 'bodyID', 4  );
    p_eb = H_eb( 1:3, 4 );

    % Get Hybrid Jacobian of a point on end-effector
    J_ee = robot.getHybridJacobian( q );
    J_ee_v = J_ee( 1:3, : );

    % Get Hybrid Jacobian of elbow
    J_eb = robot.getHybridJacobian( q, 'bodyID', 4 );
    J_eb_v = J_eb( 1:3, : );

    % Get mass matrix of robot
    M = robot.getMassMatrix( q );

    % ============================ %
    % ======== Controller   ====== %
    % ============================ %

    % Get 
    p_ee_0 = func_circularInterp( p_ee_ini, A , t, simTime/2, 2 );

    % End-effector control: Move along negative x-axis
    k_ee_l = 200;
    b_ee_l = 20;
    dp_ee = J_ee_v * dq;
    F_ee = k_ee_l * ( p_ee_0 - p_ee ) - b_ee_l * dp_ee;

    tau_ee = J_ee_v' * F_ee;

    % Hold elbow position
    k_eb = 0;
    b_eb = 0;
    dp_eb = J_eb_v * dq;
    F_eb = k_eb * ( p_eb_ini - p_eb ) - b_eb * dp_eb;

    tau_eb = J_eb_v' * F_eb;

    % Add joint damping
    B_q = 0.1;
    tau_q = - B_q * dq;

    % ============================ %
    % ====== Control torque ====== %
    % ============================ %

    tau = tau_ee + tau_q;


    % ============================================= %
    % ======== Proceed one simulation step ======== %
    % ============================================= %

    % Interpolation robot
    rhs = M \ tau;                                                          % We assume gravity and Coriolis are compensated
    [ q, dq ] = func_symplecticEuler( q, dq, rhs, dt );

    % Update the linkage plot
    robot.updateKinematics( q );

    % Update traceplot
    p_tr_x = get( tracePlot, 'XData' );
    p_tr_y = get( tracePlot, 'YData' );
    p_tr_z = get( tracePlot, 'ZData' );
    set( tracePlot, 'XData', [p_tr_x, p_ee(1,1)], 'YData', [p_tr_y, p_ee(2,1)], 'ZData', [p_tr_z, p_ee(3,1)], 'LineWidth', 4 );

    % Update control time and counter
    t = t + dt;
    cnt = cnt + 1;
    anim.update( t );

    % Do not go faster than real time
    while toc < dt
        % do nothing
    end
end

%% Close animation
anim.close();

