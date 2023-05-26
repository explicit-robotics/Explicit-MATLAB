% Project                       Robot Simulator - Triple Pendulum with Cart
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
%
% The code is heavily commented. A famous quote says:
% "Code is read more often than it is written"
%  - Guido Van Rossum, the Creator of Python

%% Cleaning up + Environment setup
clear; close all; clc;

% Simulation settings
simTime = 6;        % Total simulation time
t  = 0;             % The current time of simulation
dt = 0.01;          % Time-step of simulation


%% Initialize the robots

% Our robot is the CartPole
robot = CartPole( 1, 1, 1 );

% We want to add an DoublePendulum with two DOF
extKin = DoublePendulum( 1, 1, 1, 1 );

robot.init( );
extKin.init( );

new_robot = robot.addKinematics( extKin );

% Robot has to be initialized to adapt the joint twists for new robot
new_robot.init( );

%% Initialization of Animation

% Set figure size and attach robot to simulation

anim = Animation( 'Dimension', 2, 'xLim', [ -3 , 3 ], 'yLim', [ -5 , 1 ] );
anim.init( )
anim.attachRobot( new_robot )


% Changing the degrees to radian
q  = [ 0, 0.1,0.1,0.2 ]';

dq    = zeros( new_robot.nq, 1 );
ddq   = zeros( new_robot.nq, 1 );

% Update robot kinematics
new_robot.updateKinematics( q );

% Update animation to initial configuration
anim.update( 0 );


%% Get initial robot transformation for both tasks

% Initial transformation and position of end-effector
H_ee_ini = new_robot.getForwardKinematics( q );
p_ee_ini = H_ee_ini( 1:3 , 4 );

% Initial transformation and position of joint 3
bodyID_3 = 3;
H_j3_ini = new_robot.getForwardKinematics( q, 'bodyID', bodyID_3 , 'position', [0,0,0]');
x_j3_ini = H_j3_ini( 1:3 , 4 );

%% Draw equilibrium points for poth tasks

% End-effector task: Hold initial position
plot3( p_ee_ini( 1 , 1 ) , p_ee_ini( 2 , 1 ) , p_ee_ini( 3 , 1 ) , ...
    '-o' , 'Color' , 'r' , 'MarkerSize' , 8, 'MarkerFaceColor' , 'r');

% Joint 3: Move along x-axis
A_x = 0.6;
plot3( x_j3_ini( 1 , 1 ) + A_x , x_j3_ini( 2 , 1 ) , x_j3_ini( 3 , 1 ) , ...
    '-o' , 'Color' , 'b' , 'MarkerSize' , 8 , 'MarkerFaceColor' , 'b');

%% Running the main-loop of simulation

while t <= simTime

    % ============================ %
    % ====== Get robot data ====== %
    % ============================ %

    % Get the position on end-effector body
    H_ee = new_robot.getForwardKinematics( q );
    p_ee = H_ee( 1:3 , 4 );

    % Get the position on joint 3
    H_j3 = new_robot.getForwardKinematics( q, 'bodyID', bodyID_3, 'position', [0,0,0]');
    x_j3 = H_j3( 1:3 , 4 );

    % Get the mass matrix of the robot
    M = new_robot.getMassMatrix( q );
    M_inv = M\eye( size( M ) );

    % Get Hybrid Jacobian of point on end-effector
    JH_ee = new_robot.getHybridJacobian( q );
    JH_ee = JH_ee( 1:3 , : );

    % Get Hybrid Jacobian of point on joint 3
    JH_j3 = new_robot.getHybridJacobian( q, 'bodyID', bodyID_3 );
    JH_j3 = JH_j3( 1:3 , : );

    % ============================ %
    % ======== Trajectory ======== %
    % ============================ %

    % Move body of joint 3 along x-axis
    x_j3_0 = x_j3_ini;
    t_final = 3;

    if t <= t_final
        ratio = t / t_final;
        if ratio > 1
            ratio = 1;
        end
        x_j3_0( 1 , 1 ) = x_j3_ini( 1 , 1 ) + 0.5 * A_x * (1 - cos(pi * t/t_final));
        x_last = x_j3_0;
    else
        x_j3_0 = x_last;
    end

    % ============================ %
    % ======== Controller ======== %
    % ============================ %

    % End-effector control: Hold initial position
    K_ee = 400 * eye(3);
    B_ee = 40 * eye(3);
    B_ee( 1 , 1 ) = 3;
    dp_ee = JH_ee * dq;
    f_ee = K_ee * ( p_ee_ini - p_ee ) - B_ee * dp_ee;
    tau_ee = JH_ee' * f_ee;

    % Control of point on joint 3: Move along x-axis
    K_j3 = 60 * eye(3);
    B_j3 = 10 * eye(3);
    dp_3 = JH_j3 * dq;
    tau_j3 = JH_j3' * ( K_j3 * ( x_j3_0 - x_j3 ) - B_j3 * dp_3 );

    % Add joint damping
    B_ns = 0.1 * eye( new_robot.nq );
    tau_ns = -B_ns * dq;

    % Superposition of torques
    tau = tau_ee + tau_j3 + tau_ns;

    % ============================ %
    % ======== Animation ========= %
    % ============================ %

    % proceed one simulation step
    % rhs = M\(torque -c - g);        % if coriolis and gravity should be calculated
    rhs = M\(tau);                    % forgo coriolis and gravity for simulation

    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt);
    q  =  q1;
    dq = dq1;

    % Update the linkage plot
    new_robot.updateKinematics( q );

    anim.update( t );


    % Proceed one time step
    t = t + dt;

end
