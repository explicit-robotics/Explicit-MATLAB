% [Project]        Robot Simulator - Double Pendulum
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
simTime = 5;        % Total simulation time
t  = 0;             % The current time of simulation
dt = 0.01;          % Time-step of simulation

% Flag for turning on/off animation
is_anim = true;

%% Initialize the robot

m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
 
robot = DoublePendulum( m1, m2, l1, l2 );
robot.init( )

anim = Animation( 'Dimension', 2, 'xLim', [-1.5,1.5], 'yLim', [-2.5,0.5] );
anim.init( );
anim.attachRobot( robot )

%% Initialization of Animation

% Initial Position of the Robot
% The unit of q is degrees for revolute, and meters for prismatic.

% DO NOT CHANGE
% Changing the degrees to radian
q  = [ 50, 90 ]' * pi/180;

% Initial velocity of the robot
dq = zeros( 2, 1 );

% Update robot kinematics with qarray
robot.updateKinematics( q );

% Update animation
anim.update( 0 );


%% Running the main-loop of simulation


while t <= simTime

    % Get the mass matrix of the DoublePendulum
    M = robot.getMassMatrix( q );

    % Get the Coriolis term of the robot
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Gravity term of the robot
    G = robot.getGravityVector( q );
    ddq = M\(-C * dq - G);

    [ q1, dq1 ] = func_symplecticEuler( q, dq, ddq, dt);
    q  =  q1;
    dq = dq1;

    % Update the linkage plot
    robot.updateKinematics( q );

    anim.update( t );

    % Get the forward kinematics of the EE
    H_EE = robot.getForwardKinematics( q );
    t = t + dt;
end

anim.close( )
