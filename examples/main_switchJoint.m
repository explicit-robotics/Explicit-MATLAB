% [Project]        Robot Simulator - Demonstration for changing joints
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

%% Initialize Robot 1, which is only filled with Rotational Joints

nq = 4;
robot1 = SnakeBot( nq, ones( 1, nq ), ones( 1, nq ) );
robot1.init( )

anim1 = Animation( 'Dimension', 2, 'xLim', [-6,6], 'yLim', [-6,6] );
anim1.init( )
anim1.attachRobot( robot1 )    


%% Initialize Robot 2, where the 3rd joint is Prismatic
nq = 4;
robot2 = SnakeBot( nq, ones( 1, nq ), ones( 1, nq ) );
robot2.init( )

robot2.switchJoint( 3, 2, [ 1; 0; 0] );

anim2 = Animation( 'Dimension', 2, 'xLim', [-6,6], 'yLim', [-6,6] );
anim2.init( )
anim2.attachRobot( robot2 )    


%% Playing with the robots!

% Change the degree to radian for revolute joints
q1 = [0.4, 0.3, 0.2, 0.1]';
q2 = [0.4, 0.3, 0.2, 0.1]';

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot1.updateKinematics( q1 );
robot2.updateKinematics( q2 );

anim1.update( 0 );
anim2.update( 0 );
