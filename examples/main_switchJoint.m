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

% Select the robotID
robotID = 1;

nq = 4;
robot1 = SnakeBot( robotID, nq );
robot1.init( )

anim1 = Animation( 'Dimension', 2, 'xLim', [-6,6], 'yLim', [-6,6] );
anim1.init( )
anim1.attachRobot( robot1 )    


%% Initialize Robot 2, where the 3rd joint is Prismatic

robotID = 2;

nq = 4;
robot2 = SnakeBot( robotID, nq );
robot2.init( )

robot2.switchJoint( 3, 2, [ 1; 0; 0] );

anim2 = Animation( 'Dimension', 2, 'xLim', [-6,6], 'yLim', [-6,6] );
anim2.init( )
anim2.attachRobot( robot2 )    

set( anim1.hAxes, 'fontsize', 20, 'linewidth', 1 );
set( anim2.hAxes, 'fontsize', 20, 'linewidth', 1 );

%% Playing with the robots!

% Initial Condition of the Robot
q_deg1 = [30,22,-45,0]';
q_deg2 = [10,10,0.3,0]';

% Change the degree to radian for revolute joints
q1 = func_deg2rad( q_deg1, robot1.JointTypes );
q2 = func_deg2rad( q_deg2, robot2.JointTypes );

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot1.updateKinematics( q1 );
robot2.updateKinematics( q2 );

anim1.update( 0 );
anim2.update( 0 );
