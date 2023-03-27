% [Project]        Robot Simulator - Add Two Robots
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

%% Add robot 1 and 2

% Our robot is the CartPole
robot     = CartPole( 1, 1, 1 );
kinematic = DoublePendulum( 1, 1, 1, 1 );
new_robot = robot.addKinematics( kinematic );

% Robot has to be initialized to adapt the joint twists for new robot
robot.init( );
kinematic.init( );
new_robot.init( );

% Set figure size and attach robot to simulation
anim = Animation( 'Dimension', 2, 'xLim', [-4,4], 'yLim', [-4,4] );
anim.init( );
anim.attachRobot( new_robot )  