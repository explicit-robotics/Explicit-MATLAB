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
simTime = 5;        % Total simulation time
t  = 0;             % The current time of simulation   
dt = 0.01;          % Time-step of simulation 

% Set figure size and attach robot to simulation
robot = iiwa14( 'quality', 'high' );
robot.init( );

%% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-1,1], 'yLim', [-1,1], 'zLim', [0,2] );
anim.init( );
anim.attachRobot( robot )  

%% Update kinematics
robot.updateKinematics( robot.q_init );
anim.update( 0 );
