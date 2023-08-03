% Project                       Robot Simulator - Get Manipulator Equation
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
%
% The code is heavily commented. A famous quote says:
% "Code is read more often than it is written"
%  - Guido Vanw Rossum, the Creator of Python

%% Cleaning up + Environment setup
clear; close all; clc;

%% Initialize the robot 
% Our goal is to calculate the mass/coriolis matrices and gravity co-vector

% Our robot is the CartPole
robot = DoublePendulum( 1, 1, 1, 1 );

% Robot has to be initialized to adapt the joint twists for new robot
robot.init( )

%% Initialization of Animation

q_sym  = sym(  'q', [ robot.nq, 1 ] );
dq_sym = sym( 'dq', [ robot.nq, 1 ] );

Msym = simplify( robot.getMassMatrix( q_sym ) );
Csym = simplify( robot.getCoriolisMatrix( q_sym, dq_sym ) );
Gsym = simplify( robot.getGravityVector( q_sym ) );
