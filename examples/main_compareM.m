% [Project]        Robot Simulator - Compare Mass
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
robot = iiwa7( 'high' );
robot.init( );
robot.updateKinematics( robot.q_init );


%% Get the Mass Matrix, iiwa
tic
M1 = robot.getMassMatrix( robot.q_init );
toc

tic
M2 = robot.getMassMatrix2( robot.q_init );
toc

%% Get the Mass Matrix, Snakebot

n_arr  = [ 2,5,10,20,25,30,35,40,45,50,60,70, 80,90, 100,200 ];
% n_arr  = 100;

i = 1; 

for n = n_arr

    robot = SnakeBot( n, 1 * ones(1, n), 1 * ones(1, n) );
    robot.init( )
    q_tmp = rand( 1, n );
    tic
    M1 = robot.getMassMatrix( q_tmp );
    t1 = toc;
    
    t_arr1( i ) = t1;
    
    tic 
    M2 = robot.getMassMatrix2( q_tmp );
    t2 = toc;
    
    t_arr2( i ) = t2;
    
    tic 
    M3 = robot.getMassMatrix3( q_tmp );
    t3 = toc;
    
    t_arr3( i ) = t3;    
    
    fprintf( "%f vs. %f vs. %f\n", t1, t2, t3 );
    i = i + 1;
    
    assert( abs( max( max( M1 - M2 ) ) ) < 1e-9 )
    assert( abs( max( max( M1 - M3 ) ) ) < 1e-9 )
end



%% Get the Gravity Vector, Snakebot

n_arr  = [ 2,5,10,20,30,40,50,60,80, 100 ];
% n_arr  = 100;

i = 1; 

for n = n_arr

    robot = SnakeBot( n, 1 * ones(1, n), 1 * ones(1, n) );
    robot.init( )
    q_tmp = rand( 1, n );
    tic
    g1 = robot.getGravityVector( q_tmp );
    t1 = toc;
    
    t_arr1( i ) = t1;
    
    tic 
    g2 = robot.getGravityVector2( q_tmp );
    t2 = toc;
    
    t_arr2( i ) = t2;
    
    
    fprintf( "%f vs. %f\n", t1, t2 );
    i = i + 1;

    assert( abs( max( max( g1 - g2 ) ) ) < 1e-9 )
    
end

%% Get the Coriolis Vector, Snakebot

n_arr  = [ 3,5,10,20,30,40,50,60,80, 100 ];
% n_arr  = 100;

i = 1; 

for n = n_arr
    robot = SnakeBot( n, 1 * ones(1, n), 1 * ones(1, n) );
    robot.init( )
    q_tmp  = rand( 1, n );
    dq_tmp = rand( 1, n );
    tic
    C1 = robot.getCoriolisMatrix( q_tmp, dq_tmp );
    t1 = toc;
    
    t_arr1( i ) = t1;
    
    tic 
    C2 = robot.getCoriolisMatrix2( q_tmp, dq_tmp );
    t2 = toc;
    
    t_arr2( i ) = t2;
        
    fprintf( "%f vs. %f\n", t1, t2 );
    i = i + 1;

    assert( abs( max( max( C1 * dq_tmp' - C2 * dq_tmp' ) ) ) < 1e-9 )
    
end