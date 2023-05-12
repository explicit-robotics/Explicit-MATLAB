% [Project]        Robot Simulator - Comparison
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

%% iiwa7 Mass matrix comparison

% Set figure size and attach robot to simulation
robot = iiwa7( 'high' );
robot.init( );

% Calculate the average time
N = 1000;
t_arr = zeros( 2, N );

for i = 1:N

    q_tmp = rand( 1, robot.nq );

    tic
    M1 = robot.getMassMatrix( q_tmp );
    t_arr( 1, i ) = toc;
    
    tic
    M2 = robot.getMassMatrix2( q_tmp );
    t_arr( 2, i ) = toc;

    abs( max( max( M1 - M2 ) ) ) < 1e-9;
end

t_result = sum( t_arr, 2 );

% Printing out the average
fprintf( "Old:%f   New:%f\n", t_result( 1 ),  t_result( 2 ) );


%% Get the Mass Matrix, Snakebot
n_arr  = [ 2,5,10,20,25,30,35,40,45,50,60,70,80,90, 100 ];%,200 ];
N = 20;

i = 1;
t_arr = zeros( 2, length( n_arr ) );

for n = n_arr

    % Define the snakebot
    robot = SnakeBot( n, 1 * ones(1, n), 1 * ones(1, n) );
    robot.init( )

    tmp1 = 0;
    tmp2 = 0;

    for j = 1 : N

        q_tmp = rand( 1, n );
        tic
        M1 = robot.getMassMatrix( q_tmp );
        tmp1 = tmp1 + toc;
        
        tic 
        M2 = robot.getMassMatrix2( q_tmp );
        tmp2 = tmp2 + toc;

        assert( abs( max( max( M1 - M2 ) ) ) < 1e-9 )
    end
    
    t_arr( 1, i ) = tmp1/N;
    t_arr( 2, i ) = tmp2/N;
    
    i = i + 1;
    fprintf( "DOF:%d   Old:%f  New:%f\n", n, tmp1/N, tmp2/N );
    
end

f = figure( ); a = axes( 'parent', f );
hold on
plot( n_arr, t_arr( 1, : ) )
plot( n_arr, t_arr( 2, : ) )

%% Get the Gravity Matrix, Snakebot

n_arr  = [ 2,5,10,20,25,30,35,40,45,50,60,70,80,90, 100 ];%,200 ];
N = 20;
i = 1;
t_arr = zeros( 2, length( n_arr ) );

for n = n_arr

    % Define the snakebot
    robot = SnakeBot( n, 1 * ones(1, n), 1 * ones(1, n) );
    robot.init( )

    tmp1 = 0;
    tmp2 = 0;

    for j = 1 : N

        q_tmp = rand( 1, n );
        tic
        G1 = robot.getGravityVector( q_tmp );
        tmp1 = tmp1 + toc;
        
        tic 
        G2 = robot.getGravityVector2( q_tmp );
        tmp2 = tmp2 + toc;

        assert( abs( max( max( G1 - G2 ) ) ) < 1e-9 )
    end
    
    t_arr( 1, i ) = tmp1/N;
    t_arr( 2, i ) = tmp2/N;
    
    i = i + 1;
    fprintf( "DOF:%d   Old:%f  New:%f\n", n, tmp1/N, tmp2/N );
    
end

f = figure( ); a = axes( 'parent', f );
hold on
plot( n_arr, t_arr( 1, : ) )
plot( n_arr, t_arr( 2, : ) )

%% Get the Coriolis Vector, Snakebot

n_arr  = [ 2,5,10,20 ];%,200 ];
N = 10;
i = 1;
t_arr = zeros( 2, length( n_arr ) );

for n = n_arr

    % Define the snakebot
    robot = SnakeBot( n, ones(1, n), ones(1, n) );
    robot.init( )

    tmp1 = 0;
    tmp2 = 0;

    for j = 1 : N

        q_tmp  = rand( 1, n );
        dq_tmp = rand( 1, n );
        tic
        C1 = robot.getCoriolisMatrix( q_tmp, dq_tmp );
        tmp1 = tmp1 + toc;
        
        tic 
        C2 = robot.getCoriolisMatrix2( q_tmp, dq_tmp );
        tmp2 = tmp2 + toc;
        
        % Note that C1 and C2 are different since C is not unique
        % However, C1dq and C2dq are equivalent
        assert( abs( max( max( C1 * dq_tmp'  - C2 * dq_tmp' ) ) ) < 1e-9 )
    end
    
    t_arr( 1, i ) = tmp1/N;
    t_arr( 2, i ) = tmp2/N;
    
    i = i + 1;
    fprintf( "DOF:%d   Old:%f  New:%f\n", n, tmp1/N, tmp2/N );
    
end

f = figure( ); a = axes( 'parent', f );
hold on
plot( n_arr, t_arr( 1, : ) )
plot( n_arr, t_arr( 2, : ) )