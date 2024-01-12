
%% Cleaning up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 5;        % Total simulation time
t  = 0;             % The current time of simulation   
dt = 0.01;          % Time-step of simulation 

% Set figure size and attach robot to simulation
robot = iiwa14_2nd( );
robot.init( );

robot.getMassMatrix( zeros( 1, 7 ) )

%% Get the mass matrix in symbolic form 
 q_sym = sym(  'q', [1, 7] );
dq_sym = sym( 'dq', [1, 7] );

% M = simplify( robot.getMassMatrix( q_sym ) );
% C = simplify( robot.getCoriolisMatrix( [ 1,2,3,4,5,6,7], [ 1,2,3,4,5,6,7]' ) );
J = simplify( robot.getHybridJacobian( q_sym ) );

%% Get the dJ matrix 
dJ = 0;
for i = 1 : 7
    dJ = dJ + diff( J, q_sym( i ) ) * dq_sym( i );
end
dJ = simplify( dJ );

%% Print out the components of dJ in python friendly form 

dJ_py = cell( 6, 7 );
for i = 1 : 6
    for j = 1 : 7
        tmp = char( dJ( i, j ) );
        tmp = strrep( tmp, 'q1', 'q[0]' );
        tmp = strrep( tmp, 'q2', 'q[1]' );
        tmp = strrep( tmp, 'q3', 'q[2]' );
        tmp = strrep( tmp, 'q4', 'q[3]' );
        tmp = strrep( tmp, 'q5', 'q[4]' );
        tmp = strrep( tmp, 'q6', 'q[5]' );
        tmp = strrep( tmp, 'q7', 'q[6]' );
        
        tmp = strrep( tmp, 'dq1', 'dq[0]' );
        tmp = strrep( tmp, 'dq2', 'dq[1]' );
        tmp = strrep( tmp, 'dq3', 'dq[2]' );
        tmp = strrep( tmp, 'dq4', 'dq[3]' );
        tmp = strrep( tmp, 'dq5', 'dq[4]' );
        tmp = strrep( tmp, 'dq6', 'dq[5]' );
        tmp = strrep( tmp, 'dq7', 'dq[6]' );
        
        tmp = strrep( tmp, 'sin', 'np.sin' );
        tmp = strrep( tmp, 'cos', 'np.cos' );
        
        dJ_py{ i, j } = tmp;
    end
end

% Print out the string
for i = 1 : 6
    for j = 1 : 7
        fprintf( 'dJ[ %d, %d ] = %s\n' , i-1, j-1 , dJ_py{ i, j } );
    end
end
%% 
C_simp = sym( zeros( 7, 7 ) );
    
for i = 1 : 7
    for j = 1 : 7
        disp( i,  j )
    end
end