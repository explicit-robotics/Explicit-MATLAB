% [Project]        Robot Simulator - iiwa14
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
t  = 0.00;          % The current time of simulation   
dt = 0.01;          % Time-step of simulation 

% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

%% Calculating the Connection Coefficients

% Calculating the Structure Constants

e1 = [ zeros( 3, 3 ), [1;0;0]; zeros(1,4 )];
e2 = [ zeros( 3, 3 ), [0;1;0]; zeros(1,4 )];
e3 = [ zeros( 3, 3 ), [0;0;1]; zeros(1,4 )];

e4 = [ 0,  0,  0, 0; 
       0,  0, -1, 0;
       0,  1,  0, 0;
       zeros( 1, 4 )];

e5 = [ 0,  0, 1, 0; 
       0,  0, 0, 0;
      -1,  0, 0, 0;
       zeros( 1, 4 )];

e6 = [ 0, -1, 0, 0; 
       1,  0, 0, 0;
       0,  0, 0, 0;
       zeros( 1, 4 )];

e_arr = zeros( 4, 4, 6 );
e_arr( :, :, 1 ) = e1;
e_arr( :, :, 2 ) = e2;
e_arr( :, :, 3 ) = e3;
e_arr( :, :, 4 ) = e4;
e_arr( :, :, 5 ) = e5;
e_arr( :, :, 6 ) = e6;

v1 = se3_to_R6( e1 );
v2 = se3_to_R6( e2 );
v3 = se3_to_R6( e3 );
v4 = se3_to_R6( e4 );
v5 = se3_to_R6( e5 );
v6 = se3_to_R6( e6 );

v_arr = zeros( 6, 6 );
v_arr( :, 1 ) = v1;
v_arr( :, 2 ) = v2;
v_arr( :, 3 ) = v3;
v_arr( :, 4 ) = v4;
v_arr( :, 5 ) = v5;
v_arr( :, 6 ) = v6;

% Construction coefficients
Cijk = zeros( 6, 6, 6 );

% Calculation
for i = 1 : 6
    for j = 1 : 6
        tmp1 = e_arr( :, :, i )*e_arr( :, :, j )-e_arr( :, :, j )*e_arr( :, :, i );
        tmp2 = se3_to_R6( tmp1 );        
        for k = 1: 6
            val = sum( tmp2 .* v_arr( :, k )' );

            if val
                fprintf( "C_{%d, %d}^{%d} = %.2f\n", i, j, k, val )
            end
            Cijk( i, j, k ) = val;
        end
    end
end

% Also calculate the a values.
% Manual Calculation.
aijk = zeros( 6, 6, 6);
aijk( 4, 2, 3 ) = +1;
aijk( 4, 3, 2 ) = -1;
aijk( 5, 1, 3 ) = -1;
aijk( 6, 1, 2 ) = +1;
aijk( 5, 3, 1 ) = +1;
aijk( 6, 2, 1 ) = -1;


% Calculating Gammas
Gamma = zeros( 6, 6, 6 );
for i = 1:6
    for j = 1:6
        for k = 1:6
            if aijk( i, j , k ) || Cijk( i, j, k ) 
                    Gamma( j, i, k ) = 0.5 * ( +Cijk( i, j, k  ) + aijk( i, j , k ) );
                    Gamma( i, j, k ) = 0.5 * ( -Cijk( i, j, k  ) + aijk( i, j , k ) );
            end
        end
    end
end


for i = 1 : 6
    for j = 1 : 6   
        for k = 1: 6
            if Gamma( i, j, k )
                fprintf( "Gamma_{%d, %d}^{%d} = %.2f\n", i, j, k, Gamma( i, j, k ) )
            end
        end
    end
end

% Deriving the force
syms F1 F2 F3 F4 F5 F6 
F_arr = [ F1; F2; F3; F4; F5; F6 ];

Kasym = sym( zeros( 6, 6 ) );

for i = 1 : 6
    for j = 1 : 6
        tmp = 0;
        for k = 1 : 6
            tmp = tmp + Gamma( i,j, k ) * F_arr( k );
        end
        Kasym( i, j ) = tmp;
    end
end

%% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
anim.init( );
anim.attachRobot( robot )  

%% Checking the Connection Coefficients, Hybrid Jacobian

q0 = robot.q_init;

% Checking for 3DOF Case 
% Get the symbolic form of the Jacobian Matrix

% Get the Hybrid Jacobian
q_syms = sym( 'q', [7, 1 ] );
% F_syms = sym( 'F', [6, 1 ] );
F_syms = rand( 6, 1 );
JH = robot.getHybridJacobian( q_syms );
JB = robot.getBodyJacobian(   q_syms, 7 );

% ======================================================== %
% (Case 1) For 3DOF Case, Hybrid Jacobian, Position, With Correction Term
K_joint1 = sym( zeros( robot.nq, robot.nq ) );

tmp1 = rand( 6, 6 );
Kq = tmp1' * diag( [ 1.2, 4.3, 5.4, 1.6 ,2.5, 1.6 ] ) * tmp1;
Jac_sym = JH;
Jac_val = double( subs( JH, q_syms, q0 ) );

is_correction = true;

for a = 1 : robot.nq
    for b = 1 : robot.nq
        
        tmp = 0;
        for i = 1 : 3
            for j = 1 : 3
                tmp2 = 0;
                for k = 1 : 6
                    tmp2 = tmp2 + F_syms( k )*Gamma(i,j,k);
                end

                tmp = tmp + Jac_val( i, a )*Jac_val( j, b )*Kq( i, j ) + double( subs( diff( Jac_sym( i, a ), q_syms( b ) ), q_syms, q0 ) ) * F_syms( i ) + Jac_val( i, a )* tmp2*Jac_val( j, b );
            end
        end

        K_joint1( a, b ) = tmp;
    end
end

% Symmetric
K_joint1 = double( K_joint1 );

% ======================================================== %
% (Case 2) For 3DOF Case, Hybrid Jacobian, Position, Without Correction Term
K_joint2 = sym( zeros( robot.nq, robot.nq ) );

for a = 1 : robot.nq
    for b = 1 : robot.nq
        
        tmp = 0;
        for i = 1 : 3
            for j = 1 : 3
                tmp = tmp + Jac_val( i, a )*Jac_val( j, b )*Kq( i, j ) + double( subs( diff( Jac_sym( i, a ), q_syms( b ) ), q_syms, q0 ) ) * F_syms( i );
            end
        end

        K_joint2( a, b ) = tmp;
    end
end

% Symmetric
K_joint2 = double( K_joint2 );



% ======================================================== %
% (Case 3) For 3DOF Case, Hybrid Jacobian, Velocity, With Correction Term
K_joint3 = sym( zeros( robot.nq, robot.nq ) );

for a = 1 : robot.nq
    for b = 1 : robot.nq
        
        tmp = 0;
        for i = 4 : 6
            for j = 4 : 6
                tmp2 = 0;
                for k = 1 : 6
                    tmp2 = tmp2 + F_syms( k )*Gamma(i,j,k);
                end
                tmp = tmp + Jac_val( i, a )*Jac_val( j, b )*Kq( i, j ) + double( subs( diff( Jac_sym( i, a ), q_syms( b ) ), q_syms, q0 ) ) * F_syms( i ) + Jac_val( i, a )* tmp2*Jac_val( j, b );
            end
        end

        K_joint3( a, b ) = tmp;
    end
end

K_joint3 = double( K_joint3 );


% ======================================================== %
% (Case 4) For 3DOF Case, Hybrid Jacobian, Position, Without Correction Term
K_joint4 = sym( zeros( robot.nq, robot.nq ) );

is_correction = false;

for a = 1 : robot.nq
    for b = 1 : robot.nq
        
        tmp = 0;
        for i = 4 : 6
            for j = 4 : 6
                tmp = tmp + Jac_val( i, a )*Jac_val( j, b )*Kq( i, j ) + double( subs( diff( Jac_sym( i, a ), q_syms( b ) ), q_syms, q0 ) ) * F_syms( i ) ;
            end
        end

        K_joint4( a, b ) = tmp;
    end
end

% Symmetric
K_joint4 = double( K_joint4 );


% ======================================================== %
% (Case 5) For 6DOF Case, Hybrid Jacobian, With Correction Term
K_joint5 = sym( zeros( robot.nq, robot.nq ) );

for a = 1 : robot.nq
    for b = 1 : robot.nq
        
        tmp = 0;
        for i = 1 : 6
            for j = 1 : 6
                tmp2 = 0;
                for k = 1 : 6
                    tmp2 = tmp2 + F_syms( k )*Gamma(i,j,k);
                end
                tmp = tmp + Jac_val( i, a )*Jac_val( j, b )*Kq( i, j ) + double( subs( diff( Jac_sym( i, a ), q_syms( b ) ), q_syms, q0 ) ) * F_syms( i ) + Jac_val( i, a )* tmp2*Jac_val( j, b );
            end
        end

        K_joint5( a, b ) = tmp;
    end
end

K_joint5 = double( K_joint5 );


% ======================================================== %
% (Case 6) For 6DOF Case, Hybrid Jacobian, Without Correction Term
K_joint6 = sym( zeros( robot.nq, robot.nq ) );

for a = 1 : robot.nq
    for b = 1 : robot.nq
        
        tmp = 0;
        for i = 1 : 6
            for j = 1 : 6
                tmp = tmp + Jac_val( i, a )*Jac_val( j, b )*Kq( i, j ) + double( subs( diff( Jac_sym( i, a ), q_syms( b ) ), q_syms, q0 ) ) * F_syms( i );
            end
        end

        K_joint6( a, b ) = tmp;
    end
end

K_joint6 = double( K_joint6 );

