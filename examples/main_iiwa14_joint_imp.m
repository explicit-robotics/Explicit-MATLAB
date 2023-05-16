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
simTime = 15;        % Total simulation time
t  = 0;             % The current time of simulation   
dt = 0.001;         % Time-step of simulation 

% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
anim.init( );
anim.attachRobot( robot )  

% Update kinematics
robot.updateKinematics( robot.q_init );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

% Point-to-point Joint Space Impedance Controller
% Using Minimum-jerk trajectory as the virtual trajectory 

% Initial 
qi  = robot.q_init;
q   = qi;

% Final Posture 
qf  = qi + [ 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]';

% Initial velocity of the robot
dq = zeros( robot.nq, 1 );

% Time step for the simulation
ns = 0;

Kq = diag( [ 3.0, 2.0, 2.0, 0.4, 0.4, 0.4, 0.4 ] );
Bq = diag( [ 3.0, 1.5, 1.5, 0.5, 0.5, 0.5, 0.5 ] );

t0 = 0.1;
D  = 0.8;



while t <= simTime
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix2( q );
    
    % Get the Coriolis term of the robot
    C = robot.getCoriolisMatrix2( q, dq );
    
    % Get the Gravity term of the robot
    G = robot.getGravityVector2( q );
    
    % The joint-space impedance controller
    [ q_ref, dq_ref, ~ ] = min_jerk_traj( t0, t, D, qi, qf );
    
    tau = Kq * ( q_ref - q ) + Bq * ( dq_ref - dq );
    rhs = M\( tau ); 

%     rhs = zeros( robot.nq, 1 );
    
    % We compare the C matrices with the matrices provided from Eq.9
    % The default values of our Acrobot geometrical/inertial parameters are as follows:
    % m1 = 1, m2 = 1, lc1 = 0.5, lc2 = 0.5, l1 = 1.0, l2 = 1.0    
    % The following 
    % [REF] https://underactuated.csail.mit.edu/acrobot.html#section1
    
    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
    
    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        robot.updateKinematics( q );
        anim.update( t );    
        ns = ns + 1;
        
        q_ref_arr( ns, : ) = q_ref;
        dq_ref_arr( ns, : ) = dq_ref;
        q_arr( ns,  : ) = q;
        dq_arr( ns,  : ) = dq;
        
        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );
    
        
    end

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    

   
end

%% Minimum-jerk trajectory
function [p, dp, ddp] = min_jerk_traj( t0, t, D, qi, qf )

% Assertion
assert( t0 > 0 && ( length( qi ) == length( qf ) ) )

% The normalized time 
tau = (t - t0)/D; 

% The length of qi and qf
N = length( qi );

if t <= t0
    p   = qi;
    dp  = zeros( N, 1 );
    ddp = zeros( N, 1 );
    
elseif t0 <= t && t <= t0 + D
    p   =        qi + ( qf - qi ) * ( 10 * tau^3 -  15 * tau^4 +   6 * tau^5 );
    dp  =      1./D * ( qf - qi ) * ( 30 * tau^2 -  60 * tau^3 +  30 * tau^4 );
    ddp =  1./(D^2) * ( qf - qi ) * ( 60 * tau^1 - 180 * tau^2 + 120 * tau^3 );
    
else
    p   = qf;
    dp  = zeros( N, 1 );
    ddp = zeros( N, 1 );
end


end