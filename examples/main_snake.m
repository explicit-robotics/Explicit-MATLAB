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

%% Initialize the robot

% Geometric and Inertial Parameters of SnakeBot
nq = 5;         % The number of linkages of the Snakebot
m  = 1;         % The   mass of the each link
l  = 1;         % The length of the each link

m_arr = m * ones( 1, nq );  % The mass   array to construct SnakeBot
l_arr = l * ones( 1, nq );  % The length array to construct SnakeBot 

% Construct a 5-DOF SnakeBot
robot = SnakeBot( nq, m_arr, l_arr );
robot.init( )

% Attach the 5-DOF SnakeBot to animation for visualization
anim = Animation( 'Dimension', 2, 'xLim', [-1.5,6.5], 'yLim', [-4,4], 'isSaveVideo', true, 'VideoSpeed', 0.5 );
anim.init( )
anim.attachRobot( robot )    


%% Initialization of Animation

% Initial Condition of the Robot
q_deg = [30,22,-45,-34,13]';
dq    = zeros( nq, 1 );

% DO NOT CHANGE
% Changing the degrees to radian
q  = func_deg2rad( q_deg, robot.JointTypes );

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot.updateKinematics( q );
H_EE = robot.getForwardKinematics( q );
H_ini = H_EE;

% Update animation
if is_anim
    anim.update( 0 );            
end


%% Running the main-loop of simulation 

while t <= simTime
    
    
    % Get the p array from the SE(3) Matrix
    p_EE = H_EE( 1:3, 4 );

    % Get Hybrid Jacobian 
    JH = robot.getHybridJacobian( q );

    % Get the first 3 rows 
    JHp = JH( 1:3, : );

    % Get the mass matrix of the simulation s
    M = robot.getMassMatrix( q );
    M_inv = M\eye( size( M ) );
    
    % ============================ %
    % ======== Controller ======== %
    % ============================ %
    A_y = -2;
    t_final = 3;
    p_0 = H_ini(1:3,4);

    if t <= t_final
        ratio = t / t_final;
        if ratio > 1
            ratio = 1;
        end
        p_0( 2 ) = H_ini(2,4) + 0.5 * A_y * (1 - cos(pi * t/t_final));
        p_last = p_EE;
    else
        p_0 = p_last;
    end
    
    % End-effector control
    k = 400;
    b = 40;
    dp = JHp * dq;
    f = k * ( p_0 - p_EE ) - b * dp;
    tau_EE = JHp' * f;   
    
    % nullspace control
    b_q = 1;
    tau_NS = -b_q * dq;
    
    % Superposition of torques
    tau = tau_EE + tau_NS;

    % proceed one simulation step
    % rhs = M\(torque -c - g);        % if coriolis and gravity should be calculated
    rhs = M\(tau);                    % forgo coriolis and gravity for simulation


    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt);
    q  =  q1;
    dq = dq1;
    

    % Update the linkage plot
    robot.updateKinematics( q );
    
    if is_anim
        anim.update( t );    
    end
    
    % Get the forward kinematics of the EE
    H_EE = robot.getForwardKinematics( q );
    t = t + dt;                                                                
    
end
