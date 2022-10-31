% [Project]        Robot Simulator - Triple Pendulum
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
simTime = 10;       % Total simulation time
t  = 0;             % The current time of simulation   
dt = 0.01;          % Time-step of simulation 

%% Initialize the robot

% Select the robotID
robotID = 1;

robot = SnakeBot( robotID, 3 );
robot.init( )

anim = Animation( 'Dimension', 2, 'xLim', [-5,5], 'yLim', [-5,5] );
anim.init( )
anim.attachRobot( robot )    

%% Initialization of Animation

% Initial configuration of the Robot
q_deg = [0, 50, 50]';

% DO NOT CHANGE
% Changing the degrees to radian
q  = func_deg2rad( q_deg, robot.JointTypes );

% Initial velocity of the robot
dq = zeros( robot.nq, 1 );

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot.updateKinematics( q );

% Update animation to initial configuration
anim.update( );            

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);


%% Running the main-loop of simulation 

while t <= simTime
    
    tic
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    
    % Get the Coriolis term of the robot
    C = robot.getCoriolisMatrix( q, dq );
    
    % Get the Gravity term of the robot
    G = robot.getGravityVector( q );
    rhs = M\( -C * dq -G);  
    
    % We compare the C matrices with the matrices provided from Eq.9
    % The default values of our Acrobot geometrical/inertial parameters are as follows:
    % m1 = 1, m2 = 1, lc1 = 0.5, lc2 = 0.5, l1 = 1.0, l2 = 1.0    
    % The following 
    % [REF] https://underactuated.csail.mit.edu/acrobot.html#section1
    
    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt);
    q  =  q1;
    dq = dq1;

    % Update the linkage plot
    robot.updateKinematics( q );
    
    anim.update(  );    

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    
    % Set animation title
    set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );
    
    % do not go faster than real time
    while toc < dt
        % do nothing
    end
end

anim.close( )
