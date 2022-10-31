% [Project]        Robot Simulator - Cartpole
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

% Include all the subdirectories
func_addSubfolders( 'animation', 'helpers_geometry', 'robots', ...
                   'kinematics_dynamics', 'interpolator' );

% Simulation settings
simTime = 4;        % Total simulation time
t  = 0;             % The current time of simulation   
dt = 0.01;          % Time-step of simulation 


%% Initialize the robot

% Select the robotID
robotID = 1;

robot = CartPole( robotID, 1, 1, 1 );
robot.init( )

anim = Animation( 'Dimension', 2, 'xLim', [-1.5,1.5], 'yLim', [-1.5,1.5] );
anim.init( )
anim.attachRobot( robot )    

%% Initialization of Animation

% Initial configuration of the Robot
q_deg = [0, 70]';
q     = zeros( 2, 1 );
dq    = zeros( 2, 1 );

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot.updateKinematics( q_deg );

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
    M = robot.getMassMatrix( q_deg );
    
    % Get the Coriolis term of the robot
    tic
    C1 = robot.getCoriolisCentrifugalVector1( q_deg, dq );
    t1 = toc;
    disp( sprintf( "t1 %f", t1 ) )
    
    tic
    C2 = robot.getCoriolisCentrifugalVector2( q_deg, dq );
    t2 = toc;
    disp( sprintf( "t2 %f", t2 ) )
    
    max( abs( C1 - C2 )  )
    
    % Get the Gravity term of the robot
    G = robot.getGravityVector( q_deg );
    rhs = M\( -C1 -G);  

    % Forward Simulation 
    for i = 1 : robot.nq
        if robot.JointTypes( i ) == 1
            q( i ) = deg2rad( q_deg( i ) );
        else
            q( i ) = q_deg( i ) ;
        end
    end

    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt);
    q  =  q1;
    dq = dq1;
    
    for i = 1 : robot.nq
        if robot.JointTypes( i ) == 1
            q_deg( i ) = rad2deg( q( i ) );
        else
            q_deg( i ) = q( i );
        end
    end    
    
    % Update the linkage plot
    robot.updateKinematics( q_deg );
    
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
