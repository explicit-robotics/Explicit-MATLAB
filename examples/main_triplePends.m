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
dt = 0.01;         % Time-step of simulation 

%% Initialize the robot

% First  argument is robot ID
% Second argument is the robot's DOF
m_arr = ones( 1, 3 );
l_arr = ones( 1, 3 );
robot1 = SnakeBot( 3, m_arr, l_arr );
robot2 = SnakeBot( 3, m_arr, l_arr );
robot3 = SnakeBot( 3, m_arr, l_arr );
robot4 = SnakeBot( 3, m_arr, l_arr );
robot5 = SnakeBot( 3, m_arr, l_arr );

robot1.init( )
robot2.init( )
robot3.init( )
robot4.init( )
robot5.init( )

anim = Animation( 'Dimension', 2, 'xLim', [-5,5], 'yLim', [-5,5], 'isSaveVideo', false );
anim.init( )
anim.attachRobot( robot1 )    
anim.attachRobot( robot2 )    
anim.attachRobot( robot3 )    
anim.attachRobot( robot4 )    
anim.attachRobot( robot5 )    


%% Initialization of Animation

% Initial configuration of the Robot
q  = [0, 0.5, 0.5]';
q1 = q;
q2 = q1 + [ 0, 0, 0.01 ]';
q3 = q1 + [ 0, 0, 0.02 ]';
q4 = q1 + [ 0, 0, 0.03 ]';
q5 = q1 + [ 0, 0, 0.04 ]';

% Initial velocity of the robot
dq1 = zeros( robot1.nq, 1 );
dq2 = zeros( robot1.nq, 1 );
dq3 = zeros( robot1.nq, 1 );
dq4 = zeros( robot1.nq, 1 );
dq5 = zeros( robot1.nq, 1 );

q_arr  = [ q1,  q2,  q3,  q4,  q5]; 
dq_arr = [dq1, dq2, dq3, dq4, dq5]; 

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot1.updateKinematics( q1 );
robot2.updateKinematics( q2 );
robot3.updateKinematics( q3 );
robot4.updateKinematics( q4 );
robot5.updateKinematics( q5 );

% Update animation to initial configuration
% The initial time is 0s.
anim.update( 0 );            


%% Running the main-loop of simulation 

while t <= simTime
    
    for i = 1 : anim.nRobots
        
        q  =  q_arr( : , i );
        dq = dq_arr( : , i );
        
        % Get the mass matrix of the Acrobot
        M = anim.Robots{ i }.getMassMatrix( q );

        % Get the Coriolis term of the robot
        C = anim.Robots{ i }.getCoriolisMatrix( q, dq );

        % Get the Gravity term of the robot
        G = anim.Robots{ i }.getGravityVector( q );
        rhs = M\( -C * dq - G);  

        % We compare the C matrices with the matrices provided from Eq.9
        % The default values of our Acrobot geometrical/inertial parameters are as follows:
        % m1 = 1, m2 = 1, lc1 = 0.5, lc2 = 0.5, l1 = 1.0, l2 = 1.0    
        % The following 
        % [REF] https://underactuated.csail.mit.edu/acrobot.html#section1

        [ q, dq ] = func_symplecticEuler( q, dq, rhs, dt);
        % Update the linkage plot
        anim.Robots{ i }.updateKinematics( q );
        
        q_arr( : , i ) = q;
        dq_arr( :, i ) = dq;        

    end
    
    anim.update( t );    

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    
end

anim.close( )
