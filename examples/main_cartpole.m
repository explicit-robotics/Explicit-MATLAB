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

% Simulation settings
simTime = 5;       % Total simulation time
t  = 0;            % The current time of simulation   
dt = 1e-4;        % Time-step of simulation 

%% Initialize the robot

mc = 1;
mp = 1;
lp = 1;

robot = CartPole( mc, mp, lp );
robot.init( )

anim = Animation( 'Dimension', 2, 'xLim', [-1.5,1.5], 'yLim', [-1.5,1.5], 'isSaveVideo', true, 'VideoSpeed', 1.0 );
anim.init( )
anim.attachRobot( robot )    
set( anim.hAxes , 'visible', 'off' )
%% Initialization of Animation

% Initial configuration of the Robot
q  = [0, 90]' * pi/180;
dq = zeros( 2, 1 );

% Update robot kinematics with q_deg array
% Also get the end-effector's H matrix
robot.updateKinematics( q );

% Update animation to initial configuration
% Time input is given
anim.update( 0 );            


%% Running the main-loop of simulation 

q_arr = q;
i = 1;
while t <= simTime
    
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
    
    [ q, dq ] = func_symplecticEuler( q, dq, rhs, dt);

    % Update the linkage plot
    robot.updateKinematics( q );
    
    anim.update( t );    

    % Get the forward kinematics of the EE
    t = t + dt;                                                                

    q_arr( :, i ) = q;
    i = i + 1;    
end

anim.close( )

%% 

t_arr = 0:dt:t;

f = figure( ); a1 = axes( 'parent', f );
hold on
% yline( a1, 0, 'linewidth', 30, 'color', [0.8500 0.3250 0.0980] )
r = 1;
theta = q_arr( 2, : );
disp  = q_arr( 1, : );
x = r * cos( theta );
y = r * sin( theta ); 
z = disp;

[X,Y,Z] = cylinder( a1, r, 80 )
h = 20;
Z = Z*h - h/2;
surf(a1, X,Y,Z , 'facealpha', 0.2, 'facecolor', 'black', 'edgecolor', 'black' )

set( a1, 'xlim', [-2, 2], 'ylim', [ -2, 2 ], 'zlim', [ -2, 2 ], 'view', [150.0333, 38.1205], 'visible', 'off' )
tmp = scatter3( a1, x( 1 ), y( 1 ), z( 1 ), 1000, 'filled', 'markeredgecolor', 'black', 'markerfacecolor', 'white');

plot3( x(1:end-1), y(1:end-1), z(1:end-1), 'linewidth', 3, 'color', 'black' )

v = VideoWriter( 'video0.mp4','MPEG-4' );
v.FrameRate = 30;

open( v );
tmp_step = 333;
for i = 1 : tmp_step : length( t_arr )
    
    set( tmp, 'XData', x( i ), 'YData', y( i ), 'ZData', z( i ) )

    drawnow 
    
    tmp_frame = getframe( f );
    writeVideo( v,tmp_frame );
    i
end
close( v );

