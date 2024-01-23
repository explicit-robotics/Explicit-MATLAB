% [Project]        Robot Simulator - Double Pendulum
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
simTime = 4;        % Total simulation time
t  = 0;             % The current time of simulation
dt = 1e-4;          % Time-step of simulation

% Flag for turning on/off animation
is_anim = true;

%% Initialize the robot

m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
 
robot = DoublePendulum( m1, m2, l1, l2 );
robot.init( )

anim = Animation( 'Dimension', 2, 'xLim', [-2.0,2.0], 'yLim', [-3.5,0.5], 'isSaveVideo', true, 'VideoSpeed', 1.0 );
anim.init( );
anim.attachRobot( robot )
set( anim.hAxes , 'visible', 'off' )

%% Initialization of Animation

% Initial Position of the Robot
% The unit of q is degrees for revolute, and meters for prismatic.

% DO NOT CHANGE
% Changing the degrees to radian
q  =  [ 50, 90 ]' * pi/180;
dq = -[ 0.3, 0.5]';

% Update robot kinematics with qarray
robot.updateKinematics( q );

% Update animation
anim.update( 0 );


%% Running the main-loop of simulation
i = 1;

while t <= simTime

    % Get the mass matrix of the DoublePendulum
    M = robot.getMassMatrix( q );

    % Get the Coriolis term of the robot
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Gravity term of the robot
    G = robot.getGravityVector( q );
    ddq = M\(-C * dq );

    [ q1, dq1 ] = func_symplecticEuler( q, dq, ddq, dt);
    q  =  q1;
    dq = dq1;

    % Update the linkage plot
    robot.updateKinematics( q );

    anim.update( t );

    % Get the forward kinematics of the EE
    H_EE = robot.getForwardKinematics( q );
    t = t + dt;

    q_arr( :, i ) = q;
    i = i + 1;        
end

anim.close( )


%%

t_arr = 0:dt:t;


% Parameters
R = 1.0;  % Major radius
r = 0.5;  % Minor radius
f = figure( ); a1 = axes( 'parent', f );
hold on
% Generate torus coordinates
u = linspace(0, 2 * pi, 100);  % Azimuthal angle
v = linspace(0, 2 * pi, 100);  % Polar angle
[U, V] = meshgrid(u, v);
x = (R + r * cos(V)) .* cos(U);
y = (R + r * cos(V)) .* sin(U);
z = r * sin(V);

x1 = (R + r * cos( q_arr( 1, : ) ) ) .* cos( q_arr( 2, : ) );
y1 = (R + r * cos( q_arr( 1, : ) ) ) .* sin( q_arr( 2, : ) ); 
z1 = r * sin( q_arr( 1, : ) );

% Create the torus plot
surf( a1, x, y, z, 'facealpha', 0.1, 'facecolor', 'black', 'edgecolor', 'black' , 'edgealpha', 0.5, 'linewidth', 0.1 );

set( a1, 'xlim', [-2, 2], 'ylim', [ -2, 2 ], 'zlim', [ -2, 2 ], 'view', [67.1666,41.1220], 'visible', 'off' )
tmp = scatter3( a1, x1( 1 ), y1( 1 ), z1( 1 ), 500, 'filled', 'markeredgecolor', 'black', 'markerfacecolor', [0 0.4470 0.7410] );
plot3( x1(1:end-1), y1(1:end-1), z1(1:end-1), 'linewidth', 10, 'color', [0 0.4470 0.7410] )

v = VideoWriter( 'video0.mp4','MPEG-4' );
v.FrameRate = 30;

open( v );
tmp_step = 333;
for i = 1 : tmp_step : length( t_arr )
    
    set( tmp, 'XData', x1( i ), 'YData', y1( i ), 'ZData', z1( i ) )

    drawnow 
    
    tmp_frame = getframe( f );
    writeVideo( v,tmp_frame );
    i
end
close( v );




