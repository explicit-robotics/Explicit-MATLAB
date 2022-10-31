% ======================================================================= %
% [Project] Robot Simulator - Gravity vector Comparison
%
% [Authors] 
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
% [Description] 
%    A main .m file to compare our exp software with robotics-toolbox-matlab.
%    The Robotics Toolbox MATLAB is written by Prof. Peter Corke [REF1]
%    The files for running it are under `rvctools` 
%    The rvctools directory are downloaded form [REF2]
%    [REF1] https://github.com/petercorke/robotics-toolbox-matlab
%    [REF2] https://petercorke.com/toolboxes/robotics-toolbox/
%          RVC 2nd edition: RTB10+MVTB4 (2017) 
%
% The code is heavily commented. A famous quote says:
% "Code is read more often than it is written"
%           - Guido Van Rossum, the Creator of Python
%
% ======================================================================= %


%% (0A) Initialization 

clear; close all; clc;

% Shuffle the random number 
rng shuffle

%% (1A) Initializing of Comparison
% We will compare the computation time of the Coriolis Matrix
% We will use Snakebot for the comparison.
% Snakebot is a N-DOF planar robot with N rotational joints. 
% i.e., the (x,y,z) position of the robot's end-effector will be:
% x: lcos( q1 ) + lcos( q1+q2 ) + ... + lcos( q1+q2+...+qn )
% y: lsin( q1 ) + lcos( q1+q2 ) + ... + lsin( q1+q2+...+qn )
% z: 0
% We set the lengths and masses of N segments all identical.
l = 1;                  %[m]
m = 1;                  %[kg]
I = 1/12 * m * l^2;     %[kg-m^2]

% Given the number of degrees of freedom (N)
% We will generate a snakebot with N-DOF
% We will test multiple N-DOF robots, hence generating an array 
% N_arr is a DOFs that we will test for the comparison
N_arr = [2, 3, 5, 6, 8, 10 15, 20, 25, 30, 40, 50, 80, 100 ];

% We use 'timeit' function to calculate the computation time.
% 'timeit' already calculates the "mean" of the computation time,
%  meaning it runs multiple trials to calculate the computation time [REF].
% However, we will also run multiple trials to get the mean/std.
% [REF] https://www.mathworks.com/help/matlab/matlab_prog/measure-performance-of-your-program.html
N_trial  = 1;

% The 3D time array for the computation time.
% "rvc" is the robotic toolbox software and "exp" is our "EXPlicit" software.
% The array is sized as (N1 x N2)
% - N1 is the number of robots, i.e., the length of N_arr.
% - N2 is the number of trials running the computation.
t_arr_rvc = zeros( length( N_arr ), N_trial );
t_arr_exp = zeros( length( N_arr ), N_trial );

%% (1B) Run comparison 

clear SnakeBot*

% To check the progress of the computation, we construct a progress bar.
f = waitbar( 0, 'Initialization' );

% Titles of the waitbar
titles = 'Gravity Vector'; 

% Variables for updating the bar
% Not that much important....
idx = 1;
N_bar_max = N_trial * length( N_arr );

% Iterating through each DOF of the robot
for N = N_arr

    % ====================================================== %
    % =============== Robotics Toolbox Part================= %
    % ====================================================== %
    % Generating the Snakebot with robotics toolbox MATLAB
    % The code is a slight modification of mdl_twolink.m file, 
    % which is under ./rvctools/robot/models directory
    R_array = Revolute.empty( 0, N );

    % Generate the The N-revolute link 
    for i = 1 : N
        R_array( i ) = Revolute( 'd', 0, 'a', l, 'alpha', 0, ...
                                 'm', 1, 'r', [-0.5 0 0], 'I', [0 0 I], ...
                                 'B', 0, 'G', 0, 'Jm', 0, 'standard' );
    end

    % Generate a N-DOF Planar Robot
    % [2022.09.01] [Moses C. Nah] [Notes]
    % For the Macbook with M1 chip, the following if statement is false
    % Hence, it runs the else statement that is based on m-script.
    % For the Macbook with intel chip, the statement is true
    % Hence, it runs a mex (C-file) which is way faster than the m-script.
    % This is the reason why the mass-matrix computation on the 
    % intel chip was way faster than the M1 chip's result.
    % This is the reason why for the intel chip computer, we should set 
    % The following statement as false to make the comparison.
    % For the m1 chip, the robot.fast is set as false    
    % For details, Check the rne.m file 103th line 
    % under rvctools/robot/@SerialLink
    % Setting false for speed 'fast' to make it purly m-script driven.

    SnakeBot_rvc = SerialLink( R_array, 'name', 'snake_bot_rvc', ...
                            'comment', 'from Spong, Hutchinson, Vidyasagar' );

    % ====================================================== %
    % =================== Our Software ===================== %
    % ====================================================== %
    % Generate an N-DOF SnakeBot
    SnakeBot_exp = SnakeBot( 1, N );

    % Initialization 
    SnakeBot_exp.init( )

    % The following statement as false to make the comparison.
    % For the m1 chip, the robot.fast is set as false    
    % For details, Check the rne.m file 103th line 
    % under rvctools/robot/@SerialLink
    % Setting false for speed 'fast' to make it purly m-script driven.
    SnakeBot_rvc.fast = false;
    
    func_rvc = @SnakeBot_rvc.gravload;
    func_exp = @SnakeBot_exp.getGravityVector;

    % ====================================================== %
    % ============== RUNNING THE COMPARISON! =============== %
    % ====================================================== %
   
        
    % Running over the trial
    for i = 1 : N_trial

        % Generate a random (N-by-1) q-array for the calculation 
        % The values of q_arr are ranged between -pi to +pi
        q_arr  = 2 * pi * ( rand( 1, N ) - 0.5 );
        
        % A temporary function wrapper to conduct timeit
        tmp_rvc = @( ) func_rvc( q_arr, [0,9.81,0] );
        tmp_exp = @( ) func_exp( q_arr );

        % Save the result of timeit
        t_arr_rvc( idx, i ) = timeit( tmp_rvc );
        t_arr_exp( idx, i ) = timeit( tmp_exp );

        % Update the waitbar 
        waitbar( ( ( idx - 1 ) * N_trial + i ) / N_bar_max, f, ...
                  sprintf( '[%d-DOF Robot] [%s] %d/%d', N, titles, i, N_trial ));

        % Check whether the values between two methods are same
        % Finding the maximum error between matrices and the error
        % Should be smaller than 1e-9

        val_rvc = func_rvc( q_arr, [0,9.81,0] );
        val_exp = func_exp( q_arr );

        % A `double` wrapper is needed since `fkine` method 
        % Outputs an SE(3) instance, which is not a number itself.
        assert( max( abs( double( val_rvc' ) - val_exp ), [ ], 'all' ) < 1e-9 )
        
    end

    idx = idx + 1;
    
end
delete( f )


%% (1C) Plotting the Results
% Iterating over trials

f = figure( ); a = axes( 'parent', f );
hold( a, 'on' )

% Getting the mean and std of the value 
t_rvc_mean = mean( t_arr_rvc( :, : ), 2 );
t_exp_mean = mean( t_arr_exp( :, : ), 2 );

% Plotting the markers
plot( a, N_arr, t_rvc_mean, 'o' )
plot( a, N_arr, t_exp_mean, 'o' )

% Plotting the error bars
% I've found that the std is quite low, hence just leaving this out
% errorbar( a, N_arr, t_rvc_mean, t_rvc_std, 'o', 'MarkerSize', 30  )
% errorbar( a, N_arr,  t_JM_mean,  t_JM_std, 'o', 'MarkerSize', 30  )

set( a, 'fontsize', 40 )
xlabel( a, 'DOF [-]' , 'fontsize', 40 )
set( a, 'xtick', N_arr ) 
ylabel( a, 'Time [s]', 'fontsize', 40 )
legend( a, 'RVC', 'EXP', 'fontsize', 40, 'location', 'northwest' )
title(  a, 'Gravity', 'fontsize', 40 )    