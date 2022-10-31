% ======================================================================= %
% [Project] Robot Simulator
%
% [Authors] 
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
% [Description] 
%    Computation time comparison between func_getExponentialProducts, 
%    which is our method, and the brute force matrix multiplication
%    that is cleaner, yet slower.
%
% ======================================================================= %


%% (0A) Initialization 

clear; close all; clc;

% Shuffle the random number 
rng shuffle
                

%% (1A) Timeit

N = 1000;

t_arr = zeros( 2, N );

for i = 1 : N
    disp( i )
    mat = rand( 4, 4, 100 );

    f1 = @() func_getExponentialProduct( mat );
    f2 = @() func_tmpMethod( mat );    

    t_arr( 1, i ) = timeit( f1 );
    t_arr( 2, i ) = timeit( f2 );

end


%% Function definition

function exp_T = func_tmpMethod( exp_T_arr )
    [ ~, ~, d ] = size( exp_T_arr );

    exp_T = eye( 4 );

    for i = 1 : d
        exp_T = exp_T * exp_T_arr( :, :, d );
    end
end
