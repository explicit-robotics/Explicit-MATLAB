function [ x_cur ] = func_circularInterp( x_ini, A, t_cur, t_fin, nrRounds )
% ===========================================================================
% func_circularInterp - Simple interpolator for circular robot trajectories
%
% Input:
%       x_ini: 3d column vector
%       A: radius of circular
%       t_cur: current cycle time
%       t_fin: cycle time where endpoint is reached
% Output:
%       x_cur: column vector with interpolated array
%
% Authors                       Email                   Created
%   [1] Johannes Lachner        jlachner@mit.edu        2023
%   [2] Moses C. Nah            mosesnah@mit.edu


% Make sure x_ini and A_vec are a column vectors
if( isrow( x_ini ) )
    x_ini = x_ini';
end

% Make sure that x_ini is max. 3D
assert( length( x_ini ) <= 3 , 'Input x-array can maximal be 3D' );
assert( length( A ) >= 1 , 'Input A is 1D' );

% Circular interpolation of variable x_cur
w = 2 * pi * 1 / t_fin;
x_cur = x_ini;

if t_cur <= t_fin * nrRounds

    x_cur( 1, 1 ) = x_ini( 1 ) + A * cos( w * t_cur ) - A;
    x_cur( 2, 1 ) = x_ini( 2 ) + A * sin( w * t_cur );
    x_cur( 3, 1 ) = x_ini( 3 );

elseif t_cur > nrRounds * t_fin
    x_cur = x_ini;
end


end



