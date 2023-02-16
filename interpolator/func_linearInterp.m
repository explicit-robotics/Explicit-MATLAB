function [ x_cur ] = func_linearInterp( x_ini, A_vec, t_cur, t_fin)
% ===========================================================================
% func_cosineInterp - Simple interpolator for robot trajectories
%
% Input: 
%       x_ini: column vector (max. 3d)
%       A_vec: column vector with amplitudes [ A_x; A_y; A_z ] (max. 3d)
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

if( isrow( A_vec ) )
    A_vec = A_vec';
end

% Make sure that x_ini is max. 3D
assert( length( x_ini ) <= 3 , 'Input x-array can maximal be 3D' );
assert( length( A_vec ) <= 3 , 'Input A-array can maximal be 3D' );

% Cosine interpolation of variable x_cur
if t_cur <= t_fin

    x_cur = x_ini + 0.5 .* A_vec * ( 1 - cos( pi * t_cur / t_fin ) );

else

    x_cur = x_ini + A_vec .* ones( length( x_ini ) , 1 );

end


end



