function exp_w = expSO3( w, theta )
% ===========================================================================
% expSO3 - An exponential mapping from so(3) to SO(3)
% exp_w = expSO3( w, theta_deg )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] w - A 1x3 or 3x1 array which describes the 
%           unit angular velocity vector
% 
%   [2] theta - The angular displacement about unit-axis w, units in radian
%
% Output 
%   [1] exp_w - A 3x3 matrix of exp( w_skew * theta_deg )
%               This is a mapping from a so(3) to SO(3)
% ===========================================================================

% The input should be a 1x3 or 3x1 array
assert( isequal( size( w ), [ 3, 1 ] ) || isequal( size( w ), [ 1, 3 ] ) , 'An input must be a 3-by-1 or 1-by-3 array')

% [TODO] [Moses C. Nah] [2023.05.26]
% Must need to check if w is normalized.
% The input of the angular velocity must also be a unit vector 
% assert( sum( w .* w ) == 1 )

% A hat operator to transform a vector to a 3x3 skew-symmetric matrix
w_hat = vec_to_so3( w );

% The Rodriguez Formula
% Eq. 2.14 from Murray, R et al, A mathematical introduction to robotic manipulation 
% Eq. 3.51 from Lynch, K. and Park, F. Modern robotics.
exp_w = eye( 3 ) + w_hat * sin( theta ) + w_hat^2 * ( 1 - cos( theta ) );


end

