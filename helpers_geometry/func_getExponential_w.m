function exp_w = func_getExponential_w( w, theta )
% ===========================================================================
% func_getExponential_w - An exponential mapping from so(3) to SO(3)
% exp_w = func_getExponential_w( w, theta_deg )
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

% The input w must be a real array
assert( isreal( w ) )

% The input should be a 1x3 or 3x1 array
assert( isequal( size( w ), [ 3, 1 ] ), 'An input must be a 3-by-1 array')

% The input of the angular velocity must also be a unit vector 
% assert( sum( w .* w ) == 1 )

% A tilde operator to transform a vector to a 3x3 skew-symmetric matrix
w_tilde = func_skewSym( w );

% The Rodriguez Formula
% [REF1] Eq. 2.14 from 
% Murray, R et al, A mathematical introduction to robotic manipulation 
%
% [REF2] Eq. 3.51 from
% Lynch, K. and Park, F. Modern robotics.
exp_w = eye( 3 ) + w_tilde * sin( theta ) + w_tilde^2 * ( 1 - cos( theta ) );

% [TODO] Moses C. Nah, 2022.08.12
% It might be worth writing the more general equation, 
% which accounts for non-unit w vectors.

end

