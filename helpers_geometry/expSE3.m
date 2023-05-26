function exp_T = expSE3( T, theta )
% ========================================================================
% expSE3 - An exponential mapping from se(3) to SE(3)
% exp_T = expSE3( T, theta )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] T - A 1x6 or 6x1 twist array, T = [ v, w ] 
% 
%   [2] theta - The angular displacement about twist T
%               If revolute  joint, unit is radian
%               If prismatic joint, unit is degree
%               The data is preprocessed already.
%
% Output 
%   [1] exp_T - A 4-by-4 matrix of exp( w_skew * theta_deg )
%               This is a mapping from se(3) to SE(3)
% ========================================================================

% The input should be a 1x6 or 6x1 array
assert( isequal( size( T ), [ 6, 1 ] ) || isequal( size( T ), [ 1, 6 ] ), 'An input must be a 6-by-1 or 1-by-6 array')

% Extracting the vectors v and w 
v = T( 1 : 3 );                                        
w = T( 4 : 6 );

% Initialize 4-by-4 exp_T array
exp_T = eye( 4 );

% Wrapper for symbolic argument.
if isa( theta, 'sym' )
    exp_T = sym( exp_T );
end

% This will be the first 3-by-3 component of the SE(3) Matrix
exp_T( 1:3, 1:3 ) = expSO3( w, theta );

% Using the closed-form solution of Kevin Lynch and Frank Park (2017)
% Equation 3.88 of Modern Robotics 
w_hat = vec_to_so3( w );
exp_T( 1:3, 4 ) = ( eye( 3 ) * theta + ( 1 - cos( theta ) ) * w_hat + ( theta - sin( theta ) ) * w_hat^2 ) * v;

end