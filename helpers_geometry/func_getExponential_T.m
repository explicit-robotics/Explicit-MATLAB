function exp_T = func_getExponential_T( T, theta )
% ========================================================================
% func_getExponential_T - An exponential mapping from se(3) to SE(3)
% exp_T = func_getExponential_T( T, theta_deg )
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

% The input must be a real array
assert( isreal( T ) )

% The input should be a 1x6 or 6x1 array
assert( isequal( size( T ), [ 6, 1 ] ), 'An input must be a 6-by-1 array')

% Extracting the vectors v and w 
v = T( 1 : 3 );                                         % [TODO] WE HAVE TO CHECK IF WE WANT TO KEEP "v" FOR THIS VARIABLE
w = T( 4 : 6 );

% For advanced user, we can simply use the "symbolic" term of theta
if isa( theta, 'sym' )
    exp_T = sym( eye( 4 ) );
else
    exp_T = eye( 4 );
end
% Changing the w vector to a 3-by-3 skew-symmetric matrix
w_tilde = func_skewSym( w );

% Changing the so(3) matrix to a SO(3) matrixx
exp_w = func_getExponential_w( w, theta );


% This will be the first 3-by-3 component of the SE(3) Matrix
exp_T( 1:3, 1:3 ) = exp_w;

% For the v vector, we need to check whether it is a 
%   [a] A prismatic joint
%   [b] A  revolute joint
% [REF] For (a), Eq. 2.32 from
% [REF] For (b), Eq. 2.36 with zero pitch, from
% Murray, R et al, A mathematical introduction to robotic manipulation 

% If it's a prismatic joint, then angular velocity vector is a zero vector and theta_deg has unit [m]
if w == zeros( 3, 1 ) 
    exp_T( 1:3, 4 ) = v * theta;

% Else if it's a revolute joint.
else                                                         
    exp_T( 1:3 ,4 ) = ( eye( 3 ) - exp_w ) * ( w_tilde * v );
end

end