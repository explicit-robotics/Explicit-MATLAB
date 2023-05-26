function Adj_H = H_to_Adj( H )
% ============================================================================== 
% H_to_Adj - Getting the Adjoint Matrix representation of the Homogeneous Transformation Matrix H,
%            which is an element of SE(3)
% Adj_H = H_to_Adj( H )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] H - A 4x4 Hom. transformation matrix, which has the following form:
%       H = [ R   p ]
%           [ 0   1 ]
%
% Output 
%   [1] The 6x6 adjoint matrix, which has the following form:
%       Adj_H = [  R  p_tilde * R ]
%               [  0            R ]
%   [REF] Eq. 2.58 from:
%         Murray, R et al, A mathematical introduction to robotic manipulation 
%
% See also func_getInvAdjointMatrix
% ==============================================================================

% Check whether the input matrix is a 4x4 real matrix
assert( isequal( size( H ), [ 4, 4 ] ) , 'Input must be a 4-by-4 matrix')

% Taking out the R matrix and p vector from the H matrix
R = H( 1:3 , 1:3 );
p = H( 1:3 , 4   );

% Initialize the 6x6 adjoint matrix
Adj_H = zeros( 6,6 );

% Wrapper if symbolic form
if isa( H, 'sym' )
    Adj_H = sym( Adj_H  );
end

Adj_H( 1:3, 1:3 ) = R;
Adj_H( 4:6, 4:6 ) = R;
Adj_H( 1:3, 4:6 ) = vec_to_so3( p ) * R;

end

