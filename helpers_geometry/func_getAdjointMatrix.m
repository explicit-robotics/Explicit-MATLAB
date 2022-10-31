function Adj_H = func_getAdjointMatrix( H )
% ============================================================================== 
% func_getAdjointMatrix - A 6x6 adjoint matrix to map between se(3)-vectors
% Adj_H = func_getAdjointMatrix( H )
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

% Check whether R and p are valid values
% assert( isequal( R * R', eye( 3 ) ) && ( det( R ) == 1 ), ...
%         'Inappropriate Rotation Matrix R is given. Please check R matrix')

% Initialize the 6x6 adjoint matrix
if isa( H, 'sym' )
    Adj_H = sym( zeros( 6,6 ) );
else
    Adj_H = zeros( 6,6 );
end

Adj_H( 1:3, 1:3 ) = R;
Adj_H( 4:6, 4:6 ) = R;
Adj_H( 1:3, 4:6 ) = func_skewSym( p ) * R;

end

