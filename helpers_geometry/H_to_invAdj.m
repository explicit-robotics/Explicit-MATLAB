function Adj_Hinv = H_to_invAdj( H )
% ===============================================================================
% H_to_invAdj - Getting the inverse Adjoint Matrix representation 
%               of the Homogeneous Transformation Matrix H, which is an element of SE(3)
% Adj_Hinv = H_to_invAdj( H )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] H - A 4x4 Hom. Transformation matrix, which has the following form:
%       H = [ R   p ]
%           [ 0   1 ]
%
% Output 
%   [1] The 6x6 adjoint matrix, which has the following form:
%       Adj_H = [  R^T  -R^T * p_tilde ]
%               [  0               R^T ]
%       This is the inverse of the output of func_getAdjointMatrix
%
%   [REF] Eq. 2.58 from:
%         Murray, R et al, A mathematical introduction to robotic manipulation 
%
% See also func_getAdjointMatrix
% ===============================================================================

% Check whether the input matrix is a 4-by-4 real matrix
assert( isequal( size( H ), [ 4, 4 ] ) , ...
        'Input must be a 4-by-4 matrix')

% [Moses C. Nah] [2023.03.26]    
% While one can simply use H_to_Adj^(-1) for this function,
% this can be problematic with symbolic arguments.
% Hence, we define the inverse Adjoint matrix "from scratch".
    
% Extracting the R matrix and p array from the matrix H
R = H( 1:3 , 1:3 );
p = H( 1:3 , 4   );

% Initialize the 6x6 Adjoint matrix
Adj_Hinv = zeros( 6,6 );

% Wrapper in case if symbolic form
if isa( H, 'sym' )
    Adj_Hinv = sym( Adj_Hinv );
end

Adj_Hinv( 1:3, 1:3 ) = R.';
Adj_Hinv( 4:6, 4:6 ) = R.';
Adj_Hinv( 1:3, 4:6 ) = -R.' * vec_to_so3( p );

end