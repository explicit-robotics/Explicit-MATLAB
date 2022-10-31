function Adj_Hinv = func_getInvAdjointMatrix( H )
% ===============================================================================
% func_getInvAdjointMatrix - A 6x6 adjoint matrix to map between se(3) vectors
% Adj_Hinv = func_getInvAdjointMatrix( H )
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

% Extracting the R matrix and p array from the matrix H
R = H( 1:3 , 1:3 );
p = H( 1:3 , 4   );

% Initialize the 6x6 adjoint matrix
% For advanced user, we can use the symbolic form.
if isa( H, 'sym' )
    Adj_Hinv = sym( zeros( 6,6 ) );
else
    Adj_Hinv = zeros( 6,6 );
end

Adj_Hinv( 1:3, 1:3 ) = R.';
Adj_Hinv( 4:6, 4:6 ) = R.';
Adj_Hinv( 1:3, 4:6 ) = -R.' * func_skewSym( p );

end