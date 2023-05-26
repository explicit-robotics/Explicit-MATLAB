function [ L_Mat, W_Mat ] = LW_Mat( A_arr, H_COMs, q_arr )
% ============================================================================== 
% LW_Mat - A function for calculating the L and W matrix
% [ L_Mat, W_Mat ] = LW_Mat( A_arr, H_COMs, q_arr )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] A_arr  - A 6xnq matrix, where the i-th column is the joint-twist
%               expressed in the {i}-th frame, attached to the COM of the i-th link.
%               2nd Equation of Section 8.3.1. of Modern Robotics textbook
%
%   [2] H_COMs - A 4x4xnq matrix, where the i-th 2D matrix is the 
%                Hom. Trans. Matrix of the {i}-th frame, attached to the COM of the i-th link.
%
%   [3] q_arr  - A 1xnq (or nqx1) array which is the joint-displacement of the robot
%
% Output 
%   [1] L_mat  - A 6nqx6nq matrix, Eq. 8.72 from [REF]
%                
%   [2] W_mat  - A 6nqx6nq matrix, Eq. 8.64 from [REF]
%         [REF] Lynch, Kevin M., and Frank C. Park. Modern robotics. Cambridge University Press, 2017.
%
% ==============================================================================

% Check whether the input values are in appropriate size
% q_arr must be a vector 
assert( isvector( q_arr ) );

% Once it is clear it is a vector, get the length to use it as an assertion
n = length( q_arr );
assert( isequal(  size( A_arr ), [ 6, n ]    ) , 'First  Input must be a 6-nq matrix')
assert( isequal( size( H_COMs ), [ 4, 4, n ] ) , 'Second Input must be a 4-by-4-by-nq matrix')

% Using equation 8.64 and 8.72 from Section 8.4 of Modern Robotics.
W_Mat = zeros( 6*n, 6*n );
L_Mat = eye( 6*n );

% Wrapper of Symbolic form
if isa( q_arr, 'sym' )
    W_Mat = sym( W_Mat );
    L_Mat = sym( L_Mat );
end

for i = 2:n
   Ai = A_arr( :, i );               
   Hi = H_COMs( :, :, i   );
   Hj = H_COMs( :, :, i-1 );

   % Using the definition from 8.3.1, 3rd equation of [REF]
   Wi = H_to_Adj( expSE3( -Ai, q_arr( i ) ) * Hi^-1 * Hj );
   
   % A slight trick is used for the computation
   W_Mat( 6*(i-1)+1:6*i, 6*(i-2)+1:6*(i-1) ) = Wi;
   L_Mat( 6*(i-1)+1:6*i, 1:6*(i-1) ) = Wi * L_Mat( 6*(i-2)+1:6*(i-1), 1:6*(i-1) );
end

end

