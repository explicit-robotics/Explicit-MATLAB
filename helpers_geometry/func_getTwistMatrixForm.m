function T_matrix = func_getTwistMatrixForm( T )
% func_getTwistMatrixForm - Convert a 6D twist array to a 4x4 matrix
% T_matrix = func_getTwistMatrixForm( T )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] T - A 1x6 or 6x1 twist array, T = [ v, w ]
% 
% Output 
%   [1] T_matrix - A 4x4 matrix with the following form:
%                  Given T = [ v, w ] 
%                  where w = [ wx, wy, wz ], v = [ vx, vy, vz ], then:
%       T_matrix = [   0  -wz   wy  vx  ]
%                  [  wz    0  -wx  vy  ]
%                  [ -wy   wx    0  vz  ]
%                  [   0    0    0   1  ] 


% The input should be a 1x6 or 6x1 array
assert( isequal( size( T ), [ 6, 1 ] ), 'An input must be a 6-by-1 array')

% Extracting the vectors v and w 
v = T( 1 : 3 );
w = T( 4 : 6 );                                 % [TODO] WE HAVE TO CHECK IF WE WANT TO KEEP "v" FOR THIS VARIABLE

% Initializing the 4-by-4 T matrix with some random matrix 
T_matrix = zeros( 4,4 );

if isa( T, 'sym' )
    T_matrix = sym( T_matrix );
end

% Setting up the T_matrix
T_matrix( 1:3, 1:3 ) = func_skewSym( w );
T_matrix( 1:3,   4 ) = v;

end

