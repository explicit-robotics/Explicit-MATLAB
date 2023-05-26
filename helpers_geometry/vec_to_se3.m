function T_matrix = vec_to_se3( T )
% vec_to_se3 - Convert a 6D twist array to a 4x4 matrix
% T_matrix = vec_to_se3( T )
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
assert( isequal( size( T ), [ 6, 1 ] ) || isequal( size( T ), [ 1, 6 ] ), ...
            'An input to this function must be a 6-by-1 or 1-by-6 array' )

% Extracting the vectors v and w 
v = T( 1 : 3 );
w = T( 4 : 6 );                                 

% Initializing the 4-by-4 T matrix
T_matrix = zeros( 4, 4 );

% Wrapper of "sym" in case if the argument is symbolic form.
if isa( T, 'sym' )
    T_matrix = sym( T_matrix );
end

% Setting up the T_matrix
T_matrix( 1:3, 1:3 ) = vec_to_so3( w );
T_matrix( 1:3,   4 ) = v;

end

