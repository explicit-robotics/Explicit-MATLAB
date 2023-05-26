function x_hat = vec_to_so3( x )
% ====================================================================
% vec_to_so3 - Converts an R3 vector to so(3) matrix representation
% x_hat = vec_to_so3( x )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] x - A 1x3 or 3x1 array, given as x = [ x1, x2, x3 ]
% 
% Output 
%   [1] x_hat - A 3x3 skew-symmetric matrix, with the following form
%       x_hat = [   0   -x3    x2  ]
%               [  x3     0   -x1  ]
%               [ -x2    x1     0  ]
% ====================================================================

% The input should be a 1x3 or 3x1 array
assert( isequal( size( x ), [ 3, 1 ] ) || isequal( size( x ), [ 1, 3 ] ), ...
           'An input to this function must be a 3-by-1 or 1-by-3 array' )

x_hat = [      0,   -x( 3 ),     x( 2 ) ; ...
            x( 3 ),         0,    -x( 1 ) ; ...
           -x( 2 ),    x( 1 ),          0 ];
                
end