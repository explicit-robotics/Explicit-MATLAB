function x_skew = func_skewSym( x )
% ====================================================================
% func_skewSym - Converts an array to a matrix
% x_skew = func_skewSym( x )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] x - A 1x3 or 3x1 array, given as x = [ x1, x2, x3 ]
% 
% Output 
%   [1] x_skew - A 3x3 skew-symmetric matrix, which converts an
%                       array to a matrix
%       x_skew = [   0   -x3    x2  ]
%                [  x3     0   -x1  ]
%                [ -x2    x1     0  ]
% ====================================================================

% The input should be a 1x3 or 3x1 array
assert( isequal( size( x ), [ 3, 1 ] ), 'An input must be a 3-by-1 array')

x_skew = [      0,   -x( 3 ),     x( 2 ) ; ...
           x( 3 ),         0,    -x( 1 ) ; ...
          -x( 2 ),    x( 1 ),          0 ];
                
end