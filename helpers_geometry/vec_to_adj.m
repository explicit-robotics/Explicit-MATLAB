function adj = vec_to_adj( vec )
% ============================================================================== 
% vec_to_adj - A function for changing a ( v, w ) to an adjoint representation.
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] vec - A 1x6 or 6x1 twist array, T = [ v, w ] 
%
% Output 
%   [1] adj - A 6x6 matrix representation of the twist array. 
%           adj = [ [w] [v] ]
%                 [  0  [w] ]           
%                Note that our placement of [v] is different from [REF]
%       [REF] Lynch, Kevin M., and Frank C. Park. Modern robotics. Cambridge University Press, 2017.
%
% ==============================================================================

% The input should be a 1x6 or 6x1 array
assert( isequal( size( vec ), [ 6, 1 ] ) || isequal( size( vec ), [ 1, 6 ] ), ...
           'An input to this function must be a 6-by-1 or 1-by-6 array' )

% Initialize the 6-by-6 adjoint matrix        
% Initializing the 4-by-4 T matrix with some random matrix 
adj = zeros( 6, 6 );

% Wrapper in case if symbolic form
if isa( vec, 'sym' )
    adj = sym( adj );
end

v = vec( 1:3 ); w = vec( 4:6 );

% Define the adjoint matrix
adj( 1:3, 1:3 ) = vec_to_so3( w );
adj( 4:6, 4:6 ) = vec_to_so3( w );
adj( 1:3, 4:6 ) = vec_to_so3( v );


end

