function adj = func_vec2adj( vec )
% ============================================================================== 
% func_vec2adj - A function for changing a ( v, w ) to an adjoint representation.
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

assert( length( vec ) == 6 );

adj = zeros( 6, 6 );
v = vec( 1:3 ); w = vec( 4:6 );

% Define the adjoint matrix
adj( 1:3, 1:3 ) = func_skewSym( w );
adj( 4:6, 4:6 ) = func_skewSym( w );
adj( 1:3, 4:6 ) = func_skewSym( v );


end

