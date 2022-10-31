function t = func_LieBracket( t1, t2 )
% ====================================================================
% func_LieBracket - Conduct a Lie Bracket operation
% Eq. 4.26 of Murray, Li, Sastry
% t = func_LieBracket( t1, t2 )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] t1 - A 6 x 1 twist #1
%   [2] t2 - A 6 x 1 twist #2
% 
% Output 
%   [1] t - The output of the lie bracket, which is as follows:
%           t = [t1, t2] = [t1^ t2^ - t2^ t1^]^{v}
%           Refer to Eq. 4.26 of Murray, Li, Sastry
%           
% ====================================================================

t = zeros( 6, 1 );

if ( isa( t1, 'sym') || isa( t2, 'sym' ) )
    t = sym( t );
end

t1_mat = func_getTwistMatrixForm( t1 );
t2_mat = func_getTwistMatrixForm( t2 );

t_mat = t1_mat * t2_mat - t2_mat * t1_mat;

t( 1 : 3 ) = t_mat( 1:3, 4 );
t( 4 : 6 ) = [ -t_mat( 2, 3 ); t_mat( 1, 3 ); -t_mat( 1, 2 ) ];

                
end