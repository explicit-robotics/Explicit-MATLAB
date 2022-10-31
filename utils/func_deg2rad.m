function q_new = func_deg2rad( q_old, joint_type )
% ========================================================================
% func_deg2rad - An utility function that changes degree 2 radian
%                with an account of the joint types        
% q_new = func_deg2rad( q_old, joint_type )
%
% Authors                         Email
%   Johannes Lachner              jlachner@mit.edu
%   Moses C. Nah                  mosesnah@mit.edu
% 
% Input 
%   [1] q_old - A 1xnq or nqx1 of joint displacements
%               nq is the degrees of freedom
% 
%   [2] joint_typre - An index array of 1xnq or nqx1,
%                     each value is either 1 or 2
%                     If revolute  joint, idx = 1
%                     If prismatic joint, idx = 2
%
% Output 
%   [1] q_new - A 1xnq or nqx1 of joint displacements
%               If revolute joint, change degrees to radian
%               For prismatic joint, just leave it 
% ========================================================================

% Check whether the length of the arrays are the same.
assert( length( q_old ) == length( joint_type ) )

N = length( q_old );
q_new = zeros( N, 1 );

for i = 1 : N

    if ( joint_type( i ) == 1 )
        q_new( i ) = q_old( i ) * pi/180;
    end
    
end