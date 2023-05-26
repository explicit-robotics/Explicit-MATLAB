function [p, dp, ddp] = min_jerk_traj( t0, t, D, qi, qf )

% Assertion
assert( t0 > 0 && ( length( qi ) == length( qf ) ) )

% The normalized time 
tau = (t - t0)/D; 

% The length of qi and qf
N = length( qi );

if t <= t0
    p   = qi;
    dp  = zeros( N, 1 );
    ddp = zeros( N, 1 );
    
elseif t0 <= t && t <= t0 + D
    p   =        qi + ( qf - qi ) * ( 10 * tau^3 -  15 * tau^4 +   6 * tau^5 );
    dp  =      1./D * ( qf - qi ) * ( 30 * tau^2 -  60 * tau^3 +  30 * tau^4 );
    ddp =  1./(D^2) * ( qf - qi ) * ( 60 * tau^1 - 180 * tau^2 + 120 * tau^3 );
    
else
    p   = qf;
    dp  = zeros( N, 1 );
    ddp = zeros( N, 1 );
end


end