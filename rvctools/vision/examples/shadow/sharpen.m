%GENSHARPEN  Generate sharpening matrix
%
% M = gensharpen() is an RGB sharpening matrix (3x3).

function M = gensharpen()
    lam = [400:10:700]*1e-9;
    Q = loadspectrum(lam, 'bb2.dat')
    
    opt = optimset('display', 'iter')
    x0 = [pi/2 0.5 pi/2 pi/2 0 0.5];
    
    x = fminsearch(@(x) costfunc1(x, Q), x0, opt)
    
    [~,Qsharp,M] = costfunc1(x, Q);
     
    clf
    subplot(311)
    plot(lam*1e9, Q(:,1), lam*1e9, Qsharp(:,1), '--');
    subplot(312)
    plot(lam*1e9, Q(:,2), lam*1e9, Qsharp(:,2), '--');
    subplot(313)
    plot(lam*1e9, Q(:,3), lam*1e9, Qsharp(:,3), '--');
    xaxis(400, 700);
end

function [e,o2,o3] = costfunc1(x, spectrum)
    % state vector is 3 x (lat,long angles)
    % x = [th1 ph1 th2 ph2 th3 ph3]
    % spectrum Nx3
    
    theta = x(1); phi = x(2);
    c1 = [sin(theta)*cos(phi) sin(theta)*sin(phi) cos(theta)]';
    
    theta = x(3); phi = x(4);
    c2 = [sin(theta)*cos(phi) sin(theta)*sin(phi) cos(theta)]';
    
    theta = x(5); phi = x(6);
    c3 = [sin(theta)*cos(phi) sin(theta)*sin(phi) cos(theta)]';
    
    T = [c1 c2 c3];     % 3x3
    
    % compute sharpened spectrum
    sharp = spectrum*T;   % Nx3
        
    % compute spectral overlap
    overlap = sum( (sharp(:,1).*sharp(:,2))) + ...
        sum( (sharp(:,1).*sharp(:,3))) + ...
        sum((sharp(:,2).*sharp(:,3)));
    % compute sum of non-positive values, we don't really want this
    c = sharp(:);
    negativeness = sum(c(c<0));
    
    % final cost function
    e = overlap - 2*negativeness;
       
    if nargout > 1
        o2 = sharp;
    end
    if nargout > 2
        o3 = T;
    end
end
