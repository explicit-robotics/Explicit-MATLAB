%SHOWSHARPEN Show effect of sharpening
%
% Shows the unsharpened and sharpened response for each sensor channel.
%
% See also SHARPEN.

function showsharpen(M)

    lam = [400:5:700]*1e-9;

    % load camera response
    cones = loadspectrum(lam, 'bb2.dat');
    sharp = cones * M;

    clf
    subplot(311)
    plot(lam*1e9, cones(:,1), lam*1e9, sharp(:,1), '--');
    subplot(312)
    plot(lam*1e9, cones(:,2), lam*1e9, sharp(:,2), '--');
    subplot(313)
    plot(lam*1e9, cones(:,3), lam*1e9, sharp(:,3), '--');
    xaxis(400, 700);
