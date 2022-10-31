%SHARPENRGB Apply sharpening transform to RGB image
%
% OUT = SHARPENRGB(M, IN) is a color image (HxWx3) formed from applying the
% sharpening matrix M (3x3) to every pixel in the input image IN (HxWx3).
%
% See also SHARPEN.

function out = sharpenrgb(M, im)

    c = im2col(im) * M;
    
    out = col2im(c, im);