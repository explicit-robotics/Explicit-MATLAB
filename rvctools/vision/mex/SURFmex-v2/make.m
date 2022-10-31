function make
    fprintf('Platform : %s\n',mexext);
    
    if matlab_version < 7.5
        %Old version of Matlab
        disp('WARNING: You have a too old version of MATLAB');
        disp('I will try my best to compile anyway');
		pause
    end

    %Change this to CUDA directory
    cuda_dir = 'C:/CUDA';
    
    mex('-outdir', mexext,'surfpoints.cpp','opencv_core211.lib','opencv_features2d211.lib');
    mex('-outdir', mexext,'surfmatch.cpp');
    mex('-outdir', mexext, ...
        '-Icommon/gpusurf',...
        ['-I' cuda_dir '/include'],...
        ['-L' cuda_dir '/lib64'],...
        'surfpoints_gpu.cpp',...
        'opencv_core211.lib',...
        'opencv_features2d211.lib',...
        'opencv_highgui211.lib',...
        '-lcuda',...
        '-lcudart',...
        [mexext '/gpusurf.lib']);
    disp('Compilation OK');
end

function v = matlab_version
    %File exchange #17285
    v = sscanf (version, '%d.%d.%d');
    v = 10.^(0:-1:-(length(v)-1)) * v;
end