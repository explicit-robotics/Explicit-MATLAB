function startup_rvc
    disp('Robotics, Vision & Control: (c) Peter Corke 1992-2020 http://www.petercorke.com')
    
    if verLessThan('matlab', '7.0')
        warning('You are running a very old (and unsupported) version of MATLAB.  You will very likely encounter significant problems using the toolboxes but you are on your own with this');
    end
    tb = false;
    startup_path = fileparts( mfilename('fullpath') );
    [~,folder]=fileparts(startup_path);
    if strfind(folder, 'common')
        % startup_rvc is in common folder
        rvcpath = fileparts(startup_path);
    else
        % startup_rvc is in folder above common
        rvcpath = startup_path;
    end
    
    robotpath = fullfile(rvcpath, 'robot');
    if exist(robotpath, 'dir')
        addpath(robotpath);
        tb = true;
        if exist('startup_rtb') == 2
            startup_rtb
        end
    end
    
    visionpath = fullfile(rvcpath, 'vision');
    if exist(visionpath, 'dir')
        addpath(visionpath);
        tb = true;
        if exist('startup_mvtb') == 2
            startup_mvtb
        end
    end
    
    if tb
        % RTB or MVTB is present
        
        % add spatial math toolbox
        p = fullfile(rvcpath, 'spatial-math');
        if exist(p, 'dir')
            try
                fp = fopen( fullfile(p, 'RELEASE'), 'r');
                release = fgetl(fp);
                fclose(fp);
            catch ME
                release = [];
            end
            if release
                release = ['(release ' release ')'];
            else
                release = '';
            end
            fprintf('- Spatial Math Toolbox for MATLAB %s\n', release)
            addpath(p);
        end
                
        % add common files
        addpath(fullfile(rvcpath, 'common'));
    else
        fprintf('Neither Robotics Toolbox or MachineVision Toolbox found in %s\n', rvcpath);
    end
        
    % check for any install problems
    rvccheck(false)
end

% function startup_rvc
%     disp('Robotics, Vision & Control: (c) Peter Corke 1992-2019 http://www.petercorke.com')
% 
%     if verLessThan('matlab', '7.0')
%         warning('You are running a very old (and unsupported) version of MATLAB.  You will very likely encounter significant problems using the toolboxes but you are on your own with this');
%     end
%     tb = false;
%     rvcpath = fileparts( mfilename('fullpath') );
% 
%     robotpath = fullfile(rvcpath, 'robot');
%     if exist(robotpath,'dir')
%         addpath(robotpath);
%         tb = true;
%         if exist('startup_rtb') == 2
%             startup_rtb(robotpath);
%         end
%     end
% 
%     visionpath = fullfile(rvcpath, 'vision');
%     if exist(visionpath,'dir')
%         addpath(visionpath);
%         tb = true;
%         if exist('startup_mvtb') == 2
%             startup_mvtb(visionpath);
%         end
%     end
% 
%     if tb
%         addpath(fullfile(rvcpath, 'spatial-math'));
%         addpath(genpath(fullfile(rvcpath, 'common')));
%         addpath(fullfile(rvcpath, 'simulink'));
%     end
% 
% end
