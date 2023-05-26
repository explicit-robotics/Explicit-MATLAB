% [Project] Exp[licit] 
% - The robotic simulator based on differential geometry
% You must run this setup.m file to run the .m scripts.
%
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
%
% The code is heavily commented. A famous quote says:
% "Code is read more often than it is written"
%           - Guido Van Rossum, the Creator of Python

%% Cleaning up + Environment Setup
clear; close all; clc;

% Include all the subdirectories
func_addSubfolders( 'animation', 'helpers_geometry', 'robots',...
                   'examples', 'interpolator', 'utils', 'graphics', 'obj');

disp( 'EXPlicit (c) Moses C. Nah & Johannes Lachner 2022-2023, https://explicit-robotics.github.io' )