% SETUP  Add all project folders to the MATLAB path.
% Run this once each MATLAB session before running any other script.
%
% Requires: MATLAB, Peter Corke's Robotics Toolbox for MATLAB (RTB 10.x),
% Spatial Math Toolbox, toolbox-common, Simulink and Control System Toolbox.

root = fileparts(mfilename('fullpath'));
addpath(genpath(root));
rmpath(genpath(fullfile(root, '.git')));   % keep Git internals off the path

fprintf('ELE208 project paths added from:\n  %s\n', root);
