%% Requirement 1.ii – Transformation from the camera frame to the robot base
% --- AI assistance (Claude, Anthropic) ---
% Split by AI from AlleOppgaver.m (section 1.ii.1).
% AI changes: comments translated to English. Calculations are unchanged.
% -----------------------------------------
clear; clc

xC = 1;  % camera position from base, x [m]
yC = 2;  % y [m]
zC = 1;  % z [m]

T0C = transl(xC, yC, zC) * trotx(pi)      % camera pointing downwards (same as in createRobot)

% Test: point seen by the camera (conveyor 10 cm above base, chocolate 1 cm thick)
pC = [0.1; -0.5; 0.89; 1];
p0 = T0C * pC                             % same point in the base frame
