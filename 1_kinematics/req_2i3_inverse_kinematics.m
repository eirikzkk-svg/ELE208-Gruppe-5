%% Requirement 2.i.3 – Inverse kinematics, tested at the required reach points
% --- AI assistance (Claude, Anthropic) ---
% Split by AI from AlleOppgaver.m (section 2.i.3, both tests written by the group).
% AI changes: uses createRobot() and defines W locally, %% sections and figure
% titles added for each test. Calculations are unchanged.
% -----------------------------------------
clear; clc

[robot, T0C] = createRobot();
W = [-2 2 -2.1 2 -0.2 2.1];               % workspace for plotting

%% Test 1: pick-up height (1 m)
p0 = [1 0 1];
T_goal = CalculatePickUp(p0, pi/3, 0)

q_goal = robot.ikine(T_goal, 'mask', [1 1 1 0 1 1])

% Check
T_actual = robot.fkine(q_goal);
ik_error = norm(T_goal.T - T_actual.T)

figure
robot.plot(q_goal, 'workspace', W);
hold on
trplot(T0C);                              % camera frame
title('Test 1: pick-up height, p = [1 0 1]')

%% Test 2: drop-off reach (1.2 m)
p0 = [1.2 0 0.3];
T_goal = CalculatePickUp(p0, pi/3, 0)

q_goal = robot.ikine(T_goal, 'mask', [1 1 1 0 1 1])

% Check
T_actual = robot.fkine(q_goal);
ik_error = norm(T_goal.T - T_actual.T)

figure
robot.plot(q_goal, 'workspace', W);
hold on
trplot(T0C);                              % camera frame
title('Test 2: drop-off reach, p = [1.2 0 0.3]')
