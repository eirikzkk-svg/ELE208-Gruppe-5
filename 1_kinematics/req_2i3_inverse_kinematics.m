%% Requirement 2.i.3 – Inverse kinematics and how it is used
clear; clc

[robot, T0C] = createRobot();
W = [-2 2 -2.1 2 -0.2 2.1];               % workspace for plotting

p0 = [1 0 1];                             % example object position
T_goal = CalculatePickUp(p0, pi/3, 0)

q_goal = robot.ikine(T_goal, 'mask', [1 1 1 0 1 1])

% Check
T_actual = robot.fkine(q_goal);
ik_error = norm(T_goal.T - T_actual.T)

figure
robot.plot(q_goal, 'workspace', W);
hold on
trplot(T0C);                              % camera frame
