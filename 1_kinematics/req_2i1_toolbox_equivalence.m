%% Requirement 2.i.1 – Equivalence between own forward kinematics and Robotics Toolbox
% Runs the symbolic derivation from requirement 1.i, then compares with fkine.

req_1i_dh_forward_kinematics              % gives the symbolic T05

[robot, T0C] = createRobot();
W = [-2 2 -2.1 2 -0.2 2.1];               % workspace for plotting

% Numerical values for the symbolic parameters (same as createRobot)
d1 = 0.3; d2 = 0.2; d3 = -0.2; d4 = 0.2; d5 = 0.2;
L2 = 0.6726; L3 = 0.6726;

% Test configuration
q1 = 0; q2 = pi/4; q3 = -pi/3; q4 = -(q2+q3); q5 = 0;
q = [q1 q2 q3 q4 q5];

figure
robot.plot(q, 'workspace', W);
hold on
trplot(T0C);                              % camera frame

T05_num   = double(subs(T05));
T_toolbox = robot.fkine(q);
difference = T05_num - T_toolbox.T
