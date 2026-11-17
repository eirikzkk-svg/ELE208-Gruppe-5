%% Jacobian – static joint torques for a payload in the worst-case configuration
% Uses the geometric Jacobian to map an end-effector wrench to joint torques.
clear; clc

[robot, ~] = createRobot();

q_worstcase = [0 0 0 0 0]                 % arm fully extended
W_worstcase = [0; 0; -5; 0; 0; 0];        % wrench at end-effector [Fx Fy Fz Mx My Mz]

J = robot.jacob0(q_worstcase)
Q_worstcase = J' * W_worstcase            % joint torques
