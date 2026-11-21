%% Jacobian – static joint torques for a payload in the worst-case configuration
% Uses the geometric Jacobian to map an end-effector wrench to joint torques.
% --- AI assistance (Claude, Anthropic) ---
% Split by AI from AlleOppgaver.m (section 'Wrenches').
% AI changes: uses createRobot(), removed unused variables (p_worstcase,
% p_worstcase2, T_worstcase, q_guess), comments added. Calculations are unchanged.
% -----------------------------------------
clear; clc

[robot, ~] = createRobot();

q_worstcase = [0 0 0 0 0]                 % arm fully extended
W_worstcase = [0; 0; -5; 0; 0; 0];        % wrench at end-effector [Fx Fy Fz Mx My Mz]

J = robot.jacob0(q_worstcase)
Q_worstcase = J' * W_worstcase            % joint torques
