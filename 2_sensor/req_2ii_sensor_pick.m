%% Requirement 2.ii – Using the camera measurement to command the robot
% Camera point -> base frame -> pick trajectory (planPick2) -> joint angles.
% Requires planPick2.m (3_motion_planning).
clear; clc

[robot, T0C] = createRobot();

% Parameters
P.v            = 0.3;                     % conveyor speed [m/s]
P.yStart       = 0.3*0.15;                % start of pick-up: 0.3 m/s * 0.15 s
P.pOverDropOff = [0 -1 0.05];             % 5 cm over drop-off (incl. 1 cm chocolate)
P.tMoveStart   = 1.5;                     % home -> over start [s]
P.tDown        = 0.3;                     % down to the chocolate [s]
P.qHome        = ConstructPose(0, pi/4, -pi/4, 0);

% Chocolate 1
pC  = [0.1; -0.5; 1.05; 1];               % point seen from camera
p0  = T0C * pC                            % point in base frame
psi = pi/3;                               % rotation of chocolate

[qTotal, tTotal] = planPick2(robot, p0, psi, P);

figure
robot.plot(qTotal)

%% End-effector position over time
TAll = robot.fkine(qTotal);

figure
plot(tTotal, TAll.transl);
legend('x', 'y', 'z');
xlabel('Time [s]'); ylabel('Position [m]');
title('End-effector position during the pick cycle');

%% End-effector path in 3D
Pee = TAll.transl;
x = Pee(:,1);  y = Pee(:,2);  z = Pee(:,3);

figure
plot3(x, y, z, 'k-', 'LineWidth', 1); hold on
scatter3(x, y, z, 20, tTotal, 'filled');  % colour = time
colormap(jet); cb = colorbar; cb.Label.String = 'Time [s]';

plot3(x(1),   y(1),   z(1),   'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(0, 0, 0, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');   % robot base

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('End-effector path in 3D');
legend('Path', 'Time', 'Start', 'End', 'Base');
axis equal; grid on; view(45, 25);

%% Reach of the robot and the pick-up zone on the conveyor
L2 = 0.6726; L3 = 0.6726;
th = linspace(0, 2*pi, 100);
r  = L2 + L3;
x  = r*cos(th);  y = r*sin(th);  z = zeros(size(th));

hold on
plot3(x, y, z, 'LineWidth', 2)                                  % reach
plot3([0.8 0.8], [-2 2], [-0.05 -0.05], 'LineWidth', 2)         % conveyor border
plot3([1.2 1.2], [-2 2], [-0.05 -0.05], 'LineWidth', 2)         % conveyor border
plot3([0.8 1.2], [ 0.045  0.045], [-0.05 -0.05], 'LineWidth', 2) % start of pick-up
plot3([0.8 1.2], [-0.045 -0.045], [-0.05 -0.05], 'LineWidth', 2) % end of pick-up
axis equal
xlabel('x [m]'); ylabel('y [m]');
title('Reach of the robot and the conveyor');
