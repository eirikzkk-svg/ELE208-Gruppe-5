
clear
clc

syms q1 q2 q3 q4 q5 real
syms d1 d2 d3 d4 d5 L2 L3  real
% Initial conditions

W=[-2 2 -2.1 2 -0.2 2.1]; %Workplace for plotting


%% 1.i.1. Develop the table of DH parameters
robotSym.DH(1) = struct('type','R', 'a',0, 'alpha',pi/2, 'd',d1, 'theta',q1);
robotSym.DH(2) = struct('type','R', 'a',L2, 'alpha',0, 'd',d2, 'theta',q2);
robotSym.DH(3) = struct('type','R', 'a',L3, 'alpha',0, 'd',d3, 'theta',q3);
robotSym.DH(4) = struct('type','R', 'a',0, 'alpha',pi/2, 'd',d4, 'theta',q4);
robotSym.DH(5) = struct('type','R', 'a',0, 'alpha',0, 'd',d5, 'theta',q5);


DHmatrix = @(p) [ ...
    cos(p.theta), -sin(p.theta)*cos(p.alpha),  sin(p.theta)*sin(p.alpha), p.a*cos(p.theta);
    sin(p.theta),  cos(p.theta)*cos(p.alpha), -cos(p.theta)*sin(p.alpha), p.a*sin(p.theta);
    0,           sin(p.alpha),             cos(p.alpha),            p.d;
    0,           0,                       0,                    1];
%% 1.i.2. Develop the transformation mapping end-effector to base (for the first 4 joints only)

T01 = DHmatrix(robotSym.DH(1));
T12 = DHmatrix(robotSym.DH(2));
T23 = DHmatrix(robotSym.DH(3));
T34 = DHmatrix(robotSym.DH(4));
T45 = DHmatrix(robotSym.DH(5));

T02 = simplify(T01*T12)
T03 = simplify(T01*T12*T23)
T04 = simplify(T01*T12*T23*T34)
T05 = simplify(T01*T12*T23*T34*T45)

%% 1.ii.1. Develop the transformation from the chosen sensor system to the relevant coordinate system on the robot (world, end-effector, mobile robot, etc)
xC = 1; % Position of camera in x-direction from the base
yC = 2; % y-direction
zC = 1; % z-direction

T0C = transl(xC, yC, zC)* trotx(pi) %Camera frame seen from base, camera is pointing downwards.

%Test
pC = [0.1; -0.5; 0.89; 1]; %Point seen from camera, antar at transportbånd er 10cm over basen, og sjokoladen er 1cm tykk
p0 = T0C*pC %From base to point

%% 2. Model robot kinematics with Robotics Toolbox
alpha = pi/2;
d1 = 0.3; d2=0.2; d3=-0.2; d4=0.2; d5 = 0.2; 
L2 = 0.6726; L3 = 0.6726; 


L(1) = Link('d', d1, 'a', 0,  'alpha', alpha)
L(2) = Link('d', d2,     'a', L2, 'alpha', 0)
L(3) = Link('d', d3,    'a', L3, 'alpha', 0)
L(4) = Link('d', d4,    'a', 0, 'alpha', alpha)
L(5) = Link('d', d5,    'a', 0, 'alpha', 0)

robot = SerialLink(L, 'name', 'robot')
q1 = 0;
q2=pi/4;
q3=-pi/3;
q4= -(q2+q3);
q5=0;
q = [q1 q2 q3 q4 q5]

% Joint limits
robot.qlim = [ ...
    -pi    pi;       % Joint 1: revolute
    -pi/2    pi/2;       % Joint 2: revolute
    -5*pi/6    pi/12;      % Joint 3: revolute  Maybe try this so that yes
    -pi    pi;      % Joint 4: revolute
    -pi    pi];      % Joint 5: revolute

figure
robot.plot(q, 'workspace', W);
hold on
trplot(T0C);   % Plot camera


%% 2.i.1. Demonstrate equivalence of the forward kinematic solution obtained previously in Matlab (not Toolbox) or by hand

T05_num = double(subs(T05));
T_toolbox = robot.fkine(q);

difference = (T05_num - T_toolbox.T)


%% 2.i.3. Develop the inverse kinematics, and demonstrate how it could be used
%% Test 1: pick-up height (1 m)
p0 = [1 0 1];
T_goal = CalculatePickUp(p0,pi/3, 0) 
% --------- Inverse kinematics -----------
q_goal = robot.ikine(T_goal, 'mask', [1 1 1 0 1 1])

%Check
T_actual = robot.fkine(q_goal);
ik_error = norm(T_goal.T - T_actual.T)
% ------Plot------
figure
robot.plot(q_goal, 'workspace', W);
hold on
trplot(T0C);   % Plot camera

%% Test 2: drop-off reach (1.2 m)
p0 = [1.2 0 0.3];
T_goal = CalculatePickUp(p0,pi/3, 0) 
% --------- Inverse kinematics -----------
q_goal = robot.ikine(T_goal, 'mask', [1 1 1 0 1 1])

%Check
T_actual = robot.fkine(q_goal);
ik_error = norm(T_goal.T - T_actual.T)
% ------Plot------
figure
robot.plot(q_goal, 'workspace', W);
hold on
trplot(T0C);   % Plot camera


%% 2.ii.1. Demonstrate using the sensory system to command the robot, according to the task chosen. 

clc
clear
[robot, T0C] = createRobot();           %Returns robot and Transformation from base to camera

% Parameters

P.v = 0.3;
P.yStart = 0.3*0.15;                    %0.3m/s * 0.15 s - Start of pickup
P.pOverDropOff =  [0 -1 0.05];          %5 cm over conveyorbelt + 1 cm pga tykkelsen på sjokoladen
P.tMoveStart = 1.5;                     % home -> over start [s]
P.tDown      = 0.3;                     % ned til sjokoladen [s]
P.qHome = ConstructPose(0, pi/4, -pi/4, 0);

% Chocolate 1
pC = [0.1; -0.5; 1.05; 1];              %Point seen from camera
p0 = T0C*pC                             %From base to point
psi = pi/3;                             %Rotation of chocolate
[qTotal, tTotal] = planPick2(robot,p0,psi,P)
figure;
robot.plot(qTotal)

%% 4.iii Velocity reference from the pick trajectory
t   = tTotal(:);
q2  = qTotal(:,2);

[t, idx] = unique(t);                 % fjerner dobbelte tidspunkt i skjøtene
q2 = q2(idx);

qd2 = gradient(q2, t);                % skulderens hastighet [rad/s]
w_ref = [t, qd2];                     % for å bruke i vloop_test

max(abs(qd2))                         % sjekk: under ~2,6 rad/s?
figure; plot(t, qd2); grid on
xlabel('t [s]'); ylabel('\omega_2 [rad/s]'); title('Shoulder speed reference')

%%
%---Plot posisjon av end effector til enhver tid
TAll = robot.fkine(qTotal);


figure;
plot(tTotal, TAll.transl);
legend('x','y','z');
xlabel('Tid [s]'); ylabel('Posisjon [m]');
title('End-effector-posisjon gjennom hele operasjonen');
%%
%---End-effector-bane i 3D----------------------------------
P = TAll.transl;
x = P(:,1);  y = P(:,2);  z = P(:,3);

figure;
plot3(x, y, z, 'k-', 'LineWidth', 1); hold on;
scatter3(x, y, z, 20, tTotal, 'filled');   % farge = tid
colormap(jet); cb = colorbar; cb.Label.String = 'Tid [s]';

plot3(x(1),   y(1),   z(1),   'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(0, 0, 0, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');   % robotbase

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('End-effector-bane i 3D');
legend('Bane', 'Tid', 'Start', 'Slutt', 'Base');
axis equal; grid on; view(45, 25);
%% Plot reach of robot and where it would start and end pickup
%Reach of robot
L2 = 0.6726; L3 = 0.6726; 
th = linspace(0, 2*pi, 100);
r  = L2+L3;          % din radius
zc = 0;          % høyden
x  = r*cos(th);
y  = r*sin(th);
z  = zc*ones(size(th));

hold on
plot3(x, y, z, 'LineWidth', 2) %Radius where robot can reach
plot3([0.8 0.8], [-2 2], [-0.05 -0.05], 'LineWidth', 2) %Border of conveyor belt
plot3([1.2 1.2], [-2 2], [-0.05 -0.05], 'LineWidth', 2) %Border of conveyor belt
plot3([0.8 1.2], [0.045 0.045], [-0.05 -0.05], 'LineWidth', 2) %Where the robot would start pickup
plot3([0.8 1.2], [-0.045 -0.045], [-0.05 -0.05], 'LineWidth', 2) %Where the robot would end pickup, will have an cartesian trajectory between these lines
axis equal
xlabel('x [m]'); ylabel('y [m]');
title('Maks rekkevidde med transportbånd');


%% Wrenches
p_worstcase = [1 0 1];
p_worstcase2 = [1.2 0 0];

T_worstcase = goalPose (p_worstcase,0);
q_guess = ConstructPose(0, pi/4, -pi, 0 );
q_worstcase = [0 0 0 0 0]

W_worstcase = [0; 0; -5; 0; 0; 0]; 
J = robot.jacob0(q_worstcase)
Q_worstcase = J'*W_worstcase




%% Functions
function q = ConstructPose(q1,q2,q3,q5)
    q = [q1 q2 q3 -(q2+q3) q5];
end

function T = goalPose(p,psi)
    T = SE3(transl(p) * trotz(psi) * trotx(pi)); 
end

function T_pickup = CalculatePickUp(p_init, psi, y)
    p_pickup = [p_init(1) y p_init(3)];
    T_pickup = goalPose(p_pickup, psi);
end

