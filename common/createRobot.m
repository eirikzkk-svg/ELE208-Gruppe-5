function [robot,T0C] = createRobot()

alpha = pi/2;
d1 = 0.3; d2=0.2; d3=-0.2; d4=0.2; d5 = 0.2; 
L2 = 0.6726; L3 = 0.6726; 


L(1) = Link('d', d1, 'a', 0,  'alpha', alpha);
L(2) = Link('d', d2,     'a', L2, 'alpha', 0);
L(3) = Link('d', d3,    'a', L3, 'alpha', 0);
L(4) = Link('d', d4,    'a', 0, 'alpha', alpha);
L(5) = Link('d', d5,    'a', 0, 'alpha', 0);

robot = SerialLink(L, 'name', 'robot');

% Joint limits
robot.qlim = [ ...
    -pi    pi;       % Joint 1: revolute
    -pi/2    pi/2;       % Joint 2: revolute
    -5*pi/6    pi/12;      % Joint 3: revolute  Maybe try this so that yes
    -pi    pi;      % Joint 4: revolute
    -pi    pi];      % Joint 5: revolute

%Camera
xC = 1; % Position of camera in x-direction from the base
yC = 2; % y-direction
zC = 1; % z-direction

T0C = transl(xC, yC, zC)* trotx(pi); %Camera frame seen from base, camera is pointing downwards.

end
