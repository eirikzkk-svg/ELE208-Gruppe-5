%% Requirement 1.i – DH parameters and forward kinematics (symbolic, without Robotics Toolbox)
clear; clc

syms q1 q2 q3 q4 q5 real
syms d1 d2 d3 d4 d5 L2 L3 real

%% 1.i.1 Table of DH parameters
robotSym.DH(1) = struct('type','R', 'a',0,  'alpha',pi/2, 'd',d1, 'theta',q1);
robotSym.DH(2) = struct('type','R', 'a',L2, 'alpha',0,    'd',d2, 'theta',q2);
robotSym.DH(3) = struct('type','R', 'a',L3, 'alpha',0,    'd',d3, 'theta',q3);
robotSym.DH(4) = struct('type','R', 'a',0,  'alpha',pi/2, 'd',d4, 'theta',q4);
robotSym.DH(5) = struct('type','R', 'a',0,  'alpha',0,    'd',d5, 'theta',q5);

DHmatrix = @(p) [ ...
    cos(p.theta), -sin(p.theta)*cos(p.alpha),  sin(p.theta)*sin(p.alpha), p.a*cos(p.theta);
    sin(p.theta),  cos(p.theta)*cos(p.alpha), -cos(p.theta)*sin(p.alpha), p.a*sin(p.theta);
    0,             sin(p.alpha),               cos(p.alpha),              p.d;
    0,             0,                          0,                         1];

%% 1.i.2 Transformation from end-effector to base
T01 = DHmatrix(robotSym.DH(1));
T12 = DHmatrix(robotSym.DH(2));
T23 = DHmatrix(robotSym.DH(3));
T34 = DHmatrix(robotSym.DH(4));
T45 = DHmatrix(robotSym.DH(5));

T02 = simplify(T01*T12)
T03 = simplify(T01*T12*T23)
T04 = simplify(T01*T12*T23*T34)          % first 4 joints (requirement 1.i)
T05 = simplify(T01*T12*T23*T34*T45)
