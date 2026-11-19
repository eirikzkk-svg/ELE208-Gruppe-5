function [qTotal,tTotal] = planPick2(robot,p0,psi, P)
%---Handle time-----------------------------------
dt = 0.05; 
dt2 = 0.05;

tUpDown       = (0:dt:P.tDown)';             % ned/opp
tPick         = (0:dt:0.3)';                 % langs båndet
tOverDropOff  = (0:dt2:1.5)';                 % til avlevering / hjem
tStart        = (0:dt2:P.tMoveStart)';

%---Positions along the belt----------------------
dz = 0.05;                                   % løftehøyde
dy = P.v * P.tDown / 2;                      % strekning i y når farten går 0 <-> v
pStart = CalculatePickUp(p0, psi,  P.yStart).t';   % [x y z] start plukk
%pStop  = CalculatePickUp(p0, psi, -P.yStart).t';   % [x y z] slutt plukk

%---Define the desired end-effector poses---------
TOverStartOfPickUp = goalPose([pStart(1), pStart(2) + dy, pStart(3) + dz], psi);  % ENDRET
TStartOfPickUp     = goalPose(pStart, psi);
%TStopOfPickUp      = goalPose(pStop,  psi);
TOverDropOff       = goalPose(P.pOverDropOff, 0);
TDropOff           = SE3(0, 0, -dz) * TOverDropOff;

%---Home -> Over start of pick up-----------------
qHome = P.qHome;
qOverStart = robot.ikine(TOverStartOfPickUp, 'mask', [1 1 1 0 1 1], 'q0', qHome);
qMoveStart = mtraj(@tpoly, qHome, qOverStart, tStart);

%---NY: Down to the chocolate (y: 0 -> -v, z: myk landing)
y = tpoly(pStart(2) + dy, pStart(2),  tUpDown, 0, -P.v);
z = tpoly(pStart(3) + dz, pStart(3),  tUpDown, 0, 0);
TdownToPick = posesAlong(pStart(1), y, z, psi);
qc0 = robot.ikine(TdownToPick, 'mask', [1 1 1 0 1 1], 'q0', qOverStart);


%---Along the conveyor belt: differensialkinematikk-------
v    = [0; -P.v; 0; 0; 0; 0];       % ønsket verktøyfart: følg båndet
rows = [1 2 3 5 6];                 % samme frihetsgrader som masken [1 1 1 0 1 1]

qc = zeros(length(tPick), 5);
qc(1,:) = qc0(end, :);              % start der griperen landet
for k = 1:length(tPick)-1
    J  = robot.jacob0(qc(k,:));
    qd = J(rows,:) \ v(rows);     % leddhastigheter fra ønsket verktøyfart
    qc(k+1,:) = qc(k,:) + qd' * dt;
end
pStop = robot.fkine(qc(end,:)).t'; 

%---Up from the belt (y: -v -> 0, z: myk start)
y = tpoly(pStop(2), pStop(2) - dy, tUpDown, -P.v, 0);
z = tpoly(pStop(3), pStop(3) + dz, tUpDown, 0, 0);
Tup = posesAlong(pStop(1), y, z, psi);
qc2 = robot.ikine(Tup, 'mask', [1 1 1 0 1 1], 'q0', qc(end, :));

%---Up from belt -> Over drop off-----------------
qOverDropOff = robot.ikine(TOverDropOff, 'mask', [1 1 1 0 1 1], 'q0', qc2(end, :));
qMoveOverDrop = mtraj(@tpoly, qc2(end, :), qOverDropOff, tOverDropOff);

%---Down / up at drop off (starter og slutter i ro)
Tdown = ctraj(TOverDropOff, TDropOff, length(tUpDown));
qc3 = robot.ikine(Tdown, 'mask', [1 1 1 0 1 1], 'q0', qOverDropOff);
TupFromDrop = ctraj(TDropOff, TOverDropOff, length(tUpDown));
qc4 = robot.ikine(TupFromDrop, 'mask', [1 1 1 0 1 1], 'q0', qc3(end, :));

%---Drop off -> Home------------------------------
qMoveHome = mtraj(@tpoly, qc4(end, :), qHome, tOverDropOff);

%---Total-----------------------------------------
qTotal = [qMoveStart;
          qc0(2:end, :);
          qc(2:end, :);
          qc2(2:end, :);
          qMoveOverDrop(2:end, :);
          qc3(2:end, :);
          qc4(2:end, :);
          qMoveHome(2:end, :)];

%---Total time (samme rekkefølge som qTotal)------
t0 = 0;
tTotal = tStart;                              t0 = t0 + tStart(end);
tTotal = [tTotal; tUpDown(2:end)      + t0];  t0 = t0 + tUpDown(end);
tTotal = [tTotal; tPick(2:end)        + t0];  t0 = t0 + tPick(end);
tTotal = [tTotal; tUpDown(2:end)      + t0];  t0 = t0 + tUpDown(end);
tTotal = [tTotal; tOverDropOff(2:end) + t0];  t0 = t0 + tOverDropOff(end);
tTotal = [tTotal; tUpDown(2:end)      + t0];  t0 = t0 + tUpDown(end);
tTotal = [tTotal; tUpDown(2:end)      + t0];  t0 = t0 + tUpDown(end);
tTotal = [tTotal; tOverDropOff(2:end) + t0];
end

%% NY: lager en SE3-pose for hvert punkt langs (x, y(i), z(i))
function T = posesAlong(x, y, z, psi)
    T = goalPose([x, y(1), z(1)], psi);
    for i = 2:length(y)
        T(i) = goalPose([x, y(i), z(i)], psi);
    end
end
