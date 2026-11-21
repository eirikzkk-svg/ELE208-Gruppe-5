% Run_Robot – plans a pick and sends it to the Gazebo robot over ROS 2
% Run setup.m from the repository root first (uses common/ and 3_motion_planning/).
% Requires setupRos.m and sendAndWait.m in this folder, and the Gazebo
% simulation running with ROS_DOMAIN_ID = 17.
%
% --- AI assistance (Claude, Anthropic) ---
% Claude was used as a tutor (explanations, troubleshooting, guiding questions).
% Written by Claude, then adapted and tested by the author: q2traj,
% sendAndWait.m, the synchronisation logic (tic/toc, tWait, pause), and the
% smooth transitions and differential-kinematics loop in planPick2.
% Final clean-up by Claude: removed the local copies of ConstructPose,
% goalPose and CalculatePickUp (they are in common/), removed the
% unnecessary ';' in the for-line, added this header and the note on the
% subscriber section. The clean-up did not change calculations or ROS logic.
% Robot model, parameter values and program structure: the author's own work.
% -----------------------------------------
clc
clear
[client, joints, node] = setupRos();    %Returns client, joints and node
[robot, T0C] = createRobot();           %Returns robot and Transformation from base to camera

%% Parameters
P.v = 0.3;
P.yStart = 0.3*0.15;                    %0.3m/s * 0.15 s - Start of pickup
P.pOverDropOff =  [0 -1 0.05];          %5 cm over conveyorbelt + 1 cm pga tykkelsen på sjokoladen
P.tMoveStart = 1.5;   % home -> over start [s]
P.tDown      = 0.3;   % ned til sjokoladen [s]
P.qHome = ConstructPose(0, pi/4, -pi/4, 0);
%% Kjør til qHome først
homeTraj = q2traj(P.qHome, 3, joints);
sendAndWait(client, homeTraj);

%% Sjokoladen observeres
for k = 1:1
    % Hent koordinater til sjokoladen fra gazebo spawner

    pC  = [0.1; -0.5; 1.05; 1];
    psi = pi/3;
    tic                                          % klokka starter ved observasjon

    p0 = T0C * pC;

    % Tid til sjokoladen er på yStart, minus bevegelsene før plukket
    tWait = (p0(2) - P.yStart)/P.v - P.tDown - P.tMoveStart;
    if tWait < 0
        error("Rekker ikke sjokoladen: mangler %.2f s", -tWait);
    end

    [qTotal, tTotal] = planPick2(robot, p0, psi, P);
    pickTraj = q2traj(qTotal, tTotal, joints);

    remaining = tWait - toc;                     % trekk fra planleggingstiden
    if remaining < 0
        error("Planleggingen tok for lang tid: %.2f s for sent", -remaining);
    end
    pause(remaining);

    sendAndWait(client, pickTraj);

    % if sendAndWait(client, pickTraj) ~= 4
    %     error("Plukkbanen feilet");
    % end
end

TAll = robot.fkine(qTotal);
p = TAll.transl;                         % N x 3 posisjoner
v = diff(p) ./ diff(tTotal);             % hastighet mellom punktene
speed = vecnorm(v, 2, 2);                % fart (lengden av hastighetsvektoren)
figure;
plot(tTotal(2:end), speed); yline(0.5, '--');
xlabel('Tid [s]'); ylabel('Hastighet [m/s]');
title('End-effectors hastighet gjennom bevegelsen');
max(speed)

%% Subscribe chocolate position
% Not used in the pick above (pC is set manually). Reads and prints the
% chocolate poses published by Gazebo, for future use.
% Abonner på sjokoladeposisjonene
sub = ros2subscriber(node, ...
    "/chocolate/poses", ...
    "chocolate_interfaces/ChocolatePoseArray");

% Les en melding
[msg, status] = receive(sub, 10);

if ~status
    error("Mottok ingen posisjonsdata fra Gazebo.");
end

% Skriv ut posisjonene
for i = 1:numel(msg.chocolates)

    chocolate = msg.chocolates(i);
    pos = chocolate.pose.position;

    fprintf("%s: X=%.3f, Y=%.3f, Z=%.3f\n", ...
        string(chocolate.name), ...
        pos.x, pos.y, pos.z);
end
%% Functions
function traj = q2traj(qTotal, tTotal, joints)
    % Lager en JointTrajectory-melding fra leddvinkler (N x 5) og tider (N x 1)
    traj = ros2message("trajectory_msgs/JointTrajectory");
    traj.joint_names = joints;

    N = size(qTotal, 1);

    % Hastigheter fra numerisk derivasjon, null i start og slutt
    qd = zeros(size(qTotal));
    for j = 1:size(qTotal, 2)
        qd(:, j) = gradient(qTotal(:, j), tTotal);
    end
    qd(1, :)   = 0;
    qd(end, :) = 0;

    pt = ros2message("trajectory_msgs/JointTrajectoryPoint");
    points = repmat(pt, 1, N);

    for i = 1:N
        points(i).positions     = qTotal(i, :);
        points(i).velocities    = qd(i, :);
        points(i).accelerations = [];
        points(i).effort        = [];

        t   = tTotal(i);
        sec = floor(t);
        nsec = round((t - sec) * 1e9);
        if nsec >= 1e9          % avrunding kan gi nøyaktig 1 s
            sec = sec + 1;
            nsec = 0;
        end
        points(i).time_from_start.sec     = int32(sec);
        points(i).time_from_start.nanosec = uint32(nsec);
    end

    traj.points = points;
end
