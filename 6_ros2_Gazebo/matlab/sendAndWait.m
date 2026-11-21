% AI assistance (Claude): rewrote the author's send-and-wait loop
% (based on the Lab 2 example) as a reusable function.
% Sender en JointTrajectory til action serveren og venter til den er ferdig.
% Returnerer status: 4 = fullført, 5 = avbrutt, 6 = avbrutt med feil.
 
function exStatus = sendAndWait(client, traj)

    goalMsg = ros2message(client);
    goalMsg.trajectory = traj;
    goalMsg.goal_time_tolerance.sec     = int32(0);
    goalMsg.goal_time_tolerance.nanosec = uint32(500000000);   % 0.5 s
 
    goalHandle = sendGoal(client, goalMsg);
 
    while true
        exStatus = getStatus(client, goalHandle);
        if exStatus == 4 || exStatus == 5 || exStatus == 6
            break;
        end
        pause(0.1);
    end
 
    fprintf("Status: %d\n", exStatus);
    if exStatus ~= 4
        resultMsg = getResult(client, goalHandle);
        disp(resultMsg);
    end
end
