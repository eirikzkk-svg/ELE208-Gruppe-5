% Based on the course Lab 2 example. Written by the author with
% guidance from Claude (hints on domain ID, action name/type and outputs).

function [client,joints,matlab_ros_node] = setupRos()

ROS_DOMAIN_ID = 17;
setenv("ROS_DOMAIN_ID",string(ROS_DOMAIN_ID))
% Set the ros domain id and create a new ros node

matlab_ros_node = ros2node("/matlab_node", ROS_DOMAIN_ID);

% Check the ROS 2 interfaces

disp("ROS2 topics:");
ros2("topic", "list");

disp("ROS2 actions:")
ros2("action", "list");

% Create the action server client

controllerName = "arm_controller";

joints = { ...
    'joint_1', ...
    'joint_2', ...
    'joint_3', ...
    'joint_4', ...
    'joint_5'};

% Define the action type and name
actionType = "control_msgs/FollowJointTrajectory";
actionName = "/"+ controllerName + "/follow_joint_trajectory";


% Create the client
client = ros2actionclient(matlab_ros_node, actionName, actionType);

disp("Waiting for FollowJointTrajectory action server ...");
waitForServer(client);
disp("Action server available.");

end