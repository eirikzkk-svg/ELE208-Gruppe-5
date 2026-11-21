<!--
AI assistance (Claude, Anthropic): added the reference to the ROS 2 / Gazebo
README in "Getting started" and in the table row for requirement 5.
-->

# ELE208 Robot Manipulators – Design Challenge 5

5-DOF articulated robot arm that picks chocolate bars from a conveyor belt moving at 0.3 m/s
and places them at a drop-off position. Group project in ELE208 Robotmanipulatorer,
Western Norway University of Applied Sciences (HVL), autumn 2026.

## Getting started
1. Install MATLAB with Simulink and Control System Toolbox.
2. Install Peter Corke's Robotics Toolbox for MATLAB (tested with RTB 10.4),
   Spatial Math Toolbox and toolbox-common.
3. Clone this repository and run `setup.m` from the repository root.
4. Run the script for the requirement you want to check (see the table below).
5. For requirement 5 (ROS 2 / Gazebo simulation and control from MATLAB), see
   `6_ros2_gazebo/README.md`. This also requires Ubuntu 24.04, ROS 2 Jazzy,
   Gazebo Harmonic and MATLAB ROS Toolbox.

## Repository structure
| Folder | Contents |
|---|---|
| `common/` | Shared functions used by all parts, e.g. `createRobot.m` |
| `1_kinematics/` | DH parameters, forward and inverse kinematics, differential kinematics |
| `2_sensor/` | Camera frame and transformation of object positions to the robot base |
| `3_motion_planning/` | Trajectory generation and conveyor tracking |
| `4_dynamics/` | Dynamic model (masses, centres of mass, inertias) and recursive Newton–Euler |
| `5_control/` | Joint dynamic model, velocity and position controllers (Simulink) |
| `6_ros2_gazebo/` | URDF, Gazebo world and MATLAB–ROS 2 control |
| `docs/` | Supporting documents and derivations |

## Requirements → files
Run `setup.m` first. Each script can then be run on its own.

| Requirement | Description | File |
|---|---|---|
| 1.i | DH table and forward kinematics (symbolic) | `1_kinematics/req_1i_dh_forward_kinematics.m` |
| 1.ii | Camera-to-base transformation | `2_sensor/req_1ii_camera_transform.m` |
| 2.i.1 | Equivalence with Robotics Toolbox | `1_kinematics/req_2i1_toolbox_equivalence.m` |
| 2.i.2 | Differential kinematics (Jacobian) | `1_kinematics/req_2i2_jacobian_static_torques.m` |
| 2.i.3 | Inverse kinematics | `1_kinematics/req_2i3_inverse_kinematics.m` |
| 2.i.4, 3 | Motion planning and simulation (pick trajectory) | `3_motion_planning/planPick2.m`, run from `2_sensor/req_2ii_sensor_pick.m` |
| 2.ii | Camera measurement used to command the robot | `2_sensor/req_2ii_sensor_pick.m` |
| 4.i | Dynamic model with realistic parameters | `4_dynamics/` |
| 4.ii | Joint dynamic model and transfer function | `5_control/` |
| 4.iii | Velocity controller (shoulder joint) | `5_control/` |
| 4.iv | Position controller (shoulder joint) | `5_control/` |
| 4.v | Recursive Newton–Euler inverse dynamics | `4_dynamics/` |
| 5 | URDF, Gazebo environment, control from MATLAB over ROS 2 | `6_ros2_gazebo/` (instructions in `6_ros2_gazebo/README.md`) |

Shared functions in `common/`: `createRobot.m`, `ConstructPose.m`, `goalPose.m`, `CalculatePickUp.m`.

## Videos
- Gazebo simulation: *(add link)*
- MATLAB animation of the pick cycle: *(add link)*

## Group
Eirik, Hareth, Sondre, Stian and Susanne.
