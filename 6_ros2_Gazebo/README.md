<!--
AI-generated (ChatGPT):
This README was generated with AI assistance based on project
information, technical documentation and instructions provided
by the author. The content was reviewed and adapted before submission.

AI assistance (Claude, Anthropic):
The section "Running the MATLAB pick script (Run_Robot)" was written by
Claude based on the author's MATLAB code and instructions, and reviewed
by the author.
-->

# Chocolate Factory – Gazebo Simulation

Simulation code for the ELE208 chocolate pick-and-place project.

**Environment:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. The ROS/Gazebo control packages and build dependencies must be installed separately.

## Contents

- `chocolate_robot_description`: robot URDF and visual/collision meshes
- `chocolate_factory_sim`: Gazebo world, conveyors, chocolate SDF, launch and spawn script
- `chocolate_conveyor`: Gazebo C++ conveyor movement plugin
- `chocolate_interfaces`: custom ROS 2 chocolate pose messages
- `chocolate_tracking`: reads Gazebo model poses and publishes `/chocolate/poses`
- `MATLAB test kode.m`: example MATLAB trajectory action client

## Build

Place **all five ROS 2 package folders directly inside** `~/ros2_ws/src/` (or keep this folder directly inside `src`; `colcon` discovers its nested packages).

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Required ROS/Gazebo functionality includes `ros_gz_sim`, `gz_ros2_control`, `ros2_control`, `ros2_controllers` (joint trajectory and joint state controllers), `robot_state_publisher`, plus Gazebo Harmonic development libraries used by the C++ packages. Install missing dependencies using the corresponding Jazzy / Harmonic packages.

## Run

**Terminal 1 – Gazebo**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ros2_ws/install/chocolate_conveyor/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
ros2 launch chocolate_factory_sim chocolate_factory.launch.py
```

**Terminal 2 – spawn chocolate plates**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run chocolate_factory_sim spawn_chocolate
```

**Terminal 3 – publish chocolate poses (optional)**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run chocolate_tracking chocolate_tracking_node
```

The ROS 2 topic `/chocolate/poses` uses `chocolate_interfaces/msg/ChocolatePoseArray`.

## Robot control

- Action: `/arm_controller/follow_joint_trajectory`
- ROS 2 action type: `control_msgs/action/FollowJointTrajectory`
- Joint order: `joint_1`, `joint_2`, `joint_3`, `joint_4`, `joint_5`
- Controllers are spawned by the main launch file.

Check with:

```bash
ros2 control list_controllers
ros2 action list -t
```

Use `MATLAB test kode.m` as an example for MATLAB ROS 2 Action Client communication. Ensure ROS 2 discovery works between machines and set the same `ROS_DOMAIN_ID` (the example uses `15`).

## Running the MATLAB pick script (Run_Robot)

`Run_Robot.m` plans a complete pick-and-place trajectory in MATLAB (Robotics Toolbox) and sends it to the Gazebo robot as one `FollowJointTrajectory` goal.

### Requirements

- MATLAB R2025a–R2026a with **ROS Toolbox**
- **Robotics Toolbox for MATLAB** (Peter Corke), on the MATLAB path
- Network contact between MATLAB and the ROS 2 machine (for a VMware VM: use **Bridged** network, not NAT)
- The same `ROS_DOMAIN_ID` on both sides. `Run_Robot` uses **17** (set in `setupRos.m`)

### MATLAB files

| File | Purpose |
|---|---|
| `Run_Robot.m` | Main script: setup, parameters, move to home, plan and send the pick |
| `setupRos.m` | Creates the ROS 2 node and the action client for `/arm_controller/follow_joint_trajectory` |
| `sendAndWait.m` | Sends a trajectory goal and waits until it has finished |
| `createRobot.m` | Robot model (DH parameters, joint limits) and camera transform `T0C` |
| `planPick2.m` | Plans the full pick: home → over chocolate → down → follow belt → up → drop-off → home |
| `common/` | `ConstructPose`, `goalPose`, `CalculatePickUp` |

### Steps

1. **Start the simulation** (Terminal 1 above), with the domain ID set first:

   ```bash
   export ROS_DOMAIN_ID=17
   ros2 launch chocolate_factory_sim chocolate_factory.launch.py
   ```

   Check that the action server is available:

   ```bash
   ros2 action list -t
   ```

2. **In MATLAB**, open the repository root and run `setup.m` to add `common/` and `3_motion_planning/` to the path.

3. **Run `Run_Robot.m`.** The script will:
   - connect to ROS 2 and wait for the action server
   - move the robot to the home pose `qHome`
   - compute the chocolate position from the camera point `pC` (`p0 = T0C * pC`)
   - plan the trajectory with `planPick2`
   - wait until the chocolate reaches the pick-up point, then send the trajectory
   - plot the end-effector speed (requirement: at least 0.5 m/s)

### Parameters

Set in the `Parameters` section of `Run_Robot.m`:

| Parameter | Meaning |
|---|---|
| `P.v` | Conveyor speed [m/s] |
| `P.yStart` | y-position where the pick starts |
| `P.pOverDropOff` | Position above the drop-off point |
| `P.tMoveStart` | Duration home → over chocolate [s] |
| `P.tDown` | Duration of the move down to / up from the belt [s] |
| `P.qHome` | Home joint configuration |

### Limitations

- The chocolate pose (`pC`, `psi`) is set **manually** in the script. It is not read from `/chocolate/poses`, so the robot is not synchronised with chocolates spawned by `spawn_chocolate`.
- The subscriber section at the end of `Run_Robot.m` reads `/chocolate/poses`. It requires the custom message `chocolate_interfaces/ChocolatePoseArray` to be generated in MATLAB with `ros2genmsg`; otherwise this section fails. It also requires `chocolate_tracking_node` to be running.
- The suction gripper is not activated; the trajectory is executed without picking up an object.
- Timing uses `pause` in MATLAB and wall-clock time, so small offsets relative to Gazebo simulation time can occur.

## Scope and limitations

- Conveyor **target speed**: 0.3 m/s, implemented with a C++ P-based force regulator (`AddWorldForce`), **not** `SetLinearVelocity`. The actual achieved speed has not been quantitatively validated.
- Chocolate spawn positions and orientations are randomized.
- `chocolate_tracking` publishes simulated ground-truth poses directly from Gazebo, **not camera detections**.
- The Gazebo world currently disables gravity (`0 0 0`) to stabilize the simulated robot.
- Automated suction grasping / complete pick-and-place is not implemented in this delivery.
- The source was reviewed and cleaned statically; it was **not rebuilt or executed** in this environment.
