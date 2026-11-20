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

## Scope and limitations

- Conveyor **target speed**: 0.3 m/s, implemented with a C++ P-based force regulator (`AddWorldForce`), **not** `SetLinearVelocity`. The actual achieved speed has not been quantitatively validated.
- Chocolate spawn positions and orientations are randomized.
- `chocolate_tracking` publishes simulated ground-truth poses directly from Gazebo, **not camera detections**.
- The Gazebo world currently disables gravity (`0 0 0`) to stabilize the simulated robot.
- Automated suction grasping / complete pick-and-place is not implemented in this delivery.
- The source was reviewed and cleaned statically; it was **not rebuilt or executed** in this environment.
