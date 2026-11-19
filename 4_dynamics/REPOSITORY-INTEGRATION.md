# Checked repository integration

19 November 2026. Repository: `eirikzkk-svg/ELE208-Gruppe-5`.
Inspected branch: `main`, commit `1b560a2a07a772bbfd37b5e7ec72cf87454ef1e8`.

The local clone was clean before copying. The repository's `4_dynamics/`
contained only `.gitkeep`, which was preserved. All prepared changes are
uncommitted additions under `4_dynamics/`. No existing group script, Simulink
file, root setup/README, report or remote branch was changed.

## Constructor and paths

The group constructor is **`common/createRobot.m`**, or
`../common/createRobot.m` when working from `4_dynamics/`. It is byte-identical
to the tested `4_dynamics/support/createRobot.m`:

`870a31b670d87bb890e946daa0e7c5947749dc13dd7a17a3fa61d12fb0452453`

The handover setup deliberately selects its bundled copy and checks the
resolved path. Since the two copies match, no constructor change or geometry
adaptation was necessary. The tested dynamics, parameters and saved inputs
were copied unchanged. The existing fresh-folder MATLAB evidence remains
applicable; no additional dynamics run was needed for this copy operation.

The group's root `setup.m` adds project folders. It does not install the external
Peter Corke toolboxes. For the dynamics handover, follow this folder's README
and run `setup_shoulder_friction` after configuring those external toolboxes.
The dynamics programs do not require Simulink or Control System Toolbox.

## Verified split-program names

These filenames were verified in the checked repository and its README. These
group programs were inspected as files, not executed for this integration.

| File | Group role |
|---|---|
| `1_kinematics/req_1i_dh_forward_kinematics.m` | DH and forward kinematics |
| `1_kinematics/req_2i1_toolbox_equivalence.m` | Toolbox equivalence |
| `1_kinematics/req_2i2_jacobian_static_torques.m` | Jacobian and static wrench example |
| `1_kinematics/req_2i3_inverse_kinematics.m` | Inverse kinematics |
| `2_sensor/req_1ii_camera_transform.m` | Camera transformation |
| `2_sensor/req_2ii_sensor_pick.m` | Sensor-driven pick example |

The static wrench example applies **5 N**, not the weight of a 5 kg object.
It is not the full-trajectory RNE calculation. Use `run_4v_friction` here for
the current nominal/maximum object demands.

The sensor-pick script calls `planPick2`, but this inspected repository has
only `.gitkeep` in `3_motion_planning/` and contains no `planPick2.m`.
That group dependency remains absent. This handover uses its exact saved
trajectory data, so the missing planner does not block its RNE calculation.
No replacement was guessed or added.

## Personal upload step

The clone and file copying are already done. In GitHub Desktop choose
**File → Add Local Repository**, select this clone's root folder, review
the `4_dynamics/` additions, commit, then click **Push origin**. Hareth keeps
control of the commit and push.
