# ELE208 dynamics

This folder contains the robot dynamics model, mass and inertia parameters, saved motion inputs, RNE torque calculation and shoulder comparison. The robot geometry comes from `support/createRobot.m`.

Use MATLAB with Peter Corke's Robotics Toolbox, Spatial Math and toolbox-common. Install the toolboxes separately; [external-toolboxes.json](external-toolboxes.json) lists the tested versions. These programs do not need Simulink or Control System Toolbox.

## Run

Set MATLAB's Current Folder to `4_dynamics`. With the toolboxes already on the path, run:

```matlab
addpath(pwd,'-begin');
setup_shoulder_friction();
```

Otherwise, place the three toolbox folders inside `matlab-toolboxes` beside `ELE208-Gruppe-5` and use:

```matlab
addpath(pwd,'-begin');
setup_shoulder_friction('../../matlab-toolboxes');
```

After setup:

```matlab
maximum = example_current_model('maximum');
empty   = example_current_model('empty');
rne     = run_4v_friction();
demands = actuator_demands_4v();
shoulder = validate_shoulder_friction();
```

These commands create example models, calculate torques for the five joints, separate motor and reducer demands, and compare linear and nonlinear shoulder responses.

For a complete check against saved evidence, run `v = verify_handover();` instead of the commands above.

Generated MAT, JSON, CSV and PNG files go in `results/rne/` and `results/shoulder/`. The check writes `results/verification.json`. Git ignores `results/`. Use `validate_shoulder_friction('figures')` to redraw saved shoulder plots.

## Loads and assumptions

All cases include the same 2 kg gripper. `empty` has no object. RNE uses a 0.10 kg nominal object or a 5 kg maximum object, carried from 1.8 to 4.2 s on the saved six-second path.

Shoulder viscous friction is assumed. Motor-side `B=817e-6` gives joint-side `B=11.961697 N m s/rad`. Shoulder rotor inertia is already included. Do not add another payload or rotor reflection.

The calculated maximum-load shoulder peak is approximately 322 N·m on the 0.005 s grid. This is the torque required for the prescribed motion.

The shoulder comparison uses `q0=[0,pi/4,0,0,0]` with joints 1, 3, 4 and 5 held fixed. It compares gravity compensation at the current angle with constant equilibrium holding torque.

## Supporting material

- [Dynamics parameters, assumptions and derivation](../docs/dynamics-supporting-material.md)
- [Camera selection and optical calculations](../docs/camera-selection-and-optical-calculations.md)
