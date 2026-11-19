# ELE208 dynamics handover

Prepared 19 November 2026 for `ELE208-Gruppe-5/4_dynamics/`.
Current model: **ELE208-hollow-modular-2026-11-04-v2**, dynamics variant
**ELE208-v2-shoulder-viscous-2026-11-09-v1**. The adopted parameters are unchanged.

## Dependencies

MATLAB and **Peter Corke's** Robotics Toolbox (`Link`/`SerialLink`), Spatial Math
(`SE3`) and toolbox-common (`tb_optparse`). Tested with MATLAB R2026b on Apple
silicon and these external source versions:

| Toolbox | Tested commit |
|---|---|
| [robotics-toolbox-matlab 10.4](https://github.com/petercorke/robotics-toolbox-matlab/tree/06fddf5228cbfa1132fa63b8e336b43c5ed04899) | `06fddf5228cbfa1132fa63b8e336b43c5ed04899` |
| [spatialmath-matlab 1.0](https://github.com/petercorke/spatialmath-matlab/tree/6eeff4a79f14286705560b84f1fe72e0b7e0e7f7) | `6eeff4a79f14286705560b84f1fe72e0b7e0e7f7` |
| [toolbox-common-matlab](https://github.com/petercorke/toolbox-common-matlab/tree/6678f1a23e9b4f5abe98c23768de2b0e278a9362) | `6678f1a23e9b4f5abe98c23768de2b0e278a9362` |

Toolboxes are **not bundled**. Keep them outside the group repository. The
commands below need no Control System Toolbox, Simulink, ROS, MathWorks Robotics
System Toolbox or compiled RNE MEX. The model sets `fast=false`.

The unchanged supplied **`createRobot.m` is included in `support/`**. Setup checks
that this copy is used. It defines the five-joint DH geometry, limits and camera
transform. No other group repository file is required to run this package.
Repository inspection now succeeded through a local Git clone of `main` at
commit `1b560a2a07a772bbfd37b5e7ec72cf87454ef1e8`. The group constructor is
`../common/createRobot.m`; it is byte-identical to the bundled `support/` copy.
The actual split-program names are recorded in
[repository integration](REPOSITORY-INTEGRATION.md). No `AlleOppgaver` program
or guessed replacement is required. The RNE entry point uses saved trajectory
inputs; it does not need the currently absent `planPick2.m`.

## Run

Set MATLAB's **Current Folder to `4_dynamics`**. If the three toolboxes are
already on the path:

```matlab
addpath(pwd,'-begin');
setup_shoulder_friction();
```

Otherwise, for this example arrangement, use the relative setup command:

```text
work/
  ELE208-Gruppe-5/4_dynamics/
  matlab-toolboxes/
    robotics-toolbox-matlab/
    spatialmath-matlab/
    toolbox-common-matlab/
```

```matlab
addpath(pwd,'-begin');
setup_shoulder_friction('../../matlab-toolboxes');
```

Choose an entry point after setup:

```matlab
maximum = example_current_model('maximum'); % full robot + shoulder coefficients
empty   = example_current_model('empty');
rne     = run_4v_friction();                 % saved 0.01 / 0.005 s motion grids
demands = actuator_demands_4v();             % postprocess RNE, no new dynamics
shoulder = validate_shoulder_friction();     % existing report comparison
```

For the complete packaging check, run **`v = verify_handover();` once instead of
the entry points above**. It runs those examples/calculations and compares them
with the current saved evidence in `data/expected-results.json`.

Generated files go into `results/`, which `.gitignore` excludes from commits:

- `rne/`: `results.mat/json`, five-joint `torques.png`, trajectory/summary CSVs,
  **110-row `torque-event-limits.csv`**, and motor/reducer demand tables.
- `shoulder/`: `numerical-results.mat/json`, response/gravity/convergence CSVs,
  and four figures. **`02-constant-holding.png`** reproduces the corresponding
  report comparison. `validate_shoulder_friction('figures')` redraws saved plots.
- `verification.json`: result of `verify_handover()`.

## Expected results and assumptions

| Quantity | Empty / nominal | Maximum |
|---|---:|---:|
| Shoulder J at controller pose, kg m² (empty / maximum) | 29.9765770558 | 39.3671005891 |
| RNE shoulder peak at 0.01 s, N m (0.10 kg / 5 kg object) | 251.850445 | 321.155131 |
| RNE shoulder RMS over 0–6 s, N m (0.10 kg / 5 kg object) | 175.166357 | 196.027643 |
| RNE shoulder peak at 0.005 s, N m (0.10 kg / 5 kg object) | 252.164355 | 321.516575 |
| Constant-holding nonlinear displacement at 2.5 s, deg (empty / maximum) | 18.017430 | 13.717618 |
| Constant-holding max angle error at 2.5 s, deg (empty / maximum) | 0.872382 | 0.546404 |

All cases retain the same **2 kg gripper**. RNE carries a **0.10 kg or 5 kg object
only from 1.8 to 4.2 s** on the saved six-second path. Both event limits are
retained; trajectory CSVs select the right-hand value at an internal event.
`data/trajectory-input.mat` contains only the exact saved motion on the
0.02/0.01/0.005 s grids, including all twelve pieces and their endpoints.
No planner, IK regeneration or `AlleOppgaver` execution is required.

Shoulder comparison: `q0=[0,pi/4,0,0,0]`, joints 1/3/4/5 held, +0.5 N m step.
Current-angle gravity compensation and constant equilibrium holding torque are
different experiments. The latter uses a local, unstable gravity approximation.
Trajectory home remains `[0,pi/4,-pi/4,0,0]`.

Shoulder motor `B=817e-6` reflects once through 121² to joint
**B=11.961697 N m s/rad**. `Jm=3.45e-4` reflects once to
**5.051145 kg m²**, already included. Do not add either reflection, a separate
constant disturbance, `payload()` mass or a payload gravity wrench again.
The parameter `.m` and coefficient MAT/JSON/CSV files belong together.

These are prescribed-motion demands and mathematical comparisons. Construction
and friction are estimates; other drive spin, other friction, compliance,
transmission losses and contact transients remain omitted. These results do
not establish controller tracking, Gazebo operation or hardware capability.

See [verification](verification/VERIFICATION.md) for what actually ran and
[provenance](PROVENANCE.md) for the packaging changes and source locations.
