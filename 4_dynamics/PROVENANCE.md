# Source and packaging record

The authoritative selection is the existing project file
`output/current-model-handover-2026-11-09-v1/FILE-INDEX.md`.
The source paths below identify that retained local project, not dependencies
that must exist beside this folder after copying it into the group repository.

| Packaged files | Retained source |
|---|---|
| `ele208_source_data.m`, `ele208_dynamics_model.m`, `ele208_joint2_coefficients.m`, `joint2-coefficients.mat`, `example_current_model.m` | `output/current-model-handover-2026-11-09-v1/ELE208_current_model/` — unchanged bytes |
| `setup_shoulder_friction.m` | Same current handover — add/check bundled `support/createRobot.m` |
| `joint2-coefficients.json/csv` | `output/shoulder-friction-2026-11-09-v1/` — unchanged bytes |
| `support/createRobot.m` | `references/downloads/MATLAB Eirik/createRobot.m` — unchanged bytes; matches the checked Downloads copy |
| `run_4v_friction.m`, `plot_4v_friction.m`, `actuator_demands_4v.m` | `output/4v-friction-2026-11-09-v1/` — portable input/output/setup wiring |
| `validate_shoulder_friction.m` | `output/shoulder-validation-2026-11-09-v1/` — portable setup/input/output paths; no diary log |
| `data/trajectory-input.mat` | `traces` only from current `output/4v-friction-2026-11-09-v1/results.mat` |
| `data/expected-results.json` | Selected current numerical evidence from that RNE folder and the shoulder validation folder |
| `external-toolboxes.json` | Pinned external dependency manifest; toolbox source is not copied |

The RNE script retains the original piecewise full-robot RNE calculation,
load selection, peak/RMS calculation and one-sided event handling. Its
`inverseDynamics`, threshold-duration, summary, label and event helpers are
copied verbatim. The default selects the existing 0.01/0.005 s grids.
Historical zero-friction model/torque comparison inputs and their plots are
excluded. The current five-joint torque plot remains.

The shoulder comparison retains the original experiment, coefficients,
RNE-based nonlinear ODE, saved-state-matrix linear prediction, solver settings,
four response runs, built-in tighter-solver check and figures. No new model,
controller or trajectory has been developed. `verify_handover.m` checks this
package against selected current saved results rather than repeating the full
historical verification suite.

Repository inspection initially failed through the connected GitHub service.
A later read-only Git operation through the computer's existing Git sign-in
succeeded. The repository was cloned locally on 19 November 2026, branch `main`,
commit `1b560a2a07a772bbfd37b5e7ec72cf87454ef1e8`. Its `common/createRobot.m` is
byte-identical to the bundled snapshot. The verified split names and missing
planner observation are in [repository integration](REPOSITORY-INTEGRATION.md).
Only `4_dynamics/` was populated; existing repository code and `.gitkeep` files
were preserved. No commit or push was made.
