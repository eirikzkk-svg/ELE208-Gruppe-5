# Fresh-folder MATLAB verification

**PASS — 19 November 2026**, MATLAB **26.2.0.3386108 (R2026b), MACA64**.
The final continuation exited with code **0** and produced both explicit PASS
records. [Machine-readable results](run-summary.json) include the actual values.

The package was copied into a separate fresh folder. MATLAB began with
`restoredefaultpath`; calculations resolved the fresh folder's model,
coefficient loader/MAT, example, RNE/shoulder code and bundled constructor.
Only the three pinned Peter Corke toolboxes came from outside that folder.
No globally saved path/startup settings were changed.

## What actually ran

- Empty/maximum model examples and comparison of shoulder inertia, gravity,
  gravity slope, damping, rotor reflection and moving mass with saved evidence.
- Full five-joint RNE on the exact **0.01 and 0.005 s** saved motion, including
  all twelve pieces and both event limits. Nominal 0.10 kg and maximum 5 kg
  objects use the original load phases and same 2 kg gripper. Peak/RMS/time/speed
  summaries and all **110 primary-grid event rows** matched the saved evidence.
- Existing motor-equivalent/reducer-output postprocessing, with no new dynamics.
- Existing shoulder comparison: empty/maximum × current-angle compensation/
  constant holding, with the original +0.5 N m step, 8/2.5 s durations, solver
  settings, reaction-torque checks, gravity curves and built-in single tighter
  empty/holding solver check. Four figures and response tables were generated.
- After calculation, the original current evidence MAT was read for comparison
  only. **All three packaged trajectory structures were exactly equal** to the
  originals. Every RNE torque sample on both executed grids matched, including
  one-sided endpoints: **maximum difference 0 N m**.
- Key RNE summary/event, shoulder coefficient, endpoint and error-metric
  discrepancies were all **0** at saved precision. Current five-joint torque
  and constant-holding comparison PNGs were visually inspected.

| Reproduced quantity | Empty / nominal | Maximum |
|---|---:|---:|
| Shoulder J at controller pose, kg m² | 29.9765770558 | 39.3671005891 |
| Shoulder RNE peak, dt=0.01 s, N m | 251.850445203 | 321.155130747 |
| Shoulder RNE RMS, dt=0.01 s, N m | 175.166356906 | 196.027642754 |
| Shoulder RNE peak, dt=0.005 s, N m | 252.164354820 | 321.516575494 |
| Holding nonlinear displacement at 2.5 s, deg | 18.017430 | 13.717618 |
| Holding max angle discrepancy over 0–2.5 s, deg | 0.872382 | 0.546404 |

The RNE columns use 0.10 kg / 5 kg objects. Shoulder comparison columns use
empty / 5 kg object. All retain the same 2 kg gripper.

## Startup, interruption and limits

The first restricted launch failed before computation. A display-enabled
startup stalled and was stopped. The headless local launch completed RNE and
its comparisons, then encountered **“No space left on device”** while saving
the first shoulder CSV. Free space became available again. The check resumed
in the **same fresh folder**, reused its completed RNE output, and finished all
shoulder calculations/comparisons and original-MAT checks. A single uninterrupted
`verify_handover()` invocation therefore did not finish; its unfinished stages
and assertions completed through the continuation. No completed RNE run was
repeated for the continuation. Raw logs remain task-local, outside the upload.

Warnings about the user's `Documents/MATLAB` directory did not affect the
recorded package/toolbox resolution. Other MATLAB/toolbox versions remain
untested. The package contains no toolboxes, historical model, unrelated
coursework or raw logs. Core source/model/loader/example/MAT and bundled
constructor bytes match the current retained originals; RNE helper calculations
and shoulder numerical/plotting code are unchanged apart from portable wiring.

No full historical verification suite, trajectory/IK regeneration, new model,
controller, ROS/Gazebo or hardware test was run. No report file, group repository,
synced source, retained model or earlier dynamics evidence was edited by this
task. Repository inspection subsequently succeeded through a local Git clone. The
current `common/createRobot.m` matches the bundled snapshot byte-for-byte and
the actual split-program filenames are now recorded. See
[repository integration](../REPOSITORY-INTEGRATION.md). The copied numerical
code/input files are unchanged, so this preparation did not repeat the MATLAB
calculations. There is no repository-only runtime dependency: the tested
constructor snapshot and inputs are bundled.
