# Dynamics supporting material

Parameters, assumptions and derivation supporting the report's Dynamics and Dynamics and Payload sections. The E labels retain the original appendix equation and table identifiers.

## E.1 Link parameters and frames

We estimated the link properties from hollow 6082 aluminium at 2700 kg/m³ [6082 aluminium] and manufacturer drive masses [Panasonic motor], [Nabtesco reducer], [Harmonic Drive manual]. Each housing and output part contributes mass once. We assumed the dimensions, placements and internal mass splits, then calculated the COM tensors in the DH frames. These tensors exclude relative rotor spin.

$D_0$ is the fixed base frame. $D_i$ is the right-handed standard-DH body frame after joint $i$. Positive $q_i$ rotates about $z_{i-1}$. Base and tool transforms are identities. Each COM is measured from its $D_i$ origin and expressed along those axes. In joint order 1–5,

$$
A_i = R_z(q_i)\,T_z(d_i)\,T_x(a_i)\,R_x(\alpha_i)
\tag{E.1}
$$

$$
d = [0.30,\;0.20,\;-0.20,\;0.20,\;0.20]\;\mathrm{m},
\qquad a = [0,\;0.6726,\;0.6726,\;0,\;0]\;\mathrm{m}
$$

$$
\alpha = [\pi/2,\;0,\;0,\;\pi/2,\;0]\;\mathrm{rad}
$$

**Table E.1. Moving-body masses and COM coordinates in $D_i$.** Bodies 1–4 are unchanged between load cases. Every body 5 row includes the same 2 kg gripper.

| Body and frame | m (kg) | COM x (m) | COM y (m) | COM z (m) |
|---|---:|---:|---:|---:|
| 1 D1 | 17.562154 | 0.000000 | -0.040271 | -0.071702 |
| 2 D2 | 11.032195 | -0.229393 | 0.001020 | -0.068913 |
| 3 D3 | 6.832605 | -0.230535 | 0.001024 | 0.014144 |
| 4 D4 | 3.902489 | 0.000000 | -0.061676 | -0.018383 |
| 5 D5 empty | 3.085848 | 0.000000 | 0.000000 | -0.081791 |
| 5 D5 nominal | 3.185848 | 0.000000 | 0.000000 | -0.079067 |
| 5 D5 maximum | 8.085848 | 0.000000 | 0.000000 | -0.000296 |

**Table E.2. Symmetric COM inertia tensors, expressed in $D_i$.** All entries are in kg·m². Ixy, Ixz and Iyz are signed tensor entries. The lower triangle equals the upper triangle.

| Body and load | Ixx | Iyy | Izz | Ixy | Ixz | Iyz |
|---|---:|---:|---:|---:|---:|---:|
| 1 D1 | 0.289410 | 0.154149 | 0.196531 | 0.000000 | 0.000000 | 0.051372 |
| 2 D2 | 0.073111 | 0.998056 | 0.961980 | 0.001203 | -0.109191 | -0.000775 |
| 3 D3 | 0.072625 | 0.646196 | 0.590790 | 0.000740 | 0.160747 | 0.000099 |
| 4 D4 | 0.033293 | 0.007974 | 0.033051 | 0.000000 | 0.000000 | 0.004785 |
| 5 D5 empty | 0.014962 | 0.017962 | 0.008926 | 0.000000 | 0.000000 | 0.000000 |
| 5 D5 nominal | 0.015746 | 0.018906 | 0.009193 | 0.000000 | 0.000000 | 0.000000 |
| 5 D5 maximum | 0.061647 | 0.081314 | 0.044343 | 0.000000 | 0.000000 | 0.000000 |

Empty, nominal and maximum moving masses are 42.415290, 42.515290 and 47.415290 kg. The fixed 14.422254 kg base is excluded from moving links. Gripper and object shapes are defined in E.2. Values above are rounded for display.

## E.2 Loads friction and rotor inertia

$D_5$ is the nominal contact endpoint. Its local −z direction points back into the gripper and +z extends into the carried object. These mass placements do not add a tool transform. Uniform boxes define the load COM inertias. The old CAD cup is excluded. The object is added once to body 5, with no additional payload wrench.

**Table E.3. Adopted external-load shapes.** Dimensions and COM coordinates are in m and expressed along $D_5$. Empty has no object. The trajectory carries each object only from 1.8 to 4.2 s.

| Component | Mass (kg) | COM [x, y, z] | Dimensions [x, y, z] |
|---|---:|---|---|
| Gripper in all cases | 2 | [0, 0, −0.060] | [0.18, 0.12, 0.12] |
| Nominal object | 0.10 | [0, 0, 0.005] | [0.16, 0.08, 0.01] |
| Maximum object | 5 | [0, 0, 0.050] | [0.25, 0.15, 0.10] |

**Table E.4. Adopted shoulder and gravity parameters.** Efficiency belongs only to preliminary motoring sizing in the report's equation (2).

| Parameter | Value | Use and basis |
|---|---|---|
| N | 121 | Adopted RV-40E ratio magnitude [Nabtesco reducer] |
| Rotor inertia (kg·m²) | 3.45 × 10⁻⁴ | Motor rotor specification [Panasonic motor] |
| Reflected inertia (kg·m²) | 5.051145 | Already in M₂₂ and J, report equation (3) |
| Motor viscous B (N·m·s/rad) | 817 × 10⁻⁶ | Borrowed PUMA shoulder assumption [Robotics Toolbox] |
| B₂ (N·m·s/rad) | 11.961697 | Motor coefficient multiplied by N² |
| η | 0.75 | Assumed sizing efficiency, not a measured loss model |
| Physical gravity (m/s²) | [0, 0, −9.81] | Fixed base axes; RTB uses [0, 0, +9.81] for compensation |

The toolbox stores motor-referred rotor inertia and viscous friction and performs the $N^2$ reflection internally [Robotics Toolbox]. Required applied friction torque is the joint coefficient multiplied by joint velocity. Physical friction opposes velocity. Entering joint-referred values into motor-side properties would reflect them twice.

The report's Joint Control section (3.6) uses a separate 15.3 N·m constant disturbance for controller testing. That disturbance is not part of these inverse-dynamics or shoulder-validation runs. Viscous friction at joints 1, 3, 4 and 5, all Coulomb friction and other drive spin inertia are omitted. Compliance, backlash and contact transients are also omitted.

## E.3 Shoulder reduction and gravity linearisation

For the comparison in the report's Dynamics and Payload section (4.3), let $e_2$ select joint 2 and $x$ be displacement from the stationary test pose. Reaction torques enforce zero motion at the other joints.

$$
q = q_0 + e_2 x,\qquad q_0 = [0,\;\pi/4,\;0,\;0,\;0]^T
\tag{E.2}
$$

The held downstream assembly rotates rigidly about the shoulder. Its inertia $J=M_{22}$ is independent of $q_2$. The shoulder quadratic velocity term therefore vanishes,

$$
\left(C(q,\dot q)\dot q\right)_2
= \frac{1}{2}\frac{\partial M_{22}}{\partial q_2}\dot x^2 = 0
\tag{E.3}
$$

Taking row 2 of the coupled robot equation, equation (1) in the report, gives

$$
\tau_2 = J\ddot x + B_2\dot x + g_2(q_0+e_2 x)
\tag{E.4}
$$

**Table E.5. Held-shoulder coefficients at $q_0$.** J already includes the single 5.051145 kg·m² rotor reflection. Both loads retain the 2 kg gripper. B₂ = 11.961697 N·m·s/rad in both cases.

| Object (kg) | J (kg·m²) | τ₀ = g₂(q₀) (N·m) | Kg (N·m/rad) |
|---:|---:|---:|---:|
| 0 | 29.976577 | 153.864047 | -150.051802 |
| 5 | 39.367101 | 209.191306 | -188.037267 |

Here $\tau_0$ is equilibrium holding torque and $K_g$ is the gravity slope at $q_0$. Rigid rotation gives the gravity function and its local tangent, with $x$ in rad,

$$
g_2(q_0+e_2 x) = \tau_0\cos x + K_g\sin x
\approx \tau_0 + K_g x
\tag{E.5}
$$

$$
R(x) = g_2(q_0+e_2 x)-\tau_0-K_g x
\approx -\frac{\tau_0 x^2}{2}
\tag{E.6}
$$

Current-angle compensation adds the current gravity holding torque to the control input, yielding the plant retained in the report's Joint Control section (3.6). A constant equilibrium bias instead leaves the local gravity term,

$$
J\ddot x + B_2\dot x + K_g x = \delta\tau_2,
\qquad \tau_2 = \tau_0 + \delta\tau_2
\tag{E.7}
$$

Both $K_g$ values are negative, producing a positive open-loop pole. The nonlinear remainder adds acceleration as the angle grows. At $x=+15^\circ$, R is −4.796 N·m empty and −6.568 N·m with the 5 kg object. This explains the report's constant-holding shoulder displacement and speed figure (labelled Figure 13 in the supplied report). Moving the other joints or maintaining downward tool orientation changes this reduction.

## E.4 Numerical checks supporting the results

**Table E.6. Saved checks supporting the report's Dynamics and Payload section (4.3).** These are retained results from the completed work.

| Claim and check | Saved result |
|---|---|
| Current-angle gravity compensation, both loads, 0–8 s | Largest angle difference 1.74 × 10⁻¹³ rad |
| Tighter solver, empty constant-holding run, 0–2.5 s | Largest angle change 2.00 × 10⁻¹⁰ rad |
| 5 kg shoulder peak, Δt = 0.01 / 0.005 s | 321.155131 / 321.516575 N·m |
| All joint peaks, Δt = 0.01 to 0.005 s, both objects | Largest relative change below 0.125% |

See also the [model parameters](../4_dynamics/ele208_source_data.m) and [shoulder coefficients](../4_dynamics/joint2-coefficients.json).

## Sources

- **6082 aluminium:** Constellium Extrusions Děčín, [*6082 AlSi1MgMn*](https://assets.ctfassets.net/e62qtriulv4v/qGnte8T9Ux1h3ea1POMEW/e2649138f9abb8aab1ecf14f77d9ec3d/Material_sheet_6082.pdf), Physical properties. Density is a material input; the dimensions, mass splits and resulting tensors are our estimates and calculations.
- **Panasonic motor:** Panasonic Industry, [*MSMF152L1H5: MINAS A6 Family Servo Motor*](https://industry.panasonic.com/global/en/products/motor/fa-motor/ac-servo/number/msmf152l1h5), Spec Detail. Source for motor/brake mass and rotor inertia.
- **Nabtesco reducer:** Nabtesco Corporation, [*Precision Reduction Gear RV General Catalog*](https://www.nabtesco.de/fileadmin/04_Service/05_Downloads/01_Produktkataloge/RV_Katalog_EN.pdf), CAT.211201, Rev.004.1, 18 May 2026, printed p. 16 (PDF p. 17), RV-40E column. Source for reducer mass and ratio.
- **Harmonic Drive manual:** Harmonic Drive LLC, [*AC Servo Actuator FHA-C Series Manual*](https://www.harmonicdrive.net/_hd/content/documents1/FHA-C%20Series.pdf), TI-26007 Rev.1, July 2026, printed pp. 1-4 to 1-6 (PDF pp. 16–18). Source for modular-drive masses.
- **Robotics Toolbox:** Peter Corke, [*Robotics Toolbox for MATLAB*, version 10.4](https://github.com/petercorke/robotics-toolbox-matlab/tree/06fddf5228cbfa1132fa63b8e336b43c5ed04899), retained commit `06fddf5228cbfa1132fa63b8e336b43c5ed04899`. Relevant files are `Link.m`, `models/mdl_puma560.m` and `@SerialLink/rne_dh.m`. The PUMA coefficient is a borrowed simulation assumption.
- **Corke textbook:** Peter Corke, [*Robotics, Vision and Control: Fundamental Algorithms in MATLAB*, 2nd ed.](https://link.springer.com/book/10.1007/978-3-319-54413-7), Springer, 2017, Dynamics and Control, pp. 251–281. Background theory; the robot's numerical values and checks are our work.

[6082 aluminium]: https://assets.ctfassets.net/e62qtriulv4v/qGnte8T9Ux1h3ea1POMEW/e2649138f9abb8aab1ecf14f77d9ec3d/Material_sheet_6082.pdf
[Panasonic motor]: https://industry.panasonic.com/global/en/products/motor/fa-motor/ac-servo/number/msmf152l1h5
[Nabtesco reducer]: https://www.nabtesco.de/fileadmin/04_Service/05_Downloads/01_Produktkataloge/RV_Katalog_EN.pdf
[Harmonic Drive manual]: https://www.harmonicdrive.net/_hd/content/documents1/FHA-C%20Series.pdf
[Robotics Toolbox]: https://github.com/petercorke/robotics-toolbox-matlab/tree/06fddf5228cbfa1132fa63b8e336b43c5ed04899
