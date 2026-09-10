# Space Rover Optimization — Current Project State

Last reviewed: 2026-09-08

This file is the short, current checkpoint. The complete research rationale, equations, seven-stage roadmap, metrics, and references are in `soft_soil_rover_qp_research.md`.

## Current research position

- Current target: prepare and implement **Stage 1 — rigid uneven terrain, fixed four-contact, vertical-force (`Fz`) QP**.
- The QP module has not yet been implemented in `src/` or `include/`.
- The immediate purpose is to understand and validate the complete QP pipeline on the existing rover v1 before investing time in rover v2 visualization/collision integration.
- Required research sequence remains: rigid validation -> unchanged controller on soft soil -> causal failure analysis -> minimal soil-aware modification.

## Current simulation and model choices

- Main model selected in `src/main.cc`: `../model/mcl_rover/scene_terrain.xml` (rover v1).
- MuJoCo target/runtime used in the recent work: 3.12.0.
- Joint position and velocity feedback/control are being treated as load-side quantities.
- Joint/body angles use radians and angular rates use rad/s internally.
- Previously applied gear-ratio conversions were removed; do not restore them without an explicit decision.
- Wheel order used by the controllers: `FL, FR, RL, RR`.

## Implemented controller baseline

- `HighLevelController` exists and is connected from `src/main.cc`.
- `High_level_ctrl()` currently calls:
  1. `Kin_High_level_ctrl()`
  2. `ICR_Kinematic_mapping()`
  3. `Ori_Kinematic_mapping()`
- Current kinematic parameters in `HighLevelController.cpp`:
  - wheelbase: `1.09 m`
  - wheel track: `1.126 m`
  - wheel radius: `0.15 m`
- The ICR mapping generates steering-position and drive-velocity references.
- The orientation mapping generates load-side suspension-position references from roll and pitch.
- Orientation height control is temporarily disabled with `height_error = 0`; `chassis_height_neutral_` is not yet a verified neutral IMU height.
- Suspension position reference rate limiting is present with the current configured value `0.5`. Its exact per-step/per-second interpretation must be confirmed from `Rate_Limit` before treating it as a physical rate.

## Baseline tests completed or accepted

- Kinematic-based ICR steering/drive mapping has been tuned sufficiently to move on to the QP work, subject to preserving it as a baseline.
- Orientation kinematic mapping has operated with the intended qualitative pitch response.
- The user intends to compare the future force-based controller against this kinematic orientation baseline.

## Current observations and unresolved issues

- On uneven terrain, the rover/drive response becomes noisy during driving; flat-ground behavior is comparatively stable.
- Contact separation and recontact, position-controlled suspension motion, and contact-model parameters are suspected contributors. This remains a hypothesis, not a confirmed single cause.
- Changing `impratio` from `20` to `1` appeared visually better.
- A tested `solimp="0.9 0.9 0.001 0.5 2"` setting prevented normal forward motion and disturbed drive control; it is not the current setting.
- Current v1 tire/contact settings should be read directly from `model/mcl_rover/spaceroverMCL.xml` before any tuning because these files are actively modified by the user.
- GRF-versus-`m*a` validation still needs frame/sign-consistent force and acceleration signals. Do not compare magnitudes after removing signs.

## Active contact diagnostic baseline (2026-09-07)

- At the user's request to retain the current CSV before a `condim=4` run, archived the user-identified **condim=6** baseline in [the comparison record](../data/contact_diagnostics/20260907T233737+0900_condim6/README.md). It includes five original CSV copies, model/source snapshots, SHA-256 hashes, column mapping, phase metrics, and plots.
- Verified 26,988 numeric rows per file over **0.001–26.988 s**, with identical wheel time axes and no nonfinite values. Trunk time is inferred by row alignment.
- Captured Tire settings: `margin=0 m`, `condim=6`, `solimp="0.015 1 0.02"`, `solref="0.02 1"`, `friction="0.8 0.02 0.22"`; elliptic, impratio=1, dt=0.001 s. Hfield elevation scale=0.001 m, geom world z=0.1 m.
- On 10–26.988 s, reconstructed IMU-site body-forward speed averages **0.287648 m/s** for a 0.300 m/s reference; world angular-z/yaw-reference error RMS is **0.030834 rad/s**. Wheel drive-error RMS is **0.105–0.127 rad/s**, steering-error RMS **0.00344–0.00531 rad**. Steering follows its targets while chassis yaw response is weak; periodic drive/estimated-force oscillations remain. This is a comparison baseline, not a passed stability gate.
- Captured `main.cc` now calls **suspension and steering** control before 5 s, and drive control from 5 s. Chassis speed/yaw feedback gains remain zero. Exact executable provenance and fresh-process startup were not established.
- CSV force columns are absolute-valued sensor-based estimates, torque columns are commands, and slip/Mu columns are inactive zero estimates. Do not treat these as raw contact forces, actual actuator outputs, or measured physical slip/friction.
- The follow-up **condim=4** run is now archived and compared in [the paired report](../data/contact_diagnostics/20260907T234343+0900_condim4/README.md). It has 26,642 numeric rows over 0.001–26.642 s. All snapshot hashes verified; the only captured model/source change is Tire `condim=6→4`. Shared time axes, desired wheel/chassis commands, and first recorded states match.
- Over the common **10–26.642 s** window, condim 6→4 changes body-frame IMU lateral-speed RMS **0.004731→0.098146 m/s** (~20.7×), while yaw/world-angular-z tracking RMS improves **0.030799→0.014044 rad/s** and wheel drive-speed RMS improves from **0.103–0.128→0.033–0.051 rad/s**. Forward-speed means are similar (**0.287700→0.286386 m/s**). The new run shows substantial lateral drift and increased yaw/estimated-force fluctuations, not NaN/Inf or runaway drive speed in this recorded interval.
- Mean drive commands fall from roughly **8–15 N*m** per wheel to **0.04–0.48 N*m**; logged estimated-GRF-z standard deviation increases from roughly **3.78–5.13 N→6.26–7.51 N**. These are commanded torques and sensor-based force estimates, not raw contact wrench measurements.
- **Next task:** inspect the lateral contact velocities, normals, and signed summed wheel contact forces/moments associated with condim=4's growing body lateral motion. Both condim settings have unresolved tradeoffs; do not infer a validated contact configuration from either run. Preserve these paired archives and distinguish the earlier unlabeled 38.696 s CSV from this comparison.

## Next concrete work: Stage 1 preparation

### ProxQP installation completed (2026-09-08)

- User selected ProxQP for Stage 1/2 and requested installation alongside CasADi and Ipopt. Verified actual directory: `/home/ycm/YCM/third-party` (not `/home/YCM/third-party`).
- Installed ProxSuite **0.7.3**, official tag `v0.7.3`, commit `b93d7778ffc3299d84b5cb0851022a29bf24a596`.
- Source: `/home/ycm/YCM/third-party/proxsuite`; build: `proxsuite/build`; install prefix: `proxsuite/install`.
- Build configuration: Release, C++17, system Eigen 3.3.7, Python/tests/benchmarks/documentation disabled, optional ProxSuite SIMD support disabled (Simde not installed). Dense double backend validated.
- `CMakeLists.txt` now finds ProxSuite using this install prefix as a hint and links `simulation` to `proxsuite::proxsuite`. A relocated installation can be selected with `-Dproxsuite_DIR=<prefix>/lib/cmake/proxsuite`.
- This is a header-only C++ installation; no separate ProxQP `.so` is expected. Removing the rover project's `build/` will not remove it.
- Verified standalone synthetic Stage 1-shaped QP: equal-load solution `[100,100,100,100] N`; updated load with warm start: `[110,110,110,110] N`.
- Verified synthetic Stage 2-shaped QP with equality constraints and active `abs(Fx_i) <= 0.5*Fz_i` constraints: each `Fx_i=50 N`, `Fz_i=100 N`. Maximum equality residual across checks was below `7e-10`, no inequality violations. These are installation checks, not validated rover parameters or a passed research stage gate.
- Standalone check source/build: `/tmp/rover-proxqp-check.PdPOM6` (temporary and not guaranteed to persist).
- Eigen 3.3.7's unsupported headers require `<iostream>` before `<proxsuite/proxqp/dense/dense.hpp>` in the standalone check. Preserve this include order in future integration or use a reviewed compatible Eigen upgrade.
- Upstream install script emitted `Error: could not load cache` from an auto-uninstall hook with an empty build path; the install command returned success, headers/config files were installed, and independent CMake discovery, compilation, solving, and the full `simulation` build all passed.
- The initially downloaded project-local `.deps/proxsuite-0.7.3-src` was moved to the user-selected directory and the empty `.deps` directory removed. CasADi/Ipopt were not modified.
- Next controller task is still Stage 1 dynamics/contact extraction and QP formulation. No QP controller was added to the simulation by the installation work.

Before writing the Stage 1 QP, verify these interfaces from the repository and simulation:

1. Extract the actual wheel-terrain contact force using MuJoCo contact data and `mj_contactForce`, then transform it into one explicitly chosen frame.
2. Define the body/world coordinate convention and verify IMU quaternion-to-Euler signs.
3. Verify rover mass, CoM, inertia, wheel contact positions, control period, and suspension actuator semantics.
4. Define the desired body vertical wrench `[Fz_des, Mx_des, My_des]` from body height, roll, and pitch errors.
5. Construct the geometry-dependent mapping `A_z(q)` from the four wheel normal forces to that body wrench.
6. Use the installed ProxQP dense double backend for Stage 1/2; design the solver interface and timing/status logging.
7. Define normal-force, suspension force/torque, stroke, velocity, force-rate, and slack limits.

First QP decision variable:

```text
Fz = [Fz_FL, Fz_FR, Fz_RL, Fz_RR]^T
```

First validation sequence:

1. Flat-ground static equilibrium.
2. Asymmetric rigid uneven terrain with fixed four-contact assumption.
3. Compare against passive/nominal suspension, equal-`Fz`, and the existing kinematic orientation controller.
4. Log body tracking, all wheel forces, normal-force dispersion, minimum `Fz`, actuator saturation, constraint residuals, solver status, infeasibility count, and solve time.

## Working agreement with the user

- Explain proposed code changes first unless the user explicitly says to implement them.
- When the user provides a compiler/runtime error, identify the exact cause and edit location; let the user make the change unless they explicitly request an edit.
- Never claim an issue is solved without a build, run, log, or user confirmation appropriate to that issue.
