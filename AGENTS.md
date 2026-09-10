# Space Rover Optimization project instructions

## Persistent project context

- Before analyzing or changing this repository, read `docs/PROJECT_STATE.md`.
- For research goals, equations, stage gates, validation metrics, and the complete implementation order, read the relevant section of `docs/soft_soil_rover_qp_research.md`.
- Treat `docs/soft_soil_rover_qp_research.md` as the stable research handoff and `docs/PROJECT_STATE.md` as the current checkpoint. If they differ about current progress, use `PROJECT_STATE.md` for progress and the research handoff for the intended roadmap.

## Research objective

Develop soft-soil-aware, contact-mode-dependent integrated motion and force control for an actively suspended planetary rover.

Preserve this causal research sequence:

1. Validate the rigid-ground controller on the rigid plant.
2. Apply the unchanged rigid-ground controller to the soft-soil plant and observe failure.
3. Identify the causal failure mechanism.
4. Add the smallest justified soil-aware modification and compare it with the same baselines.

Implement and validate the numbered stages in the research handoff in order. Do not jump to a later soft-soil stage before the applicable rigid-ground stage gate has passed.

## Collaboration rules

- The user normally wants an explanation of the cause, equations, affected files, and proposed edit first so they can inspect and edit the code themselves.
- Modify source code, model files, build settings, or other existing project files only when the user explicitly asks for the modification.
- When explicitly asked to modify something, limit changes to the requested scope and preserve unrelated user changes in the dirty worktree.
- Do not silently change controller behavior, physical parameters, MJCF contact settings, coordinates, units, model selection, or initial conditions.
- State units and coordinate frames explicitly. The controller convention is SI units unless a documented interface says otherwise: position in metres, joint/body angles in radians, linear velocity in m/s, angular velocity in rad/s, force in N, and torque in N*m.
- Feedback and commands are load-side quantities unless `PROJECT_STATE.md` or the implementation explicitly says otherwise. Do not reintroduce a gear ratio without user approval.
- Use wheel order `FL, FR, RL, RR` unless the implementation being examined proves otherwise.
- Prefer evidence from the repository over asking the user. Ask only when a missing choice would materially change the result.

## Progress tracking

- After a verified milestone or an important design decision, update `docs/PROJECT_STATE.md` when the user asks for the update or when updating documentation is explicitly part of the requested work.
- Record what was verified separately from hypotheses and unresolved observations.
- Keep the next concrete task explicit so work can resume after context compaction or in a new session.

