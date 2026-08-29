# Design Documents

Feature implementation in this repository follows a design-first workflow.

## Lifecycle

`Draft -> Reviewed -> Frozen -> Implemented`

A frozen Design Doc is the contract used by Codex and by PR review. If implementation reveals a major design problem, move the document back to review and update the design before intentionally diverging.

## Backlog

1. `001_mujoco_pd_control.md` — 16-actuator MuJoCo PD control
2. `002_safety_policy.md` — home pose, limits, velocity limits, watchdog/e-stop
3. `003_joint_mapping_contract.md` — canonical/MuJoCo/STS3215 mapping
4. `004_virtual_sts3215_bus.md` — deterministic virtual hardware bus and failures
5. `005_ros2_interface.md` — `joint_command` / `joint_states`
6. `006_mediapipe_retargeting.md` — MediaPipe to LEAP 16-DOF
7. `007_recording_replay.md` — recording/replay and LeRobot/HDF5
8. `008_hardware_commissioning.md` — serial scan, direction, zero calibration, physical motion

PR #1 predates this workflow and overlaps Designs 001-003. Those documents should therefore review/adopt/amend the existing PR rather than request duplicate implementation.

## Required sections

Each Design Doc must define Context, Goals, Non-goals, Existing behavior, Proposed architecture, Interfaces, Configuration, Safety/failure behavior, Testing strategy, Acceptance Criteria, Hardware dependencies, Alternatives/trade-offs, and Codex implementation notes.
