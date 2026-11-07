# Design Documents

Feature implementation follows a design-first workflow.

## Lifecycle

`Draft -> Reviewed -> Frozen -> Implemented`

A frozen Design Doc is the contract used by implementation and PR review. If implementation reveals a major design problem, move the document back to review and update the design before intentionally diverging.

Required sections: Context, Goals, Non-goals, Existing behavior, Proposed architecture, Interfaces, Configuration, Safety/failure behavior, Testing strategy, Acceptance Criteria, Hardware dependencies, Alternatives/trade-offs, implementation notes.

## Current contract

| ID | Status | Design |
|---|---|---|
| **S7-00** | **Frozen** | [`s7_00_contract_convergence.md`](s7_00_contract_convergence.md) — canonical 16-DOF, mapping, `mayStartSend` |

Implement against S7-00. Do not start MediaPipe / LeRobot / hardware motion until that series lands and hardware gates (H-01–H-03) allow it.

## Implemented record (S4–S6)

| ID | Status | Design |
|---|---|---|
| S4 | Implemented | [`s4_realtime_control.md`](s4_realtime_control.md) |
| S5-01 … S5-09 | Implemented | `s5_01_safe_send.md` … `s5_09_control_telemetry.md` |
| S6-01 | Implemented | [`s6_01_mujoco_bridge.md`](s6_01_mujoco_bridge.md) |
| S6-02 | Implemented; **command surface superseded by S6-03** | [`s6_02_official_leap_ros2.md`](s6_02_official_leap_ros2.md) |
| S6-03 | Implemented | [`s6_03_ros2_dds.md`](s6_03_ros2_dds.md) |
| S6-04 | Implemented | [`s6_04_lifecycle_supervisor.md`](s6_04_lifecycle_supervisor.md) |

Acceptance: `docs/acceptance/s5_stage_acceptance_2026_11_05.md`, `docs/acceptance/s6_stage_acceptance_2026_11_05.md`.

Architecture baseline: `docs/architecture/system_architecture.md`.

## Historical backlog 001–008

These numbers lived only in this README; they were never separate Frozen files.

| ID | Disposition |
|---|---|
| 001 MuJoCo 16-actuator PD | **Superseded** by S6 + S7-00 |
| 002 Safety / home / limits / e-stop policy | **Superseded** in software by S5-06 / StopCoordinator; physical e-stop remains H-01 |
| 003 Joint mapping contract | **Superseded** by S7-00 mapping section |
| 004 Virtual STS3215 bus | **Superseded** by `VirtualServoBus` / S5-03 |
| 005 ROS2 `joint_command` / `joint_states` | **Superseded** by S6-03 `JointCommand` |
| 006 MediaPipe → LEAP 16-DOF | **Future / not implemented / not S7-00** |
| 007 Recording, replay, LeRobot/HDF5 | **Future / not implemented / not S7-00** |
| 008 Hardware commissioning (scan, direction, zero, physical motion) | **Future / not implemented / not S7-00**; blocked on H-01–H-03 |

### Historical: PR #1 merge gate

Early process required Designs 001–003 Frozen before merging PR #1 (`feat: add actuated LEAP simulation control`). That gate is **historical**. `main` already contains S5/S6. Do not treat PR #1 as the in-flight baseline.
