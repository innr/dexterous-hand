# Dexterous Hand System Architecture Baseline

Status: **Baseline v1**  
Date: 2026-08-29  
Scope: software architecture for the LEAP Hand / STS3215 learning platform

## 1. Purpose

This document freezes the project-level architecture before further feature implementation. Existing code is treated as the v0 implementation baseline; it is not rewritten merely to match this document. New work should follow the design-doc workflow defined here.

## 2. Product goal

Build a 16-DOF LEAP Hand software stack that can use the same canonical joint-space interface across MuJoCo simulation and STS3215 hardware, then add ROS 2 integration, human-hand retargeting, and recording/replay for robotics datasets.

The development order is simulation-first and hardware-safe: software behavior should be testable without physical hardware whenever possible.

## 3. Canonical system boundary

```text
Human input / policy / replay
            |
            v
      Retargeting / command source
            |
            v
   Canonical 16-DOF joint command
            |
     +------+------+
     |             |
     v             v
 Safety layer   State/recording
     |
     v
 Control backend abstraction
     |
 +---+------------------+
 |                      |
 v                      v
MuJoCo backend      STS3215 backend
 |                      |
 v                      v
LEAP model          Physical LEAP hand
```

The **canonical 16-DOF joint vector** is the architectural contract between command sources, simulation, hardware, ROS 2, and future data pipelines. Hardware ID / official LEAP order is the current canonical ordering. MuJoCo-specific ordering must be translated at the simulation boundary.

## 4. Current implementation baseline (`main`)

### Implemented

- Python package with MuJoCo, NumPy, pyserial and pytest.
- Pinned official `LEAP_Hand_Sim` repository as a git submodule.
- Official LEAP URDF loading.
- Explicit MuJoCo-joint-order to canonical/hardware-order conversion.
- `config/joint_mapping.json` containing 16 joint names, hardware IDs, model joint names, directions, zero offsets and position limits.
- Dependency-light STS3215 packet protocol implementation.
- STS3215 ping, scan, register read/write, position read/write and torque control.
- Serial scan and zero-capture utilities.
- Unit tests for simulation loading, joint mapping and STS3215 protocol behavior.

### Intentionally unresolved until hardware commissioning

- Real servo direction signs.
- Mechanical zero offsets.
- Verified physical joint limits.
- Real serial timing/reliability characteristics.

These values must never be inferred from simulation alone.

## 5. In-flight implementation: PR #1

PR #1 (`feat: add actuated LEAP simulation control`) is not part of `main` until merged. It currently proposes:

- generated actuated MuJoCo model with 16 position actuators;
- canonical-order PD position control;
- joint-limit projection and target velocity limiting;
- actuator force limits;
- simulation home pose;
- calibration-gated canonical joint-radian to STS3215-tick conversion;
- a `LeapHandBusController` hardware bridge;
- CI running pytest and a headless LEAP simulation smoke test.

PR #1 should be reviewed against the first two P0 design requirements before merge. Its implementation should be reused where it satisfies the frozen design rather than recreated.

## 6. Target software layers

### 6.1 Configuration / model contract

Owns canonical joint metadata and configuration:

- joint identity and ordering;
- simulation mapping;
- hardware ID mapping;
- calibrated direction and zero offset;
- position/velocity constraints;
- safe home pose;
- controller parameters.

Configuration must distinguish **simulation assumptions** from **hardware-calibrated values**.

### 6.2 Command and safety layer

All motion commands must pass through one common validation path before reaching a backend.

Responsibilities:

- shape/order validation;
- finite-value validation;
- position limits;
- velocity/rate limits;
- home pose;
- watchdog / command timeout;
- emergency-stop state;
- explicit torque-enable semantics for hardware.

Safety policy must not be duplicated independently in ROS 2, teleoperation, and hardware scripts.

### 6.3 Backend interface

Simulation and physical hardware should converge on a small conceptual interface:

```python
command_positions(q_target_rad)
read_positions() -> q_rad
set_torque(enabled)          # hardware-capable backends
stop()                       # safe backend stop
```

The exact Python API will be frozen in the relevant Design Doc before refactoring existing code.

### 6.4 MuJoCo backend

Responsibilities:

- load the pinned official LEAP model;
- expose canonical 16-DOF state/command ordering;
- apply position/PD control;
- respect configured position, velocity and actuator-force constraints;
- support deterministic headless tests.

MuJoCo ordering is an implementation detail and must not leak into higher layers.

### 6.5 STS3215 backend

Responsibilities:

- transport and packet protocol;
- deterministic errors for timeout, checksum/protocol errors and servo status errors;
- canonical radians <-> servo ticks conversion using calibrated mapping;
- multi-servo command/state operations;
- explicit torque control;
- no physical motion when calibration requirements are not satisfied.

A virtual STS3215 transport/bus must exercise these semantics before hardware is available.

### 6.6 ROS 2 adapter (planned)

ROS 2 is an adapter around the canonical command/state API, not the owner of hardware semantics.

Initial external contract:

- `joint_command`: canonical 16-DOF command;
- `joint_states`: canonical measured/simulated state.

Detailed message types, QoS, rates, lifecycle and e-stop behavior require a dedicated Design Doc.

### 6.7 Retargeting (planned)

MediaPipe hand landmarks are converted into canonical LEAP joint targets. Retargeting must not directly address servo IDs or MuJoCo indices.

### 6.8 Recording / replay (planned)

Recording consumes canonical states/commands plus sensor streams and preserves timestamps. rosbag2 / HDF5 / LeRobot conversion belongs above the control backend and must remain usable with simulation.

## 7. Dependency direction

Preferred dependency direction:

```text
applications / ROS2 / retargeting / recorder
                 |
                 v
        canonical command + safety
                 |
                 v
          backend interfaces
            /          \
           v            v
      simulation      hardware
```

Protocol-level STS3215 code must not depend on ROS 2, MediaPipe or LeRobot. Simulation must not depend on serial hardware.

## 8. Development workflow (mandatory for new feature work)

Every feature follows:

```text
Requirement
  -> ChatGPT architecture/technical discussion
  -> Design Doc
  -> Design review / freeze
  -> Codex implementation
  -> tests / simulation validation
  -> one feature PR
  -> spec-based review
  -> merge
```

Rules:

1. One scoped feature should normally map to one Design Doc and one implementation PR.
2. Codex should implement the frozen contract rather than redesign core architecture during coding.
3. Every Design Doc must contain explicit Acceptance Criteria.
4. If implementation exposes an architectural flaw, update/review the Design Doc before making a major architectural deviation.
5. PR descriptions must state the Design Doc, implementation summary, validation commands/results, and remaining limitations.
6. Hardware-dependent assumptions must be labeled and must not be marked verified until tested on hardware.

## 9. Design Doc template

Each `docs/designs/NNN_*.md` should contain at minimum:

- Status: Draft / Reviewed / Frozen / Implemented
- Context
- Goals
- Non-goals
- Existing behavior
- Proposed architecture
- Public interfaces / data structures
- Configuration changes
- Safety and failure behavior
- Testing strategy
- Acceptance Criteria
- Hardware dependencies
- Alternatives / trade-offs
- Implementation notes for Codex

## 10. Ordered design backlog

The current software backlog is frozen in this order unless a dependency requires adjustment:

| ID | Priority | Design | Hardware required |
|---|---|---|---|
| 001 | P0 | MuJoCo 16-actuator PD control | No |
| 002 | P0 | Home pose, limits, velocity limits and emergency-stop policy | No |
| 003 | P0 | MuJoCo joint order <-> STS3215 ID mapping contract/tests | No |
| 004 | P1 | Virtual STS3215 bus: read/write, timeout, checksum and servo errors | No |
| 005 | P1 | ROS 2 unified `joint_command` / `joint_states` interface | No |
| 006 | P2 | MediaPipe -> LEAP 16-DOF retargeting | Camera can be deferred |
| 007 | P2 | Recording, replay and LeRobot/HDF5 pipeline | No |
| 008 | P3 | Real serial scan, direction verification, zero calibration and physical motion | Yes |

Because PR #1 already overlaps 001 and part of 002/003, those designs must first document and review the existing implementation instead of blindly generating replacement code.

## 11. Architecture decisions frozen by this baseline

- The project uses one canonical 16-DOF joint-space contract.
- Hardware/official LEAP joint order is the canonical ordering for the current stack.
- Backend-specific ordering is translated only at backend boundaries.
- Hardware calibration data is distinct from simulation defaults.
- Hardware commands remain calibration-gated and torque enable is explicit.
- Safety is a shared layer/policy, not independently reimplemented by each application.
- ROS 2, retargeting and recording sit above the canonical control contract.
- Hardware-independent functionality should have deterministic tests before physical commissioning.

## 12. Immediate next step

Create and review `docs/designs/001_mujoco_pd_control.md` against PR #1. The goal is to decide which PR #1 behavior becomes the frozen contract, identify any gaps, then either amend PR #1 or approve it. Do not start Design 004/005 implementation until the overlapping P0 contracts are reviewed.
