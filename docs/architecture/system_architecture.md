# Dexterous Hand System Architecture

## 1. Purpose

This document is the project-level architecture baseline for the LEAP-style 16-DOF dexterous hand stack. It separates what exists on `main`, what is currently in flight, and what the target architecture requires. It also freezes the development workflow used for subsequent software tasks.

The migration principle is conservative: existing working code is not rewritten merely to match documentation. Design documents review existing behavior first, retain compatible implementation, and require changes only where a frozen contract or acceptance criterion is not met.

## 2. System scope

The stack is responsible for representing, commanding, simulating, observing, and eventually operating a 16-DOF LEAP Hand-compatible mechanism. The software boundary includes:

- official LEAP model integration and MuJoCo simulation;
- canonical 16-DOF joint representation;
- position control and shared safety policy;
- STS3215 transport, calibration, and hardware backend;
- future ROS 2 integration;
- future vision-to-hand retargeting;
- future recording/replay and LeRobot/HDF5 data paths.

Physical commissioning remains outside the software-only acceptance boundary until the required hardware is available.

## 3. Current implementation baseline

### 3.1 On `main`

The current repository already provides the foundations that new work must reuse:

- Python package layout with `simulation` and `hardware` packages;
- pinned official LEAP Hand simulation repository as a submodule;
- `config/joint_mapping.json` describing the 16 canonical joints, MuJoCo ordering, STS3215 IDs, limits, direction placeholders, and zero-offset placeholders;
- a dependency-light STS3215 packet/bus driver;
- serial scan and zero-capture tooling;
- tests around the existing model/mapping/STS3215 behavior.

The hardware mapping remains intentionally uncalibrated. Direction signs and zero offsets must not be treated as physically verified values.

### 3.2 In-flight PR #1

PR #1 (`feat: add actuated LEAP simulation control`) is implementation under review, not part of the `main` baseline. It currently introduces behavior spanning all three P0 design areas:

- temporary conversion/augmentation of the official LEAP URDF into an actuated MuJoCo model;
- 16 canonical position actuators and PD configuration;
- target velocity limiting, force limits, joint-limit projection, and simulation home pose;
- canonical joint-position <-> STS3215 tick conversion;
- a calibration-gated hardware controller and explicit torque operations;
- additional simulation and hardware-controller tests.

Because this PR crosses Designs 001, 002, and 003, it must not be approved merely because one of those designs has been reviewed. All affected P0 contracts must be frozen first.

## 4. Canonical 16-DOF contract

The system uses one canonical joint vector for all application-facing interfaces. A joint position command or state is a vector of 16 radians ordered by canonical LEAP/hardware ID `0..15`.

Backend-specific ordering is an implementation detail. MuJoCo qpos ordering must be translated at the simulation boundary. STS3215 IDs, directions, encoder offsets, and tick conversion must be translated at the hardware boundary. Higher-level modules must not duplicate these reorder/conversion rules.

The mapping configuration is the authoritative metadata source for joint identity and configured limits. Simulation-only defaults such as a MuJoCo home pose must remain explicitly distinct from calibrated physical zero offsets.

## 5. Target architecture

```text
Command Sources
  CLI / tests / future ROS 2 / future retargeting / replay
                         |
                         v
                Canonical 16-DOF API
                         |
                         v
                  Shared Safety Policy
             limits / rate / validity / stop
                         |
                         v
                   Backend Interface
                    /             \
                   v               v
          MuJoCo Backend      STS3215 Backend
          reorder + ctrl      calibration + ticks
                   |               |
                   v               v
              Simulation       Serial bus

Canonical state ---------------------------------> recording / ROS 2
```

The architecture deliberately keeps command producers independent from simulation and hardware details. A producer should be able to emit the same canonical target regardless of backend.

## 6. Layer responsibilities

### 6.1 Configuration and joint model

Owns joint names, canonical ordering, limits, backend mapping, calibration metadata, and validated control parameters. Invalid mappings must fail before commands reach a backend.

### 6.2 Shared safety/control contract

Owns validation of command shape and finite numeric values, joint-limit enforcement, velocity/rate policy, home/stop semantics, and the rules that determine whether motion or torque may be enabled.

Safety rules should be testable without physical hardware. Disabling torque or entering a safe stopped state must remain possible during recovery even when normal motion is rejected.

### 6.3 MuJoCo backend

Owns official-model loading/conversion, MuJoCo joint/actuator lookup, canonical <-> simulator ordering, actuator parameterization, and application of validated canonical targets. It must not redefine application-level joint identity.

### 6.4 STS3215 backend

Owns serial protocol details, encoder/tick conversion, servo-ID mapping, calibration transforms, read/write operations, and hardware-specific failure reporting. Hardware motion and torque enable remain calibration-gated unless an explicitly defined bench/test override is used.

### 6.5 ROS 2 interface (target)

Will expose canonical `joint_command` and `joint_states` interfaces. ROS-specific message transport must not own mapping or hardware conversion logic.

### 6.6 Retargeting (target)

Will convert MediaPipe/other hand observations into canonical LEAP 16-DOF targets. Retargeting output must pass through the same safety/control contract as any other command source.

### 6.7 Data pipeline (target)

Will record/replay canonical commands/states plus timestamps and relevant metadata. LeRobot/HDF5 serialization remains downstream of the canonical representation.

## 7. Dependency direction

Dependencies flow downward toward configuration, canonical contracts, and backend primitives. Higher-level applications may depend on lower layers; low-level transport must not depend on ROS 2, MediaPipe, or dataset code.

A desired dependency shape is:

```text
applications / ROS2 / retargeting / replay
                 |
                 v
       canonical control + safety
                 |
          +------+------+
          |             |
          v             v
     simulation      hardware
          |             |
          v             v
       MuJoCo        STS3215
                 
configuration / joint mapping is consumed by the relevant lower layers
```

## 8. Safety and calibration invariants

The following are architecture-level invariants and require explicit design review before being weakened:

- uncalibrated hardware must not accept ordinary motion commands;
- enabling torque must be explicit and subject to calibration policy;
- torque disable/recovery must remain available even when calibration or command validation fails;
- safe goal state must be established before torque activation when required by the hardware command design;
- joint direction must be validated as exactly `-1` or `+1` before calibrated conversion is trusted;
- non-finite commands/configuration values must be rejected at validation boundaries;
- simulation home pose is not a substitute for hardware calibration;
- all application-facing positions use radians in canonical order;
- backend conversion/reordering must have deterministic tests.

Exact behavior is frozen in Designs 001-003 rather than inferred from the current implementation.

## 9. Development workflow

Every implementation task follows this lifecycle:

```text
Backlog
  -> ChatGPT design discussion
  -> Design Doc (Draft)
  -> Design review
  -> Design Doc (Frozen)
  -> Codex implementation against frozen contract
  -> automated tests / simulation validation
  -> one task / one PR
  -> spec-based review
  -> merge
  -> next task
```

If implementation reveals a material architecture problem, the design returns to Draft/Review before the implementation contract changes. Codex should not silently make major architecture decisions while implementing a frozen design.

Each Design Doc must contain:

- Context / problem;
- Goals;
- Non-goals;
- Current behavior / reuse plan;
- proposed architecture;
- public interfaces and data structures;
- configuration;
- safety and failure behavior;
- testing strategy;
- Acceptance Criteria;
- alternatives / trade-offs;
- implementation notes for Codex.

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

Because PR #1 already overlaps 001, 002, and 003, those designs must first document and review the existing implementation instead of blindly generating replacement code.

## 11. Architecture decisions frozen by this baseline

- The project uses one canonical 16-DOF joint-space contract.
- Hardware/official LEAP joint order is the canonical ordering for the current stack.
- Backend-specific ordering is translated only at backend boundaries.
- Hardware calibration data is distinct from simulation defaults.
- Hardware commands remain calibration-gated and torque enable is explicit.
- Safety is a shared layer/policy, not independently reimplemented by each application.
- ROS 2, retargeting and recording sit above the canonical control contract.
- Hardware-independent functionality should have deterministic tests before physical commissioning.

## 12. Immediate next step and PR #1 merge gate

Create and review `docs/designs/001_mujoco_pd_control.md`, `002_safety_limits.md`, and `003_joint_mapping.md` against the behavior already present in PR #1. Each design should explicitly decide which existing behavior is retained, identify gaps, and freeze its Acceptance Criteria.

**PR #1 must not be approved or merged until Designs 001, 002, and 003 are all reviewed and frozen and PR #1 has been checked against all three contracts.** If gaps exist, amend PR #1 and rerun the required tests before approval. Do not start Design 004/005 implementation until this overlapping P0 review is complete.