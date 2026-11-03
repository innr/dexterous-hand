# LEAP dexterous hand: S1–S3 C++ architecture

The existing Python simulation and hardware files remain unchanged. The new `cpp/` module is a separate C++17 educational reference implementation.

```text
ROS2 / teleoperation [S4, planned]
            |
      desired positions
            v
JointTrajectoryPlanner [S3]
  -> TrajectoryState
  -> AccelerationLimiter
  -> stopping-distance and candidate checks
            |
            v
PDController + SafetyStateMachine [S3]
            |
            v
RobotController -> RobotHand -> RobotJoint [S1/S2]
            |
            v
IBus -> MockBus [implemented]
     -> STS3215Bus [NOT implemented]
```

## Component responsibilities

| Component | Purpose |
|---|---|
| RobotJoint | joint ID, position clamp |
| RobotHand | 16 commanded joint positions |
| RobotController | owns bus with unique_ptr, synchronous command send |
| PDController | PD control effort from target and measured state |
| JointTrajectoryPlanner | nominal speed/acceleration-limited position planning |
| SafetyStateMachine | NORMAL, DEGRADED, SAFE_STOP, FAULT demo transitions |

## S4 target architecture (not implemented)

```text
100 Hz control task (10 ms)
   |                     ^
   v                     |
safety gate       timestamped feedback snapshot
   |                     ^
   v                     |
latest command mailbox   |
   |                     |
   v                     |
STS3215 communication worker (~11.2 ms polling)
```

Use timestamped feedback, data-age checks, bounded command queues, stop-priority protocol and in-flight TX accounting. The current synchronous controller cannot meet the proposed real-time architecture by itself.

## Limitations

The trajectory planner is a reconstructed nominal V2 example, not a proven safe trajectory generator. Braking feasibility checks do not guarantee a feasible candidate or latency safety. `RobotController` records commanded position, not measured feedback. No hardware E-stop, watchdog, bus adapter, ROS2 or MuJoCo integration is implemented here.
