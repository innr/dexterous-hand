# Full-stack software integration design

## Goal

Validate the software path from synthetic hand landmarks to recorded and replayed LEAP joint commands without a camera or physical hardware.

## Pipeline

1. A deterministic synthetic landmark fixture provides 21 MediaPipe-style hand landmarks.
2. The C++ LEAP retargeting core converts landmarks to 16 canonical joint angles in radians.
3. The ROS 2 adapter publishes `joint_command` and receives `joint_states`.
4. The virtual STS3215 bus applies the command, validates IDs/order, and returns simulated positions.
5. The recorder stores commands and states in the HDF5 schema v1.
6. Replay feeds the recorded command sequence back to the virtual bus.

## Boundaries

- No real serial port, camera, or torque output.
- Preserve the existing 16-joint LEAP order and radian units.
- Reuse existing retargeting, ROS 2, virtual-bus, and HDF5 APIs; add only adapters and integration tests.
- Timeout behavior remains `disable_torque`.

## Test cases

- A deterministic fixture produces exactly 16 finite joint commands.
- Command IDs/order survive the ROS 2 adapter and virtual bus unchanged.
- Joint limits are enforced before the virtual write.
- A simulated timeout invokes `disable_torque`.
- A recorded episode can be replayed with matching commands and states.
- Virtual communication errors are surfaced to the integration caller.

## Acceptance criteria

The integration test suite passes on Ubuntu 24.04 / ROS 2 Jazzy with no hardware, camera, or serial device. The test output must identify each pipeline stage and must not claim physical validation.
