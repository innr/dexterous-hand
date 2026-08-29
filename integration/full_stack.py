"""Hardware-independent full-stack integration harness.

The harness uses injected retargeting and transport functions so it can run
without ROS 2, a camera, or a serial device. Production adapters can replace
the injected functions while preserving the 16-joint contract.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Sequence

JOINT_COUNT = 16

class IntegrationError(RuntimeError):
    pass

@dataclass(frozen=True)
class JointState:
    positions: tuple[float, ...]
    torque_enabled: bool = True

@dataclass(frozen=True)
class Episode:
    commands: tuple[tuple[float, ...], ...]
    states: tuple[JointState, ...]

class VirtualTransport:
    def __init__(self, *, timeout_after: int | None = None):
        self.timeout_after = timeout_after
        self.calls = 0
        self.torque_enabled = True
    def send(self, command: Sequence[float]) -> JointState:
        if len(command) != JOINT_COUNT:
            raise IntegrationError("joint command must contain exactly 16 values")
        self.calls += 1
        if self.timeout_after is not None and self.calls > self.timeout_after:
            self.torque_enabled = False
            raise TimeoutError("virtual STS3215 timeout")
        values = tuple(float(x) for x in command)
        return JointState(values, self.torque_enabled)
    def disable_torque(self) -> None:
        self.torque_enabled = False

def run_episode(
    landmarks: object,
    retarget: Callable[[object], Sequence[float]],
    transport: VirtualTransport,
) -> Episode:
    command = tuple(float(x) for x in retarget(landmarks))
    if len(command) != JOINT_COUNT:
        raise IntegrationError("retargeter must return 16 joint angles")
    if any(not (-3.141592653589793 <= x <= 3.141592653589793) for x in command):
        raise IntegrationError("joint command is outside canonical radian range")
    try:
        state = transport.send(command)
    except TimeoutError:
        transport.disable_torque()
        raise
    return Episode((command,), (state,))

def replay(episode: Episode, transport: VirtualTransport) -> Episode:
    states = []
    for command in episode.commands:
        try:
            states.append(transport.send(command))
        except TimeoutError:
            transport.disable_torque()
            raise
    return Episode(episode.commands, tuple(states))
