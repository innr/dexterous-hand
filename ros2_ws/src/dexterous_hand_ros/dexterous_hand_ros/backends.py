"""Backend protocol and virtual implementation for the ROS 2 node."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np


class HandBackend(Protocol):
    def command_positions(self, positions_rad: np.ndarray) -> None: ...
    def read_positions(self) -> np.ndarray: ...
    def set_torque(self, enabled: bool) -> None: ...


@dataclass
class VirtualBackend:
    """A deterministic backend that mirrors accepted commands in memory."""

    size: int = 16
    positions_rad: np.ndarray = field(default_factory=lambda: np.zeros(16))
    torque_enabled: bool = True
    command_count: int = 0

    def __post_init__(self) -> None:
        self.positions_rad = np.asarray(self.positions_rad, dtype=float)
        if self.positions_rad.shape != (self.size,):
            raise ValueError(f"expected {self.size} virtual positions")

    def command_positions(self, positions_rad: np.ndarray) -> None:
        values = np.asarray(positions_rad, dtype=float)
        if values.shape != (self.size,):
            raise ValueError(f"expected {self.size} positions")
        if not np.all(np.isfinite(values)):
            raise ValueError("virtual positions must be finite")
        self.positions_rad = values.copy()
        self.command_count += 1

    def read_positions(self) -> np.ndarray:
        return self.positions_rad.copy()

    def set_torque(self, enabled: bool) -> None:
        self.torque_enabled = bool(enabled)
