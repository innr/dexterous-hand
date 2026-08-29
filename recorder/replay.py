"""Deterministic episode replay helpers."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterator, Protocol

import numpy as np

from .hdf5_reader import EpisodeReader
from .schema import EpisodeFrame


class CommandSink(Protocol):
    def send_position(self, position_rad) -> None:
        ...


@dataclass
class MemoryCommandSink:
    """A safe in-memory command sink for replay tests and dry runs."""

    commands: list[np.ndarray] = field(default_factory=list)
    torque_enabled: bool = True

    def send_position(self, position_rad) -> None:
        vector = np.asarray(position_rad, dtype=np.float32)
        if vector.shape != (16,) or not np.isfinite(vector).all():
            raise ValueError("replay command must be a finite 16-vector")
        self.commands.append(vector.copy())

    def disable_torque(self) -> None:
        self.torque_enabled = False


def replay_episode(
    source: str | EpisodeReader,
    *,
    sink: CommandSink | None = None,
    speed: float = 0.0,
    skip_invalid: bool = True,
    sleep: Callable[[float], None] = time.sleep,
) -> Iterator[EpisodeFrame]:
    """Yield an episode in order and optionally send actions to a sink.

    ``speed=0`` is deterministic and never sleeps. Positive speed values use
    the recorded timestamps, where 1.0 is real-time and 2.0 is twice as fast.
    No real hardware sink is created by this function.
    """

    if speed < 0:
        raise ValueError("speed must be non-negative")
    owns_reader = isinstance(source, (str, bytes, Path))
    reader = EpisodeReader(source) if owns_reader else source
    previous_timestamp_ns: int | None = None
    try:
        for frame in reader:
            if speed > 0 and previous_timestamp_ns is not None:
                delay = (frame.timestamp_ns - previous_timestamp_ns) / 1e9 / speed
                if delay > 0:
                    sleep(delay)
            previous_timestamp_ns = frame.timestamp_ns
            if frame.valid:
                if sink is not None:
                    sink.send_position(frame.action_position_rad)
                yield frame
            elif not skip_invalid:
                yield frame
    finally:
        if owns_reader:
            reader.close()
