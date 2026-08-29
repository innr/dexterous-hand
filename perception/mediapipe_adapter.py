"""Validate and normalize MediaPipe hand landmarks without camera dependencies."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable, Sequence

LANDMARK_COUNT = 21

class LandmarkError(ValueError):
    pass

@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    z: float = 0.0

@dataclass(frozen=True)
class HandLandmarks:
    points: tuple[Landmark, ...]
    handedness: str | None = None
    timestamp_ns: int | None = None

    def as_flattened(self) -> tuple[float, ...]:
        return tuple(value for point in self.points for value in (point.x, point.y, point.z))

def normalize_landmarks(points: Iterable[object], *, handedness: str | None = None, timestamp_ns: int | None = None) -> HandLandmarks:
    rows = tuple(points)
    if len(rows) != LANDMARK_COUNT:
        raise LandmarkError(f"expected exactly {LANDMARK_COUNT} landmarks")
    normalized = []
    for index, row in enumerate(rows):
        try:
            x = float(getattr(row, "x", row[0])); y = float(getattr(row, "y", row[1])); z = float(getattr(row, "z", row[2] if len(row) > 2 else 0.0))
        except (IndexError, TypeError, ValueError, AttributeError) as exc:
            raise LandmarkError(f"landmark {index} is not an x/y/z point") from exc
        if not all(map(lambda value: value == value and abs(value) != float("inf"), (x, y, z))):
            raise LandmarkError(f"landmark {index} contains a non-finite value")
        normalized.append(Landmark(x, y, z))
    if handedness not in (None, "Left", "Right"):
        raise LandmarkError("handedness must be Left, Right, or None")
    if timestamp_ns is not None and timestamp_ns < 0:
        raise LandmarkError("timestamp_ns must be non-negative")
    return HandLandmarks(tuple(normalized), handedness, timestamp_ns)
