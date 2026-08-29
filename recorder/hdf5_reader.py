"""Validated HDF5 episode reader."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterator

import numpy as np

from .schema import (
    CANONICAL_JOINT_NAMES,
    HDF5_SCHEMA_VERSION,
    DataIntegrityError,
    EpisodeFrame,
)


def _require_h5py():
    try:
        import h5py
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError(
            "HDF5 replay requires h5py; install with `pip install 'h5py>=3.10,<4'`"
        ) from exc
    return h5py


def _attr_text(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


class EpisodeReader:
    """Read and validate one finalized HDF5 episode."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        h5py = _require_h5py()
        try:
            self._file = h5py.File(self.path, "r")
        except OSError as exc:
            raise DataIntegrityError(f"cannot open HDF5 episode: {self.path}") from exc
        try:
            self._validate()
        except Exception:
            self._file.close()
            raise

    def _validate(self) -> None:
        attrs = self._file.attrs
        if int(attrs.get("schema_version", -1)) != HDF5_SCHEMA_VERSION:
            raise DataIntegrityError("unsupported or missing HDF5 schema version")
        if not bool(attrs.get("finalized", False)):
            raise DataIntegrityError("episode was not finalized")
        if int(attrs.get("joint_count", -1)) != len(CANONICAL_JOINT_NAMES):
            raise DataIntegrityError("episode joint_count is not 16")
        try:
            joint_order = tuple(json.loads(_attr_text(attrs["joint_order_json"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise DataIntegrityError("episode joint order metadata is invalid") from exc
        if joint_order != CANONICAL_JOINT_NAMES:
            raise DataIntegrityError("episode joint order does not match LEAP canonical order")
        if _attr_text(attrs.get("position_unit", "")) != "rad":
            raise DataIntegrityError("episode position unit must be rad")
        if _attr_text(attrs.get("time_unit", "")) != "ns":
            raise DataIntegrityError("episode time unit must be ns")
        if "frames" not in self._file:
            raise DataIntegrityError("episode is missing frames group")
        frames = self._file["frames"]
        required = {
            "timestamp_ns",
            "action_position_rad",
            "observation_position_rad",
            "observation_velocity_rad_s",
            "valid",
            "valid_reason",
        }
        if not required.issubset(frames.keys()):
            raise DataIntegrityError("episode is missing one or more frame datasets")
        lengths = {int(frames[name].shape[0]) for name in required}
        if len(lengths) != 1:
            raise DataIntegrityError("episode frame datasets have inconsistent lengths")
        frame_count = next(iter(lengths))
        for name in (
            "action_position_rad",
            "observation_position_rad",
            "observation_velocity_rad_s",
        ):
            if frames[name].shape[1:] != (len(CANONICAL_JOINT_NAMES),):
                raise DataIntegrityError(f"{name} must have shape (N, 16)")
        timestamps = np.asarray(frames["timestamp_ns"][:])
        if timestamps.size and np.any(np.diff(timestamps) <= 0):
            raise DataIntegrityError("episode timestamps must be strictly increasing")
        for name in (
            "action_position_rad",
            "observation_position_rad",
            "observation_velocity_rad_s",
        ):
            if not np.isfinite(frames[name][:]).all():
                raise DataIntegrityError(f"episode dataset {name} contains non-finite values")
        expected_count = int(attrs.get("frame_count", frame_count))
        if expected_count != frame_count:
            raise DataIntegrityError("episode frame_count metadata is inconsistent")

    @property
    def frame_count(self) -> int:
        return int(self._file["frames"]["timestamp_ns"].shape[0])

    @property
    def task(self) -> str:
        return _attr_text(self._file.attrs.get("task", ""))

    @property
    def metadata(self) -> dict:
        raw = _attr_text(self._file.attrs.get("metadata_json", "{}"))
        return dict(json.loads(raw))

    def frame(self, index: int) -> EpisodeFrame:
        if not 0 <= index < self.frame_count:
            raise IndexError(index)
        frames = self._file["frames"]
        return EpisodeFrame(
            timestamp_ns=int(frames["timestamp_ns"][index]),
            action_position_rad=np.asarray(frames["action_position_rad"][index]),
            observation_position_rad=np.asarray(frames["observation_position_rad"][index]),
            observation_velocity_rad_s=np.asarray(
                frames["observation_velocity_rad_s"][index]
            ),
            valid=bool(frames["valid"][index]),
            valid_reason=int(frames["valid_reason"][index]),
        )

    def __iter__(self) -> Iterator[EpisodeFrame]:
        for index in range(self.frame_count):
            yield self.frame(index)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def __enter__(self) -> "EpisodeReader":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
