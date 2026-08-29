"""Append-only HDF5 episode writer."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Mapping

from .schema import (
    CANONICAL_JOINT_NAMES,
    HDF5_SCHEMA_VERSION,
    EpisodeFrame,
    SampleValidationError,
    metadata_json,
    normalize_frame,
)


def _require_h5py():
    try:
        import h5py
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError(
            "HDF5 recording requires h5py; install with `pip install 'h5py>=3.10,<4'`"
        ) from exc
    return h5py


class EpisodeRecorder:
    """Record one episode into ``root/episodes/<episode_id>.h5``."""

    def __init__(
        self,
        root: str | Path,
        episode_id: str | None = None,
        *,
        chunk_size: int = 256,
    ) -> None:
        if chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        self.root = Path(root)
        self.episode_id = episode_id
        self.chunk_size = int(chunk_size)
        self._file = None
        self._path: Path | None = None
        self._frame_count = 0
        self._last_timestamp_ns: int | None = None
        self._task = ""
        self._metadata: dict[str, Any] = {}

    @property
    def path(self) -> Path:
        if self._path is None:
            raise RuntimeError("episode has not started")
        return self._path

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def _next_episode_id(self) -> str:
        episodes_dir = self.root / "episodes"
        episodes_dir.mkdir(parents=True, exist_ok=True)
        used = {path.stem for path in episodes_dir.glob("*.h5")}
        index = 0
        while f"episode_{index:04d}" in used:
            index += 1
        return f"episode_{index:04d}"

    def start_episode(
        self,
        *,
        task: str = "",
        metadata: Mapping[str, Any] | None = None,
    ) -> Path:
        if self._file is not None:
            raise RuntimeError("episode is already open")
        episode_id = self.episode_id or self._next_episode_id()
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", episode_id):
            raise ValueError("episode_id may contain only letters, numbers, _, ., and -")
        self.root.mkdir(parents=True, exist_ok=True)
        episodes_dir = self.root / "episodes"
        episodes_dir.mkdir(parents=True, exist_ok=True)
        self._path = episodes_dir / f"{episode_id}.h5"
        self._task = str(task)
        self._metadata = dict(metadata or {})
        h5py = _require_h5py()
        self._file = h5py.File(self._path, "x")
        self._file.attrs["schema_version"] = HDF5_SCHEMA_VERSION
        self._file.attrs["joint_count"] = len(CANONICAL_JOINT_NAMES)
        self._file.attrs["joint_order_json"] = json.dumps(
            CANONICAL_JOINT_NAMES, ensure_ascii=False
        )
        self._file.attrs["position_unit"] = "rad"
        self._file.attrs["time_unit"] = "ns"
        self._file.attrs["task"] = self._task
        self._file.attrs["metadata_json"] = metadata_json(self._metadata)
        self._file.attrs["finalized"] = False

        frames = self._file.create_group("frames")
        shape_1d = (0,)
        maxshape_1d = (None,)
        shape_2d = (0, len(CANONICAL_JOINT_NAMES))
        maxshape_2d = (None, len(CANONICAL_JOINT_NAMES))
        frames.create_dataset(
            "timestamp_ns", shape=shape_1d, maxshape=maxshape_1d,
            dtype="<i8", chunks=(self.chunk_size,)
        )
        for name in (
            "action_position_rad",
            "observation_position_rad",
            "observation_velocity_rad_s",
        ):
            frames.create_dataset(
                name, shape=shape_2d, maxshape=maxshape_2d, dtype="<f4",
                chunks=(self.chunk_size, len(CANONICAL_JOINT_NAMES)),
                compression="gzip", compression_opts=4,
            )
        for name in ("valid", "valid_reason"):
            frames.create_dataset(
                name, shape=shape_1d, maxshape=maxshape_1d,
                dtype="u1", chunks=(self.chunk_size,)
            )
        return self._path

    def append(self, frame: EpisodeFrame) -> None:
        if self._file is None:
            raise RuntimeError("call start_episode before append")
        normalized = normalize_frame(frame)
        if (
            self._last_timestamp_ns is not None
            and normalized.timestamp_ns <= self._last_timestamp_ns
        ):
            raise SampleValidationError(
                "timestamp_ns must be strictly increasing within an episode"
            )

        index = self._frame_count
        frames = self._file["frames"]
        for name in (
            "timestamp_ns", "action_position_rad", "observation_position_rad",
            "observation_velocity_rad_s", "valid", "valid_reason",
        ):
            dataset = frames[name]
            dataset.resize((index + 1,) + dataset.shape[1:])
        frames["timestamp_ns"][index] = normalized.timestamp_ns
        frames["action_position_rad"][index] = normalized.action_position_rad
        frames["observation_position_rad"][index] = normalized.observation_position_rad
        frames["observation_velocity_rad_s"][index] = normalized.observation_velocity_rad_s
        frames["valid"][index] = int(normalized.valid)
        frames["valid_reason"][index] = normalized.valid_reason
        self._frame_count += 1
        self._last_timestamp_ns = normalized.timestamp_ns

    def finalize(self) -> Path:
        if self._file is None:
            if self._path is None:
                raise RuntimeError("episode has not started")
            return self._path
        self._file.attrs["frame_count"] = self._frame_count
        self._file.attrs["finalized"] = True
        self._file.flush()
        self._file.close()
        self._file = None
        self._write_manifest()
        return self.path

    def _write_manifest(self) -> None:
        manifest_path = self.root / "manifest.json"
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        else:
            manifest = {
                "schema_version": HDF5_SCHEMA_VERSION,
                "joint_order": list(CANONICAL_JOINT_NAMES),
                "position_unit": "rad",
                "time_unit": "ns",
                "episodes": [],
            }
        episodes = [
            entry for entry in manifest.get("episodes", [])
            if entry["id"] != self.path.stem
        ]
        episodes.append({
            "id": self.path.stem,
            "path": str(self.path.relative_to(self.root)),
            "frame_count": self._frame_count,
            "task": self._task,
            "metadata": self._metadata,
        })
        manifest["episodes"] = sorted(episodes, key=lambda entry: entry["id"])
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def close_without_finalize(self) -> None:
        """Close an interrupted file while preserving ``finalized=False``."""
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None

    def __enter__(self) -> "EpisodeRecorder":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if exc_type is None:
            self.finalize()
        else:
            self.close_without_finalize()
