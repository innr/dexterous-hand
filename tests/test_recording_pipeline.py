from __future__ import annotations

import json

import numpy as np
import pytest

from recorder.hdf5_reader import EpisodeReader
from recorder.hdf5_writer import EpisodeRecorder
from recorder.lerobot_export import export_to_lerobot
from recorder.replay import MemoryCommandSink, replay_episode
from recorder.schema import (
    CANONICAL_JOINT_NAMES,
    DataIntegrityError,
    EpisodeFrame,
    OptionalDependencyError,
    SampleValidationError,
)


def make_frame(timestamp_ns: int, value: float, *, valid: bool = True) -> EpisodeFrame:
    vector = np.full(16, value, dtype=np.float32)
    return EpisodeFrame(
        timestamp_ns=timestamp_ns,
        action_position_rad=vector,
        observation_position_rad=vector + 0.1,
        observation_velocity_rad_s=vector + 0.2,
        valid=valid,
        valid_reason=0 if valid else 3,
    )


def record_episode(tmp_path, *, episode_id="episode_0000"):
    recorder = EpisodeRecorder(tmp_path, episode_id, chunk_size=2)
    recorder.start_episode(task="open_hand", metadata={"backend": "virtual"})
    recorder.append(make_frame(1_000_000_000, 0.0))
    recorder.append(make_frame(1_033_000_000, 0.2))
    recorder.append(make_frame(1_066_000_000, 0.4, valid=False))
    path = recorder.finalize()
    return path


def test_hdf5_round_trip_and_manifest(tmp_path):
    path = record_episode(tmp_path)
    assert path.exists()
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["joint_order"] == list(CANONICAL_JOINT_NAMES)
    assert manifest["episodes"][0]["frame_count"] == 3

    with EpisodeReader(path) as reader:
        assert reader.frame_count == 3
        assert reader.task == "open_hand"
        assert reader.metadata == {"backend": "virtual"}
        frames = list(reader)
    assert frames[1].timestamp_ns == 1_033_000_000
    np.testing.assert_allclose(frames[1].action_position_rad, 0.2)
    assert not frames[2].valid


def test_append_rejects_invalid_shape_and_timestamp(tmp_path):
    recorder = EpisodeRecorder(tmp_path, "bad")
    recorder.start_episode()
    with pytest.raises(SampleValidationError, match="shape"):
        recorder.append(
            EpisodeFrame(1, [0.0] * 15, [0.0] * 16, [0.0] * 16)
        )
    recorder.append(make_frame(2, 0.0))
    with pytest.raises(SampleValidationError, match="strictly increasing"):
        recorder.append(make_frame(2, 0.1))
    recorder.close_without_finalize()


def test_reader_rejects_unfinalized_episode(tmp_path):
    recorder = EpisodeRecorder(tmp_path, "unfinished")
    path = recorder.start_episode()
    recorder.append(make_frame(1, 0.0))
    recorder.close_without_finalize()
    with pytest.raises(DataIntegrityError, match="not finalized"):
        EpisodeReader(path)


def test_replay_skips_invalid_and_sends_action(tmp_path):
    path = record_episode(tmp_path)
    sink = MemoryCommandSink()
    frames = list(replay_episode(path, sink=sink, speed=0.0))
    assert len(frames) == 2
    assert len(sink.commands) == 2
    np.testing.assert_allclose(sink.commands[-1], 0.2)


def test_replay_timing_and_include_invalid(tmp_path):
    path = record_episode(tmp_path)
    delays = []
    frames = list(
        replay_episode(
            path,
            speed=1.0,
            skip_invalid=False,
            sleep=delays.append,
        )
    )
    assert len(frames) == 3
    assert delays == pytest.approx([0.033, 0.033], abs=1e-6)


def test_lerobot_export_dependency_error(tmp_path):
    path = record_episode(tmp_path)
    with pytest.raises(OptionalDependencyError, match="lerobot"):
        export_to_lerobot(path, tmp_path / "lerobot")
