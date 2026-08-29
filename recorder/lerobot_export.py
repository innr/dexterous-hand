"""Optional exporter from canonical HDF5 episodes to LeRobotDataset."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .hdf5_reader import EpisodeReader
from .schema import CANONICAL_JOINT_NAMES, OptionalDependencyError


def _load_lerobot_dataset():
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise OptionalDependencyError(
                "LeRobot export requires `lerobot`; install the optional data extras"
            ) from exc
    return LeRobotDataset


def _episode_paths(root: Path) -> list[Path]:
    if root.is_file():
        return [root]
    return sorted((root / "episodes").glob("*.h5"))


def export_to_lerobot(
    source: str | Path,
    output: str | Path,
    *,
    repo_id: str | None = None,
    fps: float = 30.0,
    robot_type: str = "leap_hand",
) -> Path:
    """Export valid action/state rows to a LeRobotDataset directory."""

    if fps <= 0:
        raise ValueError("fps must be positive")
    dataset_cls = _load_lerobot_dataset()
    source_path = Path(source)
    output_path = Path(output)
    episode_paths = _episode_paths(source_path)
    if not episode_paths:
        raise ValueError(f"no .h5 episodes found under {source_path}")

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (16,),
            "names": list(CANONICAL_JOINT_NAMES),
        },
        "observation.velocity": {
            "dtype": "float32",
            "shape": (16,),
            "names": list(CANONICAL_JOINT_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (16,),
            "names": list(CANONICAL_JOINT_NAMES),
        },
    }
    dataset = dataset_cls.create(
        repo_id=repo_id or output_path.name,
        root=output_path,
        fps=fps,
        robot_type=robot_type,
        features=features,
        use_videos=False,
    )
    try:
        saved_episodes = 0
        for episode_path in episode_paths:
            episode_frames = 0
            with EpisodeReader(episode_path) as reader:
                for frame in reader:
                    if not frame.valid:
                        continue
                    dataset.add_frame(
                        {
                            "observation.state": np.asarray(
                                frame.observation_position_rad, dtype=np.float32
                            ),
                            "observation.velocity": np.asarray(
                                frame.observation_velocity_rad_s, dtype=np.float32
                            ),
                            "action": np.asarray(frame.action_position_rad, dtype=np.float32),
                            "task": reader.task or "dexterous_hand",
                        }
                    )
                    episode_frames += 1
            if episode_frames == 0:
                continue
            dataset.save_episode()
            saved_episodes += 1
        if saved_episodes == 0:
            raise ValueError("all recorded frames were invalid; no LeRobot episode created")
    finally:
        # v3 writes buffered parquet metadata only after finalize().
        dataset.finalize()
    return output_path
