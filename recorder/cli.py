"""Command-line entry points for recording, replay, and export."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .hdf5_writer import EpisodeRecorder
from .lerobot_export import export_to_lerobot
from .replay import MemoryCommandSink, replay_episode
from .schema import EpisodeFrame


def record_jsonl(args: argparse.Namespace) -> int:
    recorder = EpisodeRecorder(args.output, args.episode_id)
    recorder.start_episode(task=args.task, metadata={"source": "jsonl"})
    with Path(args.input).open(encoding="utf-8") as input_file:
        for line_number, line in enumerate(input_file, start=1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
                recorder.append(
                    EpisodeFrame(
                        timestamp_ns=payload["timestamp_ns"],
                        action_position_rad=payload["action_position_rad"],
                        observation_position_rad=payload["observation_position_rad"],
                        observation_velocity_rad_s=payload["observation_velocity_rad_s"],
                        valid=payload.get("valid", True),
                        valid_reason=payload.get("valid_reason", 0),
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                recorder.close_without_finalize()
                raise ValueError(f"invalid JSONL frame at line {line_number}") from exc
    print(recorder.finalize())
    return 0


def replay(args: argparse.Namespace) -> int:
    sink = MemoryCommandSink() if args.backend == "memory" else None
    frames = list(
        replay_episode(
            args.input,
            sink=sink,
            speed=args.speed,
            skip_invalid=not args.include_invalid,
        )
    )
    print(f"replayed_frames={len(frames)}")
    if sink is not None:
        print(f"sent_commands={len(sink.commands)}")
    return 0


def export(args: argparse.Namespace) -> int:
    output = export_to_lerobot(args.input, args.output, repo_id=args.repo_id, fps=args.fps)
    print(output)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="dexterous-hand-data")
    subparsers = parser.add_subparsers(dest="command", required=True)

    record_parser = subparsers.add_parser("record-jsonl")
    record_parser.add_argument("--input", required=True)
    record_parser.add_argument("--output", required=True)
    record_parser.add_argument("--episode-id", default=None)
    record_parser.add_argument("--task", default="")
    record_parser.set_defaults(function=record_jsonl)

    replay_parser = subparsers.add_parser("replay")
    replay_parser.add_argument("--input", required=True)
    replay_parser.add_argument("--speed", type=float, default=0.0)
    replay_parser.add_argument("--backend", choices=("none", "memory"), default="none")
    replay_parser.add_argument("--include-invalid", action="store_true")
    replay_parser.set_defaults(function=replay)

    export_parser = subparsers.add_parser("export-lerobot")
    export_parser.add_argument("--input", required=True)
    export_parser.add_argument("--output", required=True)
    export_parser.add_argument("--repo-id", default=None)
    export_parser.add_argument("--fps", type=float, default=30.0)
    export_parser.set_defaults(function=export)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.function(args)


if __name__ == "__main__":
    raise SystemExit(main())
