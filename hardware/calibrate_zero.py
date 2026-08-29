"""Capture STS3215 encoder readings for a reviewed zero calibration."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from .sts3215 import STS3215Bus, STS3215Error, ticks_to_radians


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAPPING_PATH = PROJECT_ROOT / "config" / "joint_mapping.json"


def capture_positions(bus: STS3215Bus, mapping: dict, ids: list[int], reference_ticks: int) -> dict:
    joints_by_id = {int(joint["hardware_id"]): joint for joint in mapping["joints"]}
    observations = []
    for servo_id in ids:
        ticks = bus.read_position_ticks(servo_id)
        joint = joints_by_id.get(servo_id)
        observations.append(
            {
                "hardware_id": servo_id,
                "name": joint["name"] if joint else None,
                "present_position_ticks": ticks,
                "offset_from_reference_rad": ticks_to_radians(
                    ticks, center_ticks=reference_ticks
                ),
            }
        )
    return {
        "schema_version": 1,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "reference_ticks": reference_ticks,
        "observations": observations,
        "note": "Review mechanical pose and direction before copying offsets into joint_mapping.json.",
    }


def parse_ids(value: str) -> list[int]:
    ids = [int(item.strip()) for item in value.split(",") if item.strip()]
    if not ids or any(not 0 <= item <= 253 for item in ids):
        raise argparse.ArgumentTypeError("IDs must be comma-separated integers from 0 to 253")
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="half-duplex TTL serial port")
    parser.add_argument("--ids", type=parse_ids, default=list(range(16)))
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--reference-ticks", type=int, default=2048)
    parser.add_argument("--mapping", type=Path, default=MAPPING_PATH)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    mapping = json.loads(args.mapping.read_text(encoding="utf-8"))

    try:
        with STS3215Bus(args.port, baudrate=args.baudrate, timeout=args.timeout) as bus:
            result = capture_positions(bus, mapping, args.ids, args.reference_ticks)
    except STS3215Error as exc:
        raise SystemExit(f"calibration read failed: {exc}") from exc

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"saved calibration capture to {args.output}")


if __name__ == "__main__":
    main()

