"""Scan a range of STS3215 IDs without enabling torque or moving a servo."""

from __future__ import annotations

import argparse

from .sts3215 import STS3215Bus, STS3215Error


def parse_ids(value: str) -> list[int]:
    ids = [int(item.strip()) for item in value.split(",") if item.strip()]
    if not ids or any(not 0 <= item <= 253 for item in ids):
        raise argparse.ArgumentTypeError("IDs must be comma-separated integers from 0 to 253")
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--ids", type=parse_ids, default=list(range(20)))
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=float, default=0.05)
    args = parser.parse_args()
    try:
        with STS3215Bus(args.port, baudrate=args.baudrate, timeout=args.timeout) as bus:
            found = bus.scan(args.ids)
    except STS3215Error as exc:
        raise SystemExit(f"scan failed: {exc}") from exc
    print("found IDs:", ", ".join(map(str, found)) if found else "none")


if __name__ == "__main__":
    main()

