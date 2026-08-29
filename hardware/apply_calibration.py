"""Apply a reviewed calibration capture to a new mapping file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .commissioning import apply_calibration, load_capture, load_json_mapping


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="confirm that the capture was manually reviewed",
    )
    parser.add_argument(
        "--mark-verified",
        action="store_true",
        help="mark mapping verified; requires direction results for every joint",
    )
    args = parser.parse_args()
    mapping = load_json_mapping(args.mapping)
    capture = load_capture(args.capture)
    updated = apply_calibration(
        mapping,
        capture,
        confirmed=args.confirm,
        mark_verified=args.mark_verified,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(updated, indent=2) + "\n", encoding="utf-8")
    print(f"saved reviewed mapping to {args.output}")


if __name__ == "__main__":
    main()
