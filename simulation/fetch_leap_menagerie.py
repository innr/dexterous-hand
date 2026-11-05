"""Download pinned, upstream MIT-licensed LEAP Hand MJCF and mesh assets.

Explicit setup step; runtime simulation never downloads code or assets.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from urllib.request import urlopen

REVISION = "98a70c569c4c1b6da467a20012f766ee1f8dc521"
ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "third_party" / "mujoco_menagerie" / "leap_hand"
FILES = (
    "right_hand.xml", "LICENSE", "README.md",
    "assets/palm_right.obj", "assets/base.obj", "assets/proximal.obj",
    "assets/medial.obj", "assets/distal.obj", "assets/tip.obj",
    "assets/thumb_base.obj", "assets/thumb_proximal.obj",
    "assets/thumb_distal.obj", "assets/thumb_tip.obj",
)


def fetch(destination: Path = DEST) -> Path:
    base = ("https://raw.githubusercontent.com/google-deepmind/"
            f"mujoco_menagerie/{REVISION}/leap_hand/")
    revision_file = destination / "SOURCE_REVISION"
    revision_matches = (revision_file.is_file() and
                        revision_file.read_text(encoding="ascii").strip() == REVISION)
    for relative in FILES:
        path = destination / relative
        if revision_matches and path.is_file() and path.stat().st_size:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        with urlopen(base + relative, timeout=30) as response:
            content = response.read()
        if not content:
            raise RuntimeError(f"empty upstream asset: {relative}")
        path.write_bytes(content)
        print(f"Fetched {relative}: {len(content)} bytes")
    (destination / "SOURCE_REVISION").write_text(REVISION + "\n", encoding="ascii")
    return destination / "right_hand.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dest", type=Path, default=DEST)
    args = parser.parse_args()
    print(f"Official LEAP model: {fetch(args.dest.resolve())}")


if __name__ == "__main__":
    main()
