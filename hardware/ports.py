"""List serial ports without opening or writing to them."""

from __future__ import annotations

import argparse


def list_ports() -> list[dict[str, str | int | None]]:
    try:
        from serial.tools import list_ports
    except ImportError as exc:  # pragma: no cover - depends on installation
        raise RuntimeError("install pyserial to enumerate serial ports") from exc
    result = []
    for port in sorted(list_ports.comports(), key=lambda item: item.device):
        result.append(
            {
                "device": port.device,
                "description": port.description or "",
                "manufacturer": port.manufacturer,
                "vid": port.vid,
                "pid": port.pid,
                "serial_number": port.serial_number,
            }
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = parser.parse_args()
    ports = list_ports()
    if args.json:
        import json
        print(json.dumps(ports, indent=2))
    elif ports:
        for port in ports:
            print(f"{port['device']}: {port['description']}")
    else:
        print("no serial ports found")


if __name__ == "__main__":
    main()
