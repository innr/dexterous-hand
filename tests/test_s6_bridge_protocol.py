"""S6-01 protocol integration: no MuJoCo dependency required."""
from __future__ import annotations
import math
from pathlib import Path
import subprocess
import sys


def step(time_us: int, target: float = 0.3, q: float = 0.0) -> str:
    values = [time_us] + [q] * 16 + [0.0] * 16 + [target] * 16
    return "STEP " + " ".join(str(v) for v in values)


def main(executable: str) -> None:
    proc = subprocess.Popen([str(Path(executable).resolve())], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, bufsize=1)
    assert proc.stdin and proc.stdout
    assert proc.stdout.readline().strip() == "READY 16 S6-01"
    commands = []
    for tick in range(10):
        proc.stdin.write(step(tick * 10000) + "\n")
        proc.stdin.flush()
        response = proc.stdout.readline().strip().split()
        assert len(response) == 17 and response[0] == "OK", response
        commands.append([float(v) for v in response[1:]])
        assert all(math.isfinite(v) and -math.pi / 2 <= v <= math.pi / 2
                   for v in commands[-1])
    assert 0.0 <= commands[0][0] < 0.001  # initial ramp near measured position, no jump
    assert commands[-1][0] > commands[0][0]
    assert commands[-1][0] < 0.3  # bounded ramp, not target pass-through
    proc.stdin.write(step(90000) + "\n")  # duplicate sim time -> latched STOP
    proc.stdin.flush()
    assert proc.stdout.readline().strip() == "STOP"
    proc.stdin.write(step(100000) + "\n")
    proc.stdin.flush()
    assert proc.stdout.readline().strip() == "STOP"
    proc.stdin.write("QUIT\n")
    proc.stdin.flush()
    assert proc.stdout.readline().strip() == "BYE"
    assert proc.wait(timeout=3) == 0
    print("S6-01 protocol and fail-closed tests passed")


if __name__ == "__main__":
    main(sys.argv[1])
