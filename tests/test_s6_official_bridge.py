"""S6-02 official MJCF smoke, time/pause/reset/stop/epoch safety regression."""
from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.fetch_leap_menagerie import DEST
from simulation.s6_official_bridge import OfficialPlant, SimSession, load_contract


def rejects(fn) -> None:
    try:
        fn()
    except (ValueError, RuntimeError):
        return
    raise AssertionError("expected rejected unsafe operation")


def main(executable: Path) -> None:
    model_path = DEST / "right_hand.xml"
    assert model_path.is_file(), "Run: python -m simulation.fetch_leap_menagerie"
    mapping = load_contract()
    assert len(mapping) == 16
    plant = OfficialPlant(model_path)
    assert plant.model.njnt == 16 and plant.model.nu == 16
    assert len(plant.names) == 16 and len(set(plant.names)) == 16
    side = [1.10 if i % 4 == 0 else 0.2 for i in range(16)]
    rejects(lambda: plant.apply(side))
    golden = Path(__file__).resolve().parents[1] / "tests" / "golden" / "official_plant_limits_rad.json"
    if golden.is_file():
        import json
        expected = json.loads(golden.read_text(encoding="utf-8"))["limits"]
        assert [[plant.lower[i], plant.upper[i]] for i in range(16)] == expected
    session = SimSession(plant, executable)
    session.session_id = 99
    try:
        names = plant.names
        initial_time = plant.time_us
        rejects(lambda: session.set_target(names[:-1], [0.2] * 15,
                                           epoch=0, stamp_us=initial_time))
        rejects(lambda: session.set_target(names, [math.nan] * 16,
                                           epoch=0, stamp_us=initial_time))
        rejects(lambda: session.set_target(names, [0.2] * 16,
                                           epoch=1, stamp_us=initial_time))
        rejects(lambda: session.set_target(names, [2.0] * 16,
                                           epoch=0, stamp_us=initial_time))
        for _ in range(30):
            session.set_target(names[::-1], [0.2] * 16,
                               epoch=0, stamp_us=plant.time_us)
            session.tick()
        assert session.state == "NORMAL"
        assert plant.time_us == 300_000, plant.time_us
        position, _ = plant.feedback()
        assert max(abs(p) for p in position) > 1e-5
        session.pause(True)
        assert session.state == "PAUSED"
        frozen_time = plant.time_us
        for _ in range(10):
            session.tick()
        assert plant.time_us == frozen_time
        rejects(lambda: session.set_target(names, [0.1] * 16,
                                           epoch=0, stamp_us=plant.time_us))
        session.pause(False)
        assert session.epoch == 1  # resume barrier invalidates pre-pause DDS messages
        assert session.state == "READY" and session.target is None
        session.tick()  # READY remains disarmed until new authorization
        assert plant.time_us == frozen_time
        session.set_target(names, [0.1] * 16, epoch=1, stamp_us=plant.time_us)
        session.tick()
        assert session.state == "NORMAL"
        # Sim-time TTL: no fresh commands for >100ms must stop and latch.
        for _ in range(10):
            session.tick()
        rejects(session.tick)
        assert session.state == "SAFE_STOP"
        session.tick()
        rejects(lambda: session.set_target(names, [0.1] * 16,
                                           epoch=0, stamp_us=plant.time_us))
        session.reset()
        assert session.epoch == 2 and session.state == "READY"
        assert plant.time_us == 0 and session.target is None
        rejects(lambda: session.set_target(names, [0.1] * 16,
                                           epoch=0, stamp_us=0))
        session.set_target(names, [0.1] * 16, epoch=2, stamp_us=0)
        session.tick()
        assert session.state == "NORMAL"
        session.stop()
        assert session.state == "SAFE_STOP"
        print("S6-02 official LEAP model, pause, reset, epoch and TTL passed")
    finally:
        session.close()


if __name__ == "__main__":
    main(Path(sys.argv[1]).resolve())
