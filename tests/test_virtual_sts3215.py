import pytest

from hardware.sts3215 import STS3215ProtocolError, STS3215ServoError, STS3215Timeout
from hardware.virtual_sts3215 import virtual_bus


def test_virtual_bus_reads_and_writes_position() -> None:
    bus, serial = virtual_bus(position_ticks={7: 0x0234})
    with bus:
        assert bus.read_position_ticks(7) == 0x0234
        bus.write_goal_position_ticks(7, 2048)
        assert bus.read_position_ticks(7) == 2048
    assert len(serial.writes) == 3


def test_virtual_bus_simulates_timeout() -> None:
    bus, _ = virtual_bus(timeout_ids={7})
    with bus, pytest.raises(STS3215Timeout):
        bus.ping(7)


def test_virtual_bus_simulates_checksum_error() -> None:
    bus, _ = virtual_bus(checksum_error_ids={7})
    with bus, pytest.raises(STS3215ProtocolError, match="checksum"):
        bus.ping(7)


def test_virtual_bus_simulates_servo_error() -> None:
    bus, _ = virtual_bus(servo_errors={7: 0x20})
    with bus, pytest.raises(STS3215ServoError) as caught:
        bus.ping(7)
    assert caught.value.servo_id == 7
    assert caught.value.error == 0x20
