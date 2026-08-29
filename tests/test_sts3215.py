from hardware.sts3215 import (
    Instruction,
    STS3215Bus,
    STS3215ProtocolError,
    decode_status_packet,
    encode_packet,
    radians_to_ticks,
    ticks_to_radians,
)


class FakeSerial:
    def __init__(self, response: bytes, **_: object) -> None:
        self.response = bytearray(response)
        self.writes: list[bytes] = []

    def reset_input_buffer(self) -> None:
        pass

    def write(self, packet: bytes) -> None:
        self.writes.append(packet)

    def flush(self) -> None:
        pass

    def read(self, size: int) -> bytes:
        chunk = bytes(self.response[:size])
        del self.response[:size]
        return chunk

    def close(self) -> None:
        pass


def test_ping_packet_matches_scs_protocol() -> None:
    assert encode_packet(1, Instruction.PING) == bytes.fromhex("ffff010201fb")


def test_decode_status_packet_and_checksum() -> None:
    packet = bytes.fromhex("ffff0104003412b4")
    status = decode_status_packet(packet, expected_id=1)
    assert status.error == 0
    assert status.params == bytes.fromhex("3412")


def test_bad_checksum_is_rejected() -> None:
    try:
        decode_status_packet(bytes.fromhex("ffff0104003412b5"))
    except STS3215ProtocolError:
        pass
    else:
        raise AssertionError("bad checksum was accepted")


def test_position_conversion_round_trip() -> None:
    for ticks in (0, 1024, 2048, 3072, 4095):
        assert abs(radians_to_ticks(ticks_to_radians(ticks)) - ticks) <= 1


def test_bus_reads_position_register() -> None:
    # Servo 7, status=0, present position=0x1234 (little-endian).
    response = bytes.fromhex("ffff0704003412ae")
    serial = FakeSerial(response)
    bus = STS3215Bus("fake", serial_factory=lambda **kwargs: serial)
    assert bus.read_position_ticks(7) == 0x1234
    assert serial.writes == [bytes.fromhex("ffff0704023802b8")]
