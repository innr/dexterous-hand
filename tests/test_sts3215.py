from hardware.sts3215 import (
    Instruction,
    STS3215ProtocolError,
    decode_status_packet,
    encode_packet,
    radians_to_ticks,
    ticks_to_radians,
)


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
