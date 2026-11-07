"""STS3215 协议编解码与总线读取的单元测试（使用假串口，无需硬件）。"""

import json
from pathlib import Path

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
    """基于预置响应字节的假串口，用于确定性测试。"""

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


GOLDEN = json.loads(
    (Path(__file__).resolve().parents[1] / "tests" / "golden" / "sts3215_packets.json")
    .read_text(encoding="utf-8"))


def test_ping_packet_matches_scs_protocol() -> None:
    """PING 指令的编码应与 SCS 协议一致。"""
    assert encode_packet(1, Instruction.PING) == bytes.fromhex(GOLDEN["ping_id1"])


def test_read_present_position_matches_golden() -> None:
    serial = FakeSerial(bytes.fromhex("ffff0704003412ae"))
    bus = STS3215Bus("fake", serial_factory=lambda **kwargs: serial)
    bus.read_position_ticks(7)
    assert serial.writes == [bytes.fromhex(GOLDEN["read_id7_present_position"])]


def test_decode_status_packet_and_checksum() -> None:
    """状态包应被正确解析，校验和校验通过。"""
    packet = bytes.fromhex(GOLDEN["status_id1_params_3412"])
    status = decode_status_packet(packet, expected_id=1)
    assert status.error == 0
    assert status.params == bytes.fromhex("3412")


def test_bad_checksum_is_rejected() -> None:
    """校验和错误的状态包应被拒绝。"""
    try:
        decode_status_packet(bytes.fromhex(GOLDEN["status_bad_checksum"]))
    except STS3215ProtocolError:
        pass
    else:
        raise AssertionError("bad checksum was accepted")


def test_position_conversion_round_trip() -> None:
    """角度与编码器读数的换算应可往返（误差不超过 1 tick）。"""
    for ticks in (0, 1024, 2048, 3072, 4095):
        assert abs(radians_to_ticks(ticks_to_radians(ticks)) - ticks) <= 1


def test_bus_reads_position_register() -> None:
    """总线应能从 PRESENT_POSITION 寄存器读回位置。"""
    # 舵机 7，状态=0，当前位置=0x1234（小端）。
    response = bytes.fromhex("ffff0704003412ae")
    serial = FakeSerial(response)
    bus = STS3215Bus("fake", serial_factory=lambda **kwargs: serial)
    assert bus.read_position_ticks(7) == 0x1234
    assert serial.writes == [bytes.fromhex("ffff0704023802b8")]
