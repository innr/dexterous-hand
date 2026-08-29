"""Deterministic in-memory STS3215 serial transport for software tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping

from .sts3215 import Instruction, Register, STS3215Bus, _checksum, encode_packet


@dataclass
class VirtualSTS3215Serial:
    """A serial-like fake that speaks enough SCS/STS for the project driver.

    Fault sets are deliberately explicit so tests can exercise timeout,
    checksum and servo-error paths without touching a real serial port.
    """

    servo_ids: Iterable[int] = range(16)
    timeout_ids: set[int] = field(default_factory=set)
    checksum_error_ids: set[int] = field(default_factory=set)
    servo_errors: Mapping[int, int] = field(default_factory=dict)
    position_ticks: dict[int, int] = field(default_factory=dict)
    voltage_decivolts: dict[int, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.servo_ids = {int(servo_id) for servo_id in self.servo_ids}
        if not self.servo_ids or any(not 0 <= servo_id <= 253 for servo_id in self.servo_ids):
            raise ValueError("servo_ids must contain IDs from 0 to 253")
        self.timeout_ids = {int(value) for value in self.timeout_ids}
        self.checksum_error_ids = {int(value) for value in self.checksum_error_ids}
        self.servo_errors = {int(key): int(value) & 0xFF for key, value in self.servo_errors.items()}
        self.position_ticks = {int(key): int(value) for key, value in self.position_ticks.items()}
        self.voltage_decivolts = {int(key): int(value) for key, value in self.voltage_decivolts.items()}
        if any(not 0 <= value <= 4095 for value in self.position_ticks.values()):
            raise ValueError("position_ticks must be in [0, 4095]")
        self._response = bytearray()
        self.writes: list[bytes] = []
        self.is_open = True

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        self._response.clear()

    def flush(self) -> None:
        pass

    def read(self, size: int) -> bytes:
        chunk = bytes(self._response[:size])
        del self._response[:size]
        return chunk

    def write(self, packet: bytes) -> None:
        packet = bytes(packet)
        self.writes.append(packet)
        if len(packet) < 6 or packet[:2] != b"\xff\xff":
            return
        servo_id, length, instruction = packet[2], packet[3], packet[4]
        params = packet[5:-1]
        if len(packet) != length + 4 or _checksum(packet[2:-1]) != packet[-1]:
            return
        if instruction == Instruction.SYNC_WRITE and servo_id == 254:
            self._handle_sync_write(params)
            return
        if servo_id not in self.servo_ids or servo_id in self.timeout_ids:
            return
        error = self.servo_errors.get(servo_id, 0)
        response_params = b""
        if not error:
            if instruction == Instruction.PING:
                pass
            elif instruction == Instruction.READ:
                if len(params) != 2:
                    error = 0x40
                else:
                    response_params = self._read_register(servo_id, params[0], params[1])
            elif instruction == Instruction.WRITE:
                if len(params) < 2:
                    error = 0x40
                else:
                    self._write_register(servo_id, params[0], params[1:])
            else:
                error = 0x40
        response = self._status_packet(servo_id, error, response_params)
        if servo_id in self.checksum_error_ids:
            response = response[:-1] + bytes((response[-1] ^ 0xFF,))
        self._response.extend(response)

    def _status_packet(self, servo_id: int, error: int, params: bytes = b"") -> bytes:
        body = bytes((servo_id, len(params) + 2, error)) + bytes(params)
        return b"\xff\xff" + body + bytes((_checksum(body),))

    def _read_register(self, servo_id: int, address: int, size: int) -> bytes:
        if address == Register.PRESENT_POSITION and size == 2:
            return int(self.position_ticks.get(servo_id, 2048)).to_bytes(2, "little")
        if address == Register.PRESENT_VOLTAGE and size == 1:
            return bytes((self.voltage_decivolts.get(servo_id, 120),))
        return bytes(size)

    def _write_register(self, servo_id: int, address: int, data: bytes) -> None:
        if address == Register.GOAL_POSITION and len(data) == 2:
            self.position_ticks[servo_id] = int.from_bytes(data, "little")

    def _handle_sync_write(self, params: bytes) -> None:
        if len(params) < 2:
            return
        address, size = params[:2]
        stride = size + 1
        payload = params[2:]
        if size == 0 or len(payload) % stride:
            return
        for offset in range(0, len(payload), stride):
            servo_id = payload[offset]
            if servo_id in self.servo_ids:
                self._write_register(servo_id, address, payload[offset + 1 : offset + stride])


def virtual_bus(**kwargs: object) -> tuple[STS3215Bus, VirtualSTS3215Serial]:
    """Return a project bus wired to a virtual serial transport."""
    serial = VirtualSTS3215Serial(**kwargs)
    bus = STS3215Bus("virtual", serial_factory=lambda **_: serial)
    return bus, serial
