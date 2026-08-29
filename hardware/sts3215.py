"""Minimal, dependency-light driver for the FEETECH STS3215 bus servo.

The driver speaks the SCS/STS half-duplex packet protocol directly.  It does
not enable torque or command motion unless the caller explicitly invokes
those methods.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import threading
from typing import Any, Callable, Iterable


class Instruction(IntEnum):
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06
    SYNC_WRITE = 0x83


class Register(IntEnum):
    """Frequently used STS3215 control-table addresses."""

    ID = 5
    BAUD_RATE = 6
    TORQUE_ENABLE = 40
    ACCELERATION = 41
    GOAL_POSITION = 42
    GOAL_TIME = 44
    GOAL_SPEED = 46
    TORQUE_LIMIT = 48
    PRESENT_POSITION = 56
    PRESENT_SPEED = 58
    PRESENT_LOAD = 60
    PRESENT_VOLTAGE = 62
    PRESENT_TEMPERATURE = 63
    MOVING = 66


class STS3215Error(Exception):
    """Base exception for protocol, transport, and servo errors."""


class STS3215Timeout(STS3215Error):
    """The servo did not return a complete status packet in time."""


class STS3215ProtocolError(STS3215Error):
    """A malformed or invalid packet was received."""


class STS3215ServoError(STS3215Error):
    """The servo returned a non-zero status/error byte."""

    def __init__(self, servo_id: int, error: int) -> None:
        super().__init__(f"servo {servo_id} returned error 0x{error:02x}")
        self.servo_id = servo_id
        self.error = error


@dataclass(frozen=True)
class StatusPacket:
    servo_id: int
    error: int
    params: bytes


def _checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def encode_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    """Encode an STS instruction packet."""
    if not 0 <= servo_id <= 254:
        raise ValueError("servo_id must be in [0, 254]")
    params = bytes(params)
    length = len(params) + 2  # instruction + checksum
    body = bytes((servo_id, length, int(instruction) & 0xFF)) + params
    return b"\xff\xff" + body + bytes((_checksum(body),))


def decode_status_packet(packet: bytes, expected_id: int | None = None) -> StatusPacket:
    """Validate and decode a returned status packet."""
    packet = bytes(packet)
    if len(packet) < 6 or packet[:2] != b"\xff\xff":
        raise STS3215ProtocolError("invalid status header")
    declared_length = packet[3]
    if len(packet) != declared_length + 4:
        raise STS3215ProtocolError(
            f"invalid packet length: declared {declared_length}, received {len(packet)}"
        )
    if _checksum(packet[2:-1]) != packet[-1]:
        raise STS3215ProtocolError("checksum mismatch")
    servo_id = packet[2]
    if expected_id is not None and servo_id != expected_id:
        raise STS3215ProtocolError(f"expected response from {expected_id}, got {servo_id}")
    return StatusPacket(servo_id=servo_id, error=packet[4], params=packet[5:-1])


def ticks_to_radians(ticks: int, *, center_ticks: int = 2048) -> float:
    """Convert a 12-bit absolute encoder value to a centered angle."""
    return (int(ticks) - center_ticks) * 2.0 * 3.141592653589793 / 4096.0


def radians_to_ticks(radians: float, *, center_ticks: int = 2048) -> int:
    """Convert a centered angle to a clamped 12-bit encoder value."""
    ticks = round(center_ticks + radians * 4096.0 / (2.0 * 3.141592653589793))
    return max(0, min(4095, ticks))


class STS3215Bus:
    """Synchronous STS3215 bus client.

    ``serial_factory`` is injectable for deterministic tests.  In production,
    pyserial is imported lazily and the port is opened at 1 Mbps by default.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial_factory = serial_factory
        self._serial: Any | None = None
        self._lock = threading.Lock()

    def open(self) -> None:
        if self._serial is not None:
            return
        if self._serial_factory is None:
            try:
                import serial
            except ImportError as exc:  # pragma: no cover - exercised on user setup
                raise STS3215Error("install pyserial to use real hardware") from exc
            if not self.port:
                raise ValueError("port is required for a real serial connection")
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
        else:
            self._serial = self._serial_factory(
                port=self.port, baudrate=self.baudrate, timeout=self.timeout
            )
            if hasattr(self._serial, "open") and getattr(self._serial, "is_open", True) is False:
                self._serial.open()

    def close(self) -> None:
        if self._serial is not None and hasattr(self._serial, "close"):
            self._serial.close()
        self._serial = None

    def __enter__(self) -> "STS3215Bus":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _read_exact(self, size: int) -> bytes:
        assert self._serial is not None
        result = bytearray()
        while len(result) < size:
            chunk = self._serial.read(size - len(result))
            if not chunk:
                raise STS3215Timeout(f"timed out reading {size} bytes")
            result.extend(chunk)
        return bytes(result)

    def _request(self, servo_id: int, instruction: int, params: bytes = b"") -> StatusPacket:
        self.open()
        assert self._serial is not None
        if servo_id == 254:
            raise ValueError("broadcast writes do not have a status response")
        with self._lock:
            if hasattr(self._serial, "reset_input_buffer"):
                self._serial.reset_input_buffer()
            self._serial.write(encode_packet(servo_id, instruction, params))
            self._serial.flush() if hasattr(self._serial, "flush") else None
            header = self._read_exact(4)
            remainder = self._read_exact(header[3])
            status = decode_status_packet(header + remainder, expected_id=servo_id)
        if status.error:
            raise STS3215ServoError(status.servo_id, status.error)
        return status

    def ping(self, servo_id: int) -> bool:
        self._request(servo_id, Instruction.PING)
        return True

    def scan(self, servo_ids: Iterable[int] = range(0, 20)) -> list[int]:
        """Ping IDs and return the ones that answer, in ascending order."""
        found: list[int] = []
        for servo_id in servo_ids:
            try:
                self.ping(int(servo_id))
            except STS3215Timeout:
                continue
            found.append(int(servo_id))
        return found

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        if not 0 <= address <= 255 or not 1 <= size <= 255:
            raise ValueError("address must be 0..255 and size must be 1..255")
        status = self._request(servo_id, Instruction.READ, bytes((address, size)))
        if len(status.params) != size:
            raise STS3215ProtocolError(
                f"read returned {len(status.params)} bytes, expected {size}"
            )
        return status.params

    def write(self, servo_id: int, address: int, data: bytes) -> None:
        if not 0 <= address <= 255 or not data:
            raise ValueError("address must be 0..255 and data must not be empty")
        self._request(servo_id, Instruction.WRITE, bytes((address,)) + bytes(data))

    def read_position_ticks(self, servo_id: int) -> int:
        return int.from_bytes(self.read(servo_id, Register.PRESENT_POSITION, 2), "little")

    def read_voltage(self, servo_id: int) -> float:
        return self.read(servo_id, Register.PRESENT_VOLTAGE, 1)[0] / 10.0

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        """Explicitly enable/disable holding torque."""
        self.write(servo_id, Register.TORQUE_ENABLE, bytes((1 if enabled else 0,)))

    def write_goal_position_ticks(self, servo_id: int, ticks: int) -> None:
        """Explicitly command a position; caller must enable torque separately."""
        if not 0 <= ticks <= 4095:
            raise ValueError("STS3215 position must be in [0, 4095]")
        self.write(servo_id, Register.GOAL_POSITION, int(ticks).to_bytes(2, "little"))

