"""FEETECH STS3215 总线舵机的最小化驱动（尽量少依赖）。

本驱动直接实现 SCS/STS 半双工数据包协议。除非调用方显式调用相应方法，
否则驱动不会使能扭矩，也不会让舵机运动。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import threading
from typing import Any, Callable, Iterable


class Instruction(IntEnum):
    """STS 协议指令码。"""

    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06
    SYNC_WRITE = 0x83


class Register(IntEnum):
    """STS3215 控制表中常用的寄存器地址。"""

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
    """协议、传输与舵机错误的基类。"""


class STS3215Timeout(STS3215Error):
    """在超时时间内未收到完整状态包时抛出。"""


class STS3215ProtocolError(STS3215Error):
    """收到格式错误或非法的数据包时抛出。"""


class STS3215ServoError(STS3215Error):
    """舵机返回非零状态/错误字节时抛出。"""

    def __init__(self, servo_id: int, error: int) -> None:
        super().__init__(f"servo {servo_id} returned error 0x{error:02x}")
        self.servo_id = servo_id
        self.error = error


@dataclass(frozen=True)
class StatusPacket:
    """解析后的状态包：舵机 ID、错误字节与参数字节。"""

    servo_id: int
    error: int
    params: bytes


def _checksum(body: bytes) -> int:
    """计算校验和：对包体逐字节求和后取反，保留低 8 位。"""
    return (~sum(body)) & 0xFF


def encode_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    """按 STS 协议编码一条指令包。"""
    if not 0 <= servo_id <= 254:
        raise ValueError("servo_id must be in [0, 254]")
    params = bytes(params)
    length = len(params) + 2  # 长度字段 = 指令 1 字节 + 校验和 1 字节 + 参数
    body = bytes((servo_id, length, int(instruction) & 0xFF)) + params
    return b"\xff\xff" + body + bytes((_checksum(body),))


def decode_status_packet(packet: bytes, expected_id: int | None = None) -> StatusPacket:
    """校验并解析舵机返回的状态包。"""
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
    """把 12 位绝对编码器读数换算为以中位为零点的角度（弧度）。"""
    return (int(ticks) - center_ticks) * 2.0 * 3.141592653589793 / 4096.0


def radians_to_ticks(radians: float, *, center_ticks: int = 2048) -> int:
    """把以中位为零点的角度（弧度）换算为 12 位编码器读数（限幅到 0..4095）。"""
    ticks = round(center_ticks + radians * 4096.0 / (2.0 * 3.141592653589793))
    return max(0, min(4095, ticks))


class STS3215Bus:
    """同步 STS3215 总线客户端。

    ``serial_factory`` 可注入以支持确定性测试。生产环境下按需惰性导入
    pyserial，默认以 1 Mbps 打开串口。
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
        # 串口读写在多线程下需串行化，避免请求/响应交错。
        self._lock = threading.Lock()

    def open(self) -> None:
        """打开串口；已打开则直接返回。"""
        if self._serial is not None:
            return

        if self._serial_factory is None:
            # 真实硬件：惰性导入 pyserial。
            try:
                import serial
            except ImportError as exc:  # pragma: no cover - 仅在用户环境缺少 pyserial 时触发
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
            # 测试环境：使用注入的串口工厂。
            self._serial = self._serial_factory(
                port=self.port, baudrate=self.baudrate, timeout=self.timeout
            )
            if hasattr(self._serial, "open") and getattr(self._serial, "is_open", True) is False:
                self._serial.open()

    def close(self) -> None:
        """关闭串口（若已打开）。"""
        if self._serial is not None and hasattr(self._serial, "close"):
            self._serial.close()
        self._serial = None

    def __enter__(self) -> "STS3215Bus":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _read_exact(self, size: int) -> bytes:
        """读取固定字节数；不足则抛超时异常。"""
        assert self._serial is not None
        result = bytearray()
        while len(result) < size:
            chunk = self._serial.read(size - len(result))
            if not chunk:
                raise STS3215Timeout(f"timed out reading {size} bytes")
            result.extend(chunk)
        return bytes(result)

    def _request(self, servo_id: int, instruction: int, params: bytes = b"") -> StatusPacket:
        """发送一条指令并读取一条状态响应（内部串行化）。"""
        self.open()
        assert self._serial is not None
        if servo_id == 254:
            raise ValueError("broadcast writes do not have a status response")

        with self._lock:
            if hasattr(self._serial, "reset_input_buffer"):
                self._serial.reset_input_buffer()
            self._serial.write(encode_packet(servo_id, instruction, params))
            if hasattr(self._serial, "flush"):
                self._serial.flush()

            # 状态包：前 4 字节含长度字段，随后按长度读取剩余部分。
            header = self._read_exact(4)
            remainder = self._read_exact(header[3])
            status = decode_status_packet(header + remainder, expected_id=servo_id)

        if status.error:
            raise STS3215ServoError(status.servo_id, status.error)
        return status

    def ping(self, servo_id: int) -> bool:
        """向指定 ID 发送 PING；有响应即返回 True。"""
        self._request(servo_id, Instruction.PING)
        return True

    def scan(self, servo_ids: Iterable[int] = range(0, 20)) -> list[int]:
        """逐个 PING，返回有响应的 ID（升序）。"""
        found: list[int] = []
        for servo_id in servo_ids:
            try:
                self.ping(int(servo_id))
            except STS3215Timeout:
                continue
            found.append(int(servo_id))
        return found

    def read(self, servo_id: int, address: int, size: int) -> bytes:
        """从指定地址读取 size 字节。"""
        if not 0 <= address <= 255 or not 1 <= size <= 255:
            raise ValueError("address must be 0..255 and size must be 1..255")
        status = self._request(servo_id, Instruction.READ, bytes((address, size)))
        if len(status.params) != size:
            raise STS3215ProtocolError(
                f"read returned {len(status.params)} bytes, expected {size}"
            )
        return status.params

    def write(self, servo_id: int, address: int, data: bytes) -> None:
        """从指定地址写入若干字节。"""
        if not 0 <= address <= 255 or not data:
            raise ValueError("address must be 0..255 and data must not be empty")
        self._request(servo_id, Instruction.WRITE, bytes((address,)) + bytes(data))

    def read_position_ticks(self, servo_id: int) -> int:
        """读取当前位置（编码器读数）。"""
        return int.from_bytes(self.read(servo_id, Register.PRESENT_POSITION, 2), "little")

    def read_voltage(self, servo_id: int) -> float:
        """读取供电电压（单位：伏，原始值 /10）。"""
        return self.read(servo_id, Register.PRESENT_VOLTAGE, 1)[0] / 10.0

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        """显式使能/关闭保持扭矩。"""
        self.write(servo_id, Register.TORQUE_ENABLE, bytes((1 if enabled else 0,)))

    def write_goal_position_ticks(self, servo_id: int, ticks: int) -> None:
        """显式下发目标位置；调用方需另行使能扭矩。"""
        if not 0 <= ticks <= 4095:
            raise ValueError("STS3215 position must be in [0, 4095]")
        self.write(servo_id, Register.GOAL_POSITION, int(ticks).to_bytes(2, "little"))
