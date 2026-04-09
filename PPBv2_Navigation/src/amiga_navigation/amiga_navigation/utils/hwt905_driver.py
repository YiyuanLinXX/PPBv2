#!/usr/bin/env python3
"""Driver helpers for reading WT901/HWT905 binary IMU frames over serial."""

from dataclasses import dataclass

import serial


GRAVITY_M_S2 = 9.80665
FRAME_HEADER = 0x55
FRAME_LENGTH = 11
ACC_FRAME = 0x51
GYRO_FRAME = 0x52
ANGLE_FRAME = 0x53


def _int16_le(low_byte, high_byte):
    raw = (high_byte << 8) | low_byte
    if raw >= 0x8000:
        raw -= 0x10000
    return raw


@dataclass
class HWT905State:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    has_accel: bool = False
    has_gyro: bool = False
    has_angle: bool = False

    @property
    def ready(self):
        return self.has_accel and self.has_gyro and self.has_angle


class HWT905:
    """Read and parse HWT905 binary frames from a serial port."""

    def __init__(self, port, baudrate=9600, timeout=0.02):
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.buffer = bytearray()
        self.state = HWT905State()

    def close(self):
        if self.serial.is_open:
            self.serial.close()

    def read_available(self):
        """Read and parse all currently buffered serial bytes."""
        bytes_waiting = self.serial.in_waiting
        if bytes_waiting <= 0:
            return False

        self.buffer.extend(self.serial.read(bytes_waiting))
        updated = False

        while True:
            header_index = self.buffer.find(bytes([FRAME_HEADER]))
            if header_index < 0:
                self.buffer.clear()
                break

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < FRAME_LENGTH:
                break

            frame = bytes(self.buffer[:FRAME_LENGTH])
            if (sum(frame[:10]) & 0xFF) != frame[10]:
                del self.buffer[0]
                continue

            del self.buffer[:FRAME_LENGTH]
            updated = self._parse_frame(frame[1], frame[2:10]) or updated

        return updated

    def _parse_frame(self, frame_type, payload):
        if frame_type == ACC_FRAME:
            self.state.ax = _int16_le(payload[0], payload[1]) / 32768.0 * 16.0 * GRAVITY_M_S2
            self.state.ay = _int16_le(payload[2], payload[3]) / 32768.0 * 16.0 * GRAVITY_M_S2
            self.state.az = _int16_le(payload[4], payload[5]) / 32768.0 * 16.0 * GRAVITY_M_S2
            self.state.has_accel = True
            return True

        if frame_type == GYRO_FRAME:
            self.state.wx = _int16_le(payload[0], payload[1]) / 32768.0 * 2000.0
            self.state.wy = _int16_le(payload[2], payload[3]) / 32768.0 * 2000.0
            self.state.wz = _int16_le(payload[4], payload[5]) / 32768.0 * 2000.0
            self.state.has_gyro = True
            return True

        if frame_type == ANGLE_FRAME:
            self.state.roll_deg = _int16_le(payload[0], payload[1]) / 32768.0 * 180.0
            self.state.pitch_deg = _int16_le(payload[2], payload[3]) / 32768.0 * 180.0
            self.state.yaw_deg = _int16_le(payload[4], payload[5]) / 32768.0 * 180.0
            self.state.has_angle = True
            return True

        return False
