#!/usr/bin/env python3
"""Publish UM982 position and dual-antenna heading from one serial stream."""

import json
import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from multi_camera_trigger.nmea import (
    evaluate_dual_solution,
    midpoint_from_ant1,
    NmeaParseError,
    parse_gga_sentence,
    parse_uniheadinga,
    robot_heading_from_baseline,
)


class GpsPublisher(Node):
    """Read UM982 GGA/heading messages and reconnect after serial failures."""

    def __init__(self):
        """Initialize publishers and start the reconnecting serial reader."""
        super().__init__('gps_publisher')

        self.initialized = False
        self.ser = None
        self._stop_event = threading.Event()
        self._read_thread = None
        self._warning_monotonic = 0.0
        self._receiver_configured = False
        self.latest_heading = None
        self.latest_heading_monotonic = None
        self.latest_heading_ros_time = None

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reconnect_delay_sec', 1.0)
        self.declare_parameter('configure_receiver_on_start', True)
        self.declare_parameter('receiver_output_period_sec', 0.1)
        self.declare_parameter('baseline_m', 1.28)
        self.declare_parameter('baseline_tolerance_m', 0.1)
        self.declare_parameter('antenna_baseline_angle_deg', 0.0)
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('pitch_multiplier', 1.0)
        self.declare_parameter('max_heading_stddev_deg', 1.0)
        self.declare_parameter('max_heading_age_sec', 0.3)
        self.declare_parameter('max_position_heading_skew_sec', 0.2)
        self.declare_parameter('max_differential_age_sec', 3.0)
        self.declare_parameter('minimum_heading_satellites', 6)
        self.declare_parameter('fix_topic', '/imaging/gps/fix')
        self.declare_parameter('utc_topic', '/gps/utc')
        self.declare_parameter('detail_topic', '/imaging/gps/fix_detail')
        self.declare_parameter('heading_topic', '/imaging/gps/heading_deg')
        self.declare_parameter('pitch_topic', '/imaging/gps/pitch_deg')

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.reconnect_delay_sec = float(
            self.get_parameter('reconnect_delay_sec').value
        )
        self.configure_receiver_on_start = bool(
            self.get_parameter('configure_receiver_on_start').value
        )
        self.receiver_output_period_sec = float(
            self.get_parameter('receiver_output_period_sec').value
        )
        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.baseline_tolerance_m = float(
            self.get_parameter('baseline_tolerance_m').value
        )
        self.antenna_baseline_angle_deg = float(
            self.get_parameter('antenna_baseline_angle_deg').value
        )
        self.heading_offset_deg = float(
            self.get_parameter('heading_offset_deg').value
        )
        self.pitch_multiplier = float(self.get_parameter('pitch_multiplier').value)
        self.max_heading_stddev_deg = float(
            self.get_parameter('max_heading_stddev_deg').value
        )
        self.max_heading_age_sec = float(
            self.get_parameter('max_heading_age_sec').value
        )
        self.max_position_heading_skew_sec = float(
            self.get_parameter('max_position_heading_skew_sec').value
        )
        self.max_differential_age_sec = float(
            self.get_parameter('max_differential_age_sec').value
        )
        self.minimum_heading_satellites = int(
            self.get_parameter('minimum_heading_satellites').value
        )
        fix_topic = str(self.get_parameter('fix_topic').value)
        utc_topic = str(self.get_parameter('utc_topic').value)
        detail_topic = str(self.get_parameter('detail_topic').value)
        heading_topic = str(self.get_parameter('heading_topic').value)
        pitch_topic = str(self.get_parameter('pitch_topic').value)

        if self.baud <= 0:
            self.get_logger().error('baud must be positive')
            return
        if self.reconnect_delay_sec <= 0.0:
            self.get_logger().error('reconnect_delay_sec must be positive')
            return
        if not self._heading_parameters_valid():
            return

        self.pub_fix = self.create_publisher(NavSatFix, fix_topic, 10)
        self.pub_utc = self.create_publisher(String, utc_topic, 10)
        self.pub_detail = self.create_publisher(String, detail_topic, 10)
        self.pub_heading = self.create_publisher(Float64, heading_topic, 10)
        self.pub_pitch = self.create_publisher(Float64, pitch_topic, 10)

        self._read_thread = threading.Thread(
            target=self.read_loop,
            name='gps-serial-reader',
            daemon=True,
        )
        self._read_thread.start()
        self.initialized = True

    @staticmethod
    def _map_navsat_status(quality: int) -> int:
        if quality <= 0:
            return NavSatStatus.STATUS_NO_FIX
        if quality == 1:
            return NavSatStatus.STATUS_FIX
        if quality == 2:
            return NavSatStatus.STATUS_SBAS_FIX
        if quality in (4, 5):
            return NavSatStatus.STATUS_GBAS_FIX
        return NavSatStatus.STATUS_FIX

    def _warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self._warning_monotonic >= 2.0:
            self.get_logger().warn(message)
            self._warning_monotonic = now

    def _heading_parameters_valid(self) -> bool:
        checks = (
            (self.receiver_output_period_sec > 0.0, 'receiver_output_period_sec'),
            (self.baseline_m > 0.0, 'baseline_m'),
            (self.baseline_tolerance_m >= 0.0, 'baseline_tolerance_m'),
            (self.max_heading_stddev_deg > 0.0, 'max_heading_stddev_deg'),
            (self.max_heading_age_sec > 0.0, 'max_heading_age_sec'),
            (
                self.max_position_heading_skew_sec > 0.0,
                'max_position_heading_skew_sec',
            ),
            (self.max_differential_age_sec > 0.0, 'max_differential_age_sec'),
            (self.minimum_heading_satellites >= 0, 'minimum_heading_satellites'),
        )
        invalid = [name for valid, name in checks if not valid]
        if invalid:
            self.get_logger().error(
                f'Invalid UM982 parameter(s): {", ".join(invalid)}'
            )
            return False
        return True

    def _configure_receiver(self):
        baseline_cm = round(self.baseline_m * 100.0)
        tolerance_cm = max(1, round(self.baseline_tolerance_m * 100.0))
        period = self.receiver_output_period_sec
        commands = (
            'UNLOG',
            'MODE ROVER',
            'CONFIG HEADING FIXLENGTH',
            f'CONFIG HEADING LENGTH {baseline_cm} {tolerance_cm}',
            f'GPGGA {period:g}',
            f'UNIHEADINGA {period:g}',
            'SAVECONFIG',
        )
        for command in commands:
            self.ser.write(f'{command}\r\n'.encode('ascii'))
            time.sleep(0.03)
        self.ser.flush()
        self._receiver_configured = True
        self.get_logger().info(
            'Configured UM982 fixed-baseline heading, GGA, and UNIHEADINGA output'
        )

    def _open_serial(self) -> bool:
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=1,
                write_timeout=1,
            )
            if self.configure_receiver_on_start and not self._receiver_configured:
                self._configure_receiver()
            self.get_logger().info(
                f'Opened GPS serial on {self.port} @ {self.baud}'
            )
            return True
        except Exception as exc:
            self._close_serial()
            self._warn_throttled(f'Failed to open GPS serial: {exc}; retrying')
            return False

    def _close_serial(self):
        serial_port = self.ser
        self.ser = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def _heading_metadata(self, now, fix):
        metadata = {
            'baseline_heading_deg': None,
            'robot_heading_deg': None,
            'pitch_deg': None,
            'heading_stddev_deg': None,
            'pitch_stddev_deg': None,
            'heading_baseline_m': None,
            'heading_satellites_tracked': None,
            'heading_satellites_used': None,
            'heading_solution_status': '',
            'heading_position_type': '',
            'heading_age_sec': None,
            'heading_ros_time': '',
            'heading_valid': False,
            'dual_solution_valid': False,
            'dual_solution_reason': 'no heading solution',
            'midpoint_latitude': None,
            'midpoint_longitude': None,
            'midpoint_altitude': None,
            'midpoint_computed': False,
        }
        heading = self.latest_heading
        if heading is None or self.latest_heading_monotonic is None:
            return metadata

        heading_age = max(0.0, now - self.latest_heading_monotonic)
        metadata.update({
            'baseline_heading_deg': heading.heading_deg,
            'robot_heading_deg': robot_heading_from_baseline(
                heading.heading_deg,
                self.antenna_baseline_angle_deg,
                self.heading_offset_deg,
            ),
            'pitch_deg': heading.pitch_deg * self.pitch_multiplier,
            'heading_stddev_deg': heading.heading_stddev_deg,
            'pitch_stddev_deg': heading.pitch_stddev_deg,
            'heading_baseline_m': heading.baseline_m,
            'heading_satellites_tracked': heading.satellites_tracked,
            'heading_satellites_used': heading.satellites_used,
            'heading_solution_status': heading.solution_status,
            'heading_position_type': heading.position_type,
            'heading_age_sec': heading_age,
            'heading_ros_time': self.latest_heading_ros_time or '',
        })
        if (
            fix.latitude is not None
            and fix.longitude is not None
            and fix.altitude is not None
        ):
            midpoint_lat, midpoint_lon, midpoint_altitude = midpoint_from_ant1(
                latitude=fix.latitude,
                longitude=fix.longitude,
                altitude_m=fix.altitude,
                baseline_heading_deg=heading.heading_deg,
                pitch_deg=heading.pitch_deg * self.pitch_multiplier,
                baseline_m=self.baseline_m,
            )
            metadata.update({
                'midpoint_latitude': midpoint_lat,
                'midpoint_longitude': midpoint_lon,
                'midpoint_altitude': midpoint_altitude,
                'midpoint_computed': True,
            })

        validity = evaluate_dual_solution(
            heading=heading,
            heading_age_sec=heading_age,
            fix_quality=fix.fix_quality,
            differential_age_sec=fix.differential_age_sec,
            expected_baseline_m=self.baseline_m,
            baseline_tolerance_m=self.baseline_tolerance_m,
            max_heading_stddev_deg=self.max_heading_stddev_deg,
            max_heading_age_sec=self.max_heading_age_sec,
            max_position_heading_skew_sec=self.max_position_heading_skew_sec,
            max_differential_age_sec=self.max_differential_age_sec,
            minimum_heading_satellites=self.minimum_heading_satellites,
        )
        metadata['heading_valid'] = validity.heading_valid
        metadata['dual_solution_valid'] = validity.dual_solution_valid
        metadata['dual_solution_reason'] = validity.reason
        return metadata

    def _publish_detail(self, stamp, fix, navsat_status: int, heading_metadata):
        detail = String()
        payload = {
            'ros_time': f'{stamp.sec}.{stamp.nanosec:09d}',
            'satellite_utc': fix.satellite_utc,
            'latitude': fix.latitude,
            'longitude': fix.longitude,
            'altitude': fix.altitude,
            'ant1_latitude': fix.latitude,
            'ant1_longitude': fix.longitude,
            'ant1_altitude': fix.altitude,
            'fix_quality': fix.fix_quality,
            'differential_age_sec': fix.differential_age_sec,
            'navsat_status': navsat_status,
        }
        payload.update(heading_metadata)
        detail.data = json.dumps(payload, separators=(',', ':'))
        self.pub_detail.publish(detail)

    def _publish_fix(self, fix):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        navsat_status = self._map_navsat_status(fix.fix_quality)
        heading_metadata = self._heading_metadata(now, fix)

        if fix.satellite_utc:
            utc_msg = String()
            utc_msg.data = fix.satellite_utc
            self.pub_utc.publish(utc_msg)

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps'
        msg.latitude = fix.latitude if fix.latitude is not None else math.nan
        msg.longitude = fix.longitude if fix.longitude is not None else math.nan
        msg.altitude = fix.altitude if fix.altitude is not None else math.nan
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.status.status = navsat_status
        self.pub_fix.publish(msg)
        self._publish_detail(stamp, fix, navsat_status, heading_metadata)

        if heading_metadata['dual_solution_valid']:
            self.pub_heading.publish(
                Float64(data=heading_metadata['robot_heading_deg'])
            )
            self.pub_pitch.publish(Float64(data=heading_metadata['pitch_deg']))

    def read_loop(self):
        """Read, validate, and publish GGA messages until shutdown."""
        while rclpy.ok() and not self._stop_event.is_set():
            if self.ser is None and not self._open_serial():
                self._stop_event.wait(self.reconnect_delay_sec)
                continue

            try:
                line = self.ser.readline().decode(
                    'ascii',
                    errors='ignore',
                ).strip()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._warn_throttled(
                        f'GPS serial read failed: {exc}; reconnecting'
                    )
                self._close_serial()
                self._stop_event.wait(self.reconnect_delay_sec)
                continue

            if not line:
                continue
            if line.startswith('#UNIHEADINGA,'):
                try:
                    self.latest_heading = parse_uniheadinga(line)
                    self.latest_heading_monotonic = time.monotonic()
                    heading_stamp = self.get_clock().now().to_msg()
                    self.latest_heading_ros_time = (
                        f'{heading_stamp.sec}.{heading_stamp.nanosec:09d}'
                    )
                except NmeaParseError as exc:
                    self._warn_throttled(
                        f'Ignoring malformed UNIHEADINGA message: {exc}'
                    )
                continue
            try:
                fix = parse_gga_sentence(line)
            except NmeaParseError as exc:
                if line.startswith('$') and line[3:6] == 'GGA':
                    self._warn_throttled(f'Ignoring malformed GGA message: {exc}')
                continue
            self._publish_fix(fix)

    def destroy_node(self):
        """Stop the reader thread and close the serial port."""
        self._stop_event.set()
        self._close_serial()
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """Run the GPS publisher node."""
    rclpy.init(args=args)
    node = None
    success = False
    try:
        node = GpsPublisher()
        if node.initialized and rclpy.ok():
            success = True
            rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
