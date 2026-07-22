#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish RTK GPS fixes and RTK status from GGA serial messages."""

import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool
import pynmea2


DEFAULT_GPS_PORT = '/dev/serial/by-id/usb-Emlid_ReachRS3_8243877019144177-if02'
DEFAULT_BAUDRATE = 115200


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gnss_publisher')

        self.declare_parameter('gps_port', DEFAULT_GPS_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('poll_period_sec', 0.02)
        self.declare_parameter('serial_timeout_sec', 0.02)

        port = self.get_parameter('gps_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        poll_period = self.get_parameter('poll_period_sec').get_parameter_value().double_value
        serial_timeout = self.get_parameter('serial_timeout_sec').get_parameter_value().double_value

        self.get_logger().info(f"Starting GPS Publisher on {port} @ {baud}bps")

        # Publishers
        self.pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_rtk_status = self.create_publisher(Bool, '/gps/rtk_status_flag', 10)

        # Serial port
        self.ser = serial.Serial(port, baud, timeout=serial_timeout)

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

        # Used to avoid spamming repeated warning messages
        self._last_warn_time = {}

        self.create_timer(poll_period, self.timer_callback)

    def warn_limited(self, key, message, interval_sec=2.0):
        """Print warning messages at a limited rate."""
        now = time.monotonic()
        last = self._last_warn_time.get(key, 0.0)

        if now - last >= interval_sec:
            self.get_logger().warning(message)
            self._last_warn_time[key] = now

    def timer_callback(self):
        latest_gga_msg = None
        latest_gga_raw = None

        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()

                if not raw:
                    continue

                # Try to recover if there is garbage before the NMEA sentence.
                dollar_index = raw.find('$')
                if dollar_index < 0:
                    self.warn_limited(
                        'no_dollar',
                        f"Skipping non-NMEA serial line: raw='{raw}'"
                    )
                    continue

                raw = raw[dollar_index:]

                try:
                    msg = pynmea2.parse(raw)
                except pynmea2.ParseError as e:
                    self.warn_limited(
                        'parse_error',
                        f"Skipping invalid NMEA sentence: raw='{raw}' | error={e}"
                    )
                    continue
                except Exception as e:
                    self.warn_limited(
                        'parse_unknown',
                        f"Skipping unreadable NMEA sentence: raw='{raw}' | error={e}"
                    )
                    continue

                # pynmea2 normalizes both $GPGGA and $GNGGA to sentence_type == 'GGA'.
                if getattr(msg, 'sentence_type', None) != 'GGA':
                    continue

                latest_gga_msg = msg
                latest_gga_raw = raw

        except serial.SerialException as e:
            self.warn_limited('serial_error', f"Serial read error: {e}")
            return
        except Exception as e:
            self.warn_limited('serial_unknown', f"Unexpected serial read error: {e}")
            return

        if latest_gga_msg is None:
            return

        self.process_gga(latest_gga_msg, latest_gga_raw)

    def process_gga(self, msg, raw):
        """Process one GGA message safely."""

        # Read RTK/GNSS quality first.
        try:
            qual = int(getattr(msg, 'gps_qual', 0))
        except Exception:
            self.warn_limited(
                'bad_quality',
                f"Skipping GGA with invalid fix quality: raw='{raw}'"
            )
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return

        # Only RTK Fixed is accepted.
        # GGA quality:
        # 0 = invalid
        # 1 = GPS fix
        # 2 = DGPS fix
        # 4 = RTK fixed
        # 5 = RTK float
        if qual != 4:
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return

        # Check raw NMEA coordinate fields before accessing msg.latitude/msg.longitude.
        # Accessing msg.latitude can raise ValueError if the raw field is malformed.
        raw_lat = getattr(msg, 'lat', '')
        raw_lon = getattr(msg, 'lon', '')
        raw_lat_dir = getattr(msg, 'lat_dir', '')
        raw_lon_dir = getattr(msg, 'lon_dir', '')

        if not raw_lat or not raw_lon or not raw_lat_dir or not raw_lon_dir:
            self.warn_limited(
                'empty_coordinate',
                f"Skipping RTK fixed GGA with empty coordinates: raw='{raw}'"
            )
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return

        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
        except ValueError as e:
            self.warn_limited(
                'bad_coordinate',
                f"Skipping RTK fixed GGA with invalid coordinates: raw='{raw}' | error={e}"
            )
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return
        except Exception as e:
            self.warn_limited(
                'coordinate_unknown',
                f"Skipping RTK fixed GGA due to coordinate conversion error: raw='{raw}' | error={e}"
            )
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return

        # Sanity check after conversion.
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.warn_limited(
                'coordinate_range',
                f"Skipping RTK fixed GGA with out-of-range coordinates: "
                f"lat={lat}, lon={lon}, raw='{raw}'"
            )
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return

        try:
            alt = float(getattr(msg, 'altitude', 0.0))
        except Exception:
            self.warn_limited(
                'bad_altitude',
                f"Skipping RTK fixed GGA with invalid altitude: raw='{raw}'"
            )
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX
            return

        self.latitude = lat
        self.longitude = lon
        self.altitude = alt

        now = self.get_clock().now().to_msg()

        gps_msg = NavSatFix()
        gps_msg.header.stamp = now
        gps_msg.header.frame_id = 'gps_link'

        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt

        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_msg.position_covariance = [
            0.02**2, 0.0, 0.0,
            0.0, 0.02**2, 0.0,
            0.0, 0.0, 0.05**2
        ]

        self.pub_gps.publish(gps_msg)

        # Keep your original convention:
        # False = RTK FIX OK
        # True  = NOT FIX / invalid
        self.pub_rtk_status.publish(Bool(data=False))

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()