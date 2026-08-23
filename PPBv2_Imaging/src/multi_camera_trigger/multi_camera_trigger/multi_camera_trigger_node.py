#!/usr/bin/env python3
"""Hardware-triggered multi-camera acquisition with synchronized GPS logging."""

import csv
import datetime
import json
import os
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import PySpin
import rclpy
import serial as pyserial
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from multi_camera_trigger.image_io import (
    normalize_image_format,
    save_frame_atomic,
)


class MultiCameraTriggerNode(Node):
    """Acquire synchronized frames from all detected cameras."""

    def __init__(self):
        """Declare parameters and initialize every detected camera safely."""
        super().__init__('multi_camera_trigger')

        self.initialized = False
        self.initialization_error = ''
        self._cleaned_up = False
        self._gps_warning_monotonic = 0.0
        self._gps_lock = threading.Lock()
        self._arduino_lock = threading.Lock()
        self._arduino_stop_event = threading.Event()
        self._arduino_failed_event = threading.Event()
        self._heartbeat_thread = None
        self._save_executor = None
        self.gps_history = deque()
        self.cameras = []
        self.arduino = None
        self.system = None
        self.cam_list = None
        self.gps_subscription = None

        self._declare_parameters()

        try:
            self._read_parameters()
            self._validate_parameters()
            self._create_gps_subscription()
            os.makedirs(self.output_dir, exist_ok=True)
            self._initialize_cameras()
            self._save_executor = ThreadPoolExecutor(
                max_workers=len(self.cameras),
                thread_name_prefix='image-writer',
            )
            self.initialized = True
        except Exception as exc:
            self.initialization_error = str(exc)
            self.get_logger().error(f'Camera initialization failed: {exc}')
            self.cleanup()

    def _declare_parameters(self):
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('png_compress_level', 3)
        self.declare_parameter('overwrite_existing', False)
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('arduino_baud', 9600)
        self.declare_parameter('arduino_startup_delay_sec', 2.0)
        self.declare_parameter('arduino_heartbeat_period_sec', 0.5)
        self.declare_parameter('exposure_time', 400.0)
        self.declare_parameter('gain', 5.0)
        self.declare_parameter('wb_red', 1.34)
        self.declare_parameter('wb_blue', 2.98)
        self.declare_parameter('gps_detail_topic', '/imaging/gps/fix_detail')
        self.declare_parameter('gps_qos_depth', 200)
        self.declare_parameter('gps_match_max_age_sec', 0.25)
        self.declare_parameter('camera_timeout_ms', 1000)
        self.declare_parameter('resync_max_attempts', 3)
        self.declare_parameter('resync_max_drop_frames', 10)
        self.declare_parameter('resync_timeout_sec', 4.0)

    def _read_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self.output_dir = str(value('output_dir'))
        self.image_format = normalize_image_format(str(value('image_format')))
        self.jpeg_quality = int(value('jpeg_quality'))
        self.png_compress_level = int(value('png_compress_level'))
        self.overwrite_existing = bool(value('overwrite_existing'))
        self.arduino_port = str(value('arduino_port'))
        self.arduino_baud = int(value('arduino_baud'))
        self.arduino_startup_delay_sec = float(value('arduino_startup_delay_sec'))
        self.arduino_heartbeat_period_sec = float(
            value('arduino_heartbeat_period_sec')
        )
        self.exposure_time = float(value('exposure_time'))
        self.gain_value = float(value('gain'))
        self.wb_red = float(value('wb_red'))
        self.wb_blue = float(value('wb_blue'))
        self.gps_detail_topic = str(value('gps_detail_topic'))
        self.gps_qos_depth = int(value('gps_qos_depth'))
        self.gps_match_max_age_sec = float(value('gps_match_max_age_sec'))
        self.camera_timeout_ms = int(value('camera_timeout_ms'))
        self.resync_max_attempts = int(value('resync_max_attempts'))
        self.resync_max_drop_frames = int(value('resync_max_drop_frames'))
        self.resync_timeout_sec = float(value('resync_timeout_sec'))

    def _validate_parameters(self):
        if not self.output_dir:
            raise ValueError('output_dir must not be empty')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        if not 0 <= self.png_compress_level <= 9:
            raise ValueError('png_compress_level must be in [0, 9]')
        if self.arduino_baud <= 0:
            raise ValueError('arduino_baud must be positive')
        if self.arduino_startup_delay_sec < 0.0:
            raise ValueError('arduino_startup_delay_sec must not be negative')
        if not 0.1 <= self.arduino_heartbeat_period_sec <= 1.0:
            raise ValueError('arduino_heartbeat_period_sec must be in [0.1, 1.0]')
        if self.exposure_time <= 0.0:
            raise ValueError('exposure_time must be positive')
        if self.gain_value < 0.0:
            raise ValueError('gain must not be negative')
        if self.wb_red <= 0.0 or self.wb_blue <= 0.0:
            raise ValueError('white-balance ratios must be positive')
        if self.gps_qos_depth <= 0:
            raise ValueError('gps_qos_depth must be positive')
        if self.gps_match_max_age_sec < 0.0:
            raise ValueError('gps_match_max_age_sec must not be negative')
        if self.camera_timeout_ms <= 0:
            raise ValueError('camera_timeout_ms must be positive')
        if self.resync_max_attempts < 0:
            raise ValueError('resync_max_attempts must not be negative')
        if self.resync_max_drop_frames < 0:
            raise ValueError('resync_max_drop_frames must not be negative')
        if self.resync_timeout_sec <= 0.0:
            raise ValueError('resync_timeout_sec must be positive')

    def _create_gps_subscription(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.gps_qos_depth,
        )
        self.gps_history = deque(maxlen=self.gps_qos_depth)
        self.gps_subscription = self.create_subscription(
            String,
            self.gps_detail_topic,
            self._gps_callback,
            qos,
        )

    def _initialize_cameras(self):
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        camera_count = self.cam_list.GetSize()
        if camera_count == 0:
            raise RuntimeError('no cameras detected')

        self.get_logger().info(
            f'Configuring {camera_count} camera(s); output format={self.image_format}'
        )
        for index in range(camera_count):
            cam = self.cam_list[index]
            entry = {
                'cam': cam,
                'nodemap': None,
                'dir': None,
                'serial': f'cam{index}',
                'csv_file': None,
                'csv_writer': None,
                'counter': 1,
                'initialized': False,
                'acquiring': False,
            }
            self.cameras.append(entry)

            cam.Init()
            entry['initialized'] = True
            nodemap = cam.GetNodeMap()
            entry['nodemap'] = nodemap

            serial = self._camera_serial(cam, index)
            entry['serial'] = serial
            self._configure_camera(nodemap, serial)
            self._prepare_camera_output(entry)

            cam.BeginAcquisition()
            entry['acquiring'] = True

    @staticmethod
    def _camera_serial(cam, index):
        transport_map = cam.GetTLDeviceNodeMap()
        serial_node = PySpin.CStringPtr(
            transport_map.GetNode('DeviceSerialNumber')
        )
        if PySpin.IsAvailable(serial_node) and PySpin.IsReadable(serial_node):
            return serial_node.GetValue()
        return f'cam{index}'

    def _unavailable(self, serial, setting, required):
        message = f'{serial}: {setting} is unavailable, unreadable, or not writable'
        if required:
            raise RuntimeError(message)
        self.get_logger().warn(message)
        return False

    def _set_enum(self, nodemap, serial, name, entry_name, required=True):
        node = PySpin.CEnumerationPtr(nodemap.GetNode(name))
        if (
            not PySpin.IsAvailable(node)
            or not PySpin.IsReadable(node)
            or not PySpin.IsWritable(node)
        ):
            return self._unavailable(serial, name, required)
        enum_entry = node.GetEntryByName(entry_name)
        if not PySpin.IsAvailable(enum_entry) or not PySpin.IsReadable(enum_entry):
            return self._unavailable(
                serial,
                f'{name}={entry_name}',
                required,
            )
        node.SetIntValue(enum_entry.GetValue())
        return True

    def _set_boolean(self, nodemap, serial, name, value, required=True):
        node = PySpin.CBooleanPtr(nodemap.GetNode(name))
        if (
            not PySpin.IsAvailable(node)
            or not PySpin.IsReadable(node)
            or not PySpin.IsWritable(node)
        ):
            return self._unavailable(serial, name, required)
        node.SetValue(value)
        return True

    def _set_integer(self, nodemap, serial, name, value, required=False):
        node = PySpin.CIntegerPtr(nodemap.GetNode(name))
        if (
            not PySpin.IsAvailable(node)
            or not PySpin.IsReadable(node)
            or not PySpin.IsWritable(node)
        ):
            return self._unavailable(serial, name, required)
        requested = int(value)
        applied = min(max(requested, node.GetMin()), node.GetMax())
        if applied != requested:
            self.get_logger().warn(
                f'{serial}: clamped {name} from {requested} to {applied}'
            )
        node.SetValue(applied)
        self.get_logger().info(f'{serial}: {name}={node.GetValue()}')
        return True

    def _set_float(self, nodemap, serial, name, value, required=True):
        node = PySpin.CFloatPtr(nodemap.GetNode(name))
        if (
            not PySpin.IsAvailable(node)
            or not PySpin.IsReadable(node)
            or not PySpin.IsWritable(node)
        ):
            return self._unavailable(serial, name, required)
        requested = float(value)
        applied = min(max(requested, node.GetMin()), node.GetMax())
        if applied != requested:
            self.get_logger().warn(
                f'{serial}: clamped {name} from {requested} to {applied}'
            )
        node.SetValue(applied)
        self.get_logger().info(f'{serial}: {name}={node.GetValue()}')
        return True

    def _configure_camera(self, nodemap, serial):
        self._set_integer(
            nodemap,
            serial,
            'DeviceLinkThroughputLimit',
            100_000_000,
            required=False,
        )
        self._set_enum(nodemap, serial, 'AcquisitionMode', 'Continuous')
        self._set_enum(nodemap, serial, 'ExposureAuto', 'Off')
        self._set_enum(nodemap, serial, 'ExposureMode', 'Timed')
        self._set_float(nodemap, serial, 'ExposureTime', self.exposure_time)

        if self._set_enum(
            nodemap,
            serial,
            'GainAuto',
            'Off',
            required=False,
        ):
            self._set_float(
                nodemap,
                serial,
                'Gain',
                self.gain_value,
                required=False,
            )

        if self._set_enum(
            nodemap,
            serial,
            'BalanceWhiteAuto',
            'Off',
            required=False,
        ):
            self._configure_white_balance(nodemap, serial)

        self._set_enum(nodemap, serial, 'PixelFormat', 'BayerRG8')
        self._set_boolean(nodemap, serial, 'ChunkModeActive', True)
        for chunk_name in ('FrameID', 'Timestamp'):
            self._set_enum(nodemap, serial, 'ChunkSelector', chunk_name)
            self._set_boolean(nodemap, serial, 'ChunkEnable', True)

        self._set_enum(nodemap, serial, 'TriggerMode', 'Off')
        self._set_enum(nodemap, serial, 'TriggerSelector', 'FrameStart')
        self._set_enum(nodemap, serial, 'TriggerSource', 'Line0')
        self._set_enum(
            nodemap,
            serial,
            'TriggerActivation',
            'FallingEdge',
        )
        self._set_enum(nodemap, serial, 'TriggerMode', 'On')

    def _configure_white_balance(self, nodemap, serial):
        selector = PySpin.CEnumerationPtr(
            nodemap.GetNode('BalanceRatioSelector')
        )
        ratio = PySpin.CFloatPtr(nodemap.GetNode('BalanceRatio'))
        if (
            not PySpin.IsAvailable(selector)
            or not PySpin.IsWritable(selector)
            or not PySpin.IsAvailable(ratio)
            or not PySpin.IsReadable(ratio)
            or not PySpin.IsWritable(ratio)
        ):
            self._unavailable(serial, 'white-balance ratio', required=False)
            return

        for color, requested in (('Red', self.wb_red), ('Blue', self.wb_blue)):
            color_entry = selector.GetEntryByName(color)
            if not PySpin.IsAvailable(color_entry) or not PySpin.IsReadable(
                color_entry
            ):
                self._unavailable(
                    serial,
                    f'BalanceRatioSelector={color}',
                    required=False,
                )
                continue
            selector.SetIntValue(color_entry.GetValue())
            applied = min(max(requested, ratio.GetMin()), ratio.GetMax())
            if applied != requested:
                self.get_logger().warn(
                    f'{serial}: clamped {color} WB from {requested} to {applied}'
                )
            ratio.SetValue(applied)

    def _prepare_camera_output(self, entry):
        camera_dir = Path(self.output_dir) / entry['serial']
        if camera_dir.exists() and any(camera_dir.iterdir()):
            if not self.overwrite_existing:
                raise FileExistsError(
                    f'camera output directory is not empty: {camera_dir}; '
                    'choose a new output directory'
                )
            self.get_logger().warn(
                f'{entry["serial"]}: overwriting files in non-empty {camera_dir}'
            )
        camera_dir.mkdir(parents=True, exist_ok=True)
        entry['dir'] = str(camera_dir)

        csv_path = camera_dir / 'Timestamp_GPS.csv'
        csv_mode = 'w' if self.overwrite_existing else 'x'
        csv_file = csv_path.open(csv_mode, newline='')
        entry['csv_file'] = csv_file
        entry['csv_writer'] = csv.writer(csv_file)
        entry['csv_writer'].writerow([
            'Frame ID',
            'Image File',
            'Computer Time',
            'ROS Time Stamp(s.ns)',
            'Chunk Frame ID',
            'Chunk Time(ns)',
            'Satellite UTC',
            'GPS ROS Time(s.ns)',
            'GPS Match Delta(s)',
            'Midpoint Latitude',
            'Midpoint Longitude',
            'Midpoint Altitude',
            'Midpoint Computed',
            'FixQuality',
            'Differential Age(s)',
            'Baseline Heading(deg)',
            'Robot Heading(deg)',
            'Pitch(deg)',
            'Heading StdDev(deg)',
            'Pitch StdDev(deg)',
            'Heading Baseline(m)',
            'Heading Satellites Tracked',
            'Heading Satellites Used',
            'Heading Solution Status',
            'Heading Position Type',
            'Heading ROS Time(s.ns)',
            'GGA-Heading Delta(s)',
            'Frame-Heading Delta(s)',
            'Heading Valid',
            'Dual Solution Valid',
            'Dual Solution Reason',
            'CrossCameraSyncOk',
        ])
        csv_file.flush()

    def _gps_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            ros_time_str = payload['ros_time']
            sample = {
                'ros_time': float(ros_time_str),
                'ros_time_str': str(ros_time_str),
                'satellite_utc': payload.get('satellite_utc', ''),
                'latitude': self._optional_float(
                    payload.get('midpoint_latitude')
                ),
                'longitude': self._optional_float(
                    payload.get('midpoint_longitude')
                ),
                'altitude': self._optional_float(
                    payload.get('midpoint_altitude')
                ),
                'midpoint_computed': payload.get('midpoint_computed'),
                'fix_quality': payload.get('fix_quality', ''),
                'differential_age_sec': self._optional_float(
                    payload.get('differential_age_sec')
                ),
                'baseline_heading_deg': self._optional_float(
                    payload.get('baseline_heading_deg')
                ),
                'robot_heading_deg': self._optional_float(
                    payload.get('robot_heading_deg')
                ),
                'pitch_deg': self._optional_float(payload.get('pitch_deg')),
                'heading_stddev_deg': self._optional_float(
                    payload.get('heading_stddev_deg')
                ),
                'pitch_stddev_deg': self._optional_float(
                    payload.get('pitch_stddev_deg')
                ),
                'heading_baseline_m': self._optional_float(
                    payload.get('heading_baseline_m')
                ),
                'heading_satellites_tracked': payload.get(
                    'heading_satellites_tracked', ''
                ),
                'heading_satellites_used': payload.get(
                    'heading_satellites_used', ''
                ),
                'heading_solution_status': payload.get(
                    'heading_solution_status', ''
                ),
                'heading_position_type': payload.get('heading_position_type', ''),
                'heading_ros_time_str': str(
                    payload.get('heading_ros_time', '')
                ),
                'heading_ros_time': self._optional_float(
                    payload.get('heading_ros_time')
                ),
                'heading_age_sec': self._optional_float(
                    payload.get('heading_age_sec')
                ),
                'heading_valid': payload.get('heading_valid'),
                'dual_solution_valid': payload.get('dual_solution_valid'),
                'dual_solution_reason': payload.get('dual_solution_reason', ''),
            }
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Ignoring malformed GPS detail payload: {exc}')
            return

        with self._gps_lock:
            self.gps_history.append(sample)

    @staticmethod
    def _optional_float(value):
        return None if value is None or value == '' else float(value)

    def _acquire_frame(self, entry):
        image = None
        converted = None
        try:
            image = entry['cam'].GetNextImage(self.camera_timeout_ms)
            stamp = self.get_clock().now().to_msg()
            if image.IsIncomplete():
                self.get_logger().warn(
                    f"{entry['serial']}: incomplete image "
                    f'{image.GetImageStatus()}'
                )
                return None

            chunk = image.GetChunkData()
            if self.image_format == 'pgm':
                frame_data = bytes(image.GetData())
            else:
                converted = image.Convert(
                    PySpin.PixelFormat_RGB8,
                    PySpin.HQ_LINEAR,
                )
                frame_data = bytes(converted.GetData())

            return {
                'entry': entry,
                'computer_time': datetime.datetime.now().strftime(
                    '%Y-%m-%d-%H-%M-%S-%f'
                )[:-3],
                'ros_time_str': f'{stamp.sec}.{stamp.nanosec:09d}',
                'ros_time_float': stamp.sec + stamp.nanosec * 1e-9,
                'chunk_frame_id': chunk.GetFrameID(),
                'chunk_time': chunk.GetTimestamp(),
                'width': image.GetWidth(),
                'height': image.GetHeight(),
                'data': frame_data,
            }
        except PySpin.SpinnakerException as exc:
            self.get_logger().warn(
                f"{entry['serial']}: failed waiting for or converting image: {exc}"
            )
            return None
        finally:
            converted = None
            if image is not None:
                try:
                    image.Release()
                except PySpin.SpinnakerException:
                    pass

    @staticmethod
    def _chunk_ids_aligned(frames) -> bool:
        return len({frame['chunk_frame_id'] for frame in frames}) == 1

    def _resync_batch(self, frames):
        attempts = 0
        dropped = 0
        deadline = time.monotonic() + self.resync_timeout_sec

        while (
            not self._chunk_ids_aligned(frames)
            and attempts < self.resync_max_attempts
            and dropped < self.resync_max_drop_frames
            and time.monotonic() < deadline
        ):
            attempts += 1
            target_chunk_id = max(frame['chunk_frame_id'] for frame in frames)
            mismatch = ', '.join(
                f"{frame['entry']['serial']}:{frame['chunk_frame_id']}"
                for frame in frames
            )
            self.get_logger().warn(
                f'Chunk Frame ID mismatch ({mismatch}); '
                f'resync target={target_chunk_id}, attempt '
                f'{attempts}/{self.resync_max_attempts}'
            )

            new_frames = []
            for frame in frames:
                current = frame
                while (
                    current is not None
                    and current['chunk_frame_id'] < target_chunk_id
                    and dropped < self.resync_max_drop_frames
                    and time.monotonic() < deadline
                ):
                    self.get_logger().warn(
                        f"Dropping unsynced frame from {current['entry']['serial']}: "
                        f"chunk {current['chunk_frame_id']} < {target_chunk_id}"
                    )
                    dropped += 1
                    current = self._acquire_frame(current['entry'])
                if current is None:
                    return [], False
                new_frames.append(current)
            frames = new_frames

        sync_ok = self._chunk_ids_aligned(frames)
        if not sync_ok:
            mismatch = ', '.join(
                f"{frame['entry']['serial']}:{frame['chunk_frame_id']}"
                for frame in frames
            )
            self.get_logger().error(
                f'Unable to resync frames ({mismatch}); attempts={attempts}, '
                f'dropped={dropped}'
            )
        return frames, sync_ok

    def _find_best_gps_sample(self, reference_time):
        with self._gps_lock:
            samples = list(self.gps_history)
        if not samples:
            return None, None
        best = min(samples, key=lambda item: abs(item['ros_time'] - reference_time))
        return best, abs(best['ros_time'] - reference_time)

    def _warn_stale_gps(self, message: str):
        now = time.monotonic()
        if now - self._gps_warning_monotonic >= 2.0:
            self.get_logger().warn(message)
            self._gps_warning_monotonic = now

    @staticmethod
    def _format_optional(value, precision):
        return '' if value is None else f'{value:.{precision}f}'

    @staticmethod
    def _format_optional_bool(value):
        return '' if value is None else int(bool(value))

    def _save_frame(self, frame, gps_sample, gps_age, sync_ok):
        entry = frame['entry']
        filename = f"{entry['counter']:06d}.{self.image_format}"
        output_path = os.path.join(entry['dir'], filename)
        save_frame_atomic(
            output_path=output_path,
            data=frame['data'],
            width=frame['width'],
            height=frame['height'],
            image_format=self.image_format,
            jpeg_quality=self.jpeg_quality,
            png_compress_level=self.png_compress_level,
        )

        latitude = longitude = altitude = ''
        fix_quality = satellite_utc = gps_ros_time = gps_delta = ''
        if gps_sample is not None:
            latitude = self._format_optional(gps_sample['latitude'], 9)
            longitude = self._format_optional(gps_sample['longitude'], 9)
            altitude = self._format_optional(gps_sample['altitude'], 2)
            fix_quality = gps_sample['fix_quality']
            satellite_utc = gps_sample['satellite_utc']
            gps_ros_time = gps_sample['ros_time_str']
            gps_delta = f'{gps_age:.6f}' if gps_age is not None else ''

        frame_heading_delta = ''
        if gps_sample is not None and gps_sample['heading_ros_time'] is not None:
            frame_heading_delta = (
                f"{abs(frame['ros_time_float'] - gps_sample['heading_ros_time']):.6f}"
            )

        entry['csv_writer'].writerow([
            entry['counter'],
            filename,
            frame['computer_time'],
            frame['ros_time_str'],
            frame['chunk_frame_id'],
            frame['chunk_time'],
            satellite_utc,
            gps_ros_time,
            gps_delta,
            latitude,
            longitude,
            altitude,
            self._format_optional_bool(
                gps_sample.get('midpoint_computed') if gps_sample else None
            ),
            fix_quality,
            self._format_optional(
                gps_sample.get('differential_age_sec') if gps_sample else None,
                3,
            ),
            self._format_optional(
                gps_sample.get('baseline_heading_deg') if gps_sample else None,
                6,
            ),
            self._format_optional(
                gps_sample.get('robot_heading_deg') if gps_sample else None,
                6,
            ),
            self._format_optional(
                gps_sample.get('pitch_deg') if gps_sample else None,
                6,
            ),
            self._format_optional(
                gps_sample.get('heading_stddev_deg') if gps_sample else None,
                6,
            ),
            self._format_optional(
                gps_sample.get('pitch_stddev_deg') if gps_sample else None,
                6,
            ),
            self._format_optional(
                gps_sample.get('heading_baseline_m') if gps_sample else None,
                4,
            ),
            gps_sample.get('heading_satellites_tracked', '') if gps_sample else '',
            gps_sample.get('heading_satellites_used', '') if gps_sample else '',
            gps_sample.get('heading_solution_status', '') if gps_sample else '',
            gps_sample.get('heading_position_type', '') if gps_sample else '',
            gps_sample.get('heading_ros_time_str', '') if gps_sample else '',
            self._format_optional(
                gps_sample.get('heading_age_sec') if gps_sample else None,
                6,
            ),
            frame_heading_delta,
            self._format_optional_bool(
                gps_sample.get('heading_valid') if gps_sample else None
            ),
            self._format_optional_bool(
                gps_sample.get('dual_solution_valid') if gps_sample else None
            ),
            gps_sample.get('dual_solution_reason', '') if gps_sample else '',
            int(sync_ok),
        ])
        entry['csv_file'].flush()

        self.get_logger().info(
            f"camera {entry['serial']} saved {filename}, "
            f"chunk={frame['chunk_frame_id']}, "
            f"gps_delta={gps_delta or 'NA'}s, sync_ok={int(sync_ok)}"
        )
        entry['counter'] += 1

    def _save_batch(self, frames, gps_sample, gps_age, sync_ok):
        futures = [
            self._save_executor.submit(
                self._save_frame,
                frame,
                gps_sample,
                gps_age,
                sync_ok,
            )
            for frame in frames
        ]
        first_error = None
        for future in futures:
            try:
                future.result()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

    def _write_arduino(self, payload: bytes):
        with self._arduino_lock:
            if self.arduino is None or not self.arduino.is_open:
                raise pyserial.SerialException('Arduino serial port is not open')
            self.arduino.write(payload)
            self.arduino.flush()

    def _heartbeat_loop(self):
        while not self._arduino_stop_event.wait(
            self.arduino_heartbeat_period_sec
        ):
            try:
                self._write_arduino(b'h')
            except Exception as exc:
                self.get_logger().error(f'Arduino heartbeat failed: {exc}')
                self._arduino_failed_event.set()
                return

    def _start_arduino(self):
        try:
            self.arduino = pyserial.Serial(
                self.arduino_port,
                self.arduino_baud,
                timeout=1,
                write_timeout=1,
            )
            if self.arduino_startup_delay_sec:
                time.sleep(self.arduino_startup_delay_sec)
            self.arduino.reset_input_buffer()
            self._write_arduino(b's')
            self._arduino_stop_event.clear()
            self._arduino_failed_event.clear()
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                name='arduino-heartbeat',
                daemon=True,
            )
            self._heartbeat_thread.start()
            self.get_logger().info(
                f"Sent 's' to Arduino on {self.arduino_port}; heartbeat enabled"
            )
            return True
        except Exception as exc:
            self.get_logger().error(
                f'Error opening Arduino serial port {self.arduino_port}: {exc}'
            )
            return False

    def run(self):
        """Run acquisition until ROS shutdown or a hardware error occurs."""
        if not self.initialized:
            return False

        self.get_logger().info('Starting acquisition loop...')
        if not self._start_arduino():
            self.cleanup()
            return False

        success = True
        try:
            while rclpy.ok() and not self._arduino_failed_event.is_set():
                frames = []
                for entry in self.cameras:
                    frame = self._acquire_frame(entry)
                    if frame is None:
                        frames = []
                        break
                    frames.append(frame)

                if not frames:
                    continue

                frames, sync_ok = self._resync_batch(frames)
                if not frames:
                    continue

                reference_time = sum(
                    frame['ros_time_float'] for frame in frames
                ) / len(frames)
                gps_sample, gps_age = self._find_best_gps_sample(reference_time)

                if gps_sample is None:
                    self._warn_stale_gps(
                        'No GPS samples available yet; leaving GPS fields blank.'
                    )
                elif gps_age > self.gps_match_max_age_sec:
                    self._warn_stale_gps(
                        f'Closest GPS sample is too old ({gps_age:.3f}s > '
                        f'{self.gps_match_max_age_sec:.3f}s); '
                        'leaving GPS fields blank.'
                    )
                    gps_sample = None

                self._save_batch(frames, gps_sample, gps_age, sync_ok)

            if self._arduino_failed_event.is_set():
                success = False
                self.get_logger().error(
                    'Stopping acquisition because Arduino communication failed'
                )
        except KeyboardInterrupt:
            self.get_logger().info('Stopping acquisition')
        except Exception as exc:
            success = False
            self.get_logger().error(f'Acquisition stopped by an error: {exc}')
        finally:
            self.cleanup()
        return success

    def cleanup(self):
        """Stop triggering and release all serial, camera, and CSV resources."""
        if self._cleaned_up:
            return
        self._cleaned_up = True

        self._arduino_stop_event.set()
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=2.0)

        if self.arduino is not None:
            try:
                self._write_arduino(b'e' * 10)
            except Exception as exc:
                self.get_logger().warn(f'Unable to send Arduino stop command: {exc}')
            try:
                self.arduino.close()
            except Exception:
                pass
            self.arduino = None

        if self._save_executor is not None:
            self._save_executor.shutdown(wait=True)
            self._save_executor = None

        for entry in self.cameras:
            cam = entry['cam']
            try:
                if entry['acquiring']:
                    cam.EndAcquisition()
                    entry['acquiring'] = False
            except Exception as exc:
                self.get_logger().warn(
                    f"{entry['serial']}: EndAcquisition failed: {exc}"
                )
            try:
                if entry['nodemap'] is not None:
                    self._set_enum(
                        entry['nodemap'],
                        entry['serial'],
                        'TriggerMode',
                        'Off',
                        required=False,
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f"{entry['serial']}: disabling trigger failed: {exc}"
                )
            try:
                if entry['initialized']:
                    cam.DeInit()
                    entry['initialized'] = False
            except Exception as exc:
                self.get_logger().warn(
                    f"{entry['serial']}: DeInit failed: {exc}"
                )
            try:
                if entry['csv_file'] is not None:
                    entry['csv_file'].close()
            except Exception:
                pass

        self.cameras.clear()
        cam = None
        entry = None
        try:
            if self.cam_list is not None:
                self.cam_list.Clear()
        except Exception as exc:
            self.get_logger().warn(f'Unable to clear camera list: {exc}')
        self.cam_list = None

        try:
            if self.system is not None:
                self.system.ReleaseInstance()
        except Exception as exc:
            self.get_logger().warn(f'Unable to release Spinnaker system: {exc}')
        self.system = None

    def destroy_node(self):
        """Release hardware before destroying the ROS node."""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    """Run GPS callbacks and the blocking multi-camera acquisition loop."""
    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    success = False

    try:
        node = MultiCameraTriggerNode()
        if node.initialized and rclpy.ok():
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            success = node.run()
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
