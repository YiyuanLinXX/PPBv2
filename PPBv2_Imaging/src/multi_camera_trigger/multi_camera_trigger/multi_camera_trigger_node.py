#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# multi_camera_trigger_node.py
# ROS2 node: use single hardware trigger to drive multiple cameras concurrently,
# continuously save raw BayerRG8 images as PGM until interrupted,
# record camera chunk timestamps, ROS timestamps, and synchronized GPS/UTC data
# into per-camera CSVs.
# =============================================================================

import csv
import datetime
import json
import os
import threading
import time
from collections import deque

import PySpin
import rclpy
import serial as pyserial
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MultiCameraTriggerNode(Node):
    def __init__(self):
        super().__init__('multi_camera_trigger')

        self.initialized = False
        self._cleaned_up = False
        self._gps_warning_monotonic = 0.0
        self._gps_lock = threading.Lock()
        self.gps_history = deque()
        self.cameras = []
        self.arduino = None
        self.system = None
        self.cam_list = None

        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('arduino_baud', 9600)
        self.declare_parameter('exposure_time', 400.0)
        self.declare_parameter('gain', 5.0)
        self.declare_parameter('wb_red', 1.34)
        self.declare_parameter('wb_blue', 2.98)
        self.declare_parameter('gps_detail_topic', '/gps/fix_detail')
        self.declare_parameter('gps_qos_depth', 200)
        self.declare_parameter('gps_match_max_age_sec', 0.25)
        self.declare_parameter('camera_timeout_ms', 1000)
        self.declare_parameter('resync_max_attempts', 3)

        p = self.get_parameter
        self.output_dir = p('output_dir').get_parameter_value().string_value
        self.arduino_port = p('arduino_port').get_parameter_value().string_value
        self.arduino_baud = p('arduino_baud').get_parameter_value().integer_value
        self.exposure_time = p('exposure_time').get_parameter_value().double_value
        self.gain_value = p('gain').get_parameter_value().double_value
        self.wb_red = p('wb_red').get_parameter_value().double_value
        self.wb_blue = p('wb_blue').get_parameter_value().double_value
        self.gps_detail_topic = p('gps_detail_topic').get_parameter_value().string_value
        self.gps_qos_depth = p('gps_qos_depth').get_parameter_value().integer_value
        self.gps_match_max_age_sec = p('gps_match_max_age_sec').get_parameter_value().double_value
        self.camera_timeout_ms = p('camera_timeout_ms').get_parameter_value().integer_value
        self.resync_max_attempts = p('resync_max_attempts').get_parameter_value().integer_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.gps_qos_depth,
        )
        self.gps_history = deque(maxlen=self.gps_qos_depth)
        self.create_subscription(String, self.gps_detail_topic, self._gps_callback, qos)

        os.makedirs(self.output_dir, exist_ok=True)

        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        if self.cam_list.GetSize() == 0:
            self.get_logger().error('No cameras detected. Exiting.')
            self.cleanup()
            return

        for i in range(self.cam_list.GetSize()):
            cam = self.cam_list[i]
            cam.Init()
            nodemap = cam.GetNodeMap()

            dltl_node = PySpin.CIntegerPtr(nodemap.GetNode('DeviceLinkThroughputLimit'))
            if PySpin.IsAvailable(dltl_node) and PySpin.IsWritable(dltl_node):
                dltl_node.SetValue(100_000_000)
                self.get_logger().info(
                    f'Set Device Link Throughput Limit to {dltl_node.GetValue()} Bytes/sec'
                )
            else:
                self.get_logger().warn('DeviceLinkThroughputLimit not available or not writable.')

            exp_auto = PySpin.CEnumerationPtr(nodemap.GetNode('ExposureAuto'))
            if PySpin.IsAvailable(exp_auto) and PySpin.IsWritable(exp_auto):
                entry_off = exp_auto.GetEntryByName('Off')
                if PySpin.IsAvailable(entry_off) and PySpin.IsReadable(entry_off):
                    exp_auto.SetIntValue(entry_off.GetValue())
                    self.get_logger().info('ExposureAuto set to Off')

            exp_mode = PySpin.CEnumerationPtr(nodemap.GetNode('ExposureMode'))
            if PySpin.IsAvailable(exp_mode) and PySpin.IsWritable(exp_mode):
                entry_timed = exp_mode.GetEntryByName('Timed')
                if PySpin.IsAvailable(entry_timed) and PySpin.IsReadable(entry_timed):
                    exp_mode.SetIntValue(entry_timed.GetValue())
                    self.get_logger().info('ExposureMode set to Timed')

            exp_node = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime'))
            if PySpin.IsWritable(exp_node):
                max_exp = exp_node.GetMax()
                exp_to_set = min(self.exposure_time, max_exp)
                exp_node.SetValue(exp_to_set)
                self.get_logger().info(
                    f'Set ExposureTime to {exp_node.GetValue()} µs (max {max_exp} µs)'
                )
            else:
                self.get_logger().warn('ExposureTime is not writable')

            gain_auto = PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto'))
            if PySpin.IsAvailable(gain_auto) and PySpin.IsWritable(gain_auto):
                gain_auto.SetIntValue(gain_auto.GetEntryByName('Off').GetValue())
                gain_node = PySpin.CFloatPtr(nodemap.GetNode('Gain'))
                if PySpin.IsWritable(gain_node):
                    gain_node.SetValue(min(gain_node.GetMax(), self.gain_value))
                    self.get_logger().info(f'Set Gain to {gain_node.GetValue()} dB')
                else:
                    self.get_logger().warn('Gain is not writable')
            else:
                self.get_logger().warn('GainAuto not available or not writable')

            wb = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceWhiteAuto'))
            if PySpin.IsAvailable(wb) and PySpin.IsWritable(wb):
                wb.SetIntValue(wb.GetEntryByName('Off').GetValue())
                selector = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceRatioSelector'))
                ratio = PySpin.CFloatPtr(nodemap.GetNode('BalanceRatio'))
                if PySpin.IsWritable(selector) and PySpin.IsWritable(ratio):
                    for name, value in [('Red', self.wb_red), ('Blue', self.wb_blue)]:
                        selector.SetIntValue(selector.GetEntryByName(name).GetValue())
                        ratio.SetValue(value)
                    selector.SetIntValue(selector.GetEntryByName('Red').GetValue())
                    red_value = ratio.GetValue()
                    selector.SetIntValue(selector.GetEntryByName('Blue').GetValue())
                    blue_value = ratio.GetValue()
                    self.get_logger().info(
                        f'Set WB ratios: Red={red_value:.2f}, Blue={blue_value:.2f}'
                    )
                else:
                    self.get_logger().warn('Balance selector or ratio not writable')
            else:
                self.get_logger().warn('White balance not available or not writable')

            pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            pixel_format.SetIntValue(pixel_format.GetEntryByName('BayerRG8').GetValue())

            chunk_mode = PySpin.CBooleanPtr(nodemap.GetNode('ChunkModeActive'))
            chunk_mode.SetValue(True)
            chunk_selector = PySpin.CEnumerationPtr(nodemap.GetNode('ChunkSelector'))
            for name in ['FrameID', 'Timestamp']:
                chunk_selector.SetIntValue(chunk_selector.GetEntryByName(name).GetValue())
                chunk_enable = PySpin.CBooleanPtr(nodemap.GetNode('ChunkEnable'))
                chunk_enable.SetValue(True)

            trigger_mode = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
            trigger_mode.SetIntValue(trigger_mode.GetEntryByName('Off').GetValue())
            trigger_selector = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSelector'))
            trigger_selector.SetIntValue(trigger_selector.GetEntryByName('FrameStart').GetValue())
            trigger_source = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
            trigger_source.SetIntValue(trigger_source.GetEntryByName('Line0').GetValue())

            trigger_activation = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerActivation'))
            if PySpin.IsAvailable(trigger_activation) and PySpin.IsWritable(trigger_activation):
                trigger_activation.SetIntValue(
                    trigger_activation.GetEntryByName('FallingEdge').GetValue()
                )
                self.get_logger().info('TriggerActivation set to FallingEdge')
            else:
                self.get_logger().warn('TriggerActivation not available or not writable')

            trigger_mode.SetIntValue(trigger_mode.GetEntryByName('On').GetValue())
            cam.BeginAcquisition()

            tl = cam.GetTLDeviceNodeMap()
            serial_node = PySpin.CStringPtr(tl.GetNode('DeviceSerialNumber'))
            serial = serial_node.GetValue() if PySpin.IsReadable(serial_node) else f'cam{i}'
            cam_dir = os.path.join(self.output_dir, serial)
            os.makedirs(cam_dir, exist_ok=True)

            csv_path = os.path.join(cam_dir, 'Timestamp_GPS.csv')
            csv_file = open(csv_path, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                'Frame ID',
                'Computer Time',
                'ROS Time Stamp(s.ns)',
                'Chunk Frame ID',
                'Chunk Time',
                'Satellite UTC',
                'GPS ROS Time(s.ns)',
                'GPS Match Delta(s)',
                'Latitude',
                'Longitude',
                'Altitude',
                'FixQuality',
                'CrossCameraSyncOk',
            ])
            csv_file.flush()

            self.cameras.append({
                'cam': cam,
                'nodemap': nodemap,
                'dir': cam_dir,
                'serial': serial,
                'csv_file': csv_file,
                'csv_writer': csv_writer,
                'counter': 1,
            })

        self.initialized = True

    def _gps_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            ros_time_str = payload['ros_time']
            sample = {
                'ros_time': float(ros_time_str),
                'ros_time_str': ros_time_str,
                'satellite_utc': payload.get('satellite_utc', ''),
                'latitude': payload.get('latitude'),
                'longitude': payload.get('longitude'),
                'altitude': payload.get('altitude'),
                'fix_quality': payload.get('fix_quality', ''),
            }
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Ignoring malformed GPS detail payload: {exc}')
            return

        with self._gps_lock:
            self.gps_history.append(sample)

    def _acquire_frame(self, entry):
        img = None
        try:
            img = entry['cam'].GetNextImage(self.camera_timeout_ms)
            stamp = self.get_clock().now().to_msg()
            if img.IsIncomplete():
                self.get_logger().warn(
                    f"{entry['serial']}: incomplete image {img.GetImageStatus()}"
                )
                return None

            chunk = img.GetChunkData()
            return {
                'entry': entry,
                'computer_time': datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3],
                'ros_time_str': f'{stamp.sec}.{stamp.nanosec:09d}',
                'ros_time_float': stamp.sec + stamp.nanosec * 1e-9,
                'chunk_frame_id': chunk.GetFrameID(),
                'chunk_time': chunk.GetTimestamp(),
                'width': img.GetWidth(),
                'height': img.GetHeight(),
                'data': bytes(img.GetData()),
            }
        except PySpin.SpinnakerException as exc:
            self.get_logger().warn(f"{entry['serial']}: timeout waiting for image: {exc}")
            return None
        finally:
            if img is not None:
                try:
                    img.Release()
                except PySpin.SpinnakerException:
                    pass

    @staticmethod
    def _chunk_ids_aligned(frames) -> bool:
        return len({frame['chunk_frame_id'] for frame in frames}) == 1

    def _resync_batch(self, frames):
        sync_ok = self._chunk_ids_aligned(frames)
        attempts = 0

        while not sync_ok and attempts < self.resync_max_attempts:
            attempts += 1
            target_chunk_id = max(frame['chunk_frame_id'] for frame in frames)
            mismatch = ', '.join(
                f"{frame['entry']['serial']}:{frame['chunk_frame_id']}" for frame in frames
            )
            self.get_logger().warn(
                f'Chunk Frame ID mismatch ({mismatch}); '
                f'attempting resync to {target_chunk_id} '
                f'({attempts}/{self.resync_max_attempts})'
            )

            new_frames = []
            for frame in frames:
                current = frame
                while current is not None and current['chunk_frame_id'] < target_chunk_id:
                    self.get_logger().warn(
                        f"Dropping unsynced frame from {current['entry']['serial']}: "
                        f"chunk frame {current['chunk_frame_id']} < target {target_chunk_id}"
                    )
                    current = self._acquire_frame(current['entry'])
                if current is None:
                    return [], False
                new_frames.append(current)

            frames = new_frames
            sync_ok = self._chunk_ids_aligned(frames)

        if not sync_ok:
            mismatch = ', '.join(
                f"{frame['entry']['serial']}:{frame['chunk_frame_id']}" for frame in frames
            )
            self.get_logger().error(
                f'Unable to fully resync Chunk Frame IDs after {attempts} attempts: {mismatch}'
            )

        return frames, sync_ok

    def _find_best_gps_sample(self, reference_time):
        with self._gps_lock:
            samples = list(self.gps_history)

        if not samples:
            return None, None

        best = min(samples, key=lambda sample: abs(sample['ros_time'] - reference_time))
        return best, abs(best['ros_time'] - reference_time)

    def _warn_stale_gps(self, message: str) -> None:
        now = time.monotonic()
        if now - self._gps_warning_monotonic >= 2.0:
            self.get_logger().warn(message)
            self._gps_warning_monotonic = now

    def _save_frame(self, frame, gps_sample, gps_age, sync_ok):
        entry = frame['entry']
        output_path = os.path.join(entry['dir'], f"{entry['counter']:06d}.pgm")

        with open(output_path, 'wb') as pgm_file:
            pgm_file.write(b'P5\n')
            pgm_file.write(f"{frame['width']} {frame['height']}\n".encode())
            pgm_file.write(b'255\n')
            pgm_file.write(frame['data'])

        latitude = ''
        longitude = ''
        altitude = ''
        fix_quality = ''
        satellite_utc = ''
        gps_ros_time = ''
        gps_delta = ''
        if gps_sample is not None:
            latitude = f"{gps_sample['latitude']:.9f}"
            longitude = f"{gps_sample['longitude']:.9f}"
            altitude = f"{gps_sample['altitude']:.2f}"
            fix_quality = gps_sample['fix_quality']
            satellite_utc = gps_sample['satellite_utc']
            gps_ros_time = gps_sample['ros_time_str']
            gps_delta = f'{gps_age:.6f}' if gps_age is not None else ''

        entry['csv_writer'].writerow([
            entry['counter'],
            frame['computer_time'],
            frame['ros_time_str'],
            frame['chunk_frame_id'],
            f"{frame['chunk_time']:.5E}",
            satellite_utc,
            gps_ros_time,
            gps_delta,
            latitude,
            longitude,
            altitude,
            fix_quality,
            int(sync_ok),
        ])
        entry['csv_file'].flush()

        self.get_logger().info(
            f"camera {entry['serial']} frame {entry['counter']:06d} "
            f"chunk {frame['chunk_frame_id']} saved "
            f"(gps_delta={gps_delta or 'NA'}s, sync_ok={int(sync_ok)})"
        )
        entry['counter'] += 1

    def run(self):
        if not self.initialized:
            return

        self.get_logger().info('Starting acquisition loop...')

        try:
            self.arduino = pyserial.Serial(self.arduino_port, self.arduino_baud, timeout=1)
            self.arduino.write(b's')
            self.get_logger().info(f"Sent 's' to Arduino on {self.arduino_port}")
        except Exception as exc:
            self.get_logger().error(f'Error opening serial port {self.arduino_port}: {exc}')
            return

        try:
            while rclpy.ok():
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

                reference_time = sum(frame['ros_time_float'] for frame in frames) / len(frames)
                gps_sample, gps_age = self._find_best_gps_sample(reference_time)

                if gps_sample is None:
                    self._warn_stale_gps('No GPS samples available yet; leaving GPS fields blank.')
                elif gps_age is not None and gps_age > self.gps_match_max_age_sec:
                    self._warn_stale_gps(
                        f'Closest GPS sample is too old ({gps_age:.3f}s > '
                        f'{self.gps_match_max_age_sec:.3f}s); leaving GPS fields blank.'
                    )
                    gps_sample = None

                for frame in frames:
                    self._save_frame(frame, gps_sample, gps_age, sync_ok)
        except KeyboardInterrupt:
            self.get_logger().info('Stopping acquisition')
        finally:
            self.cleanup()

    def cleanup(self):
        if self._cleaned_up:
            return
        self._cleaned_up = True

        if self.arduino is not None:
            try:
                for _ in range(10):
                    self.arduino.write(b'e')
                self.arduino.close()
            except Exception:
                pass

        for entry in self.cameras:
            try:
                entry['cam'].EndAcquisition()
                trigger_mode = PySpin.CEnumerationPtr(entry['nodemap'].GetNode('TriggerMode'))
                trigger_mode.SetIntValue(trigger_mode.GetEntryByName('Off').GetValue())
                entry['cam'].DeInit()
            except Exception:
                pass
            try:
                entry['csv_file'].close()
            except Exception:
                pass

        try:
            if self.cam_list is not None:
                self.cam_list.Clear()
        except Exception:
            pass

        try:
            if self.system is not None:
                self.system.ReleaseInstance()
        except Exception:
            pass

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraTriggerNode()
    executor = None
    spin_thread = None

    try:
        if node.initialized and rclpy.ok():
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            node.run()
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
