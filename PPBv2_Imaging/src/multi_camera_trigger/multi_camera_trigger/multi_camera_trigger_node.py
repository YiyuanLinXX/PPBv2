#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# multi_camera_trigger_node.py
# ROS2 node: use single hardware trigger to drive multiple cameras concurrently,
# continuously save raw BayerRG8 images as PGM until interrupted,
# record camera chunk timestamps, ROS timestamps, and synchronized GPS coords into per-camera CSVs.
# Configure exposure, white balance, and gain at initialization.
# =============================================================================

import os
import time
import datetime
import PySpin
import serial as pyserial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from collections import deque

class MultiCameraTriggerNode(Node):
    def __init__(self):
        super().__init__('multi_camera_trigger')
        # Declare parameters
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('arduino_baud', 9600)
        self.declare_parameter('exposure_time', 400.0)
        self.declare_parameter('gain', 5.0)
        self.declare_parameter('wb_red', 1.34)
        self.declare_parameter('wb_blue', 2.98)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('gps_qos_depth', 50)

        # Read parameters
        p = self.get_parameter
        self.output_dir = p('output_dir').get_parameter_value().string_value
        self.arduino_port = p('arduino_port').get_parameter_value().string_value
        self.arduino_baud = p('arduino_baud').get_parameter_value().integer_value
        self.exposure_time = p('exposure_time').get_parameter_value().double_value
        self.gain_value = p('gain').get_parameter_value().double_value
        self.wb_red = p('wb_red').get_parameter_value().double_value
        self.wb_blue = p('wb_blue').get_parameter_value().double_value
        self.gps_topic = p('gps_topic').get_parameter_value().string_value
        qdepth = p('gps_qos_depth').get_parameter_value().integer_value

        # Prepare GPS subscription with history cache
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qdepth
        )
        self.gps_history = deque(maxlen=qdepth)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_callback, qos)

        # Prepare output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize camera system
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        if self.cam_list.GetSize() == 0:
            self.get_logger().error('No cameras detected. Exiting.')
            self.cleanup()
            rclpy.shutdown()
            return

        # Configure each camera
        self.cameras = []
        for i in range(self.cam_list.GetSize()):
            cam = self.cam_list[i]
            cam.Init()
            nodemap = cam.GetNodeMap()

            # --- DeviceLinkThroughputLimit ---
            dltl_node = PySpin.CIntegerPtr(nodemap.GetNode('DeviceLinkThroughputLimit'))
            if PySpin.IsAvailable(dltl_node) and PySpin.IsWritable(dltl_node):
                dltl_node.SetValue(100_000_000)
                self.get_logger().info(f'Set Device Link Throughput Limit to {dltl_node.GetValue()} Bytes/sec')
            else:
                self.get_logger().warn('DeviceLinkThroughputLimit not available or not writable.')

            # --- Exposure ---
            exp_auto = PySpin.CEnumerationPtr(nodemap.GetNode('ExposureAuto'))
            if PySpin.IsAvailable(exp_auto) and PySpin.IsWritable(exp_auto):
                entry_off = exp_auto.GetEntryByName("Off")
                if PySpin.IsAvailable(entry_off) and PySpin.IsReadable(entry_off):
                    exp_auto.SetIntValue(entry_off.GetValue())
                    self.get_logger().info("ExposureAuto set to Off")

            exp_mode = PySpin.CEnumerationPtr(nodemap.GetNode('ExposureMode'))
            if PySpin.IsAvailable(exp_mode) and PySpin.IsWritable(exp_mode):
                entry_timed = exp_mode.GetEntryByName("Timed")
                if PySpin.IsAvailable(entry_timed) and PySpin.IsReadable(entry_timed):
                    exp_mode.SetIntValue(entry_timed.GetValue())
                    self.get_logger().info("ExposureMode set to Timed")

            exp_node = PySpin.CFloatPtr(nodemap.GetNode('ExposureTime'))
            if PySpin.IsWritable(exp_node):
                max_exp = exp_node.GetMax()
                exp_to_set = min(self.exposure_time, max_exp)
                exp_node.SetValue(exp_to_set)
                self.get_logger().info(f'Set ExposureTime to {exp_node.GetValue()} µs (max {max_exp} µs)')
            else:
                self.get_logger().warn("ExposureTime is not writable")

            # --- Gain ---
            ga = PySpin.CEnumerationPtr(nodemap.GetNode('GainAuto'))
            if PySpin.IsAvailable(ga) and PySpin.IsWritable(ga):
                ga.SetIntValue(ga.GetEntryByName('Off').GetValue())
                gn = PySpin.CFloatPtr(nodemap.GetNode('Gain'))
                if PySpin.IsWritable(gn):
                    gn.SetValue(min(gn.GetMax(), self.gain_value))
                    self.get_logger().info(f'Set Gain to {gn.GetValue()} dB')
                else:
                    self.get_logger().warn('Gain is not writable')
            else:
                self.get_logger().warn('GainAuto not available or not writable')

            # --- White Balance ---
            wb = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceWhiteAuto'))
            if PySpin.IsAvailable(wb) and PySpin.IsWritable(wb):
                wb.SetIntValue(wb.GetEntryByName('Off').GetValue())
                sel = PySpin.CEnumerationPtr(nodemap.GetNode('BalanceRatioSelector'))
                br = PySpin.CFloatPtr(nodemap.GetNode('BalanceRatio'))
                if PySpin.IsWritable(sel) and PySpin.IsWritable(br):
                    for name, val in [('Red', self.wb_red), ('Blue', self.wb_blue)]:
                        sel.SetIntValue(sel.GetEntryByName(name).GetValue())
                        br.SetValue(val)
                    # Debug actual set value
                    sel.SetIntValue(sel.GetEntryByName('Red').GetValue())
                    red_val = br.GetValue()
                    sel.SetIntValue(sel.GetEntryByName('Blue').GetValue())
                    blue_val = br.GetValue()
                    self.get_logger().info(f"Set WB ratios: Red={red_val:.2f}, Blue={blue_val:.2f}")
                else:
                    self.get_logger().warn("Balance selector or ratio not writable")
            else:
                self.get_logger().warn('White balance not available or not writable')

            # --- Pixel Format ---
            pf = PySpin.CEnumerationPtr(nodemap.GetNode('PixelFormat'))
            pf.SetIntValue(pf.GetEntryByName('BayerRG8').GetValue())

            # --- Chunk Data ---
            cm = PySpin.CBooleanPtr(nodemap.GetNode('ChunkModeActive')); cm.SetValue(True)
            cs = PySpin.CEnumerationPtr(nodemap.GetNode('ChunkSelector'))
            for name in ['FrameID', 'Timestamp']:
                cs.SetIntValue(cs.GetEntryByName(name).GetValue())
                ce = PySpin.CBooleanPtr(nodemap.GetNode('ChunkEnable')); ce.SetValue(True)

            # --- Trigger ---
            tm = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
            tm.SetIntValue(tm.GetEntryByName('Off').GetValue())
            ts = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSelector'))
            ts.SetIntValue(ts.GetEntryByName('FrameStart').GetValue())
            src = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
            src.SetIntValue(src.GetEntryByName('Line0').GetValue())

            # Set Trigger Activation to FallingEdge
            ta = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerActivation'))
            if PySpin.IsAvailable(ta) and PySpin.IsWritable(ta):
                ta.SetIntValue(ta.GetEntryByName('FallingEdge').GetValue())
                self.get_logger().info("TriggerActivation set to FallingEdge")
            else:
                self.get_logger().warn("TriggerActivation not available or not writable")

            tm.SetIntValue(tm.GetEntryByName('On').GetValue())

            cam.BeginAcquisition()

            # --- Output Directory and CSV ---
            tl = cam.GetTLDeviceNodeMap()
            sn_node = PySpin.CStringPtr(tl.GetNode('DeviceSerialNumber'))
            serial = sn_node.GetValue() if PySpin.IsReadable(sn_node) else f'cam{i}'
            cam_dir = os.path.join(self.output_dir, serial)
            os.makedirs(cam_dir, exist_ok=True)
            csv_path = os.path.join(cam_dir, 'Timestamp_GPS.csv')
            csv_file = open(csv_path, 'w', newline='')
            header = 'Frame ID,Computer Time,ROS Time Stamp(s.ns),Chunk Frame ID,Chunk Time,Latitude,Longitude,FixQuality\n'
            csv_file.write(header)
            csv_file.flush()

            self.cameras.append({
                'cam': cam,
                'nodemap': nodemap,
                'dir': cam_dir,
                'serial': serial,
                'csv': csv_file,
                'counter': 1
            })

    def _gps_callback(self, msg: NavSatFix):
        t = msg.header.stamp
        ts = t.sec + t.nanosec * 1e-9
        self.gps_history.append((ts, msg.latitude, msg.longitude, msg.status.status))

    def run(self):
        self.get_logger().info('Starting acquisition loop...')

        # Open Arduino serial and send start
        try:
            self.arduino = pyserial.Serial(self.arduino_port, self.arduino_baud, timeout=1)
            self.arduino.write(b's')  # start trigger
            self.get_logger().info(f"Sent 's' to Arduino on {self.arduino_port}")
        except Exception as e:
            self.get_logger().error(f"Error opening serial port {self.arduino_port}: {e}")
            rclpy.shutdown()
            return

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                for entry in self.cameras:
                    try:
                        img = entry['cam'].GetNextImage(1000)
                    except PySpin.SpinnakerException as ex:
                        self.get_logger().warn(f"{entry['serial']}: timeout waiting for image: {ex}")
                        continue

                    if not img.IsIncomplete():
                        t0 = time.perf_counter()
                        comp_str = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]
                        t = self.get_clock().now().to_msg()
                        ros_ts = f"{t.sec}.{t.nanosec:09d}"
                        ros_float = t.sec + t.nanosec * 1e-9
                        cd = img.GetChunkData()
                        cfid = cd.GetFrameID()
                        cts = cd.GetTimestamp()
                        lat = ''; lon = ''; fq = ''
                        if self.gps_history:
                            best = min(self.gps_history, key=lambda x: abs(x[0] - ros_float))
                            _, lat, lon, fq = best
                        w, h = img.GetWidth(), img.GetHeight()
                        data = img.GetData()
                        fn = os.path.join(entry['dir'], f"{entry['counter']:06d}.pgm")
                        with open(fn, 'wb') as f:
                            f.write(b'P5\n')
                            f.write(f'{w} {h}\n'.encode())
                            f.write(b'255\n')
                            f.write(data)

                        t1 = time.perf_counter()
                        elapsed = t1 - t0
                        self.get_logger().info(
                            f"camera {entry['serial']} frame {entry['counter']:06d} saved in {elapsed:.3f}s"
                        )
                        line = (
                            f"{entry['counter']},{comp_str},{ros_ts},"
                            f"{cfid},{cts:.5E},{lat},{lon},{fq}\n"
                        )
                        entry['csv'].write(line)
                        entry['csv'].flush()
                        entry['counter'] += 1
                    img.Release()
        except KeyboardInterrupt:
            self.get_logger().info('Stopping acquisition')
        finally:
            self.cleanup()

    def cleanup(self):
        try:
            for _ in range(10):
                self.arduino.write(b'e')
            self.arduino.close()
        except:
            pass
        for entry in self.cameras:
            try:
                entry['cam'].EndAcquisition()
                tm = PySpin.CEnumerationPtr(entry['nodemap'].GetNode('TriggerMode'))
                tm.SetIntValue(tm.GetEntryByName('Off').GetValue())
                entry['cam'].DeInit()
            except:
                pass
            try:
                entry['csv'].close()
            except:
                pass
        try:
            self.cam_list.Clear()
            self.system.ReleaseInstance()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraTriggerNode()
    if rclpy.ok():
        node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
