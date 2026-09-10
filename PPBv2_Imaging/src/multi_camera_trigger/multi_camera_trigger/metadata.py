"""Capture metadata without inferring unreadable hardware settings."""

import datetime
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def utc_now():
    """Return an explicit UTC host-clock timestamp."""
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write_json(path, value):
    """Commit a complete JSON file atomically."""
    path = Path(path)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    try:
        with temporary.open('x') as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def load_layout(path):
    """Read nominal positions; absent orientation remains unknown."""
    if not path:
        return {'schema_version': 1, 'cameras': {}, 'status': 'not_provided'}
    layout = json.loads(Path(path).read_text())
    if layout.get('schema_version') != 1 or not isinstance(layout.get('cameras'), dict):
        raise ValueError('invalid camera layout schema')
    if layout.get('reference_frame', {}).get('position_unit') != 'm':
        raise ValueError('camera layout positions must use meters')
    for serial, camera in layout['cameras'].items():
        position = camera.get('position_m')
        if (not isinstance(position, list) or len(position) != 3
                or any(type(v) not in (float, int) or not math.isfinite(v)
                       for v in position)):
            raise ValueError(f'invalid position_m for camera {serial}')
    return layout


def read_setting(sdk, nodemap, name, kind, unit=None):
    """Return SDK readback or an explicit unknown value with its reason."""
    result = {'value': None, 'status': 'unavailable'}
    if unit:
        result['unit'] = unit
    try:
        pointer = getattr(sdk, f'C{kind}Ptr')(nodemap.GetNode(name))
        if not sdk.IsAvailable(pointer) or not sdk.IsReadable(pointer):
            return result
        if kind == 'Enumeration':
            value = pointer.GetCurrentEntry().GetSymbolic()
        else:
            value = pointer.GetValue()
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError('nonfinite hardware value')
        result.update(value=value, status='read_back')
    except Exception as exc:
        result['reason'] = str(exc)
    return result


def camera_readback(sdk, nodemap, transport_map):
    """Read parameters after configuration, restoring the WB selector."""
    groups = {
        'Float': ['ExposureTime', 'Gain', 'Gamma', 'BlackLevel',
                  'AcquisitionFrameRate', 'DeviceTemperature'],
        'Integer': ['Width', 'Height', 'OffsetX', 'OffsetY', 'BinningHorizontal',
                    'BinningVertical', 'DecimationHorizontal', 'DecimationVertical',
                    'DeviceLinkThroughputLimit'],
        'Boolean': ['ReverseX', 'ReverseY', 'GammaEnable', 'ChunkModeActive',
                    'AcquisitionFrameRateEnable'],
        'Enumeration': ['PixelFormat', 'PixelColorFilter', 'ExposureAuto',
                        'ExposureMode', 'GainAuto', 'BalanceWhiteAuto',
                        'AcquisitionMode', 'TriggerMode', 'TriggerSelector',
                        'TriggerSource', 'TriggerActivation'],
        'String': ['DeviceFirmwareVersion'],
    }
    units = {'ExposureTime': 'us', 'Gain': 'dB', 'DeviceTemperature': 'degC',
             'AcquisitionFrameRate': 'Hz', 'DeviceLinkThroughputLimit': 'byte/s'}
    values = {name: read_setting(sdk, nodemap, name, kind, units.get(name))
              for kind, names in groups.items() for name in names}
    for name in ['DeviceSerialNumber', 'DeviceModelName', 'DeviceVendorName']:
        values[name] = read_setting(sdk, transport_map, name, 'String')
    unknown = {'value': None, 'status': 'unavailable'}
    values['BalanceRatioRed'] = dict(unknown)
    values['BalanceRatioBlue'] = dict(unknown)
    selector = None
    original = None
    try:
        selector = sdk.CEnumerationPtr(nodemap.GetNode('BalanceRatioSelector'))
        if sdk.IsAvailable(selector) and sdk.IsReadable(selector) and sdk.IsWritable(selector):
            original = selector.GetIntValue()
            for color in ('Red', 'Blue'):
                choice = selector.GetEntryByName(color)
                if sdk.IsAvailable(choice) and sdk.IsReadable(choice):
                    selector.SetIntValue(choice.GetValue())
                    values[f'BalanceRatio{color}'] = read_setting(
                        sdk, nodemap, 'BalanceRatio', 'Float'
                    )
    except Exception as exc:
        values['white_balance_readback_error'] = str(exc)
    finally:
        if original is not None:
            # Failure to restore is an initialization error, not silently ignored.
            selector.SetIntValue(original)
    return values


def encoding_metadata(fmt, jpeg_quality, jpeg_subsampling, png_level):
    """Describe the actual writer path and its information loss."""
    return {
        'file_format': fmt,
        'camera_output_pixel_format': 'BayerRG8',
        'bayer_pattern_before_any_ROI_or_flip': 'RGGB',
        'bits_per_channel': 8,
        'stored_pixel_encoding': 'BayerRG8' if fmt == 'pgm' else 'RGB8',
        'lossy_compression': fmt == 'jpg',
        'demosaicing': None if fmt == 'pgm' else {
            'implementation': 'PySpin.ImageProcessor.Convert',
            'algorithm': 'SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR',
            'output': 'PixelFormat_RGB8',
        },
        'writer': 'binary P5 PGM' if fmt == 'pgm' else 'Pillow',
        'jpeg_quality': jpeg_quality if fmt == 'jpg' else None,
        'jpeg_subsampling': jpeg_subsampling if fmt == 'jpg' else None,
        'jpeg_subsampling_name': {0: '4:4:4', 1: '4:2:2', 2: '4:2:0'}.get(
            jpeg_subsampling) if fmt == 'jpg' else None,
        'png_compress_level': png_level if fmt == 'png' else None,
        'software_resize': False, 'software_crop': False,
        'undistorted': False, 'stereo_rectified': False,
        'note': 'PGM preserves the configured 8-bit camera output, not higher-bit '
                'sensor data. Camera-side settings are listed in actual_settings. '
                'ROI, flips and PixelColorFilter must be considered when decoding Bayer.',
    }


def software_versions():
    """Record installed processing packages and host software."""
    versions = {'python': sys.version, 'platform': platform.platform()}
    for name in ('Pillow', 'numpy', 'spinnaker-python', 'rclpy', 'multi-camera-trigger'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def snapshot_launch(output, workspace, launcher, settings):
    """Save effective launcher values plus the source used for this run."""
    output, workspace = Path(output), Path(workspace)
    snapshot = output / 'config_snapshot'
    snapshot.mkdir()
    files = [Path(launcher), workspace / 'convert_pgm2png.py',
             workspace / 'Arduino/Strobe_Light_Serial_Trigger_Init.ino']
    files += sorted((workspace / 'src/multi_camera_trigger/multi_camera_trigger').glob('*.py'))
    if settings.get('camera_layout_file'):
        files.append(Path(settings['camera_layout_file']))
    manifest = []
    for index, source in enumerate(files):
        target = snapshot / f'{index:02d}_{source.name}'
        shutil.copyfile(source, target)
        manifest.append({'source': str(source), 'snapshot': str(target.relative_to(output)),
                         'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    def git(*args):
        result = subprocess.run(['git', '-C', str(workspace), *args],
                                capture_output=True, text=True, check=False)
        return result.stdout.strip() if result.returncode == 0 else None
    write_json(output / 'launch_metadata.json', {
        'schema_version': 1, 'created_at_host_utc': utc_now(),
        'effective_launcher_settings': settings,
        'git_commit': git('rev-parse', 'HEAD'),
        'git_status': git('status', '--short'),
        'source_files': manifest,
        'note': 'Arduino source is a reference snapshot; its presence does not '
                'verify the firmware currently flashed on the Arduino. '
                'Launcher settings are requested values; use camera_metadata.json '
                'for camera hardware readback.',
    })


if __name__ == '__main__':
    # Called by the launcher before any hardware processes start.
    values = dict(arg.split('=', 1) for arg in sys.argv[4:])
    load_layout(values.get('camera_layout_file', ''))
    snapshot_launch(sys.argv[1], sys.argv[2], sys.argv[3], values)
