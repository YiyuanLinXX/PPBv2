"""Metadata preserves measured values, unknowns and coordinate conventions."""

import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from multi_camera_trigger.metadata import (
    camera_readback, encoding_metadata, load_layout, read_setting, write_json,
)


def fake_sdk():
    sdk = Mock()
    for kind in ('Float', 'Integer', 'Boolean', 'Enumeration', 'String'):
        getattr(sdk, f'C{kind}Ptr').side_effect = lambda node: node
    sdk.IsAvailable.side_effect = lambda node: node is not None
    sdk.IsReadable.side_effect = lambda node: node is not None
    sdk.IsWritable.side_effect = lambda node: node is not None
    return sdk


def test_readback_records_applied_value_and_unknowns():
    sdk = fake_sdk()
    exposure = Mock()
    exposure.GetValue.return_value = 253.0
    node_map = Mock()
    node_map.GetNode.side_effect = lambda name: exposure if name == 'ExposureTime' else None
    values = camera_readback(sdk, node_map, node_map)
    assert values['ExposureTime'] == {'value': 253.0, 'status': 'read_back', 'unit': 'us'}
    assert values['Gain']['value'] is None
    assert values['Gain']['status'] == 'unavailable'
    assert values['BalanceRatioRed']['value'] is None


def test_white_balance_readback_restores_selector():
    sdk = fake_sdk()
    selector = Mock()
    selected = [7]
    selector.GetIntValue.side_effect = lambda: selected[0]
    selector.SetIntValue.side_effect = lambda value: selected.__setitem__(0, value)
    entries = {'Red': Mock(), 'Blue': Mock()}
    entries['Red'].GetValue.return_value = 1
    entries['Blue'].GetValue.return_value = 2
    selector.GetEntryByName.side_effect = entries.get
    ratio = Mock()
    ratio.GetValue.side_effect = lambda: {1: 1.34, 2: 2.98}[selected[0]]
    node_map = Mock()
    node_map.GetNode.side_effect = lambda name: {
        'BalanceRatioSelector': selector, 'BalanceRatio': ratio,
    }.get(name)
    values = camera_readback(sdk, node_map, node_map)
    assert values['BalanceRatioRed']['value'] == 1.34
    assert values['BalanceRatioBlue']['value'] == 2.98
    assert selected[0] == 7


def test_read_error_never_substitutes_requested_value():
    sdk = fake_sdk()
    node_map = Mock()
    node_map.GetNode.side_effect = RuntimeError('device unavailable')
    value = read_setting(sdk, node_map, 'ExposureTime', 'Float', 'us')
    assert value['value'] is None
    assert 'device unavailable' in value['reason']


def test_nominal_installation_is_not_calibration():
    path = Path(__file__).resolve().parents[3] / 'config/camera_layout.json'
    layout = load_layout(path)
    assert layout['reference_frame']['axes'] == {'x': 'robot_forward', 'y': 'robot_left', 'z': 'up'}
    assert layout['cameras']['24071782']['position_m'] == [0, 0.545, -0.560]
    assert layout['cameras']['24071775']['position_m'] == [-0.050, 0.545, -0.560]
    assert layout['cameras']['24071784']['position_m'] == [0, -0.545, -0.560]
    assert layout['cameras']['24071783']['position_m'] == [-0.050, -0.545, -0.560]
    assert all(c['rotation_camera_to_reference'] is None for c in layout['cameras'].values())
    assert not layout['calibration']['performed_by_acquisition']


def test_invalid_layout_rejected(tmp_path):
    path = tmp_path / 'layout.json'
    path.write_text(json.dumps({'schema_version': 1, 'reference_frame': {'position_unit': 'm'},
                                'cameras': {'123': {'position_m': [0, 1, 'unknown']}}}))
    with pytest.raises(ValueError, match='position_m'):
        load_layout(path)


def test_encoding_separates_raw_and_rgb():
    raw = encoding_metadata('pgm', 95, 2, 3)
    jpeg = encoding_metadata('jpg', 95, 2, 3)
    assert raw['stored_pixel_encoding'] == 'BayerRG8'
    assert raw['demosaicing'] is None and raw['jpeg_quality'] is None
    assert not raw['lossy_compression']
    assert jpeg['stored_pixel_encoding'] == 'RGB8'
    assert jpeg['lossy_compression'] and jpeg['jpeg_quality'] == 95
    assert jpeg['jpeg_subsampling_name'] == '4:2:0'
    assert not jpeg['stereo_rectified']


def test_json_commit_preserves_old_file_on_invalid_data(tmp_path):
    path = tmp_path / 'metadata.json'
    write_json(path, {'value': 1})
    with pytest.raises(ValueError):
        write_json(path, {'value': float('nan')})
    assert json.loads(path.read_text()) == {'value': 1}
    assert list(tmp_path.iterdir()) == [path]


def test_session_metadata_uses_declared_parameters_without_list_api(tmp_path):
    from types import SimpleNamespace
    from multi_camera_trigger.multi_camera_trigger_node import MultiCameraTriggerNode

    node = MultiCameraTriggerNode.__new__(MultiCameraTriggerNode)
    parameters = {}
    node.declare_parameter = lambda name, default: parameters.setdefault(name, default)
    node._declare_parameters()
    parameters['use_sim_time'] = False
    parameters['jpeg_quality'] = 91
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.output_dir = str(tmp_path)
    node.camera_layout = load_layout('')
    node.cameras = [{'serial': '123', 'image_format': 'jpg'}]
    node.system = Mock()
    node.system.GetLibraryVersion.return_value = SimpleNamespace(major=4, minor=2, type=0, build=88)
    node._created_at_utc = '2026-09-09T00:00:00+00:00'
    node._write_session_metadata()
    saved = json.loads((tmp_path / 'session_metadata.json').read_text())
    assert saved['effective_node_parameters']['jpeg_quality'] == 91
    assert saved['software_versions']['spinnaker_library']['build'] == 88
    assert saved['time_conventions']['host_clock_sync_source'] is None
    assert saved['cameras']['123']['metadata_file'] == '123/camera_metadata.json'
    assert node._metadata_ready
