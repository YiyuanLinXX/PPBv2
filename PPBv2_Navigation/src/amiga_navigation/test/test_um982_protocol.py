"""Tests for UM982 parsing and midpoint geometry."""

import math

import pytest

from amiga_navigation.um982_protocol import (
    heading_to_enu_yaw,
    midpoint_from_master,
    parse_gga,
    parse_uniheadinga,
    robot_heading_from_baseline,
)


def test_parse_rtk_fixed_gga():
    fix = parse_gga(
        '$GNGGA,123519.00,4250.000000,N,07630.000000,W,4,20,'
        '0.7,100.000,M,-30.000,M,0.8,0001*64'
    )
    assert fix.quality == 4
    assert math.isclose(fix.latitude, 42.8333333333)
    assert math.isclose(fix.longitude, -76.5)
    assert math.isclose(fix.differential_age_sec, 0.8)


def test_parse_uniheading():
    solution = parse_uniheadinga(
        '#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;'
        'SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,'
        '0.1000,0.2000,"0",20,16,18,12,0,00,0,0*5cdb44d1'
    )
    assert solution.solution_status == 'SOL_COMPUTED'
    assert solution.position_type == 'NARROW_INT'
    assert solution.baseline_m == 1.0
    assert solution.heading_deg == 90.0
    assert solution.pitch_deg == 10.0
    assert solution.satellites_used == 16


def test_uniheading_rejects_bad_crc():
    with pytest.raises(ValueError, match='CRC mismatch'):
        parse_uniheadinga(
            '#UNIHEADINGA,0,GPS,FINE,0,0,0,0,0,0;'
            'SOL_COMPUTED,NARROW_INT,1,0,0,0,0.1,0.1,"",10,8,8,8'
            ',0,00,0,0*00000000'
        )


def test_midpoint_moves_half_baseline_north():
    lat, lon, altitude = midpoint_from_master(
        42.0, -76.0, 100.0, 0.0, 0.0, 1.0
    )
    assert lat > 42.0
    assert math.isclose(lon, -76.0, abs_tol=1e-10)
    assert math.isclose(altitude, 100.0)


def test_pitch_changes_horizontal_and_vertical_offset():
    lat, lon, altitude = midpoint_from_master(
        42.0, -76.0, 100.0, 90.0, 30.0, 1.0
    )
    assert lon > -76.0
    assert math.isclose(lat, 42.0, abs_tol=1e-9)
    assert math.isclose(altitude, 100.25, abs_tol=1e-9)


def test_heading_to_enu_yaw():
    assert math.isclose(heading_to_enu_yaw(0.0), math.pi / 2.0)
    assert math.isclose(heading_to_enu_yaw(90.0), 0.0)
    assert math.isclose(heading_to_enu_yaw(180.0), -math.pi / 2.0)


@pytest.mark.parametrize(
    ('baseline_heading', 'layout_angle', 'robot_heading'),
    [
        (30.0, 0.0, 30.0),       # ANT2 in front
        (210.0, 180.0, 30.0),    # ANT2 behind
        (120.0, 90.0, 30.0),     # ANT2 to robot-right
        (300.0, -90.0, 30.0),    # ANT2 to robot-left
    ],
)
def test_robot_heading_for_common_antenna_layouts(
    baseline_heading,
    layout_angle,
    robot_heading,
):
    assert math.isclose(
        robot_heading_from_baseline(baseline_heading, layout_angle),
        robot_heading,
    )


def test_robot_heading_applies_calibration_after_layout():
    assert math.isclose(
        robot_heading_from_baseline(120.0, 90.0, 2.5),
        32.5,
    )
