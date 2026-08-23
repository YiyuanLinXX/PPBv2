"""Unit tests for the ROS-independent NMEA parser."""

import pytest

from multi_camera_trigger.nmea import (
    evaluate_dual_solution,
    midpoint_from_ant1,
    NmeaParseError,
    parse_gga_sentence,
    parse_uniheadinga,
    robot_heading_from_baseline,
)


def _with_checksum(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f'${body}*{checksum:02X}'


@pytest.mark.parametrize('talker', ['GP', 'GN'])
def test_parse_gga_accepts_gps_and_multi_constellation_talkers(talker):
    sentence = _with_checksum(
        f'{talker}GGA,123519.00,4807.038,N,01131.000,E,'
        '4,12,0.7,545.4,M,46.9,M,,'
    )

    fix = parse_gga_sentence(sentence)

    assert fix.satellite_utc == '12:35:19.00'
    assert fix.latitude == pytest.approx(48.1173)
    assert fix.longitude == pytest.approx(11.5166666667)
    assert fix.altitude == pytest.approx(545.4)
    assert fix.fix_quality == 4


def test_parse_gga_handles_no_fix_without_fake_coordinates():
    sentence = _with_checksum(
        'GNGGA,123520.00,4807.038,N,01131.000,E,0,00,99.9,545.4,M,,M,,'
    )

    fix = parse_gga_sentence(sentence)

    assert fix.fix_quality == 0
    assert fix.latitude is None
    assert fix.longitude is None
    assert fix.altitude is None


def test_parse_gga_applies_south_and_west_signs():
    sentence = _with_checksum(
        'GPGGA,010203.00,3345.000,S,07030.000,W,1,08,1.0,10.0,M,0.0,M,,'
    )

    fix = parse_gga_sentence(sentence)

    assert fix.latitude == pytest.approx(-33.75)
    assert fix.longitude == pytest.approx(-70.5)


def test_parse_um982_gga_differential_age():
    fix = parse_gga_sentence(
        '$GNGGA,123519.00,4250.000000,N,07630.000000,W,4,20,'
        '0.7,100.000,M,-30.000,M,0.8,0001*64'
    )

    assert fix.fix_quality == 4
    assert fix.differential_age_sec == pytest.approx(0.8)


def test_parse_gga_rejects_bad_checksum():
    with pytest.raises(NmeaParseError, match='checksum mismatch'):
        parse_gga_sentence(
            '$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00'
        )


def test_parse_gga_rejects_out_of_range_coordinates():
    sentence = _with_checksum(
        'GPGGA,123519,9100.000,N,01131.000,E,1,08,0.9,10.0,M,0.0,M,,'
    )
    with pytest.raises(NmeaParseError, match='valid range'):
        parse_gga_sentence(sentence)


def test_parse_gga_rejects_other_message_types():
    with pytest.raises(NmeaParseError, match='unsupported'):
        parse_gga_sentence('$GPRMC,123519,A,4807.038,N,01131.000,E')


def test_parse_um982_uniheadinga():
    solution = parse_uniheadinga(
        '#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;'
        'SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,'
        '0.1000,0.2000,"0",20,16,18,12,0,00,0,0*5cdb44d1'
    )

    assert solution.solution_status == 'SOL_COMPUTED'
    assert solution.position_type == 'NARROW_INT'
    assert solution.baseline_m == pytest.approx(1.0)
    assert solution.heading_deg == pytest.approx(90.0)
    assert solution.pitch_deg == pytest.approx(10.0)
    assert solution.heading_stddev_deg == pytest.approx(0.1)
    assert solution.satellites_used == 16


def test_uniheadinga_rejects_bad_crc():
    with pytest.raises(NmeaParseError, match='CRC mismatch'):
        parse_uniheadinga(
            '#UNIHEADINGA,0,GPS,FINE,0,0,0,0,0,0;'
            'SOL_COMPUTED,NARROW_INT,1,0,0,0,0.1,0.1,"",10,8,8,8,'
            '0,00,0,0*00000000'
        )


def _valid_heading_solution():
    return parse_uniheadinga(
        '#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;'
        'SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,'
        '0.1000,0.2000,"0",20,16,18,12,0,00,0,0*5cdb44d1'
    )


def _evaluate_heading(**overrides):
    arguments = {
        'heading': _valid_heading_solution(),
        'heading_age_sec': 0.1,
        'fix_quality': 4,
        'differential_age_sec': 0.8,
        'expected_baseline_m': 1.0,
        'baseline_tolerance_m': 0.1,
        'max_heading_stddev_deg': 1.0,
        'max_heading_age_sec': 0.3,
        'max_position_heading_skew_sec': 0.2,
        'max_differential_age_sec': 3.0,
        'minimum_heading_satellites': 6,
    }
    arguments.update(overrides)
    return evaluate_dual_solution(**arguments)


def test_dual_solution_validity_accepts_synchronized_rtk_fixed_data():
    validity = _evaluate_heading()

    assert validity.heading_valid is True
    assert validity.dual_solution_valid is True
    assert validity.reason == 'ok'


def test_dual_solution_distinguishes_heading_from_position_validity():
    validity = _evaluate_heading(fix_quality=1)

    assert validity.heading_valid is True
    assert validity.dual_solution_valid is False
    assert 'GGA quality=1' in validity.reason


def test_dual_solution_rejects_stale_heading():
    validity = _evaluate_heading(heading_age_sec=0.5)

    assert validity.heading_valid is False
    assert validity.dual_solution_valid is False
    assert 'heading stale' in validity.reason


def test_midpoint_moves_half_baseline_north():
    latitude, longitude, altitude = midpoint_from_ant1(
        latitude=42.0,
        longitude=-76.0,
        altitude_m=100.0,
        baseline_heading_deg=0.0,
        pitch_deg=0.0,
        baseline_m=1.0,
    )

    assert latitude > 42.0
    assert longitude == pytest.approx(-76.0, abs=1e-10)
    assert altitude == pytest.approx(100.0)


def test_midpoint_applies_pitch_to_horizontal_and_vertical_offsets():
    latitude, longitude, altitude = midpoint_from_ant1(
        latitude=42.0,
        longitude=-76.0,
        altitude_m=100.0,
        baseline_heading_deg=90.0,
        pitch_deg=30.0,
        baseline_m=1.0,
    )

    assert longitude > -76.0
    assert latitude == pytest.approx(42.0, abs=1e-9)
    assert altitude == pytest.approx(100.25, abs=1e-9)


@pytest.mark.parametrize(
    'baseline_heading, layout_angle, expected',
    [
        (30.0, 0.0, 30.0),
        (210.0, 180.0, 30.0),
        (120.0, 90.0, 30.0),
        (300.0, -90.0, 30.0),
    ],
)
def test_robot_heading_from_antenna_layout(
    baseline_heading,
    layout_angle,
    expected,
):
    assert robot_heading_from_baseline(
        baseline_heading,
        layout_angle,
    ) == pytest.approx(expected)
