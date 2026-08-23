"""ROS-independent protocol helpers for UM982 GGA and UNIHEADINGA data."""

from dataclasses import dataclass
import math
from typing import Optional

from pyproj import Geod


WGS84_GEOD = Geod(ellps='WGS84')


class NmeaParseError(ValueError):
    """Raised when an NMEA sentence is malformed or unsupported."""


@dataclass(frozen=True)
class GgaFix:
    """Fields used by the imaging nodes from an NMEA GGA sentence."""

    satellite_utc: str
    latitude: Optional[float]
    longitude: Optional[float]
    altitude: Optional[float]
    fix_quality: int
    differential_age_sec: Optional[float]


@dataclass(frozen=True)
class HeadingSolution:
    """Dual-antenna solution reported by UM982 UNIHEADINGA."""

    solution_status: str
    position_type: str
    baseline_m: float
    heading_deg: float
    pitch_deg: float
    heading_stddev_deg: float
    pitch_stddev_deg: float
    satellites_tracked: int
    satellites_used: int


@dataclass(frozen=True)
class DualSolutionValidity:
    """Validity of heading alone and of the synchronized position/heading pair."""

    heading_valid: bool
    dual_solution_valid: bool
    reason: str


def _validate_checksum(sentence: str) -> str:
    if not sentence.startswith('$'):
        raise NmeaParseError('sentence does not start with $')

    payload = sentence[1:]
    if '*' not in payload:
        return payload

    body, checksum_text = payload.rsplit('*', 1)
    if len(checksum_text) != 2:
        raise NmeaParseError('invalid checksum field')

    try:
        expected = int(checksum_text, 16)
    except ValueError as exc:
        raise NmeaParseError('checksum is not hexadecimal') from exc

    actual = 0
    for character in body:
        actual ^= ord(character)
    if actual != expected:
        raise NmeaParseError(
            f'checksum mismatch: expected {expected:02X}, calculated {actual:02X}'
        )
    return body


def _format_utc(raw_utc: str) -> str:
    if not raw_utc:
        return ''
    if len(raw_utc) < 6:
        raise NmeaParseError('UTC field is too short')
    try:
        numeric_utc = float(raw_utc)
    except ValueError as exc:
        raise NmeaParseError('UTC field is not numeric') from exc
    if not math.isfinite(numeric_utc):
        raise NmeaParseError('UTC field is not finite')
    return f'{raw_utc[0:2]}:{raw_utc[2:4]}:{raw_utc[4:]}'


def _parse_coordinate(
    raw_value: str,
    hemisphere: str,
    degree_digits: int,
    max_degrees: int,
    positive_hemisphere: str,
    negative_hemisphere: str,
) -> Optional[float]:
    if not raw_value:
        return None
    if hemisphere not in (positive_hemisphere, negative_hemisphere):
        raise NmeaParseError(f'invalid hemisphere {hemisphere!r}')

    try:
        degrees = float(raw_value[:degree_digits])
        minutes = float(raw_value[degree_digits:])
    except ValueError as exc:
        raise NmeaParseError('invalid coordinate') from exc
    if not math.isfinite(degrees) or not math.isfinite(minutes):
        raise NmeaParseError('coordinate is not finite')
    if not 0.0 <= minutes < 60.0:
        raise NmeaParseError('coordinate minutes are outside [0, 60)')
    if degrees > max_degrees or (degrees == max_degrees and minutes > 0.0):
        raise NmeaParseError('coordinate degrees are outside the valid range')

    coordinate = degrees + minutes / 60.0
    if hemisphere == negative_hemisphere:
        coordinate = -coordinate
    return coordinate


def parse_gga_sentence(sentence: str) -> GgaFix:
    """Parse a GP/GN/etc. GGA sentence and validate its checksum when present."""
    body = _validate_checksum(sentence.strip())
    parts = body.split(',')
    message_id = parts[0]
    if len(message_id) != 5 or not message_id.endswith('GGA'):
        raise NmeaParseError(f'unsupported NMEA message {message_id!r}')
    if len(parts) < 10:
        raise NmeaParseError('GGA sentence has too few fields')

    try:
        fix_quality = int(parts[6] or '0')
    except ValueError as exc:
        raise NmeaParseError('fix quality is not an integer') from exc

    latitude = _parse_coordinate(parts[2], parts[3], 2, 90, 'N', 'S')
    longitude = _parse_coordinate(parts[4], parts[5], 3, 180, 'E', 'W')

    altitude = None
    if parts[9]:
        try:
            altitude = float(parts[9])
        except ValueError as exc:
            raise NmeaParseError('altitude is not numeric') from exc
        if not math.isfinite(altitude):
            raise NmeaParseError('altitude is not finite')

    if fix_quality <= 0:
        latitude = None
        longitude = None
        altitude = None

    differential_age_sec = None
    if len(parts) > 13 and parts[13]:
        try:
            differential_age_sec = float(parts[13])
        except ValueError as exc:
            raise NmeaParseError('differential age is not numeric') from exc
        if not math.isfinite(differential_age_sec):
            raise NmeaParseError('differential age is not finite')

    return GgaFix(
        satellite_utc=_format_utc(parts[1]),
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        fix_quality=fix_quality,
        differential_age_sec=differential_age_sec,
    )


def parse_uniheadinga(sentence: str) -> HeadingSolution:
    """Parse and CRC-check a UM982 UNIHEADINGA ASCII message."""
    text = sentence.strip()
    if not text.startswith('#UNIHEADINGA,'):
        raise NmeaParseError('not a UNIHEADINGA message')
    if ';' not in text or '*' not in text:
        raise NmeaParseError('incomplete UNIHEADINGA message')

    crc_payload, crc_text = text[1:].rsplit('*', 1)
    try:
        expected_crc = int(crc_text[:8], 16)
    except ValueError as exc:
        raise NmeaParseError('UNIHEADINGA CRC is not hexadecimal') from exc
    if _unicore_crc32(crc_payload.encode('ascii')) != expected_crc:
        raise NmeaParseError('UNIHEADINGA CRC mismatch')

    fields = crc_payload.split(';', 1)[1].split(',')
    if len(fields) < 14:
        raise NmeaParseError(
            f'UNIHEADINGA body has {len(fields)} fields; expected at least 14'
        )

    try:
        solution = HeadingSolution(
            solution_status=fields[0].strip(),
            position_type=fields[1].strip(),
            baseline_m=float(fields[2]),
            heading_deg=float(fields[3]) % 360.0,
            pitch_deg=float(fields[4]),
            heading_stddev_deg=float(fields[6]),
            pitch_stddev_deg=float(fields[7]),
            satellites_tracked=int(fields[9]),
            satellites_used=int(fields[10]),
        )
    except (ValueError, IndexError) as exc:
        raise NmeaParseError('invalid UNIHEADINGA field') from exc

    numeric_fields = (
        solution.baseline_m,
        solution.heading_deg,
        solution.pitch_deg,
        solution.heading_stddev_deg,
        solution.pitch_stddev_deg,
    )
    if not all(math.isfinite(value) for value in numeric_fields):
        raise NmeaParseError('UNIHEADINGA contains a non-finite value')
    return solution


def robot_heading_from_baseline(
    baseline_heading_deg: float,
    antenna_baseline_angle_deg: float,
    heading_offset_deg: float = 0.0,
) -> float:
    """Convert ANT1-to-ANT2 heading to robot-forward true heading."""
    return (
        baseline_heading_deg
        - antenna_baseline_angle_deg
        + heading_offset_deg
    ) % 360.0


def midpoint_from_ant1(
    latitude: float,
    longitude: float,
    altitude_m: float,
    baseline_heading_deg: float,
    pitch_deg: float,
    baseline_m: float,
) -> tuple[float, float, float]:
    """Move from ANT1 to the 3-D midpoint of the ANT1-to-ANT2 baseline."""
    half_length = baseline_m / 2.0
    pitch_rad = math.radians(pitch_deg)
    horizontal_distance = half_length * math.cos(pitch_rad)
    vertical_distance = half_length * math.sin(pitch_rad)
    midpoint_lon, midpoint_lat, _ = WGS84_GEOD.fwd(
        longitude,
        latitude,
        baseline_heading_deg,
        horizontal_distance,
    )
    return midpoint_lat, midpoint_lon, altitude_m + vertical_distance


def evaluate_dual_solution(
    heading: HeadingSolution,
    heading_age_sec: float,
    fix_quality: int,
    differential_age_sec: Optional[float],
    expected_baseline_m: float,
    baseline_tolerance_m: float,
    max_heading_stddev_deg: float,
    max_heading_age_sec: float,
    max_position_heading_skew_sec: float,
    max_differential_age_sec: float,
    minimum_heading_satellites: int,
) -> DualSolutionValidity:
    """Apply the same dual-GNSS validity rules used by PPBv2 Navigation."""
    heading_reasons = []
    if heading_age_sec > max_heading_age_sec:
        heading_reasons.append('heading stale')
    if heading.solution_status != 'SOL_COMPUTED':
        heading_reasons.append(f'heading status={heading.solution_status}')
    if heading.position_type != 'NARROW_INT':
        heading_reasons.append(f'heading type={heading.position_type}')
    if abs(heading.baseline_m - expected_baseline_m) > baseline_tolerance_m:
        heading_reasons.append('baseline outside tolerance')
    if heading.heading_stddev_deg > max_heading_stddev_deg:
        heading_reasons.append('heading stddev too high')
    if heading.satellites_used < minimum_heading_satellites:
        heading_reasons.append('too few heading satellites')

    dual_reasons = list(heading_reasons)
    if heading_age_sec > max_position_heading_skew_sec:
        dual_reasons.append('position/heading skew too large')
    if fix_quality != 4:
        dual_reasons.append(f'GGA quality={fix_quality}')
    if differential_age_sec is None:
        dual_reasons.append('differential age unavailable')
    elif differential_age_sec > max_differential_age_sec:
        dual_reasons.append('differential age too high')

    return DualSolutionValidity(
        heading_valid=not heading_reasons,
        dual_solution_valid=not dual_reasons,
        reason='ok' if not dual_reasons else '; '.join(dual_reasons),
    )


def _unicore_crc32(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = ((crc >> 8) & 0x00FFFFFF) ^ _crc32_value((crc ^ byte) & 0xFF)
    return crc


def _crc32_value(value: int) -> int:
    crc = value
    for _ in range(8):
        if crc & 1:
            crc = (crc >> 1) ^ 0xEDB88320
        else:
            crc >>= 1
    return crc
