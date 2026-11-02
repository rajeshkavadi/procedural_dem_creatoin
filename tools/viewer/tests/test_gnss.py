"""Tests for the GNSS/NMEA log reader (pc_snapshot.gnss)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pc_snapshot.gnss import load_gnss, parse_nmea_text


def nmea(body: str) -> str:
    """Build a checksum-valid NMEA sentence from its comma-separated body."""
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"${body}*{checksum:02X}\r"


VALID_GGA = nmea("GPGGA,110852,1728.1051,N,07826.8324,E,1,03,9.51,460.41,M,-73.196,M,,")
INVALID_GGA = nmea("GPGGA,110852.00,,,,,0,03,9.52,,,,,,")
RMC = nmea("GPRMC,110852,A,1728.1051,N,07826.8324,E,0.8017,212.752,301026,,")
ZDA = nmea("GPZDA,110852.00,30,10,2026,00,00")
BAD_CHECKSUM = "$GPGGA,110853,1728.1048,N,07826.8327,E,1,03,9.52,454.89,M,-73.196,M,,*00"


def test_parses_valid_fix_and_skips_gpspipe_json():
    text = "\n".join([
        '{"class":"VERSION","release":"3.17"}',
        '{"class":"DEVICES","devices":[]}',
        ZDA, VALID_GGA, INVALID_GGA, RMC, BAD_CHECKSUM,
    ])
    s = parse_nmea_text(text)
    assert s.available is True
    assert s.fixes_valid == 1
    assert s.fixes_invalid == 1
    assert s.gga_total == 2
    assert s.checksum_errors == 1
    assert s.sentences == 5          # ZDA + 2 GGA + RMC + bad checksum
    assert s.fixes[0].lat == pytest.approx(17.4684183, abs=1e-6)
    assert s.fixes[0].lon == pytest.approx(78.4472067, abs=1e-6)
    assert s.fixes[0].quality == 1
    assert s.fixes[0].satellites == 3
    assert s.fixes[0].hdop == pytest.approx(9.51)
    assert s.fixes[0].alt_m == pytest.approx(460.41)
    assert s.first_utc.startswith("2026-10-30T11:08:52")


def test_no_valid_fix_reports_reason_and_warning():
    s = parse_nmea_text(INVALID_GGA + "\n" + INVALID_GGA.replace("110852", "110853"))
    assert s.available is False
    assert "no valid GNSS fix" in s.reason
    assert s.anchor() is None
    assert any("unavailable" in w for w in s.warnings)
    assert s.health()["available"] is False


def test_missing_file_is_reported_not_raised():
    s = load_gnss("/nonexistent/ublox.txt")
    assert s is not None
    assert s.available is False
    assert "not found" in s.reason


def test_empty_path_means_no_gnss():
    assert load_gnss("") is None


def test_southern_west_coordinates_and_midnight_span():
    south = nmea("GNGGA,235959,3352.0000,S,15112.0000,W,1,09,0.90,12.0,M,0.0,M,,")
    north = nmea("GNGGA,000001,3352.0100,S,15112.0100,W,1,09,0.90,12.5,M,0.0,M,,")
    s = parse_nmea_text(south + "\n" + north)
    assert s.fixes_valid == 2
    assert s.fixes[0].lat < 0 and s.fixes[0].lon < 0
    # 2 s apart across midnight is 2 s, not -86398 s
    assert s.duration_sec == pytest.approx(2.0)


def test_health_and_dem_blocks_expose_anchor():
    s = parse_nmea_text(ZDA + "\n" + VALID_GGA)
    health = s.health()
    assert health["enabled"] is True
    assert health["fixes_valid"] == 1
    anchor = health["anchor"]
    assert anchor["lat"] == pytest.approx(17.4684183, abs=1e-6)
    assert anchor["alt_m"] == pytest.approx(460.41)

    dem = s.dem_block()
    assert dem["available"] is True
    assert dem["georeferenced"] is False
    assert "not georeferenced" in dem["note"]
    assert dem["anchor"]["lat"] == anchor["lat"]


def test_duplicate_interleaved_fixes_are_collapsed():
    s = parse_nmea_text("\n".join([VALID_GGA, VALID_GGA]))
    assert s.gga_total == 2
    assert s.fixes_valid == 1


def test_dem_render_gnss_line_is_sensor_frame_honest():
    s = parse_nmea_text(ZDA + "\n" + VALID_GGA)
    from pc_snapshot.dem import _gnss_line
    text = _gnss_line(s.dem_block())
    assert "17.468418" in text and "E" in text
    assert "not georeferenced" in text
    assert _gnss_line(None) == ""
    assert _gnss_line({"anchor": None}) == ""
