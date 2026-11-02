"""GNSS log reading (gpspipe raw NMEA) for the snapshot service.

The recordings produced by ``recording/record_all_in_one.sh`` keep the GPS in
a separate text file (``gpspipe -r > ublox7_*.txt``), not in the bag.  This
module parses that file so the service can report **where** the dataset was
recorded and annotate the DEM with the GNSS position.

What it does
------------
* ignores the gpspipe JSON preamble (``{"class":"VERSION",...}``) and device
  chatter, keeping only ``$`` sentences,
* verifies NMEA checksums and counts bad ones,
* decodes ``GGA`` (position fix, quality, satellites, HDOP, MSL altitude) and
  ``RMC``/``ZDA`` (date, so timestamps can be reconstructed),
* keeps only **valid** fixes (GGA quality 1/2/4/5) for the position summary,
* reports an anchor position (median of the valid fixes) and its UTM
  coordinates.

What it deliberately does *not* do
----------------------------------
It does not align the GNSS track to the LiDAR trajectory.  The GPS file is a
separate recording with its own clock, and this service does no scan
registration, so the DEM stays in the sensor frame; the anchor only says
where the data was collected.  Claiming a georeferenced DEM from this input
would be dishonest, so the summary carries a warning that says exactly that.
"""

import math
import os
import statistics
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Dict, List, Optional

__all__ = ["GnssSummary", "GnssFix", "load_gnss", "parse_nmea_text"]

# GGA fix quality -> human label (1 = GPS, 2 = DGPS, 4 = RTK fixed, 5 = RTK float)
_QUALITY_LABEL = {
    0: "invalid",
    1: "GPS",
    2: "DGPS",
    3: "PPS",
    4: "RTK fixed",
    5: "RTK float",
    6: "estimated",
    7: "manual",
    8: "simulation",
}
_VALID_QUALITIES = (1, 2, 4, 5)


@dataclass(frozen=True)
class GnssFix:
    """One valid GGA position fix."""

    utc: str                 # ISO-8601 UTC string (date filled in when known)
    time_s: float            # seconds of day
    lat: float               # decimal degrees, north positive
    lon: float               # decimal degrees, east positive
    alt_m: float             # MSL altitude (metres)
    quality: int
    satellites: int
    hdop: Optional[float]


@dataclass
class GnssSummary:
    """Result of reading one GNSS log (never raises on bad input)."""

    path: str
    available: bool = False
    reason: Optional[str] = None
    size_bytes: int = 0
    sentences: int = 0
    checksum_errors: int = 0
    gga_total: int = 0
    fixes_valid: int = 0
    fixes_invalid: int = 0
    quality_counts: Dict[int, int] = field(default_factory=dict)
    fixes: List[GnssFix] = field(default_factory=list)
    first_utc: Optional[str] = None
    last_utc: Optional[str] = None
    duration_sec: Optional[float] = None
    warnings: List[str] = field(default_factory=list)

    # ------------------------------------------------------------- summary
    def anchor(self) -> Optional[dict]:
        """Median of the valid fixes + its UTM position (None if no fix)."""
        if not self.fixes:
            return None
        lat = statistics.median(f.lat for f in self.fixes)
        lon = statistics.median(f.lon for f in self.fixes)
        alt = statistics.median(f.alt_m for f in self.fixes)
        anchor = {"lat": round(lat, 7), "lon": round(lon, 7),
                  "alt_m": round(alt, 2)}
        anchor.update(_to_utm(lat, lon))
        return anchor

    def health(self) -> dict:
        """Compact block for /api/health."""
        if not self.available:
            return {"enabled": True, "path": self.path, "available": False,
                    "reason": self.reason}
        sats = [f.satellites for f in self.fixes]
        hdops = [f.hdop for f in self.fixes if f.hdop is not None]
        return {
            "enabled": True,
            "path": self.path,
            "available": True,
            "size_bytes": self.size_bytes,
            "sentences": self.sentences,
            "checksum_errors": self.checksum_errors,
            "gga_total": self.gga_total,
            "fixes_valid": self.fixes_valid,
            "fixes_invalid": self.fixes_invalid,
            "fix_rate": (round(self.fixes_valid / self.gga_total, 3)
                         if self.gga_total else None),
            "quality_counts": {str(k): v for k, v in
                               sorted(self.quality_counts.items())},
            "satellites_median": (statistics.median(sats) if sats else None),
            "satellites_max": (max(sats) if sats else None),
            "hdop_median": (round(statistics.median(hdops), 2)
                            if hdops else None),
            "first_fix_utc": self.first_utc,
            "last_fix_utc": self.last_utc,
            "duration_sec": self.duration_sec,
            "anchor": self.anchor(),
            "warnings": list(self.warnings),
        }

    def dem_block(self) -> dict:
        """Annotation carried by every DEM render."""
        anchor = self.anchor()
        return {
            "source": self.path,
            "available": self.available,
            "fixes_valid": self.fixes_valid,
            "anchor": anchor,
            # The grid itself is sensor-frame: say so instead of implying a
            # georeferenced DEM.
            "georeferenced": False,
            "note": ("GNSS anchor of the recording; the DEM grid itself is "
                     "sensor-frame and not georeferenced"
                     if anchor else (self.reason or "no GNSS position")),
        }


# ---------------------------------------------------------------------------
# NMEA decoding
# ---------------------------------------------------------------------------
def _checksum_ok(sentence: str) -> bool:
    """Validate ``$....*HH``.  Returns True for sentences without a checksum."""
    star = sentence.rfind("*")
    if star < 0:
        return True                       # some drivers omit it; accept
    if star + 3 > len(sentence):
        return False
    body = sentence[1:star]
    want = sentence[star + 1:star + 3]
    try:
        got = 0
        for ch in body:
            got ^= ord(ch)
        return want.upper() == f"{got:02X}"
    except Exception:
        return False


def _dm_to_degrees(value: str, hemi: str) -> Optional[float]:
    """NMEA ``ddmm.mmmm`` / ``dddmm.mmmm`` -> signed decimal degrees."""
    if not value:
        return None
    try:
        dot = value.index(".")
        deg_digits = (dot - 2) if dot >= 2 else len(value) - 2
        deg = float(value[:deg_digits])
        minutes = float(value[deg_digits:])
    except (ValueError, IndexError):
        return None
    out = deg + minutes / 60.0
    if hemi in ("S", "W"):
        out = -out
    return out


def _parse_hhmmss(value: str) -> Optional[float]:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    hours = int(v // 10000)
    minutes = int((v % 10000) // 100)
    seconds = v % 100
    return hours * 3600.0 + minutes * 60.0 + seconds


def _parse_date(value: str) -> Optional[datetime]:
    try:
        return datetime.strptime(value, "%d%m%y")
    except (TypeError, ValueError):
        return None


def _iso(day: Optional[datetime], time_s: float) -> str:
    if day is None:
        h = int(time_s // 3600)
        m = int((time_s % 3600) // 60)
        s = time_s % 60
        return f"{h:02d}:{m:02d}:{s:06.3f}Z"
    return (day + timedelta(seconds=time_s)).isoformat(timespec="seconds") + "Z"


def _to_utm(lat: float, lon: float) -> dict:
    """UTM for one point, using the shared dem_mapping implementation."""
    try:
        from dem_mapping.georef import latlon_to_utm
    except Exception:
        return {}
    try:
        e, n, zone, north = latlon_to_utm(lat, lon)
        return {
            "utm_easting": round(float(e), 2),
            "utm_northing": round(float(n), 2),
            "utm_zone": int(zone),
            "utm_hemisphere": "N" if north else "S",
            "epsg": (32600 if north else 32700) + int(zone),
        }
    except Exception:
        return {}


def parse_nmea_text(text: str, path: str = "<memory>") -> GnssSummary:
    """Parse gpspipe/NMEA text into a :class:`GnssSummary`."""
    summary = GnssSummary(path=path, size_bytes=len(text.encode("utf-8", "ignore")))
    fixes: List[GnssFix] = []
    day: Optional[datetime] = None
    seen = set()

    for raw in text.splitlines():
        line = raw.strip().lstrip("\x00")
        if not line.startswith("$"):
            continue                      # gpspipe JSON / device chatter
        summary.sentences += 1
        if not _checksum_ok(line):
            summary.checksum_errors += 1
            continue

        fields = line.split("*")[0].split(",")
        tag = fields[0][3:]                # strip $GP / $GN / $GL ...

        if tag == "ZDA" and len(fields) >= 5:
            d = _parse_date(fields[2] + fields[3] + fields[4][2:])
            if d is not None:
                day = d
            continue

        if tag == "RMC" and len(fields) >= 10:
            d = _parse_date(fields[9])
            if d is not None:
                day = d
            continue

        if tag != "GGA" or len(fields) < 10:
            continue

        summary.gga_total += 1
        try:
            quality = int(fields[6] or 0)
        except ValueError:
            quality = 0
        summary.quality_counts[quality] = summary.quality_counts.get(quality, 0) + 1

        lat = _dm_to_degrees(fields[2], fields[3])
        lon = _dm_to_degrees(fields[4], fields[5])
        time_s = _parse_hhmmss(fields[1])
        if (quality not in _VALID_QUALITIES or lat is None or lon is None
                or time_s is None or abs(lat) > 90 or abs(lon) > 180):
            summary.fixes_invalid += 1
            continue

        try:
            alt = float(fields[9])
        except (ValueError, IndexError):
            alt = 0.0
        try:
            sats = int(fields[7] or 0)
        except ValueError:
            sats = 0
        try:
            hdop = float(fields[8]) if fields[8] else None
        except ValueError:
            hdop = None

        # The u-blox emits two interleaved GGA streams (one with a fix, one
        # without); identical valid fixes are collapsed by time+position.
        key = (round(time_s, 2), round(lat, 7), round(lon, 7))
        if key in seen:
            continue
        seen.add(key)
        fixes.append(GnssFix(utc=_iso(day, time_s), time_s=time_s, lat=lat,
                             lon=lon, alt_m=alt, quality=quality,
                             satellites=sats, hdop=hdop))

    # Keep file order: a gpspipe log is chronological, and sorting by
    # seconds-of-day would put 00:00:01 before 23:59:59 across midnight.
    summary.fixes = fixes
    summary.fixes_valid = len(fixes)

    if fixes:
        first, last = fixes[0], fixes[-1]
        # Re-render ISO strings now that a date may have been seen later.
        summary.first_utc = _iso(day, first.time_s)
        summary.last_utc = _iso(day, last.time_s)
        summary.duration_sec = round(last.time_s - first.time_s, 2)
        if summary.duration_sec < 0:      # log crosses midnight
            summary.duration_sec = round(summary.duration_sec + 86400, 2)
        summary.available = True
    else:
        summary.reason = ("no valid GNSS fix in the file"
                          if summary.gga_total else
                          "no GGA sentences found in the file")

    summary.warnings = _warnings(summary)
    return summary


def _warnings(s: GnssSummary) -> List[str]:
    out: List[str] = []
    if not s.available:
        out.append(f"GNSS position unavailable: {s.reason}")
        return out
    if s.gga_total and s.fixes_valid / s.gga_total < 0.5:
        out.append(f"only {s.fixes_valid}/{s.gga_total} GGA sentences had a "
                   f"valid fix")
    qualities = {f.quality for f in s.fixes}
    if qualities and qualities.isdisjoint((2, 4, 5)):
        out.append("single-point GPS fixes (~2-5 m); not survey grade")
    if s.checksum_errors:
        out.append(f"{s.checksum_errors} NMEA sentences failed checksum")
    out.append("GPS altitude is MSL; the anchor is informational and the DEM "
               "is not georeferenced")
    return out


def load_gnss(path: str) -> Optional[GnssSummary]:
    """Read a GNSS log.  Returns a summary with ``available=False`` on error."""
    if not path:
        return None
    expanded = os.path.expanduser(path)
    if not os.path.isfile(expanded):
        return GnssSummary(path=path, available=False,
                           reason=f"GNSS file not found: {expanded}")
    try:
        with open(expanded, "r", encoding="utf-8", errors="ignore") as fh:
            text = fh.read()
    except OSError as exc:
        return GnssSummary(path=path, available=False,
                           reason=f"cannot read GNSS file: {exc}")
    return parse_nmea_text(text, path=expanded)
