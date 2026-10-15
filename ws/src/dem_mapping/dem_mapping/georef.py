"""Georeferencing helpers: WGS84 latitude/longitude -> UTM (pure NumPy).

Conventions
-----------
* A *local* DEM lives in the mapper frame (e.g. the FAST-LIO2 map frame or
  the robot odometry frame).  X/Y are metres, the origin is arbitrary.
* A *geographically referenced* DEM lives in a projected CRS such as UTM.
  Latitude/longitude **must never** be treated as Cartesian metres.

The conversion below implements the standard UTM (transverse Mercator,
Karney/Snyder series) on the WGS84 ellipsoid and is accurate to well under
a millimetre inside a UTM zone - no pyproj/PROJ dependency required.

If the pose stream and the GNSS stream are not expressed in the same frame,
they must first be aligned (see ``gnss_align``); without the extrinsic
calibration between the GNSS antenna, the IMU and the LiDAR such alignment
is impossible and global georeferencing must not be claimed.
"""

import math

import numpy as np

# WGS84 ellipsoid
A = 6378137.0
F = 1.0 / 298.257223563
B = A * (1.0 - F)
E2 = F * (2.0 - F)                       # first eccentricity squared
EP2 = E2 / (1.0 - E2)                    # second eccentricity squared
K0 = 0.9996


def utm_zone(lat, lon):
    """Return (zone number, is_northern) for a WGS84 lat/lon in degrees."""
    zone = int(math.floor((lon + 180.0) / 6.0)) % 60 + 1
    return zone, bool(lat >= 0.0)


def latlon_to_utm(lat, lon, zone=None, northern=None):
    """Vectorised WGS84 lat/lon (degrees) -> UTM easting/northing (metres).

    Returns (easting, northing, zone, is_northern).
    """
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)

    if zone is None:
        zone_f = math.floor((float(np.mean(lon)) + 180.0) / 6.0)
        zone = int(zone_f) % 60 + 1
    if northern is None:
        northern = bool(np.mean(lat) >= 0.0)

    lon0 = math.radians(zone * 6 - 183)      # central meridian
    lat_r = np.radians(lat)
    lon_r = np.radians(lon)

    n = F / (2.0 - F)                        # third flattening (unused, kept
                                             # for clarity of the series)
    # -- standard Snyder (1987) transverse-Mercator series --------------
    e4 = E2 * E2
    e6 = e4 * E2
    def meridional_arc(p):                       # M(p) in metres (Snyder 3-21)
        return A * ((1 - E2/4 - 3*e4/64 - 5*e6/256) * p
                    - (3*E2/8 + 3*e4/32 + 45*e6/1024) * np.sin(2*p)
                    + (15*e4/256 + 45*e6/1024) * np.sin(4*p)
                    - (35*e6/3072) * np.sin(6*p))
    M = meridional_arc(lat_r)
    ep2 = EP2
    nu = A / np.sqrt(1 - E2 * np.sin(lat_r) ** 2)
    rho = A * (1 - E2) / (1 - E2 * np.sin(lat_r) ** 2) ** 1.5
    psi = nu / rho

    dlon = lon_r - lon0
    t_ = np.tan(lat_r)
    t2 = t_ ** 2
    t4 = t2 ** 2
    c1 = ep2 * np.cos(lat_r) ** 2          # eta^2
    c2 = c1 ** 2                            # eta^4
    cosphi = np.cos(lat_r)
    # Snyder (1987) eq. 8-9..8-11 written in powers of dlon
    x = nu * cosphi * (dlon
                       + dlon ** 3 * cosphi ** 2 * (1 - t2 + c1) / 6
                       + dlon ** 5 * cosphi ** 4
                       * (5 - 18 * t2 + t4 + 14 * c2 - 58 * ep2 * t2) / 120)
    y = (M + nu * t_ * dlon ** 2 * cosphi ** 2 / 2
         + nu * t_ * dlon ** 4 * cosphi ** 4
         * (5 - t2 + 9 * c1 + 4 * c2) / 24
         + nu * t_ * dlon ** 6 * cosphi ** 6
         * (61 - 58 * t2 + t4 + 270 * c2 - 330 * ep2 * t2) / 720)

    easting = K0 * x + 500000.0
    if northern:
        northing = K0 * y
    else:
        northing = K0 * y + 10000000.0
    return easting, northing, zone, bool(northern)


def utm_to_latlon(easting, northing, zone, northern=True):
    """Inverse transform (useful for verification / map export)."""
    x = (np.asarray(easting, dtype=np.float64) - 500000.0) / K0
    y = np.asarray(northing, dtype=np.float64)
    if not northern:
        y = y - 10000000.0
    y = y / K0

    e4 = E2 * E2
    e6 = e4 * E2
    e1 = (1 - math.sqrt(1 - E2)) / (1 + math.sqrt(1 - E2))
    M = y
    mu = M / (A * (1 - E2/4 - 3*e4/64 - 5*e6/256))
    phi1 = (mu + (3*e1/2 - 27*e1**3/32) * np.sin(2*mu)
            + (21*e1**2/16 - 55*e1**4/32) * np.sin(4*mu)
            + (151*e1**3/96) * np.sin(6*mu)
            + (1097*e1**4/512) * np.sin(8*mu))
    C1 = EP2 * np.cos(phi1) ** 2
    T1 = np.tan(phi1) ** 2
    N1 = A / np.sqrt(1 - E2 * np.sin(phi1) ** 2)
    R1 = A * (1 - E2) / (1 - E2 * np.sin(phi1) ** 2) ** 1.5
    D = x / N1

    lat = phi1 - (N1 * np.tan(phi1) / R1) * (
        D**2/2 - (5 + 3*T1 + 10*C1 - 4*C1**2 - 9*EP2) * D**4/24
        + (61 + 90*T1 + 298*C1 + 45*T1**2 - 252*EP2 - 3*C1**2) * D**6/720)
    lon = lon0_from_zone(zone) + (
        D - (1 + 2*T1 + C1) * D**3/6
        + (5 - 2*C1 + 28*T1 - 3*C1**2 + 8*EP2 + 24*T1**2) * D**5/120) \
        / np.cos(phi1)
    return np.degrees(lat), np.degrees(lon)


def lon0_from_zone(zone):
    return math.radians(zone * 6 - 183)


def is_geographic(value):
    """Heuristic: does a coordinate pair look like WGS84 degrees?"""
    x, y = float(value[0]), float(value[1])
    return abs(x) <= 180.0 and abs(y) <= 90.0 and abs(x) > 0.001
