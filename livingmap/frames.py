"""Frame translation: private robot coordinates <-> real-world GPS.

Local frame (Writer's private frame):
  - origin = the tunnel entrance (the one point with a known GPS fix)
  - +x     = the direction the Writer faced when it started
  - +y     = 90 degrees to the left of +x (counter-clockwise, right-handed)
  - units  = meters

The entrance anchor is (lat0, lon0, heading_deg), where heading_deg is the compass
bearing of +x, in degrees clockwise from true north.

  East  =  x*sin(psi) - y*cos(psi)
  North =  x*cos(psi) + y*sin(psi)

North/East offsets are converted to degrees with WGS-84 meters-per-degree
formulas evaluated at the anchor latitude (error is negligible over a few hundred m).
"""
from __future__ import annotations

import math


def meters_per_degree(lat_deg: float) -> tuple[float, float]:
    """Return (meters per degree of latitude, meters per degree of longitude) at lat_deg."""
    phi = math.radians(lat_deg)
    m_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return m_lat, m_lon


def local_to_enu(x: float, y: float, heading_deg: float) -> tuple[float, float]:
    """Rotate local (x, y) into (east, north) meters."""
    psi = math.radians(heading_deg)
    east = x * math.sin(psi) - y * math.cos(psi)
    north = x * math.cos(psi) + y * math.sin(psi)
    return east, north


def enu_to_local(east: float, north: float, heading_deg: float) -> tuple[float, float]:
    """Inverse rotation: (east, north) meters -> local (x, y)."""
    psi = math.radians(heading_deg)
    x = east * math.sin(psi) + north * math.cos(psi)
    y = -east * math.cos(psi) + north * math.sin(psi)
    return x, y


def local_to_gps(x: float, y: float, lat0: float, lon0: float, heading_deg: float) -> tuple[float, float]:
    """Local meters -> (lat, lon) in degrees."""
    east, north = local_to_enu(x, y, heading_deg)
    m_lat, m_lon = meters_per_degree(lat0)
    return lat0 + north / m_lat, lon0 + east / m_lon


def gps_to_local(lat: float, lon: float, lat0: float, lon0: float, heading_deg: float) -> tuple[float, float]:
    """(lat, lon) -> local meters. Inverse of local_to_gps."""
    m_lat, m_lon = meters_per_degree(lat0)
    return enu_to_local((lon - lon0) * m_lon, (lat - lat0) * m_lat, heading_deg)
