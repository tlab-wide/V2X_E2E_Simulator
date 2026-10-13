"""Coordinate helpers: simulator map frame <-> WGS84, oriented boxes on the map."""
import math

from pyproj import Transformer

EARTH_RADIUS_M = 6378137.0


class MapFrame:
    """Autoware / AWSIM 'map' frame = local coordinates of the MGRS square (utm = map_xy + offset)."""

    def __init__(self, utm_epsg, offset):
        self.offset = offset
        self._to_wgs84 = Transformer.from_crs(f'EPSG:{utm_epsg}', 'EPSG:4326', always_xy=True)

    def to_lonlat(self, x, y):
        return self._to_wgs84.transform(x + self.offset[0], y + self.offset[1])


def destination(lon, lat, bearing_deg, distance_m):
    """Point reached from (lon, lat) after distance_m along bearing_deg (clockwise from north)."""
    b = math.radians(bearing_deg)
    dlat = distance_m * math.cos(b) / EARTH_RADIUS_M
    dlon = distance_m * math.sin(b) / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lon + math.degrees(dlon), lat + math.degrees(dlat)


def box(lon, lat, heading_deg, length_m, width_m):
    """Closed polygon ring of a length x width box centred on (lon, lat), heading clockwise from north."""
    h = math.radians(heading_deg)
    fwd = (math.sin(h), math.cos(h))          # east, north components of the heading
    left = (-math.cos(h), math.sin(h))
    m_per_deg_lat = math.pi * EARTH_RADIUS_M / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(lat))
    ring = []
    for f, s in ((0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)):
        east = f * length_m * fwd[0] + s * width_m * left[0]
        north = f * length_m * fwd[1] + s * width_m * left[1]
        ring.append([round(lon + east / m_per_deg_lon, 8), round(lat + north / m_per_deg_lat, 8)])
    return ring


def yaw_from_quaternion(q):
    """Yaw (rad, counter-clockwise from east / x axis) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_heading(yaw_rad):
    """ENU yaw -> heading in degrees clockwise from north."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0
