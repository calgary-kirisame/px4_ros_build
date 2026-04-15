import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'px4_single_plan'
DEFAULT_FIELD_CONFIG_FILE = 'michigan_stadium_gps.yaml'
EARTH_RADIUS_M = 6_378_137.0


def package_share_path(filename: str, package_name: str = PACKAGE_NAME) -> str:
    return os.path.join(get_package_share_directory(package_name), filename)


def load_package_yaml(filename: str, package_name: str = PACKAGE_NAME):
    path = package_share_path(filename, package_name)
    with open(path, 'r') as stream:
        return yaml.safe_load(stream) or {}, path


def wrap_degrees_180(angle_deg: float) -> float:
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def _as_float(value, default: float = 0.0) -> float:
    if value is None:
        return float(default)
    return float(value)


def _parse_geo_point(point_cfg, label: str):
    if not isinstance(point_cfg, dict):
        raise ValueError(f'Missing GPS point for {label}')
    return {
        'lat': _as_float(point_cfg.get('lat')),
        'lon': _as_float(point_cfg.get('lon')),
        'alt_m': _as_float(point_cfg.get('alt_m', point_cfg.get('alt', 0.0)), 0.0),
    }


def geodetic_delta_m(origin, point):
    avg_lat_rad = math.radians((float(origin['lat']) + float(point['lat'])) / 2.0)
    east_m = math.radians(float(point['lon']) - float(origin['lon'])) * EARTH_RADIUS_M * math.cos(avg_lat_rad)
    north_m = math.radians(float(point['lat']) - float(origin['lat'])) * EARTH_RADIUS_M
    return east_m, north_m


def distance_m(point_a, point_b) -> float:
    east_m, north_m = geodetic_delta_m(point_a, point_b)
    return math.hypot(east_m, north_m)


def bearing_deg(point_a, point_b) -> float:
    lat1_rad = math.radians(float(point_a['lat']))
    lon1_rad = math.radians(float(point_a['lon']))
    lat2_rad = math.radians(float(point_b['lat']))
    lon2_rad = math.radians(float(point_b['lon']))
    delta_lon = lon2_rad - lon1_rad
    y_term = math.sin(delta_lon) * math.cos(lat2_rad)
    x_term = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    )
    return (math.degrees(math.atan2(y_term, x_term)) + 360.0) % 360.0


def _normalize_vector(east_m: float, north_m: float):
    magnitude = math.hypot(east_m, north_m)
    if magnitude <= 1e-9:
        raise ValueError('Field axis magnitude is too small to normalize')
    return east_m / magnitude, north_m / magnitude


def _average_points(points):
    return {
        'lat': sum(point['lat'] for point in points) / len(points),
        'lon': sum(point['lon'] for point in points) / len(points),
        'alt_m': sum(point.get('alt_m', 0.0) for point in points) / len(points),
    }


def build_field_setup(config, source_path: str = ''):
    field_cfg = dict(config.get('field', {}) or {})
    launch_points = dict(config.get('launch_points', {}) or {})
    field_setup = {
        'source_path': source_path,
        'field': field_cfg,
        'launch_points': launch_points,
        'width_m': _as_float(field_cfg.get('width_m', 0.0), 0.0),
        'height_m': _as_float(field_cfg.get('height_m', 0.0), 0.0),
        'heading_deg': _as_float(field_cfg.get('heading_deg', 0.0), 0.0),
        'heading_rad': math.radians(_as_float(field_cfg.get('heading_deg', 0.0), 0.0)),
        'forward_bearing_deg': wrap_degrees_180(-_as_float(field_cfg.get('heading_deg', 0.0), 0.0)),
        'forward_yaw_deg': wrap_degrees_180(-_as_float(field_cfg.get('heading_deg', 0.0), 0.0)),
        'has_gps_geometry': False,
        'origin_gps': None,
        'home_gps': None,
        'corners_gps': {},
        'x_axis_east': 1.0,
        'x_axis_north': 0.0,
        'y_axis_east': 0.0,
        'y_axis_north': 1.0,
    }

    corners_cfg = field_cfg.get('corners_gps') or field_cfg.get('gps_corners') or {}
    if corners_cfg:
        southwest = _parse_geo_point(corners_cfg.get('southwest'), 'field.corners_gps.southwest')
        southeast = _parse_geo_point(corners_cfg.get('southeast'), 'field.corners_gps.southeast')
        northwest = _parse_geo_point(corners_cfg.get('northwest'), 'field.corners_gps.northwest')
        northeast_cfg = corners_cfg.get('northeast')
        northeast = _parse_geo_point(northeast_cfg, 'field.corners_gps.northeast') if northeast_cfg else None

        south_axis_east, south_axis_north = geodetic_delta_m(southwest, southeast)
        north_axis_east, north_axis_north = geodetic_delta_m(southwest, northwest)
        x_axis_east, x_axis_north = _normalize_vector(south_axis_east, south_axis_north)
        y_axis_east, y_axis_north = _normalize_vector(north_axis_east, north_axis_north)
        heading_deg = math.degrees(math.atan2(x_axis_north, x_axis_east))
        forward_bearing = bearing_deg(southwest, northwest)

        field_setup.update({
            'width_m': distance_m(southwest, southeast),
            'height_m': distance_m(southwest, northwest),
            'heading_deg': heading_deg,
            'heading_rad': math.radians(heading_deg),
            'forward_bearing_deg': forward_bearing,
            'forward_yaw_deg': wrap_degrees_180(forward_bearing),
            'has_gps_geometry': True,
            'origin_gps': southwest,
            'corners_gps': {
                'southwest': southwest,
                'southeast': southeast,
                'northwest': northwest,
                'northeast': northeast,
            },
            'x_axis_east': x_axis_east,
            'x_axis_north': x_axis_north,
            'y_axis_east': y_axis_east,
            'y_axis_north': y_axis_north,
        })

        home_cfg = field_cfg.get('home') or field_cfg.get('home_gps')
        if home_cfg:
            field_setup['home_gps'] = _parse_geo_point(home_cfg, 'field.home')
        else:
            corner_points = [southwest, southeast, northwest]
            if northeast is not None:
                corner_points.append(northeast)
            field_setup['home_gps'] = _average_points(corner_points)
    else:
        home_cfg = field_cfg.get('home') or field_cfg.get('home_gps')
        if home_cfg:
            field_setup['home_gps'] = _parse_geo_point(home_cfg, 'field.home')

    return field_setup


def load_field_setup(field_config_file: str, package_name: str = PACKAGE_NAME):
    config, path = load_package_yaml(field_config_file, package_name=package_name)
    return build_field_setup(config, source_path=path)


def gps_to_field_xy(lat: float, lon: float, field_setup):
    if not field_setup.get('has_gps_geometry'):
        raise ValueError('GPS launch points require field GPS geometry in the field config file')

    east_m, north_m = geodetic_delta_m(
        field_setup['origin_gps'],
        {'lat': float(lat), 'lon': float(lon), 'alt_m': 0.0},
    )
    field_x_m = east_m * field_setup['x_axis_east'] + north_m * field_setup['x_axis_north']
    field_y_m = east_m * field_setup['y_axis_east'] + north_m * field_setup['y_axis_north']
    return field_x_m, field_y_m


def field_xy_to_local(field_x: float, field_y: float, origin_x: float = 0.0, origin_y: float = 0.0, heading_rad: float = 0.0):
    rel_east = float(field_x) - float(origin_x)
    rel_north = float(field_y) - float(origin_y)

    if abs(float(heading_rad)) < 1e-9:
        return rel_north, rel_east

    cos_h = math.cos(float(heading_rad))
    sin_h = math.sin(float(heading_rad))
    local_east = rel_east * cos_h - rel_north * sin_h
    local_north = rel_east * sin_h + rel_north * cos_h
    return local_north, local_east


def launch_point_to_home_local(launch_point, field_setup):
    home_gps = field_setup.get('home_gps')
    if home_gps and ('lat' in launch_point) and ('lon' in launch_point):
        east_m, north_m = geodetic_delta_m(
            home_gps,
            {
                'lat': float(launch_point['lat']),
                'lon': float(launch_point['lon']),
                'alt_m': 0.0,
            },
        )
        return north_m, east_m

    origin_x = 0.0
    origin_y = 0.0
    if home_gps and field_setup.get('has_gps_geometry'):
        origin_x, origin_y = gps_to_field_xy(
            float(home_gps['lat']),
            float(home_gps['lon']),
            field_setup,
        )

    return field_xy_to_local(
        launch_point['x'],
        launch_point['y'],
        origin_x,
        origin_y,
        field_setup.get('heading_rad', 0.0),
    )


def _vehicle_namespace_candidates(vehicle_namespace: str):
    stripped = str(vehicle_namespace or '').strip('/')
    candidates = []
    if stripped:
        candidates.append(stripped)
    candidates.extend(['default', 'shared'])
    return candidates


def resolve_vehicle_launch_point(vehicle_namespace: str, launch_points, field_setup):
    launch_points = launch_points or {}
    for key in _vehicle_namespace_candidates(vehicle_namespace):
        launch_cfg = launch_points.get(key)
        if not isinstance(launch_cfg, dict):
            continue

        yaw_deg = _as_float(launch_cfg.get('yaw_deg', field_setup.get('forward_yaw_deg', 0.0)), field_setup.get('forward_yaw_deg', 0.0))
        if 'lat' in launch_cfg and 'lon' in launch_cfg:
            field_x_m, field_y_m = gps_to_field_xy(launch_cfg['lat'], launch_cfg['lon'], field_setup)
            return {
                'x': field_x_m,
                'y': field_y_m,
                'yaw_deg': yaw_deg,
                'lat': _as_float(launch_cfg['lat']),
                'lon': _as_float(launch_cfg['lon']),
            }

        return {
            'x': _as_float(launch_cfg.get('x', launch_cfg.get('x_m', 0.0)), 0.0),
            'y': _as_float(launch_cfg.get('y', launch_cfg.get('y_m', 0.0)), 0.0),
            'yaw_deg': yaw_deg,
        }

    return {
        'x': 0.0,
        'y': 0.0,
        'yaw_deg': _as_float(field_setup.get('forward_yaw_deg', 0.0), 0.0),
    }
