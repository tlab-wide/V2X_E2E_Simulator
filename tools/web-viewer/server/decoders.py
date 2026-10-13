"""Turn the simulator's ROS messages into GeoJSON features for the map.

Every decoder returns {'layer': <layer kind>, 'features': [...], 'summary': {...}}.
Feature properties are what the inspector shows when a feature is clicked.
"""
import math

from geo import box, destination, yaw_from_quaternion, yaw_to_heading

# CooL4 DM constants (CooL4 データ連携 PF API 仕様 §5)
DM_CLASS = {0: 'unknown', 1: 'vehicle', 2: 'person', 3: 'animal', 4: 'other'}
DM_MAIN_LIGHT = {0: 'unknown', 1: 'dark', 2: 'flashing red', 3: 'red', 5: 'green', 7: 'yellow', 9: 'flashing yellow'}
DM_PED_MAIN_LIGHT = {**DM_MAIN_LIGHT, 7: 'flashing green'}
DM_LIGHT_COLOR = {0: 'gray', 1: 'dark', 2: 'red', 3: 'red', 5: 'green', 7: 'yellow', 9: 'yellow'}
ITS_EPOCH_UNIX_MS = 1072882800000   # epoch used by the simulator (2004-01-01 00:00 JST), see docs
UNKNOWN_LAT, UNKNOWN_LON = 900000001, 1800000001

# Autoware constants
AW_LABEL = {0: 'unknown', 1: 'car', 2: 'truck', 3: 'bus', 4: 'trailer', 5: 'motorcycle', 6: 'bicycle', 7: 'pedestrian'}
AW_COLOR = {0: 'unknown', 1: 'red', 2: 'yellow', 3: 'green', 4: 'white'}
AW_LIGHT_COLOR = {0: 'gray', 1: 'red', 2: 'yellow', 3: 'green', 4: 'gray'}
AW_SHAPE = {1: 'circle', 2: 'left', 3: 'right', 4: 'up', 5: 'up-left', 6: 'up-right', 7: 'down', 8: 'down-left',
            9: 'down-right', 10: 'cross'}
AW_STATUS = {1: 'off', 2: 'on', 3: 'flashing'}

MAP_FRAMES = {'map', 'obj', 'world', ''}

LAYER_FOR_TYPE = {
    'dm_object_info_msgs/msg/ObjectInfoArray': 'dm_objects',
    'dm_signal_info_msgs/msg/SignalInfoArray': 'dm_signals',
    'v2x_msgs/msg/V2XSignals': 'traffic_signals',
    'autoware_perception_msgs/msg/TrafficSignalArray': 'traffic_signals',
    'autoware_perception_msgs/msg/TrafficLightGroupArray': 'traffic_signals',
    'autoware_perception_msgs/msg/PredictedObjects': 'autoware_objects',
    'autoware_auto_perception_msgs/msg/PredictedObjects': 'autoware_objects',
    'autoware_perception_msgs/msg/DetectedObjects': 'autoware_objects',
    'autoware_auto_perception_msgs/msg/DetectedObjects': 'autoware_objects',
    'autoware_perception_msgs/msg/TrackedObjects': 'autoware_objects',
    'autoware_auto_perception_msgs/msg/TrackedObjects': 'autoware_objects',
    'geometry_msgs/msg/PoseStamped': 'poses',
    'geometry_msgs/msg/PoseWithCovarianceStamped': 'poses',
    'nav_msgs/msg/Odometry': 'poses',
}


def _feature(geometry_type, coordinates, **props):
    return {'type': 'Feature', 'properties': props, 'geometry': {'type': geometry_type, 'coordinates': coordinates}}


def _hex(value, width=0):
    return f'0x{value:0{width}x}'


def _uuid(uuid_msg):
    return ''.join(f'{b:02x}' for b in bytes(uuid_msg.uuid))[:8]


class Decoders:
    def __init__(self, map_frame, meta, intersections):
        self.map_frame = map_frame
        self.lights_by_re = meta.get('traffic_lights_by_regulatory_element', {})
        self.lights_by_way = meta.get('traffic_lights_by_way', {})
        self.intersections = {i['crp_id']: i for i in intersections['intersections']}
        self.vehicle_distance = intersections.get('vehicle_distance_m', 24)
        self.pedestrian_distance = intersections.get('pedestrian_distance_m', 14)
        self.ego = None            # (x, y, yaw) in the map frame, from the ego pose topic
        self.ego_topic = '/awsim/ground_truth/vehicle/pose'

    @staticmethod
    def layer_of(type_name):
        return LAYER_FOR_TYPE.get(type_name)

    def decode(self, topic, type_name, msg):
        layer = LAYER_FOR_TYPE[type_name]
        if layer == 'dm_objects':
            return self._dm_objects(msg)
        if layer == 'dm_signals':
            return self._dm_signals(msg)
        if layer == 'traffic_signals':
            return self._traffic_signals(type_name, msg)
        if layer == 'autoware_objects':
            return self._autoware_objects(type_name, msg)
        return self._pose(topic, type_name, msg)

    # ---------------------------------------------------------------- CooL4 DM object info
    def _dm_objects(self, msg):
        features = []
        for o in msg.array:
            loc = o.object_location
            if loc.latitude.value == UNKNOWN_LAT or loc.longitude.value == UNKNOWN_LON:
                continue
            lat, lon = loc.latitude.value * 1e-7, loc.longitude.value * 1e-7
            orientation = o.orientation.value.value
            direction = o.direction.value.value
            heading = orientation * 0.0125 if orientation != 28800 else (direction * 0.0125 if direction != 28800 else 0.0)

            def size(dim, default):
                v = dim.value.value
                return v * 0.01 if v not in (0, 65535) else default
            length, width, height = size(o.size.length, 1.0), size(o.size.width, 1.0), size(o.size.height, 1.0)

            cls = o.object_class[0].id.value if len(o.object_class) else 0
            oid = o.id.value
            speed = o.speed.value.value
            props = {
                'kind': DM_CLASS.get(cls, str(cls)),
                'label': DM_CLASS.get(cls, '?'),
                'object_id': _hex(oid, 16),
                'id_type': {0: 'roadside unit', 1: 'pseudonym (self)', 2: 'recognised by RSU', 3: 'recognised by vehicle'}[oid >> 62],
                'rsu_id': _hex(oid & 0xFFFFFFFF, 8),
                'sensor_id': (oid >> 48) & 0xFF,
                'object_number': (oid >> 32) & 0xFFFF,
                'lat': round(lat, 7), 'lon': round(lon, 7),
                'heading_deg': round(heading, 2),
                'direction_deg': round(direction * 0.0125, 2) if direction != 28800 else None,
                'speed_mps': round(speed * 0.01, 2) if speed != 16383 else None,
                'size_lwh_m': [round(length, 2), round(width, 2), round(height, 2)],
                'existency': o.existency.value,
                'its_time_ms': o.time.value,
                'sources': [_hex(s.value, 16) for s in o.information_source_list],
            }
            features.append(_feature('Polygon', [box(lon, lat, heading, length, width)], **props))
        return {'layer': 'dm_objects', 'features': features, 'summary': {'objects': len(features)}}

    # ---------------------------------------------------------------- CooL4 DM signal info
    def _dm_signals(self, msg):
        features, unplaced = [], set()
        for s in msg.array:
            crp = s.crp_id.value
            ids = [i.value for i in s.signal_id_list if i.value]
            lights = [(li.main_light.value, li.arrow_light.value, li.min_time_to_change.value, li.max_time_to_change.value)
                      for li in s.signal_light_info_list]
            if not ids or not lights:
                continue
            inter = self.intersections.get(crp)
            if inter is None:
                unplaced.add(crp)
                continue
            main, arrow, tmin, tmax = lights[0]
            pedestrian = all(i & 0x0F == 0 for i in ids)
            names = DM_PED_MAIN_LIGHT if pedestrian else DM_MAIN_LIGHT
            remaining = None if tmin == 65535 else round(tmin / 10.0, 1)
            schedule = [f'{names.get(m, m)} {"?" if a == 65535 else f"{a / 10:.1f}s"}' for m, _, a, _ in lights]
            legs = sorted({i >> 4 for i in ids})
            for leg in legs:
                bearing = inter['approaches'].get(str(leg))
                if bearing is None:
                    continue
                distance = self.pedestrian_distance if pedestrian else self.vehicle_distance
                # Pedestrian markers are pushed sideways so they do not hide the vehicle marker of the same leg.
                lon, lat = destination(*inter['center'], bearing + (12 if pedestrian else 0), distance)
                props = {
                    'kind': 'pedestrian' if pedestrian else 'vehicle',
                    'intersection': inter['name'],
                    'crp_id': crp,
                    'approach': leg,
                    'signal_ids': ' '.join(_hex(i, 2) for i in ids),
                    'main_light': main,
                    'light': names.get(main, str(main)),
                    'color': DM_LIGHT_COLOR.get(main, 'gray'),
                    'arrow_bits': arrow,
                    'remaining_s': remaining,
                    'countdown_stopped': bool(s.count_down_stop_flag.value),
                    'schedule': ' → '.join(schedule),
                    'its_time_ms': s.time.value,
                    'label': ('🚶 ' if pedestrian else '') + (f'{remaining:.0f}s' if remaining is not None else '—'),
                }
                features.append(_feature('Point', [lon, lat], **props))
        return {'layer': 'dm_signals', 'features': features,
                'summary': {'signal_groups': len(msg.array), 'unplaced_crp_ids': sorted(unplaced)}}

    # ---------------------------------------------------------------- Autoware / V2X traffic signals
    def _traffic_signals(self, type_name, msg):
        if type_name.endswith('V2XSignals'):
            signals = [(s.traffic_signal_id, s.elements) for s in msg.traffic_signals.signals]
            station = msg.station_id
        elif type_name.endswith('TrafficLightGroupArray'):
            signals = [(s.traffic_light_group_id, s.elements) for s in msg.traffic_light_groups]
            station = None
        else:
            signals = [(s.traffic_signal_id, s.elements) for s in msg.signals]
            station = None
        features, unplaced = [], 0
        for sid, elements in signals:
            pos = self.lights_by_re.get(str(sid)) or self.lights_by_way.get(str(sid))
            if pos is None:
                unplaced += 1
                continue
            on = [e for e in elements if AW_STATUS.get(e.status) in ('on', 'flashing')] or list(elements)
            color = AW_LIGHT_COLOR.get(on[0].color, 'gray') if on else 'gray'
            desc = ', '.join(f'{AW_COLOR.get(e.color, e.color)} {AW_SHAPE.get(e.shape, e.shape)} {AW_STATUS.get(e.status, e.status)}'
                             for e in elements)
            features.append(_feature('Point', pos, kind='traffic_light', signal_id=sid, color=color,
                                     elements=desc or 'none', station_id=station, label=''))
        return {'layer': 'traffic_signals', 'features': features,
                'summary': {'signals': len(signals), 'unplaced': unplaced}}

    # ---------------------------------------------------------------- Autoware objects
    def _autoware_objects(self, type_name, msg):
        frame = msg.header.frame_id
        relative = frame not in MAP_FRAMES
        if relative and self.ego is None:
            return {'layer': 'autoware_objects', 'features': [],
                    'summary': {'objects': len(msg.objects), 'note': f'frame "{frame}" is ego-relative; waiting for {self.ego_topic}'}}
        features = []
        for o in msg.objects:
            if hasattr(o.kinematics, 'initial_pose_with_covariance'):
                pose = o.kinematics.initial_pose_with_covariance.pose
            else:
                pose = o.kinematics.pose_with_covariance.pose
            x, y, yaw = pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
            if relative:
                ex, ey, eyaw = self.ego
                x, y = ex + x * math.cos(eyaw) - y * math.sin(eyaw), ey + x * math.sin(eyaw) + y * math.cos(eyaw)
                yaw += eyaw
            lon, lat = self.map_frame.to_lonlat(x, y)
            dims = o.shape.dimensions
            label = AW_LABEL.get(o.classification[0].label, '?') if len(o.classification) else 'unknown'
            props = {
                'kind': label, 'label': label,
                'object_id': _uuid(o.object_id) if hasattr(o, 'object_id') else None,
                'existence_probability': round(o.existence_probability, 2),
                'map_xy': [round(x, 2), round(y, 2)],
                'heading_deg': round(yaw_to_heading(yaw), 1),
                'size_lwh_m': [round(dims.x, 2), round(dims.y, 2), round(dims.z, 2)],
                'frame': frame,
            }
            features.append(_feature('Polygon', [box(lon, lat, yaw_to_heading(yaw), max(dims.x, 0.3), max(dims.y, 0.3))], **props))
        return {'layer': 'autoware_objects', 'features': features, 'summary': {'objects': len(features), 'frame': frame}}

    # ---------------------------------------------------------------- poses (ego vehicle, …)
    def _pose(self, topic, type_name, msg):
        if type_name.endswith('Odometry') or type_name.endswith('PoseWithCovarianceStamped'):
            pose = msg.pose.pose
        else:
            pose = msg.pose
        frame = msg.header.frame_id
        x, y, yaw = pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
        if frame not in MAP_FRAMES:
            return {'layer': 'poses', 'features': [], 'summary': {'note': f'frame "{frame}" is not the map frame'}}
        if topic == self.ego_topic:
            self.ego = (x, y, yaw)
        lon, lat = self.map_frame.to_lonlat(x, y)
        heading = yaw_to_heading(yaw)
        props = {'kind': 'ego' if topic == self.ego_topic else 'pose', 'label': 'ego' if topic == self.ego_topic else '',
                 'map_xy': [round(x, 2), round(y, 2)], 'heading_deg': round(heading, 1),
                 'lat': round(lat, 7), 'lon': round(lon, 7), 'frame': frame}
        features = [_feature('Polygon', [box(lon, lat, heading, 4.8, 1.9)], **props)]
        return {'layer': 'poses', 'features': features, 'summary': {'map_xy': props['map_xy']}}
