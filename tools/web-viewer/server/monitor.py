"""ROS 2 side of the viewer: discovers topics, measures their rate and keeps the latest message of each."""
import collections
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# Raw sensor data is listed but never subscribed to (heavy, and not what the viewer is for).
RAW_SENSOR_TYPES = {
    'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Imu', 'sensor_msgs/msg/NavSatFix',
    'sensor_msgs/msg/Range', 'radar_msgs/msg/RadarScan', 'radar_msgs/msg/RadarTracks',
}
IGNORED_TOPICS = {'/rosout', '/parameter_events'}
RATE_WINDOW_S = 3.0

# Best effort matches both reliable and best-effort publishers.
QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                 reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


def category(name, type_name):
    if type_name in RAW_SENSOR_TYPES:
        return 'Raw sensors'
    if type_name.startswith('dm_signal_info_msgs'):
        return 'DM signal info'
    if type_name.startswith('dm_'):
        return 'DM object info'
    if 'TrafficSignal' in type_name or 'TrafficLight' in type_name or 'V2XSignals' in type_name:
        return 'Traffic signals'
    if 'Objects' in type_name:
        return 'Autoware objects'
    if name.startswith('/awsim') or name.startswith('/localization') or name.startswith('/vehicle'):
        return 'Ego vehicle'
    if name.startswith('/control'):
        return 'Control'
    if name == '/clock':
        return 'Clock'
    return 'Other'


class TopicState:
    def __init__(self, name, type_name):
        self.name = name
        self.type_name = type_name
        self.category = category(name, type_name)
        self.raw_sensor = type_name in RAW_SENSOR_TYPES
        self.msg_class = None
        self.error = None
        self.subscription = None
        self.publishers = 0
        self.stamps = collections.deque()
        self.count = 0
        self.bytes = 0
        self.first_rx = None
        self.last_rx = None
        self.latest = None
        self.seq = 0


class TopicMonitor(Node):
    def __init__(self, renderable):
        super().__init__('v2x_web_viewer')
        self._renderable = renderable       # callable(type_name) -> bool
        self.lock = threading.Lock()
        self.topics = {}
        self.create_timer(1.0, self.discover)
        self.discover()

    def discover(self):
        seen = set()
        for name, types in self.get_topic_names_and_types():
            if name in IGNORED_TOPICS or not types:
                continue
            seen.add(name)
            with self.lock:
                state = self.topics.get(name)
                if state is None or state.type_name != types[0]:
                    state = self.topics[name] = TopicState(name, types[0])
            state.publishers = self.count_publishers(name)
            if state.subscription is None and not state.raw_sensor and state.error is None and state.publishers > 0:
                self._subscribe(state)
        with self.lock:
            for name, state in self.topics.items():
                if name not in seen:
                    state.publishers = 0

    def _subscribe(self, state):
        try:
            state.msg_class = get_message(state.type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as e:
            state.error = f'message type not installed ({e.__class__.__name__})'
            return
        state.subscription = self.create_subscription(
            state.msg_class, state.name, lambda data, s=state: self._on_message(s, data), QOS, raw=True)

    def _on_message(self, state, data):
        now = time.monotonic()
        with self.lock:
            state.stamps.append(now)
            while state.stamps and now - state.stamps[0] > RATE_WINDOW_S:
                state.stamps.popleft()
            state.count += 1
            state.bytes += len(data)
            state.first_rx = state.first_rx or now
            state.last_rx = now
            state.latest = data
            state.seq += 1

    def snapshot(self):
        """Health of every topic, safe to JSON-encode."""
        now = time.monotonic()
        rows = []
        with self.lock:
            for s in self.topics.values():
                while s.stamps and now - s.stamps[0] > RATE_WINDOW_S:
                    s.stamps.popleft()
                hz = 0.0
                if len(s.stamps) >= 2 and s.stamps[-1] > s.stamps[0]:
                    hz = (len(s.stamps) - 1) / (s.stamps[-1] - s.stamps[0])
                age = None if s.last_rx is None else now - s.last_rx
                if s.raw_sensor:
                    status = 'raw'
                elif s.error:
                    status = 'unsupported'
                elif s.publishers == 0:
                    status = 'offline'
                elif age is None:
                    status = 'waiting'
                elif age <= max(1.5, 3.0 / hz if hz else 1.5):
                    status = 'live'
                else:
                    status = 'stale'
                rows.append({
                    'name': s.name, 'type': s.type_name, 'category': s.category, 'status': status,
                    'publishers': s.publishers, 'hz': round(hz, 1), 'age_s': None if age is None else round(age, 1),
                    'count': s.count, 'kbps': round(s.bytes / 1024.0 / max(now - s.first_rx, 1e-3), 1) if s.first_rx else 0.0,
                    'renderable': self._renderable(s.type_name), 'error': s.error,
                })
        rows.sort(key=lambda r: (r['category'], r['name']))
        return rows

    def latest_messages(self):
        """[(name, type_name, msg_class, raw_bytes, seq)] of topics that have received something."""
        with self.lock:
            return [(s.name, s.type_name, s.msg_class, s.latest, s.seq)
                    for s in self.topics.values() if s.latest is not None]
