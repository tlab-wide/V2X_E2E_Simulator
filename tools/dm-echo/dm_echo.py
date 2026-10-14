#!/usr/bin/env python3
"""Print CooL4 DM object info and signal info in readable form.

    source /opt/ros/humble/setup.bash && source <v2x_e2e_msgs>/install/setup.bash
    python3 tools/dm-echo/dm_echo.py                      # /v2x/object_info and /signal_info
    python3 tools/dm-echo/dm_echo.py /v2x/object_info     # one topic (type found from the graph)
    python3 tools/dm-echo/dm_echo.py --once /signal_info  # one message, then exit

Object IDs are split into their parts (owner bits, sensor ID, object number, RSU ID) and named with the RSUs and sensors
of the default scenario in config/ (rsus.yaml). Units are converted: 1e-7 deg -> deg, 0.01 m -> m, 0.0125 deg -> deg,
0.1 s -> s; TimestampIts -> local time (simulator epoch, see the docs).
"""
import argparse
import datetime
import os
import sys

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
ITS_EPOCH_MS = 1072882800000          # what the simulator adds back to get Unix ms (2004-01-01 00:00 JST)
OWNER = {0b00: 'roadside unit itself', 0b01: 'own pseudonym', 0b10: 'recognised by RSU', 0b11: 'recognised by vehicle'}
OBJECT_CLASS = {0: 'unknown', 1: 'vehicle', 2: 'person', 3: 'animal', 4: 'other'}
VEHICLE_LIGHT = {1: 'dark', 2: 'red flashing', 3: 'red', 5: 'green', 7: 'yellow', 9: 'yellow flashing'}
PEDESTRIAN_LIGHT = {1: 'dark', 2: 'red flashing', 3: 'red', 5: 'green', 7: 'green flashing', 9: 'yellow flashing'}
ARROWS = [(0x40, 'left'), (0x10, 'straight'), (0x04, 'right')]


def load_names(config_root=os.path.join(REPO, 'config')):
    """{rsu_id: (rsu name, {sensor_id: sensor name})} of the default scenario, {crp_id: intersection name}."""
    rsus, crps = {}, {}
    try:
        scenario = open(os.path.join(config_root, 'default_scenario.txt')).read().strip()
        data = yaml.safe_load(open(os.path.join(config_root, 'scenarios', scenario, 'rsus.yaml'), encoding='utf-8'))
        for r in data.get('rsus') or []:
            rid = r['rsu_id'] if isinstance(r['rsu_id'], int) else int(str(r['rsu_id']), 16)
            rsus[rid] = (r['name'], {s['sensor_id']: s['name'] for s in r.get('sensors') or []})
    except (OSError, KeyError, ValueError, TypeError):
        pass
    try:
        lights = yaml.safe_load(open(os.path.join(REPO, 'tools', 'web-viewer', 'config', 'traffic_lights.yaml'), encoding='utf-8'))
        crps = {i['crp_id']: i['name'] for i in lights.get('intersections') or []}
    except (OSError, KeyError, TypeError):
        pass
    return rsus, crps


def split_object_id(value):
    """64-bit CooL4 object ID -> dict of its parts (spec §3.3.3; the simulator uses owner 10)."""
    owner = value >> 62
    if owner in (0b00, 0b10):
        return {'owner': owner, 'sensor_id': (value >> 48) & 0xFF, 'object_number': (value >> 32) & 0xFFFF,
                'rsu_id': value & 0xFFFFFFFF}
    return {'owner': owner, 'number': (value >> 50) & 0xFFF, 'pseudonym': value & ((1 << 50) - 1)}


def describe_object_id(value, rsus):
    p = split_object_id(value)
    if 'rsu_id' not in p:
        return f'0x{value:016x} ({OWNER[p["owner"]]}, pseudonym {p["pseudonym"]})'
    rsu, sensors = rsus.get(p['rsu_id'], (None, {}))
    where = f'{rsu} / {sensors.get(p["sensor_id"], "sensor " + str(p["sensor_id"]))}' if rsu else \
        f'RSU 0x{p["rsu_id"]:08x} / sensor {p["sensor_id"]}'
    return f'0x{value:016x} = object {p["object_number"]} seen by {where} (RSU 0x{p["rsu_id"]:08x}, sensor {p["sensor_id"]})'


def its_time(ms):
    return datetime.datetime.fromtimestamp((ms + ITS_EPOCH_MS) / 1000.0).strftime('%H:%M:%S.%f')[:-3]


def format_objects(msg, rsus):
    lines = [f'ObjectInfoArray: {len(msg.array)} objects']
    for o in msg.array:
        cls = OBJECT_CLASS.get(o.object_class[0].id.value, o.object_class[0].id.value) if o.object_class else 'unknown'
        loc = o.object_location
        p = split_object_id(o.id.value)
        lines.append(f'- object {p.get("object_number", "?")} ({cls}), {its_time(o.time.value)}, existence {o.existency.value}/101')
        lines.append(f'    position  {loc.latitude.value / 1e7:.7f}, {loc.longitude.value / 1e7:.7f}, '
                     f'altitude {loc.altitude.value / 100:.2f} m')
        lines.append(f'    heading   {o.orientation.value.value * 0.0125:.1f} deg, moving to {o.direction.value.value * 0.0125:.1f} deg '
                     f'at {o.speed.value.value / 100:.2f} m/s ({o.speed.value.value * 0.036:.1f} km/h)')
        lines.append(f'    size      {o.size.length.value.value / 100:.2f} x {o.size.width.value.value / 100:.2f} x '
                     f'{o.size.height.value.value / 100:.2f} m (length x width x height)')
        lines.append(f'    id        {describe_object_id(o.id.value, rsus)}')
        for s in o.information_source_list[1:]:
            lines.append(f'    also      {describe_object_id(s.value, rsus)}')
    return '\n'.join(lines)


def format_signals(msg, crps):
    lines = [f'SignalInfoArray: {len(msg.array)} signal groups']
    for s in msg.array:
        ids = [i.value for i in s.signal_id_list if i.value]
        pedestrian = all(i & 0x0F == 0 for i in ids)
        names = PEDESTRIAN_LIGHT if pedestrian else VEHICLE_LIGHT
        what = ', '.join(f'crossing approach {i >> 4}' if pedestrian else f'approach {i >> 4} -> {i & 0x0F}' for i in ids)
        crp = s.crp_id.value
        lines.append(f'- {crps.get(crp, "CRP")} (crp 0x{crp:08x}) {"pedestrian" if pedestrian else "vehicle"} '
                     f'[{" ".join(f"0x{i:02x}" for i in ids)}] = {what}, {its_time(s.time.value)}')
        steps = []
        for n, li in enumerate(s.signal_light_info_list):
            t = li.min_time_to_change.value
            dur = 'unknown' if t == 65535 else f'{t / 10:.1f} s'
            arrows = '+'.join(a for bit, a in ARROWS if li.arrow_light.value & bit)
            light = names.get(li.main_light.value, str(li.main_light.value)) + (f' ({arrows} arrow)' if arrows else '')
            steps.append(f'{light} {"for another " if n == 0 else "for "}{dur}')
        lines.append('    ' + ' -> '.join(steps) + ('  [countdown stopped]' if s.count_down_stop_flag.value else ''))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('topics', nargs='*', default=['/v2x/object_info', '/signal_info'])
    ap.add_argument('--once', action='store_true', help='print one message per topic, then exit')
    ap.add_argument('--config-root', default=os.path.join(REPO, 'config'))
    args = ap.parse_args()

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    try:
        from dm_object_info_msgs.msg import ObjectInfoArray
        from dm_signal_info_msgs.msg import SignalInfoArray
    except ImportError:
        sys.exit('dm_object_info_msgs / dm_signal_info_msgs not found: source the v2x_e2e_msgs workspace first')

    rsus, crps = load_names(args.config_root)
    rclpy.init()
    node = rclpy.create_node('dm_echo')
    pending = set(args.topics)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # matches reliable and best-effort publishers

    def callback(topic, fmt):
        def cb(msg):
            print(f'=== {topic}\n{fmt(msg)}\n', flush=True)
            pending.discard(topic)
        return cb

    for topic in args.topics:
        is_signal = 'signal' in topic
        node.create_subscription(SignalInfoArray if is_signal else ObjectInfoArray, topic,
                                 callback(topic, (lambda m: format_signals(m, crps)) if is_signal else (lambda m: format_objects(m, rsus))),
                                 qos)
    try:
        while rclpy.ok() and (pending or not args.once):
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
