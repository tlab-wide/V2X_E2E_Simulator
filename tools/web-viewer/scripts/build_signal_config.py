#!/usr/bin/env python3
"""Fill in traffic-light head positions of config/traffic_lights.yaml from a Lanelet2 map.

Usage:
    python3 build_signal_config.py <lanelet2_map.osm> [config/traffic_lights.yaml] [--force]

For every approach (方路) of every intersection in the YAML:
- vehicle heads   = the traffic-light poles (Lanelet2 `traffic_light` ways) of the regulatory elements
                    whose lanes enter the intersection from that approach's bearing;
- pedestrian heads = the two ends of the crosswalk across that approach.
Approaches that already have heads are left alone (hand-edited positions are kept) unless --force is given.
The YAML is rewritten in the same layout; comments are regenerated.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
VEHICLE_MATCH_DEG = 35      # lane entry bearing must be this close to the approach bearing
CROSSWALK_MATCH_DEG = 35
STOP_LINE_RADIUS_M = 45    # the lane must end (at its stop line) this close to the intersection centre


def bearing(a, b):
    dy = (b[0] - a[0]) * 111320
    dx = (b[1] - a[1]) * 111320 * math.cos(math.radians(a[0]))
    return (math.degrees(math.atan2(dx, dy)) + 360) % 360


def dist(a, b):
    return math.hypot((a[0] - b[0]) * 111320, (a[1] - b[1]) * 111320 * math.cos(math.radians(a[0])))


def angle_diff(a, b):
    return abs((a - b + 180) % 360 - 180)


class Lanelet2:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.nodes = {n.get('id'): (float(n.get('lat')), float(n.get('lon'))) for n in root.iter('node')}
        self.ways = {w.get('id'): [nd.get('ref') for nd in w.iter('nd')] for w in root.iter('way')}
        self.way_tags = {w.get('id'): {t.get('k'): t.get('v') for t in w.iter('tag')} for w in root.iter('way')}
        self.rels = {r.get('id'): ({t.get('k'): t.get('v') for t in r.iter('tag')},
                                   [(m.get('ref'), m.get('role')) for m in r.iter('member')]) for r in root.iter('relation')}

    def centroid(self, way_ids):
        pts = [self.nodes[n] for w in way_ids for n in self.ways.get(w, []) if n in self.nodes]
        return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)) if pts else None

    def lanelet_ends(self, lid):
        m = {role: ref for ref, role in self.rels[lid][1] if role in ('left', 'right')}
        left, right = self.ways[m['left']], self.ways[m['right']]
        mid = lambda a, b: ((self.nodes[a][0] + self.nodes[b][0]) / 2, (self.nodes[a][1] + self.nodes[b][1]) / 2)
        return mid(left[0], right[0]), mid(left[-1], right[-1])

    def vehicle_signals(self, center, approach_bearing):
        """(regulatory element ids, light way ids) whose lanes enter from approach_bearing."""
        res, light_ways = [], []
        for rid, (tags, members) in self.rels.items():
            if tags.get('type') != 'lanelet' or tags.get('subtype', 'road') != 'road':
                continue
            for ref, role in members:
                if role != 'regulatory_element' or self.rels.get(ref, ({}, []))[0].get('subtype') != 'traffic_light':
                    continue
                start, end = self.lanelet_ends(rid)
                if dist(end, center) > STOP_LINE_RADIUS_M:
                    continue
                if angle_diff(bearing(center, start), approach_bearing) > VEHICLE_MATCH_DEG:
                    continue
                if ref not in res:
                    res.append(ref)
                    for wref, wrole in self.rels[ref][1]:
                        if wrole == 'refers' and self.way_tags.get(wref, {}).get('type') == 'traffic_light' and wref not in light_ways:
                            light_ways.append(wref)
        return [int(r) for r in res], [int(w) for w in light_ways]

    def crosswalk_ends(self, center, approach_bearing):
        """The two outermost ends of the crosswalk(s) across the approach."""
        ends = []
        for rid, (tags, members) in self.rels.items():
            if tags.get('type') != 'lanelet' or tags.get('subtype') != 'crosswalk':
                continue
            m = {role: ref for ref, role in members if role in ('left', 'right')}
            c = self.centroid([m.get('left'), m.get('right')])
            # (bearings are meaningless for crosswalk pieces right at the centre)
            if c is None or not 6 < dist(c, center) < 40 or angle_diff(bearing(center, c), approach_bearing) > CROSSWALK_MATCH_DEG:
                continue
            ends.extend(self.lanelet_ends(rid))
        if len(ends) < 2:
            return []
        a, b = max(((p, q) for i, p in enumerate(ends) for q in ends[i + 1:]), key=lambda pq: dist(*pq))
        return [a, b]


def head(pos, facing):
    return {'lat': round(pos[0], 7), 'lon': round(pos[1], 7), 'facing_deg': round(facing)}


def fill(config, osm, force):
    for inter in config['intersections']:
        center = (inter['center']['lat'], inter['center']['lon'])
        for app in inter['approaches']:
            veh = app.setdefault('vehicle', {})
            if force or not veh.get('heads'):
                res, ways = osm.vehicle_signals(center, app['bearing_deg'])
                veh['lanelet_regulatory_elements'] = res
                veh['lanelet_light_ways'] = ways
                veh['heads'] = [head(osm.centroid([str(w)]), app['bearing_deg']) for w in ways]
            ped = app.get('pedestrian')
            if ped is not None and (force or not ped.get('heads')):
                ped['heads'] = [head(p, app['bearing_deg']) for p in osm.crosswalk_ends(center, app['bearing_deg'])]
            n_veh, n_ped = len(veh.get('heads', [])), len((ped or {}).get('heads', []))
            print(f"{inter['name']:<20} 方路{app['id']}: {n_veh} vehicle head(s), {n_ped} pedestrian head(s)")


def hexlist(ids):
    return '[' + ', '.join(f'0x{i:02x}' for i in ids) + ']'


def dump(config, path):
    out = [
        '# Traffic signals shown by the web viewer (tools/web-viewer).',
        '# Positions are WGS84 latitude/longitude. Edit them freely: scripts/build_signal_config.py only fills',
        '# approaches that have no heads yet (use --force to re-derive everything from the Lanelet2 map).',
        '#',
        '# How live data is matched to a head:',
        '#   dm_signal_ids                -> CooL4 SignalInfo (crp_id + any of these signal IDs); has remaining time',
        '#   lanelet_regulatory_elements  -> v2x_msgs/V2XSignals, Autoware TrafficSignalArray / TrafficLightGroupArray',
        '#   lanelet_light_ways           -> the same messages when they carry Lanelet2 way IDs instead',
        '# Signal IDs follow CooL4 §3.6.3: vehicle = inbound approach << 4 | outbound approach,',
        '# pedestrian = crossed approach << 4.  facing_deg = direction the head faces (clockwise from north).',
        '',
        'intersections:',
    ]
    for inter in config['intersections']:
        out += [f"  - name: {inter['name']}",
                f"    crp_id: {inter['crp_id']}            # 0x{inter['crp_id']:08x}",
                f"    unity_intersection: {inter.get('unity_intersection', '')!r}",
                f"    center: {{lat: {inter['center']['lat']}, lon: {inter['center']['lon']}}}",
                '    approaches:']
        for app in inter['approaches']:
            out += [f"      - id: {app['id']}                  # 方路{app['id']}",
                    f"        direction: {app.get('direction', '')}",
                    f"        bearing_deg: {app['bearing_deg']}        # from the centre towards this leg",
                    '        vehicle:',
                    f"          dm_signal_ids: {hexlist(app['vehicle'].get('dm_signal_ids', []))}",
                    f"          lanelet_regulatory_elements: {app['vehicle'].get('lanelet_regulatory_elements', [])}",
                    f"          lanelet_light_ways: {app['vehicle'].get('lanelet_light_ways', [])}",
                    '          heads:']
            for h in app['vehicle'].get('heads', []) or []:
                out.append(f"            - {{lat: {h['lat']}, lon: {h['lon']}, facing_deg: {h['facing_deg']}}}")
            if not app['vehicle'].get('heads'):
                out[-1] = '          heads: []'
            ped = app.get('pedestrian')
            if ped is not None:
                out += ['        pedestrian:                 # crosswalk across this approach',
                        f"          dm_signal_ids: {hexlist(ped.get('dm_signal_ids', []))}",
                        '          heads:']
                for h in ped.get('heads', []) or []:
                    out.append(f"            - {{lat: {h['lat']}, lon: {h['lon']}, facing_deg: {h['facing_deg']}}}")
                if not ped.get('heads'):
                    out[-1] = '          heads: []'
        out.append('')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not args:
        sys.exit(__doc__)
    path = args[1] if len(args) > 1 else os.path.join(HERE, '..', 'config', 'traffic_lights.yaml')
    with open(path, encoding='utf-8') as f:
        config = yaml.safe_load(f)
    fill(config, Lanelet2(args[0]), '--force' in sys.argv)
    dump(config, path)
    yaml.safe_load(open(path, encoding='utf-8'))   # round-trip check
    print('written', path)


if __name__ == '__main__':
    main()
