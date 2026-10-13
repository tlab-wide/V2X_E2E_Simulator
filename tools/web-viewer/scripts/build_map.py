#!/usr/bin/env python3
"""Convert a Lanelet2 OSM map into the compact GeoJSON used by the web viewer.

Usage:
    python3 build_map.py <lanelet2_map.osm> [output_dir]

Writes into output_dir (default: ../maps):
    lanelet2.geojson   drawable geometry (lane borders, road surface, crosswalks, stop lines, traffic lights)
    lanelet2_meta.json frame conversion (map frame <-> UTM) and traffic-light positions by regulatory element ID
"""
import json
import math
import os
import statistics
import sys
import xml.etree.ElementTree as ET

from pyproj import Transformer

DIGITS = 7  # ~1 cm


def rounded(lat, lon):
    return [round(lon, DIGITS), round(lat, DIGITS)]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    osm_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(__file__), '..', 'maps')
    os.makedirs(out_dir, exist_ok=True)

    root = ET.parse(osm_path).getroot()

    nodes, local = {}, {}
    for n in root.iter('node'):
        lat, lon = float(n.get('lat')), float(n.get('lon'))
        nodes[n.get('id')] = (lat, lon)
        tags = {t.get('k'): t.get('v') for t in n.iter('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            local[n.get('id')] = (float(tags['local_x']), float(tags['local_y']))

    ways, way_tags = {}, {}
    for w in root.iter('way'):
        ways[w.get('id')] = [nd.get('ref') for nd in w.iter('nd')]
        way_tags[w.get('id')] = {t.get('k'): t.get('v') for t in w.iter('tag')}

    relations = []
    for r in root.iter('relation'):
        relations.append((r.get('id'),
                          {t.get('k'): t.get('v') for t in r.iter('tag')},
                          [(m.get('type'), m.get('ref'), m.get('role')) for m in r.iter('member')]))

    def coords(way_id):
        return [rounded(*nodes[n]) for n in ways.get(way_id, []) if n in nodes]

    features = []

    def add(geometry_type, coordinates, **props):
        features.append({'type': 'Feature', 'properties': props,
                         'geometry': {'type': geometry_type, 'coordinates': coordinates}})

    # Lanelets: road surface, crosswalks
    used_bounds = set()
    for rid, tags, members in relations:
        if tags.get('type') != 'lanelet':
            continue
        bounds = {role: ref for _, ref, role in members if role in ('left', 'right')}
        if len(bounds) != 2:
            continue
        left, right = coords(bounds['left']), coords(bounds['right'])
        if len(left) < 2 or len(right) < 2:
            continue
        ring = left + right[::-1] + [left[0]]
        subtype = tags.get('subtype', 'road')
        kind = 'crosswalk' if subtype == 'crosswalk' else ('walkway' if subtype == 'walkway' else 'lane')
        add('Polygon', [ring], kind=kind, id=int(rid))
        used_bounds.update(bounds.values())

    # Lane borders, stop lines, traffic lights
    border_types = {'line_thin', 'line_thick', 'curbstone', 'road_border', 'fence', 'guard_rail', 'wall'}
    traffic_light_ways = {}
    for wid, tags in way_tags.items():
        t = tags.get('type')
        line = coords(wid)
        if len(line) < 2:
            continue
        if t in border_types and wid in used_bounds:
            add('LineString', line, kind='border', style=t, dashed=tags.get('subtype') == 'dashed')
        elif t == 'stop_line':
            add('LineString', line, kind='stop_line', id=int(wid))
        elif t == 'traffic_light':
            traffic_light_ways[wid] = line
            add('LineString', line, kind='traffic_light', id=int(wid))

    # Traffic-light regulatory elements -> position (centre of the referred lights)
    regulatory_lights = {}
    for rid, tags, members in relations:
        if tags.get('subtype') != 'traffic_light':
            continue
        pts = [p for _, ref, role in members if role == 'refers' for p in traffic_light_ways.get(ref, [])]
        if pts:
            regulatory_lights[rid] = [round(sum(p[0] for p in pts) / len(pts), DIGITS),
                                      round(sum(p[1] for p in pts) / len(pts), DIGITS)]
    # Lights are also addressed by their own way ID (TrafficSignalID.WayID in the simulator)
    way_lights = {wid: [round(sum(p[0] for p in pts) / len(pts), DIGITS), round(sum(p[1] for p in pts) / len(pts), DIGITS)]
                  for wid, pts in traffic_light_ways.items()}

    # Map frame (local_x/local_y) -> UTM offset. Autoware/AWSIM map frame = MGRS square local coordinates.
    lat0, lon0 = next(iter(nodes.values()))
    zone = int((lon0 + 180) // 6) + 1
    epsg = (32600 if lat0 >= 0 else 32700) + zone
    to_utm = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}', always_xy=True)
    dx, dy = [], []
    for nid, (x, y) in list(local.items())[::max(1, len(local) // 2000)]:
        lat, lon = nodes[nid]
        e, n = to_utm.transform(lon, lat)
        dx.append(e - x)
        dy.append(n - y)
    offset = [round(statistics.median(dx), 3), round(statistics.median(dy), 3)] if dx else None
    residual = max((abs(a - offset[0]) + abs(b - offset[1]) for a, b in zip(dx, dy)), default=None) if dx else None

    lats = [p[0] for p in nodes.values()]
    lons = [p[1] for p in nodes.values()]
    meta = {
        'source': os.path.basename(osm_path),
        'utm_epsg': epsg,
        'map_to_utm_offset': offset,          # utm = map_xy + offset
        'map_to_utm_max_residual_m': round(residual, 3) if residual is not None else None,
        'bounds': [min(lons), min(lats), max(lons), max(lats)],
        'traffic_lights_by_regulatory_element': regulatory_lights,
        'traffic_lights_by_way': way_lights,
    }

    with open(os.path.join(out_dir, 'lanelet2.geojson'), 'w') as f:
        json.dump({'type': 'FeatureCollection', 'features': features}, f, separators=(',', ':'))
    with open(os.path.join(out_dir, 'lanelet2_meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)

    counts = {}
    for feat in features:
        counts[feat['properties']['kind']] = counts.get(feat['properties']['kind'], 0) + 1
    print(f'{len(features)} features {counts}')
    print(f'EPSG:{epsg} offset {offset} (max residual {meta["map_to_utm_max_residual_m"]} m), '
          f'{len(regulatory_lights)} traffic-light regulatory elements')


if __name__ == '__main__':
    main()
