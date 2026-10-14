"""Settings API of the web viewer: read, check and write the simulator configuration (config/ of the repository).

config/defaults/*.yaml            shared defaults (sensor types, noise, network)
config/scenarios/<name>/*.yaml    one folder per scenario, scenario.yaml lists its files
config/factory/...                the shipped version of every file ("reset to default" copies it back)

The simulator reads the files when Play starts, so saved changes apply on the next run.
"""
import json
import os
import re
import shutil

import yaml
from aiohttp import web
from pyproj import Transformer

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
CONFIG = os.path.join(ROOT, 'config')
FACTORY = os.path.join(CONFIG, 'factory')
DEFAULT_FILES = {'sensor_types': 'sensor_types.yaml', 'noise': 'noise.yaml', 'network': 'network.yaml'}
SCENARIO_KEYS = ['rsus', 'vehicles', 'pedestrians', 'teleports', 'raw_sensors']
NAME_RE = re.compile(r'^[A-Za-z0-9_\-]+$')
HEX_KEYS = {'rsu_id'}


def _hex_strings(node):
    """YAML reads 0x12020001 as an int; keep RSU IDs as hex strings like the simulator."""
    if isinstance(node, dict):
        for k, v in node.items():
            if k in HEX_KEYS and isinstance(v, int):
                node[k] = f'0x{v:08x}'
            else:
                _hex_strings(v)
    elif isinstance(node, list):
        for v in node:
            _hex_strings(v)
    return node


def _read(path):
    with open(path, encoding='utf-8') as f:
        text = f.read()
    return _hex_strings(yaml.safe_load(text) or {}), _header(text)


def _header(text):
    lines = []
    for line in text.splitlines():
        if line.startswith('#'):
            lines.append(line)
        elif line.strip():
            break
    return '\n'.join(lines)


def _strip_private(node):
    """Drop keys the page adds for display (start with _)."""
    if isinstance(node, dict):
        return {k: _strip_private(v) for k, v in node.items() if not k.startswith('_')}
    if isinstance(node, list):
        return [_strip_private(v) for v in node]
    return node


class _Dumper(yaml.SafeDumper):
    """Short lists of plain values and the noise selection in flow style, as in the shipped files."""


def _represent_list(dumper, data):
    flow = len(data) <= 12 and all(isinstance(v, (str, int, float, bool)) for v in data)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=flow)


def _represent_dict(dumper, data):
    flow = set(data) == {'position', 'rotation', 'dimension', 'probability'}
    return dumper.represent_mapping('tag:yaml.org,2002:map', data.items(), flow_style=flow)


_Dumper.add_representer(list, _represent_list)
_Dumper.add_representer(dict, _represent_dict)
_HEX_QUOTED = re.compile(r"^(\s*(?:- )?(?:%s): )'(0x[0-9A-Fa-f]+)'$" % '|'.join(HEX_KEYS), re.M)


def _comments(text):
    """Comments of an existing file below its header: blocks before a line and comments at the end of a line,
    keyed by the line they belong to (content without the comment)."""
    lines = text.splitlines()
    i = 0
    while i < len(lines) and (lines[i].startswith('#') or not lines[i].strip()):
        i += 1
    before, inline, block = [], {}, []
    for line in lines[i:]:
        st = line.strip()
        if st.startswith('#'):
            block.append(st)
            continue
        if not st:
            continue
        m = re.match(r'^(.*?\S)\s+(#.*)$', line)
        if m and m.group(1).count("'") % 2 == 0 and m.group(1).count('"') % 2 == 0:
            inline[m.group(1).strip()] = m.group(2)
            st = m.group(1).strip()
        if block:
            before.append((st, block))
            block = []
    return before, inline


def _write(path, data, header):
    old = open(path, encoding='utf-8').read() if os.path.exists(path) else ''
    before, inline = _comments(old)
    body = yaml.dump(data, Dumper=_Dumper, sort_keys=False, allow_unicode=True, default_flow_style=False, width=140)
    body = _HEX_QUOTED.sub(r'\1\2', body)
    out, pending = [], list(before)
    for line in body.splitlines():
        st = line.strip()
        for n, (anchor, block) in enumerate(pending):
            if anchor == st:
                indent = line[:len(line) - len(line.lstrip())]
                out.extend(indent + c for c in block)
                del pending[n]
                break
        out.append(line + ('   ' + inline[st] if st in inline else ''))
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        if header:
            f.write(header.rstrip() + '\n\n')
        f.write('\n'.join(out) + '\n')
    os.replace(tmp, path)


class Settings:
    def __init__(self, utm_epsg, offset):
        self.offset = offset
        self._to_ll = Transformer.from_crs(f'EPSG:{utm_epsg}', 'EPSG:4326', always_xy=True)
        self._to_utm = Transformer.from_crs('EPSG:4326', f'EPSG:{utm_epsg}', always_xy=True)

    # ------------------------------------------------------------------ files
    @staticmethod
    def scenarios():
        d = os.path.join(CONFIG, 'scenarios')
        if not os.path.isdir(d):
            return []
        return sorted(n for n in os.listdir(d) if os.path.isfile(os.path.join(d, n, 'scenario.yaml')))

    @staticmethod
    def default_scenario():
        p = os.path.join(CONFIG, 'default_scenario.txt')
        return open(p).read().strip() if os.path.exists(p) else None

    @staticmethod
    def _scenario_dir(name):
        if not NAME_RE.match(name or ''):
            raise web.HTTPBadRequest(text=f'bad scenario name {name!r}')
        return os.path.join(CONFIG, 'scenarios', name)

    def _file_path(self, scenario, key):
        """(path relative to config/, absolute path) of a scenario or defaults file."""
        if key in DEFAULT_FILES:
            rel = os.path.join('defaults', DEFAULT_FILES[key])
        elif key == 'scenario':
            rel = os.path.join('scenarios', scenario, 'scenario.yaml')
        elif key in SCENARIO_KEYS:
            meta, _ = _read(os.path.join(self._scenario_dir(scenario), 'scenario.yaml'))
            fname = meta.get(key) or f'{key}.yaml'
            if os.path.basename(fname) != fname:
                raise web.HTTPBadRequest(text=f'bad file name {fname!r}')
            rel = os.path.join('scenarios', scenario, fname)
        else:
            raise web.HTTPBadRequest(text=f'unknown file {key!r}')
        return rel, os.path.join(CONFIG, rel)

    def load(self, scenario):
        out = {'scenario': scenario, 'files': {}, 'headers': {}, 'factory': {}}
        for key in ['scenario', *SCENARIO_KEYS, *DEFAULT_FILES]:
            rel, path = self._file_path(scenario, key)
            if os.path.exists(path):
                out['files'][key], out['headers'][key] = _read(path)
            else:
                out['files'][key] = None
            out['factory'][key] = os.path.exists(os.path.join(FACTORY, rel))
        return out

    # ------------------------------------------------------------------ checks (same rules as ConfigLoader.cs)
    @staticmethod
    def validate(files):
        errors = []
        st, noise, net = files.get('sensor_types') or {}, files.get('noise') or {}, files.get('network') or {}
        pseudo = {t.get('name') for t in st.get('pseudo_sensor_types') or []}
        raw_types = {t.get('name') for t in st.get('raw_sensor_types') or []}
        noises = {n.get('name') for n in noise.get('noise_profiles') or []}
        nets = {n.get('name') for n in net.get('network_profiles') or []}
        for kind, names in (('pseudo sensor type', [t.get('name') for t in st.get('pseudo_sensor_types') or []]),
                            ('noise profile', [n.get('name') for n in noise.get('noise_profiles') or []]),
                            ('network profile', [n.get('name') for n in net.get('network_profiles') or []])):
            dup = {n for n in names if names.count(n) > 1}
            if dup:
                errors.append(f'{kind} names used twice: {", ".join(map(str, dup))}')
        for t in st.get('pseudo_sensor_types') or []:
            if t.get('kind') not in ('lidar', 'camera'):
                errors.append(f'pseudo sensor type {t.get("name")}: kind must be lidar or camera')

        def check_noise(where, sel):
            for k in ('position', 'rotation', 'dimension', 'probability'):
                if (sel or {}).get(k, 'Default') not in noises:
                    errors.append(f'{where}: unknown noise profile {(sel or {}).get(k)!r}')

        rsus = files.get('rsus')
        rsu_names = set()
        if rsus:
            ids = set()
            for r in rsus.get('rsus') or []:
                name = r.get('name')
                if name in rsu_names:
                    errors.append(f'RSU name {name!r} is used twice')
                rsu_names.add(name)
                try:
                    rid = int(str(r.get('rsu_id')), 16)
                    if not 0 <= rid <= 0xFFFFFFFF:
                        raise ValueError
                    if rid in ids:
                        errors.append(f'RSU {name}: rsu_id {r.get("rsu_id")} is used by another RSU')
                    ids.add(rid)
                except (ValueError, TypeError):
                    errors.append(f'RSU {name}: rsu_id {r.get("rsu_id")!r} is not a 32-bit hex number')
                sids = set()
                for s in r.get('sensors') or []:
                    where = f'RSU {name} / {s.get("name")}'
                    if s.get('type') not in pseudo:
                        errors.append(f'{where}: unknown sensor type {s.get("type")!r}')
                    sid = s.get('sensor_id')
                    if not isinstance(sid, int) or not 0 <= sid <= 255:
                        errors.append(f'{where}: sensor_id {sid!r} is not 0-255')
                    elif s.get('enabled', True) and sid in sids:
                        errors.append(f'{where}: sensor_id {sid} is used twice in this RSU')
                    sids.add(sid)
            for kind in ('object_info_publishers', 'autoware_object_publishers'):
                for p in rsus.get(kind) or []:
                    where = f'{kind[:-1].replace("_", " ")} {p.get("name")}'
                    for rn in p.get('rsus') or []:
                        if rn != 'all' and rn not in rsu_names:
                            errors.append(f'{where}: unknown RSU {rn!r}')
                    for n in p.get('network') or []:
                        if n not in nets:
                            errors.append(f'{where}: unknown network profile {n!r}')
                    if not p.get('topic'):
                        errors.append(f'{where}: topic is empty')
                    check_noise(where, p.get('noise'))
            check_noise('per_sensor_object_info', (rsus.get('per_sensor_object_info') or {}).get('noise'))
            for p in rsus.get('traffic_signal_publishers') or []:
                if not p.get('topics'):
                    errors.append(f'traffic signal publisher {p.get("name")}: no topics')
                if p.get('signal_id', 'relation') not in ('relation', 'way'):
                    errors.append(f'traffic signal publisher {p.get("name")}: signal_id must be relation or way')

        v = files.get('vehicles')
        if v:
            if not v.get('lanes_root'):
                errors.append('vehicles: lanes_root is empty')
            if not v.get('vehicle_types'):
                errors.append('vehicles: no vehicle_types')
            for s in v.get('spawn_points') or []:
                if (s.get('vehicles_per_minute') or 0) < 0:
                    errors.append(f'vehicles: spawn point {s.get("lanelet")}: vehicles_per_minute < 0')
            for b in v.get('branches') or []:
                if not b.get('to') or any((t.get('ratio') or 0) < 0 for t in b['to']):
                    errors.append(f'vehicles: branch from {b.get("from_lanelet")}: needs targets with ratio >= 0')

        p = files.get('pedestrians')
        if p:
            ids = []
            for path in p.get('paths') or []:
                pid = path.get('id')
                if not pid or pid in ids:
                    errors.append(f'pedestrians: path id {pid!r} is empty or used twice')
                ids.append(pid)
                if not path.get('enabled', True):
                    continue
                if not 0 <= (path.get('cyclist_ratio') or 0) <= 1:
                    errors.append(f'pedestrians: path {pid}: cyclist_ratio must be 0-1')
                if len(path.get('waypoints') or []) < 2:
                    errors.append(f'pedestrians: path {pid}: needs at least 2 waypoints')

        t = files.get('teleports')
        if t:
            for g in t.get('teleports') or []:
                if not g.get('enabled', True):
                    continue
                strat = g.get('strategy')
                if strat not in ('lane_midpoint', 'random_point'):
                    errors.append(f'teleport {g.get("name")}: strategy must be lane_midpoint or random_point')
                if strat == 'lane_midpoint' and not (g.get('lane_midpoint') or {}).get('lanelets'):
                    errors.append(f'teleport {g.get("name")}: lane_midpoint needs lanelets')
                if strat == 'random_point' and not g.get('random_points'):
                    errors.append(f'teleport {g.get("name")}: random_point needs random_points')

        rs = files.get('raw_sensors')
        if rs:
            names = []
            for s in rs.get('raw_sensors') or []:
                n = s.get('name')
                if n in names:
                    errors.append(f'raw sensor name {n!r} is used twice')
                names.append(n)
                if s.get('mount') not in ('ego', 'scene', 'world'):
                    errors.append(f'raw sensor {n}: mount must be ego, scene or world')
                if s.get('kind') not in ('lidar', 'camera'):
                    errors.append(f'raw sensor {n}: kind must be lidar or camera')
                if s.get('mount') in ('ego', 'scene') and not s.get('object'):
                    errors.append(f'raw sensor {n}: {s.get("mount")} sensors need object')
                if s.get('mount') == 'world' and s.get('enabled', True) and (s.get('type') not in raw_types or not s.get('pose')):
                    errors.append(f'raw sensor {n}: world sensors need a known type and a pose')
                w, h = s.get('width'), s.get('height')
                if (w is None) != (h is None) or (w and (w * h) % 4):
                    errors.append(f'raw sensor {n}: set width and height together (width × height a multiple of 4)')
        return errors

    # ------------------------------------------------------------------ HTTP handlers
    async def index(self, request):
        return web.json_response({'scenarios': self.scenarios(), 'default_scenario': self.default_scenario(),
                                  'config_root': CONFIG})

    async def get_scenario(self, request):
        name = request.match_info['name']
        if not os.path.isdir(self._scenario_dir(name)):
            raise web.HTTPNotFound(text=f'no scenario {name}')
        return web.json_response(self.load(name))

    async def save(self, request):
        """Body: {files: {key: data}} for the keys that changed. All files are checked together before anything is written."""
        name = request.match_info['name']
        body = await request.json()
        changed = {k: _hex_strings(_strip_private(v)) for k, v in (body.get('files') or {}).items()}
        current = self.load(name)
        merged = dict(current['files'])
        merged.update(changed)
        errors = self.validate(merged)
        if errors:
            return web.json_response({'ok': False, 'errors': errors}, status=422)
        for key, data in changed.items():
            rel, path = self._file_path(name, key)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            _write(path, data, current['headers'].get(key) or self._factory_header(rel))
        return web.json_response({'ok': True, 'saved': sorted(changed)})

    def _factory_header(self, rel):
        p = os.path.join(FACTORY, rel)
        return _header(open(p, encoding='utf-8').read()) if os.path.exists(p) else ''

    async def reset(self, request):
        """Copy the factory version of one file (?file=key) back."""
        name = request.match_info['name']
        key = request.query.get('file', '')
        rel, path = self._file_path(name, key)
        src = os.path.join(FACTORY, rel)
        if not os.path.exists(src):
            # a scenario created in the UI: fall back to the factory version of the default scenario
            alt = rel.replace(os.path.join('scenarios', name), os.path.join('scenarios', self.default_scenario() or ''), 1)
            src = os.path.join(FACTORY, alt)
        if not os.path.exists(src):
            return web.json_response({'ok': False, 'errors': [f'no factory default for {rel}']}, status=404)
        shutil.copyfile(src, path)
        return web.json_response({'ok': True, 'data': self.load(name)})

    async def copy(self, request):
        """Body: {from, to}: a new scenario folder with the files of another one."""
        body = await request.json()
        src, dst = self._scenario_dir(body.get('from')), self._scenario_dir(body.get('to'))
        if os.path.exists(dst):
            return web.json_response({'ok': False, 'errors': [f'scenario {body.get("to")} exists']}, status=409)
        shutil.copytree(src, dst)
        meta, header = _read(os.path.join(dst, 'scenario.yaml'))
        meta['name'] = body.get('to')
        meta['description'] = f'copy of {body.get("from")}'
        _write(os.path.join(dst, 'scenario.yaml'), meta, header)
        return web.json_response({'ok': True, 'scenarios': self.scenarios()})

    async def set_default(self, request):
        body = await request.json()
        self._scenario_dir(body.get('name'))
        with open(os.path.join(CONFIG, 'default_scenario.txt'), 'w') as f:
            f.write(body.get('name') + '\n')
        return web.json_response({'ok': True})

    async def geo(self, request):
        """Body: {to: 'lonlat' | 'map', points: [[a, b], ...]} -> {points}."""
        body = await request.json()
        ox, oy = self.offset
        if body.get('to') == 'lonlat':
            pts = [list(self._to_ll.transform(x + ox, y + oy)) for x, y in body.get('points', [])]
        else:
            pts = []
            for lon, lat in body.get('points', []):
                e, n = self._to_utm.transform(lon, lat)
                pts.append([round(e - ox, 3), round(n - oy, 3)])
        return web.json_response({'points': pts})

    async def lanes(self, request):
        """Traffic lanes of the simulator (config/maps/traffic_lanes.json, written by V2X > Config > Export Traffic Lanes)
        as GeoJSON line strings; their ids are the lanelet numbers used in vehicles.yaml and teleports.yaml."""
        path = os.path.join(CONFIG, 'maps', 'traffic_lanes.json')
        if not os.path.exists(path):
            return web.json_response({'type': 'FeatureCollection', 'features': [], 'missing': True})
        mtime = os.path.getmtime(path)
        if getattr(self, '_lanes_mtime', None) != mtime:
            data = json.load(open(path, encoding='utf-8'))
            ox, oy = self.offset
            feats = []
            for lane in data.get('lanes', []):
                xs = [p[0] + ox for p in lane['points']]
                ys = [p[1] + oy for p in lane['points']]
                lon, lat = self._to_ll.transform(xs, ys)
                feats.append({'type': 'Feature',
                              'properties': {'id': lane['id'], 'next': lane.get('next', []), 'prev': lane.get('prev', []),
                                             'z0': lane['points'][0][2],
                                             'turn': lane.get('turn'), 'intersection': lane.get('intersection', False)},
                              'geometry': {'type': 'LineString', 'coordinates': [[round(a, 8), round(b, 8)] for a, b in zip(lon, lat)]}})
            self._lanes = {'type': 'FeatureCollection', 'features': feats, 'lanes_root': data.get('lanes_root')}
            self._lanes_mtime = mtime
        return web.json_response(self._lanes)

    def add_routes(self, app):
        app.router.add_get('/api/settings/lanes', self.lanes)
        app.router.add_get('/api/settings', self.index)
        app.router.add_get('/api/settings/scenario/{name}', self.get_scenario)
        app.router.add_put('/api/settings/scenario/{name}', self.save)
        app.router.add_post('/api/settings/scenario/{name}/reset', self.reset)
        app.router.add_post('/api/settings/copy', self.copy)
        app.router.add_post('/api/settings/default', self.set_default)
        app.router.add_post('/api/settings/geo', self.geo)
