/* Settings mode: edit the simulator configuration (config/ of the repository) in a list view and on the map.
   The simulator reads the files when Play starts, so saved changes apply on the next run. */
'use strict';

// ------------------------------------------------------------------------------------------------ map frame <-> WGS84
// UTM (Krüger series, mm accurate) + the MGRS offset of the map frame.
const Geo = (() => {
  const a = 6378137, f = 1 / 298.257223563, k0 = 0.9996, FE = 500000;
  const n = f / (2 - f), A = a / (1 + n) * (1 + n * n / 4 + n ** 4 / 64), c = 2 * Math.sqrt(n) / (1 + n);
  const al = [n / 2 - 2 * n * n / 3 + 5 * n ** 3 / 16 + 41 * n ** 4 / 180, 13 * n * n / 48 - 3 * n ** 3 / 5 + 557 * n ** 4 / 1440,
              61 * n ** 3 / 240 - 103 * n ** 4 / 140, 49561 * n ** 4 / 161280];
  const be = [n / 2 - 2 * n * n / 3 + 37 * n ** 3 / 96 - n ** 4 / 360, n * n / 48 + n ** 3 / 15 - 437 * n ** 4 / 1440,
              17 * n ** 3 / 480 - 37 * n ** 4 / 840, 4397 * n ** 4 / 161280];
  const de = [2 * n - 2 * n * n / 3 - 2 * n ** 3 + 116 * n ** 4 / 45, 7 * n * n / 3 - 8 * n ** 3 / 5 - 227 * n ** 4 / 45,
              56 * n ** 3 / 15 - 136 * n ** 4 / 35, 4279 * n ** 4 / 630];
  let lon0 = 141 * Math.PI / 180, off = [400000, 3900000];
  const rad = Math.PI / 180;
  return {
    init(epsg, offset) { lon0 = ((epsg % 100) * 6 - 183) * rad; off = offset; },
    toLonLat(x, y) {
      const E = x + off[0], N = y + off[1];
      const xi = N / (k0 * A), eta = (E - FE) / (k0 * A);
      let xp = xi, ep = eta;
      for (let j = 1; j <= 4; j++) {
        xp -= be[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
        ep -= be[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
      }
      const chi = Math.asin(Math.sin(xp) / Math.cosh(ep));
      let phi = chi;
      for (let j = 1; j <= 4; j++) phi += de[j - 1] * Math.sin(2 * j * chi);
      return [(lon0 + Math.atan2(Math.sinh(ep), Math.cos(xp))) / rad, phi / rad];
    },
    toMap(lon, lat) {
      const phi = lat * rad, lam = lon * rad - lon0;
      const t = Math.sinh(Math.atanh(Math.sin(phi)) - c * Math.atanh(c * Math.sin(phi)));
      const xi = Math.atan2(t, Math.cos(lam)), eta = Math.atanh(Math.sin(lam) / Math.sqrt(1 + t * t));
      let E = eta, N = xi;
      for (let j = 1; j <= 4; j++) {
        E += al[j - 1] * Math.cos(2 * j * xi) * Math.sinh(2 * j * eta);
        N += al[j - 1] * Math.sin(2 * j * xi) * Math.cosh(2 * j * eta);
      }
      return [Math.round((FE + k0 * A * E - off[0]) * 1000) / 1000, Math.round((k0 * A * N - off[1]) * 1000) / 1000];
    },
    /** Polygon ring of a sector: apex (lon, lat), heading clockwise from north, full angle, radius (m). */
    sector(lon, lat, heading, fov, range) {
      const pts = [[lon, lat]], m = 111320, ml = m * Math.cos(lat * rad);
      const half = Math.min(fov, 359.9) / 2, steps = Math.max(6, Math.round(fov / 6));
      for (let i = 0; i <= steps; i++) {
        const b = (heading - half + (2 * half * i) / steps) * rad;
        pts.push([lon + range * Math.sin(b) / ml, lat + range * Math.cos(b) / m]);
      }
      pts.push([lon, lat]);
      return pts;
    },
    box(lon, lat, heading, length, width) {
      const h = heading * rad, m = 111320, ml = m * Math.cos(lat * rad);
      const fwd = [Math.sin(h), Math.cos(h)], left = [-Math.cos(h), Math.sin(h)];
      return [[.5, .5], [.5, -.5], [-.5, -.5], [-.5, .5], [.5, .5]].map(([f, s]) =>
        [lon + (f * length * fwd[0] + s * width * left[0]) / ml, lat + (f * length * fwd[1] + s * width * left[1]) / m]);
    },
  };
})();

const Settings = (() => {
  const TABS = [['rsus', 'RSUs & sensors'], ['vehicles', 'Vehicles'], ['pedestrians', 'Pedestrians'],
                ['teleports', 'Teleports'], ['raw_sensors', 'Raw sensors'], ['defaults', 'Defaults']];
  const S = { scenario: null, scenarios: [], files: {}, saved: {}, factory: {}, tab: 'rsus', sel: null, tool: null, markers: [] };
  const panel = document.getElementById('settings');
  let active = false;

  // -------------------------------------------------------------------------------------------- helpers
  const esc = s => escapeHtml(s == null ? '' : s);
  const get = (o, path) => path.split('.').reduce((x, k) => (x == null ? undefined : x[k]), o);
  function set(o, path, v) {
    const keys = path.split('.');
    let x = o;
    for (const k of keys.slice(0, -1)) x = x[k] = (x[k] && typeof x[k] === 'object') ? x[k] : {};
    if (v === undefined) delete x[keys[keys.length - 1]]; else x[keys[keys.length - 1]] = v;
  }
  const fileOfTab = tab => tab === 'defaults' ? ['sensor_types', 'noise', 'network'] : [tab];
  const dirtyKeys = () => Object.keys(S.files).filter(k => JSON.stringify(S.files[k]) !== S.saved[k]);
  function changed(file) { setStatus(); redrawMap(); }

  /** One labelled input bound to obj[path]. spec: {type: text|number|int|bool|select|list|hex, options, opt (blank = unset)} */
  function field(obj, path, label, spec, file) {
    spec = spec || {};
    const type = spec.type || 'text';
    const wrap = document.createElement('label');
    wrap.className = 'f' + (type === 'bool' ? ' f-bool' : '') + (spec.wide ? ' f-wide' : '');
    const cur = get(obj, path);
    let el;
    if (type === 'bool') {
      el = document.createElement('input'); el.type = 'checkbox';
      el.checked = cur == null ? (spec.dflt ?? path.endsWith('enabled')) : !!cur;
    } else if (type === 'select') {
      el = document.createElement('select');
      for (const o of spec.options || []) el.add(new Option(o, o, false, o === cur));
      if (cur != null && !(spec.options || []).includes(cur)) el.add(new Option(cur + ' (unknown)', cur, false, true));
    } else {
      el = document.createElement('input');
      el.type = (type === 'number' || type === 'int') ? 'number' : 'text';
      if (type === 'number') el.step = spec.step || 'any';
      el.value = type === 'list' ? (cur || []).join(', ') : (cur == null ? '' : cur);
      if (spec.opt) el.placeholder = spec.placeholder || 'default';
    }
    el.addEventListener('change', () => {
      let v;
      if (type === 'bool') v = el.checked;
      else if (type === 'number' || type === 'int') {
        if (el.value === '' && spec.opt) v = undefined;
        else { v = type === 'int' ? parseInt(el.value, 10) : parseFloat(el.value); if (Number.isNaN(v)) v = spec.opt ? undefined : 0; }
      } else if (type === 'list') v = el.value.split(',').map(s => s.trim()).filter(Boolean).map(s => spec.ints ? Number(s) : s);
      else v = (el.value === '' && spec.opt) ? undefined : el.value;
      set(obj, path, v);
      if (spec.onChange) spec.onChange(v);
      changed(file);
    });
    wrap.innerHTML = `<span>${esc(label)}</span>`;
    wrap.appendChild(el);
    if (spec.hint) { const h = document.createElement('small'); h.textContent = spec.hint; wrap.appendChild(h); }
    return wrap;
  }

  function section(title, parent) {
    const d = document.createElement('div');
    d.className = 'sec';
    d.innerHTML = `<div class="sec-title">${esc(title)}</div>`;
    parent.appendChild(d);
    return d;
  }

  function fields(parent, obj, defs, file) {
    const grid = document.createElement('div');
    grid.className = 'grid';
    for (const [path, label, spec] of defs) grid.appendChild(field(obj, path, label, spec, file));
    parent.appendChild(grid);
    return grid;
  }

  /** One line of fields followed by a delete button. */
  function rowFields(parent, obj, defs, file, onDelete) {
    const grid = fields(parent, obj, defs.map(([p, l, sp]) => [p, l, Object.assign({}, sp, { wide: false })]), file);
    grid.style.gridTemplateColumns = `repeat(${defs.length}, minmax(0, 1fr)) auto`;
    grid.appendChild(button('×', () => { onDelete(); changed(file); renderPanel(); }, 'small'));
    return grid;
  }

  function button(label, onClick, cls) {
    const b = document.createElement('button');
    b.className = 'btn' + (cls ? ' ' + cls : '');
    b.textContent = label;
    b.addEventListener('click', onClick);
    return b;
  }

  /** Selectable list of items with Add / Delete; the selected item's form is drawn by renderItem. */
  const isOpen = it => S.sel && (S.sel.item === it || (S.sel.parents || []).includes(it));

  function itemList(parent, items, { title, label, kind, newItem, renderItem, file, extra, owner }) {
    const sec = section(`${title} (${items.length})`, parent);
    const bar = document.createElement('div'); bar.className = 'bar';
    if (newItem) bar.appendChild(button('+ Add', () => {
      const it = newItem(); if (!it) return;
      items.push(it); select(kind, it, owner, false); changed(file); renderPanel();
    }));
    if (extra) for (const b of extra) bar.appendChild(b);
    sec.appendChild(bar);
    const list = document.createElement('div'); list.className = 'items';
    items.forEach((it, i) => {
      const row = document.createElement('div');
      row.className = 'item' + (isOpen(it) ? ' sel' : '') + (it.enabled === false ? ' off' : '');
      row.innerHTML = `<span class="lbl">${esc(label(it, i))}</span>`;
      const del = document.createElement('span'); del.className = 'del'; del.title = 'Delete'; del.textContent = '×';
      del.addEventListener('click', ev => {
        ev.stopPropagation();
        if (!confirm(`Delete ${label(it, i)}?`)) return;
        items.splice(items.indexOf(it), 1);
        if (isOpen(it)) S.sel = null;
        changed(file); renderPanel();
      });
      row.appendChild(del);
      row.addEventListener('click', () => {
        if (S.sel && S.sel.item === it) select(owner ? 'parent' : null, owner || null, null);   // click again: close
        else { select(kind, it, owner); focusItem(kind, it); }
      });
      list.appendChild(row);
      if (isOpen(it)) {
        const form = document.createElement('div'); form.className = 'item-form';
        renderItem(form, it);
        list.appendChild(form);
      }
    });
    sec.appendChild(list);
    return sec;
  }

  function setTool(tool, hint) {
    S.tool = tool;
    map.getCanvas().style.cursor = tool ? 'crosshair' : '';
    document.getElementById('set-tool').textContent = tool ? hint + ' (Esc to stop)' : '';
  }

  // -------------------------------------------------------------------------------------------- tabs
  const types = () => ((S.files.sensor_types || {}).pseudo_sensor_types || []);
  const noiseNames = () => ((S.files.noise || {}).noise_profiles || []).map(n => n.name);
  const netNames = () => ((S.files.network || {}).network_profiles || []).map(n => n.name);
  const NOISE = file => ['position', 'rotation', 'dimension', 'probability'].map(k =>
    [`noise.${k}`, `noise: ${k}`, { type: 'select', options: noiseNames() }]);

  function renderRsus(root) {
    const f = S.files.rsus;
    if (!f) { root.innerHTML = '<p class="muted">This scenario has no rsus.yaml.</p>'; return; }
    f.rsus = f.rsus || [];
    itemList(root, f.rsus, {
      title: 'RSUs', kind: 'rsu', file: 'rsus', label: r => `${r.name} · ${r.rsu_id} · ${(r.sensors || []).length} sensors`,
      newItem: () => ({ name: 'NewRSU' + (f.rsus.length + 1), rsu_id: '0x1202' + String(f.rsus.length + 1).padStart(4, '0'),
                        location: '', enabled: true, sensors: [] }),
      renderItem: (form, r) => {
        fields(form, r, [['name', 'name'], ['rsu_id', 'RSU ID (hex)'], ['location', 'location'], ['enabled', 'enabled', { type: 'bool' }]], 'rsus');
        r.sensors = r.sensors || [];
        itemList(form, r.sensors, {
          title: 'Sensors', kind: 'sensor', file: 'rsus', owner: r,
          label: s => `${s.sensor_id} · ${s.name} · ${s.type}`,
          extra: [button('+ Place on map', () => { S.sel = { kind: 'rsu', item: r }; setTool('place-sensor', `Click the map to place a sensor of ${r.name}`); })],
          renderItem: (sf, s) => {
            const t = types().find(t => t.name === s.type) || {};
            fields(sf, s, [
              ['name', 'name'], ['type', 'type', { type: 'select', options: types().map(t => t.name) }],
              ['sensor_id', 'sensor ID (0-255)', { type: 'int' }], ['enabled', 'enabled', { type: 'bool' }],
              ['pose.x', 'x (m)', { type: 'number' }], ['pose.y', 'y (m)', { type: 'number' }], ['pose.z', 'z (m)', { type: 'number' }],
              ['pose.heading_deg', 'heading (° from north)', { type: 'number' }], ['pose.pitch_deg', 'pitch (°, + up)', { type: 'number' }],
              ['h_fov_deg', 'h FOV (°)', { type: 'number', opt: true, placeholder: `${t.h_fov_deg ?? ''} (type)` }],
              ['v_fov_deg', 'v FOV (°)', { type: 'number', opt: true, placeholder: `${t.v_fov_deg ?? ''} (type)` }],
              ['range_m', 'range (m)', { type: 'number', opt: true, placeholder: `${t.range_m ?? ''} (type)` }],
              ['object_info.topic', 'own topic (optional)', { opt: true, wide: true, placeholder: 'none' }],
              ['object_info.ground_truth_topic', 'own ground-truth topic', { opt: true, wide: true, placeholder: 'none' }],
            ], 'rsus');
          },
        });
      },
    });
    const pubs = (key, title, defs, mk) => {
      f[key] = f[key] || [];
      itemList(root, f[key], { title, kind: key, file: 'rsus', label: p => `${p.name} · ${p.topic || (p.topics || []).join(', ')}`,
        newItem: mk, renderItem: (form, p) => fields(form, p, defs, 'rsus') });
    };
    const rsuHint = { type: 'list', wide: true, hint: 'RSU names, or all' };
    pubs('object_info_publishers', 'Object-info publishers (CooL4 DM)', [
      ['name', 'name'], ['rsus', 'RSUs', rsuHint], ['topic', 'topic', { wide: true }],
      ['ground_truth_topic', 'ground-truth topic', { wide: true }], ['hz', 'Hz', { type: 'number' }], ...NOISE(),
      ['network', 'network copies', { type: 'list', wide: true, hint: netNames().join(', ') }],
    ], () => ({ name: 'new', rsus: ['all'], topic: '/v2x/new/object_info', ground_truth_topic: '', hz: 10,
               noise: { position: 'Default', rotation: 'Default', dimension: 'Default', probability: 'Default' }, network: [] }));
    pubs('autoware_object_publishers', 'Autoware object publishers', [
      ['name', 'name'], ['rsus', 'RSUs', rsuHint], ['topic', 'topic', { wide: true }], ['station_id', 'station ID', { type: 'int' }],
      ['hz', 'Hz', { type: 'number' }], ...NOISE(), ['network', 'network copies', { type: 'list', wide: true, hint: netNames().join(', ') }],
    ], () => ({ name: 'new', rsus: [], topic: '/v2x/new/predicted_object', station_id: 1, hz: 10,
               noise: { position: 'Default', rotation: 'Default', dimension: 'Default', probability: 'Default' }, network: [] }));
    pubs('traffic_signal_publishers', 'V2X traffic-signal publishers', [
      ['name', 'name'], ['station_id', 'station ID', { type: 'int' }],
      ['intersections', 'intersections', { type: 'list', wide: true, hint: 'TrafficIntersection names, e.g. Kakeiken' }],
      ['topics', 'topics', { type: 'list', wide: true }], ['hz', 'Hz', { type: 'int' }],
      ['signal_id', 'signal IDs', { type: 'select', options: ['relation', 'way'] }],
    ], () => ({ name: 'new', station_id: 1, intersections: [], topics: ['/v2x/new/traffic_signals'], hz: 10, signal_id: 'relation' }));
    const ps = section('Per-sensor topics: rate and noise', root);
    f.per_sensor_object_info = f.per_sensor_object_info || { hz: 10, noise: {} };
    fields(ps, f.per_sensor_object_info, [['hz', 'Hz', { type: 'number' }], ...NOISE()], 'rsus');
  }

  function renderVehicles(root) {
    const f = S.files.vehicles;
    if (!f) { root.innerHTML = '<p class="muted">This scenario has no vehicles.yaml.</p>'; return; }
    if (S.lanesMissing) root.insertAdjacentHTML('beforeend', '<p class="muted">No traffic lanes for the map: run V2X &gt; Config &gt; ' +
      'Export Traffic Lanes For The Web Viewer in Unity (writes config/maps/traffic_lanes.json).</p>');
    const g = section('Traffic', root);
    fields(g, f, [
      ['max_vehicles', 'max vehicles', { type: 'int' }], ['seed', 'random seed', { type: 'int' }],
      ['spawn_distance_to_ego_m', 'min distance to ego (m)', { type: 'number' }], ['spawn_clearance_m', 'spawn clearance (m)', { type: 'number' }],
      ['maximum_spawns', 'max spawns (0 = ∞)', { type: 'int' }],
      ['behaviour.deadlock_timeout_s', 'deadlock timeout (s)', { type: 'number', hint: 'standing while yielding → right of way' }],
      ['behaviour.forced_pass_s', 'right of way for (s)', { type: 'number' }],
      ['behaviour.stuck_despawn_s', 'remove stuck after (s)', { type: 'number', hint: '0 = never' }],
      ['lanes_root', 'lanes root (scene path)', { wide: true }], ['template_traffic_manager', 'template traffic manager', { wide: true }],
    ], 'vehicles');
    f.vehicle_types = f.vehicle_types || [];
    itemList(root, f.vehicle_types, { title: 'Vehicle types', kind: 'vtype', file: 'vehicles', label: t => `${t.prefab} · weight ${t.weight}`,
      newItem: () => ({ prefab: '', weight: 1 }),
      renderItem: (form, t) => fields(form, t, [['prefab', 'prefab name', { wide: true }], ['weight', 'weight', { type: 'number' }]], 'vehicles') });
    f.spawn_points = f.spawn_points || [];
    itemList(root, f.spawn_points, {
      title: 'Spawn points', kind: 'spawn', file: 'vehicles', label: s => `lanelet ${s.lanelet} · ${s.vehicles_per_minute}/min`,
      extra: [button('+ Click a lane', () => setTool('add-spawn', 'Click a lane on the map to add a spawn point'))],
      renderItem: (form, s) => fields(form, s, [['name', 'name'], ['lanelet', 'lanelet ID', { type: 'int' }],
        ['vehicles_per_minute', 'vehicles / minute', { type: 'number' }]], 'vehicles'),
    });
    f.branches = f.branches || [];
    itemList(root, f.branches, {
      title: 'Branch ratios', kind: 'branch', file: 'vehicles',
      label: b => `${b.from_lanelet} → ${(b.to || []).map(t => `${t.lanelet}:${t.ratio}`).join(' ')}`,
      newItem: () => ({ from_lanelet: 0, to: [{ lanelet: 0, ratio: 1 }] }),
      renderItem: (form, b) => {
        fields(form, b, [['from_lanelet', 'from lanelet', { type: 'int' }]], 'vehicles');
        b.to = b.to || [];
        b.to.forEach((t, i) => {
          rowFields(form, t, [['lanelet', `to lanelet #${i + 1}`, { type: 'int' }], ['ratio', 'ratio', { type: 'number' }]], 'vehicles',
                    () => b.to.splice(i, 1));
        });
        form.appendChild(button('+ target', () => { b.to.push({ lanelet: 0, ratio: 1 }); changed('vehicles'); renderPanel(); }, 'small'));
        const p = document.createElement('p'); p.className = 'muted';
        p.textContent = 'Ratios are normalised over the listed targets; other next lanelets get 0. The from lanelet is blue, targets orange. Click a lane on the map to see its next lanes.';
        form.appendChild(p);
      },
    });
  }

  function renderPedestrians(root) {
    const f = S.files.pedestrians;
    if (!f) { root.innerHTML = '<p class="muted">This scenario has no pedestrians.yaml.</p>'; return; }
    fields(section('Pedestrians and cyclists', root), f, [
      ['max_active', 'max active agents', { type: 'int', hint: '0 = unlimited' }], ['snap_to_ground', 'snap to ground', { type: 'bool' }]], 'pedestrians');
    f.paths = f.paths || [];
    itemList(root, f.paths, {
      title: 'Paths', kind: 'path', file: 'pedestrians',
      label: p => `${p.id} · ${p.spawns_per_minute}/min · ${Math.round((p.cyclist_ratio || 0) * 100)}% cyclists · ${(p.waypoints || []).length} pts`,
      newItem: () => { const p = { id: 'Path' + (f.paths.length + 1), spawns_per_minute: 1, cyclist_ratio: 0, enabled: true, waypoints: [] };
                       setTimeout(() => setTool('add-waypoint', `Click the map to add waypoints to ${p.id}`)); return p; },
      renderItem: (form, p) => {
        fields(form, p, [['id', 'id'], ['spawns_per_minute', 'spawns / minute', { type: 'number' }],
          ['cyclist_ratio', 'cyclist share (0-1)', { type: 'number', step: 0.05, hint: '0 = pedestrians only, 1 = cyclists only' }],
          ['enabled', 'enabled', { type: 'bool' }]], 'pedestrians');
        const bar = document.createElement('div'); bar.className = 'bar';
        bar.appendChild(button('+ Add waypoints on map', () => setTool('add-waypoint', `Click the map to add waypoints to ${p.id}`)));
        bar.appendChild(button('Reverse', () => { p.waypoints.reverse(); changed('pedestrians'); renderPanel(); }));
        form.appendChild(bar);
        (p.waypoints || []).forEach((w, i) => {
          rowFields(form, w, [['traffic_light', `point ${i + 1}: wait for light`, { opt: true, placeholder: 'none' }]], 'pedestrians',
                    () => p.waypoints.splice(i, 1));
        });
        const hint = document.createElement('p'); hint.className = 'muted';
        hint.textContent = 'Drag the points on the map to move them. A light name (e.g. Kakeiken_P_0x30_1) makes agents wait there for green.';
        form.appendChild(hint);
      },
    });
  }

  function renderTeleports(root) {
    const f = S.files.teleports;
    if (!f) { root.innerHTML = '<p class="muted">This scenario has no teleports.yaml.</p>'; return; }
    fields(section('Teleports', root), f, [['lanes_root', 'lanes root', { opt: true, wide: true, placeholder: 'from vehicles.yaml' }]], 'teleports');
    f.teleports = f.teleports || [];
    itemList(root, f.teleports, {
      title: 'Gates', kind: 'gate', file: 'teleports', label: t => `${t.name} · ${t.strategy}`,
      extra: [button('+ Place gate on map', () => setTool('add-gate', 'Click the map to place a teleport gate'))],
      renderItem: (form, t) => {
        fields(form, t, [
          ['name', 'name'], ['enabled', 'enabled', { type: 'bool' }],
          ['trigger.heading_deg', 'trigger heading (°)', { type: 'number' }],
          ['trigger_size.x', 'trigger width (m)', { type: 'number' }], ['trigger_size.z', 'trigger length (m)', { type: 'number' }],
          ['trigger_size.y', 'trigger height (m)', { type: 'number' }],
          ['strategy', 'strategy', { type: 'select', options: ['lane_midpoint', 'random_point'] }],
          ['clearance_radius_m', 'clear NPCs within (m)', { type: 'number' }], ['delay_s', 'delay (s)', { type: 'number' }],
          ['random_delay_s', 'random extra delay min, max (s)', { type: 'list', ints: true }],
        ], 'teleports');
        if (t.strategy === 'lane_midpoint') {
          t.lane_midpoint = t.lane_midpoint || { lanelets: [], spacing_m: 10, include_endpoints: true, midpoint_lanelet: 0,
            midpoint_fraction: 0.5, cruise_speed_kmh: 30, acceleration_mps2: 2, arrival_window_s: [0, 90] };
          const bar = document.createElement('div'); bar.className = 'bar';
          bar.appendChild(button('Pick target lanes on map', () => setTool('pick-lanes', 'Click lanes to add or remove them')));
          form.appendChild(bar);
          fields(form, t, [
            ['lane_midpoint.lanelets', 'target lanelets', { type: 'list', ints: true, wide: true }],
            ['lane_midpoint.midpoint_lanelet', 'midpoint lanelet', { type: 'int' }],
            ['lane_midpoint.midpoint_fraction', 'midpoint fraction (0-1)', { type: 'number' }],
            ['lane_midpoint.arrival_window_s', 'arrival window min, max (s)', { type: 'list', ints: true }],
            ['lane_midpoint.cruise_speed_kmh', 'cruise speed (km/h)', { type: 'number' }],
            ['lane_midpoint.acceleration_mps2', 'acceleration (m/s²)', { type: 'number' }],
            ['lane_midpoint.spacing_m', 'candidate spacing (m)', { type: 'number' }],
          ], 'teleports');
        } else {
          const bar = document.createElement('div'); bar.className = 'bar';
          bar.appendChild(button('+ Add target point on map', () => setTool('add-target', `Click the map to add a target of ${t.name}`)));
          form.appendChild(bar);
          (t.random_points = t.random_points || []).forEach((p, i) => {
            rowFields(form, p, [['heading_deg', `target #${i + 1} heading (°)`, { type: 'number' }]], 'teleports',
                      () => t.random_points.splice(i, 1));
          });
        }
      },
    });
  }

  function renderRaw(root) {
    const f = S.files.raw_sensors;
    if (!f) { root.innerHTML = '<p class="muted">This scenario has no raw_sensors.yaml.</p>'; return; }
    f.raw_sensors = f.raw_sensors || [];
    const rawTypes = ((S.files.sensor_types || {}).raw_sensor_types || []);
    itemList(root, f.raw_sensors, {
      title: 'Raw sensors', kind: 'raw', file: 'raw_sensors',
      label: s => `${s.enabled === false ? '○' : '●'} ${s.kind} · ${s.name}`,
      extra: [button('+ Place new sensor on map', () => setTool('add-raw', 'Click the map to place a new raw sensor'))],
      renderItem: (form, s) => {
        const defs = [['name', 'name', { wide: true }], ['enabled', 'enabled', { type: 'bool' }],
          ['mount', 'mount', { type: 'select', options: ['ego', 'scene', 'world'] }], ['hz', 'Hz', { type: 'int', opt: true }],
          ['frame_id', 'frame ID', { opt: true }]];
        if (s.mount === 'ego') defs.push(['vehicle', 'vehicle'], ['object', 'object (path in vehicle)', { wide: true }]);
        if (s.mount === 'scene') defs.push(['object', 'scene path', { wide: true }]);
        if (s.mount === 'world') defs.push(['type', 'type', { type: 'select', options: rawTypes.filter(t => t.kind === s.kind).map(t => t.name) }],
          ['pose.z', 'z (m)', { type: 'number' }], ['pose.heading_deg', 'heading (°)', { type: 'number' }], ['pose.pitch_deg', 'pitch (°)', { type: 'number' }]);
        if (s.kind === 'camera') defs.push(['image_topic', 'image topic', { opt: true, wide: true }],
          ['camera_info_topic', 'camera-info topic', { opt: true, wide: true }], ['width', 'width', { type: 'int', opt: true }], ['height', 'height', { type: 'int', opt: true }]);
        fields(form, s, defs, 'raw_sensors');
        if (s.kind === 'lidar') {
          s.point_cloud_topics = s.point_cloud_topics || [];
          s.point_cloud_topics.forEach((t, i) => {
            rowFields(form, t, [['topic', `point cloud #${i + 1}`], ['format', 'format'], ['publish', 'publish', { type: 'bool' }]], 'raw_sensors',
                      () => s.point_cloud_topics.splice(i, 1));
          });
          form.appendChild(button('+ point cloud topic', () => { s.point_cloud_topics.push({ topic: 'lidar/pointcloud', format: 'Pcl24', publish: true }); changed('raw_sensors'); renderPanel(); }, 'small'));
        }
      },
    });
  }

  function renderDefaults(root) {
    const st = S.files.sensor_types = S.files.sensor_types || {};
    st.pseudo_sensor_types = st.pseudo_sensor_types || [];
    itemList(root, st.pseudo_sensor_types, { title: 'Pseudo-sensor types (shared by all scenarios)', kind: 'ptype', file: 'sensor_types',
      label: t => `${t.name} · ${t.kind} ${t.h_fov_deg}×${t.v_fov_deg}° ${t.range_m} m`,
      newItem: () => ({ name: 'new_type', kind: 'lidar', h_fov_deg: 90, v_fov_deg: 30, range_m: 100, description: '' }),
      renderItem: (form, t) => fields(form, t, [['name', 'name', { wide: true }], ['kind', 'kind', { type: 'select', options: ['lidar', 'camera'] }],
        ['h_fov_deg', 'h FOV (°, full)', { type: 'number' }], ['v_fov_deg', 'v FOV (°, full)', { type: 'number' }],
        ['range_m', 'range (m)', { type: 'number' }], ['description', 'description', { wide: true }]], 'sensor_types') });
    const nz = S.files.noise = S.files.noise || {}; nz.noise_profiles = nz.noise_profiles || [];
    itemList(root, nz.noise_profiles, { title: 'Noise profiles', kind: 'noise', file: 'noise',
      label: n => `${n.name} · ${n.distribution} ${n.x_m}/${n.y_m}/${n.z_m} m`,
      newItem: () => ({ name: 'new', distribution: 'uniform', x_m: 0.1, y_m: 0.1, z_m: 0, gaussian_mean: 0, gaussian_std_dev: 0.25 }),
      renderItem: (form, n) => fields(form, n, [['name', 'name'], ['distribution', 'distribution', { type: 'select', options: ['uniform', 'gaussian'] }],
        ['x_m', 'x (m)', { type: 'number' }], ['y_m', 'y (m)', { type: 'number' }], ['z_m', 'z (m)', { type: 'number' }],
        ['gaussian_mean', 'gaussian mean', { type: 'number' }], ['gaussian_std_dev', 'gaussian std dev (× amplitude)', { type: 'number' }]], 'noise') });
    const nw = S.files.network = S.files.network || {}; nw.network_profiles = nw.network_profiles || [];
    itemList(root, nw.network_profiles, { title: 'Network profiles', kind: 'net', file: 'network',
      label: n => `${n.name} · ${n.delay_ms} ms · ${Math.round((n.packet_loss || 0) * 100)}% loss`,
      newItem: () => ({ name: 'new', topic_suffix: '/new', delay_ms: 100, delay_variance_ms: 0, packet_loss: 0, packet_loss_variance: 0, ground_truth: false }),
      renderItem: (form, n) => fields(form, n, [['name', 'name'], ['topic_suffix', 'topic suffix'], ['delay_ms', 'delay (ms)', { type: 'number' }],
        ['delay_variance_ms', '± variance (ms)', { type: 'number' }], ['packet_loss', 'packet loss (0-1)', { type: 'number' }],
        ['packet_loss_variance', '± loss variance', { type: 'number' }], ['ground_truth', 'copy ground truth', { type: 'bool' }]], 'network') });
  }

  // -------------------------------------------------------------------------------------------- panel
  function setStatus(msg, cls) {
    const dirty = dirtyKeys();
    const el = document.getElementById('set-status');
    if (msg) { el.innerHTML = msg; el.className = 'set-status ' + (cls || ''); return; }
    el.className = 'set-status' + (dirty.length ? ' dirty' : '');
    el.textContent = dirty.length ? `Unsaved: ${dirty.join(', ')}` : 'Saved. Changes apply the next time the simulation starts (Play).';
    document.getElementById('set-save').disabled = !dirty.length;
    document.getElementById('set-discard').disabled = !dirty.length;
  }

  function renderPanel() {
    const body = document.getElementById('set-body');
    const scroll = body.scrollTop;
    body.innerHTML = '';
    document.querySelectorAll('#set-tabs .tab').forEach(t => t.classList.toggle('active', t.dataset.tab === S.tab));
    ({ rsus: renderRsus, vehicles: renderVehicles, pedestrians: renderPedestrians, teleports: renderTeleports,
       raw_sensors: renderRaw, defaults: renderDefaults })[S.tab](body);
    body.scrollTop = scroll;
    setStatus();
  }

  async function api(url, opts) {
    const r = await fetch(url, opts);
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw Object.assign(new Error(j.errors ? j.errors.join('\n') : r.statusText), { errors: j.errors });
    return j;
  }

  async function load(name) {
    const d = await api('/api/settings/scenario/' + encodeURIComponent(name));
    S.scenario = name; S.files = d.files; S.factory = d.factory; S.sel = null;
    S.saved = Object.fromEntries(Object.entries(S.files).map(([k, v]) => [k, JSON.stringify(v)]));
    renderPanel(); redrawMap();
  }

  async function save() {
    const files = Object.fromEntries(dirtyKeys().map(k => [k, S.files[k]]));
    try {
      await api('/api/settings/scenario/' + encodeURIComponent(S.scenario), {
        method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ files }) });
      for (const k of Object.keys(files)) S.saved[k] = JSON.stringify(S.files[k]);
      setStatus();
    } catch (e) {
      setStatus('<b>Not saved:</b><br>' + (e.errors || [e.message]).map(esc).join('<br>'), 'error');
    }
  }

  async function reset() {
    const keys = fileOfTab(S.tab);
    if (!confirm(`Replace ${keys.join(', ')} of ${S.scenario} with the shipped default? Unsaved and saved edits of these files are lost.`)) return;
    try {
      let d;
      for (const k of keys) d = (await api(`/api/settings/scenario/${encodeURIComponent(S.scenario)}/reset?file=${k}`, { method: 'POST' })).data;
      for (const k of keys) { S.files[k] = d.files[k]; S.saved[k] = JSON.stringify(d.files[k]); }
      S.sel = null; renderPanel(); redrawMap();
    } catch (e) { setStatus(esc(e.message), 'error'); }
  }

  async function newScenario() {
    const to = prompt(`New scenario (a copy of ${S.scenario}); name with letters, digits, _ or -:`);
    if (!to) return;
    try {
      const d = await api('/api/settings/copy', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                                  body: JSON.stringify({ from: S.scenario, to }) });
      S.scenarios = d.scenarios; fillScenarios(); await load(to);
    } catch (e) { setStatus(esc(e.message), 'error'); }
  }

  async function makeDefault() {
    await api('/api/settings/default', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name: S.scenario }) });
    S.defaultScenario = S.scenario; fillScenarios();
    setStatus(`${esc(S.scenario)} is now the default scenario (config/default_scenario.txt).`);
  }

  function fillScenarios() {
    const sel = document.getElementById('set-scenario');
    sel.innerHTML = '';
    for (const s of S.scenarios) sel.add(new Option(s + (s === S.defaultScenario ? ' (default)' : ''), s, false, s === S.scenario));
  }

  // -------------------------------------------------------------------------------------------- map
  const SRC = 'settings';
  const LAYER_IDS = ['set-simlanes', 'set-simlanes-hit', 'set-lanes-hl', 'set-lanes-target', 'set-sector', 'set-sector-line', 'set-path', 'set-gate'];
  const LANE_TABS = ['vehicles', 'teleports'];
  let popup = null;
  const ll = p => Geo.toLonLat(p.x, p.y);
  function clearMarkers() { for (const m of S.markers) m.remove(); S.markers = []; }

  function marker(lonlat, cls, title, onDrag, onClick) {
    const el = document.createElement('div');
    el.className = 'set-marker ' + cls;
    el.title = title;
    const m = new maplibregl.Marker({ element: el, draggable: !!onDrag }).setLngLat(lonlat).addTo(map);
    if (onDrag) m.on('dragend', () => { const p = m.getLngLat(); onDrag(Geo.toMap(p.lng, p.lat)); });
    if (onClick) el.addEventListener('click', ev => { ev.stopPropagation(); onClick(); });
    S.markers.push(m);
    return m;
  }

  /** Select an item; a sensor keeps its RSU open. kind 'parent' re-selects the owner of a closed child. */
  function select(kind, item, owner, render = true) {
    if (!item) S.sel = null;
    else {
      if (kind === 'parent') kind = 'rsu';
      if (kind === 'sensor' && !owner) owner = ((S.files.rsus || {}).rsus || []).find(r => (r.sensors || []).includes(item));
      S.sel = { kind, item, parents: owner ? [owner] : [] };
    }
    if (render) { renderPanel(); redrawMap(); }
  }

  function redrawMap() {
    if (!active || !map.getSource(SRC)) return;
    clearMarkers();
    const feats = [];
    const sel = S.sel && S.sel.item;
    const highlightLanes = [], highlightTargets = [];
    if (S.tab === 'rsus' && S.files.rsus) {
      for (const r of S.files.rsus.rsus || []) for (const s of r.sensors || []) {
        if (!s.pose) continue;
        const t = types().find(t => t.name === s.type) || {};
        const [lon, lat] = ll(s.pose);
        const on = r.enabled !== false && s.enabled !== false;
        feats.push({ type: 'Feature', properties: { kind: 'sector', cam: t.kind === 'camera', on, sel: s === sel },
          geometry: { type: 'Polygon', coordinates: [Geo.sector(lon, lat, s.pose.heading_deg || 0, s.h_fov_deg ?? t.h_fov_deg ?? 60, s.range_m ?? t.range_m ?? 50)] } });
        marker([lon, lat], `sensor ${t.kind === 'camera' ? 'cam' : 'lidar'}${s === sel ? ' sel' : ''}${on ? '' : ' off'}`,
          `${r.name} / ${s.name} (sensor ${s.sensor_id})`,
          xy => { s.pose.x = xy[0]; s.pose.y = xy[1]; changed('rsus'); if (S.sel && S.sel.item === s) renderPanel(); },
          () => select('sensor', s));
      }
    }
    if (S.tab === 'vehicles' && S.files.vehicles) {
      for (const s of S.files.vehicles.spawn_points || []) {
        if (s.position) marker(ll(s.position), 'spawn' + (s === sel ? ' sel' : ''), `spawn lanelet ${s.lanelet}: ${s.vehicles_per_minute}/min`,
          null, () => select('spawn', s));
        if (s === sel) highlightLanes.push(s.lanelet);
      }
      if (S.sel && S.sel.kind === 'branch') {
        highlightLanes.push(sel.from_lanelet);
        for (const t of sel.to || []) highlightTargets.push(t.lanelet);
      }
    }
    if (S.tab === 'pedestrians' && S.files.pedestrians) {
      for (const p of S.files.pedestrians.paths || []) {
        const coords = (p.waypoints || []).map(ll);
        if (coords.length > 1) feats.push({ type: 'Feature', properties: { kind: 'path', sel: p === sel, on: p.enabled !== false },
          geometry: { type: 'LineString', coordinates: coords } });
        if (p === sel) p.waypoints.forEach((w, i) => marker(ll(w), 'wp' + (w.traffic_light ? ' light' : ''), `#${i + 1}${w.traffic_light ? ' waits for ' + w.traffic_light : ''}`,
          xy => { w.x = xy[0]; w.y = xy[1]; changed('pedestrians'); }));
        else if (coords.length) marker(coords[0], 'wp start', p.id, null, () => select('path', p));
      }
    }
    if (S.tab === 'teleports' && S.files.teleports) {
      for (const t of S.files.teleports.teleports || []) {
        if (!t.trigger) continue;
        const [lon, lat] = ll(t.trigger);
        const size = t.trigger_size || { x: 10, z: 4 };
        feats.push({ type: 'Feature', properties: { kind: 'gate', sel: t === sel, on: t.enabled !== false },
          geometry: { type: 'Polygon', coordinates: [Geo.box(lon, lat, t.trigger.heading_deg || 0, size.z || 4, size.x || 10)] } });
        marker([lon, lat], 'gate' + (t === sel ? ' sel' : ''), `teleport ${t.name}`,
          xy => { t.trigger.x = xy[0]; t.trigger.y = xy[1]; changed('teleports'); }, () => select('gate', t));
        if (t === sel) {
          for (const p of t.random_points || []) marker(ll(p), 'target', 'target', xy => { p.x = xy[0]; p.y = xy[1]; changed('teleports'); });
          if (t.lane_midpoint) { highlightTargets.push(...(t.lane_midpoint.lanelets || [])); highlightLanes.push(t.lane_midpoint.midpoint_lanelet); }
        }
      }
    }
    if (S.tab === 'raw_sensors' && S.files.raw_sensors) {
      for (const s of S.files.raw_sensors.raw_sensors || []) {
        if (s.mount !== 'world' || !s.pose) continue;
        marker(ll(s.pose), 'sensor raw' + (s === sel ? ' sel' : ''), s.name, xy => { s.pose.x = xy[0]; s.pose.y = xy[1]; changed('raw_sensors'); },
          () => select('raw', s));
      }
    }
    map.getSource(SRC).setData({ type: 'FeatureCollection', features: feats });
    for (const id of ['set-simlanes', 'set-simlanes-hit'])
      map.setLayoutProperty(id, 'visibility', LANE_TABS.includes(S.tab) ? 'visible' : 'none');
    map.setFilter('set-lanes-hl', ['in', ['get', 'id'], ['literal', highlightLanes.map(Number)]]);
    map.setFilter('set-lanes-target', ['in', ['get', 'id'], ['literal', highlightTargets.map(Number)]]);
  }

  function focusItem(kind, it) {
    const p = it.pose || it.position || it.trigger || (it.waypoints && it.waypoints[0]);
    if (p && p.x != null) map.easeTo({ center: ll(p), duration: 400 });
  }

  function addLayers() {
    map.addSource(SRC, { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
    // The simulator's traffic lanes (config/maps/traffic_lanes.json): their ids are the lanelet numbers of the config.
    map.addSource('sim-lanes', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
    const w = (a, b) => ['interpolate', ['linear'], ['zoom'], 15, a, 19, b];
    map.addLayer({ id: 'set-simlanes', type: 'line', source: 'sim-lanes', layout: { 'line-cap': 'round' },
      paint: { 'line-color': ['case', ['get', 'intersection'], '#566173', '#3f6b8f'], 'line-width': w(0.6, 2.2), 'line-opacity': 0.8 } });
    map.addLayer({ id: 'set-simlanes-hit', type: 'line', source: 'sim-lanes', paint: { 'line-color': '#000', 'line-opacity': 0, 'line-width': w(6, 16) } });
    map.addLayer({ id: 'set-lanes-hl', type: 'line', source: 'sim-lanes', filter: ['in', ['get', 'id'], ['literal', []]],
      layout: { 'line-cap': 'round' }, paint: { 'line-color': '#4c8dff', 'line-width': w(3, 7) } });
    map.addLayer({ id: 'set-lanes-target', type: 'line', source: 'sim-lanes', filter: ['in', ['get', 'id'], ['literal', []]],
      layout: { 'line-cap': 'round' }, paint: { 'line-color': '#ff9f43', 'line-width': w(3, 7) } });
    map.on('mouseenter', 'set-simlanes-hit', () => { if (active && !S.tool) map.getCanvas().style.cursor = 'pointer'; });
    map.on('mouseleave', 'set-simlanes-hit', () => { if (active && !S.tool) map.getCanvas().style.cursor = ''; });
    map.addLayer({ id: 'set-sector', type: 'fill', source: SRC, filter: ['==', ['get', 'kind'], 'sector'], paint: {
      'fill-color': ['case', ['get', 'cam'], '#c792ea', '#4cc2ff'],
      'fill-opacity': ['case', ['get', 'sel'], 0.35, ['get', 'on'], 0.13, 0.04] } });
    map.addLayer({ id: 'set-sector-line', type: 'line', source: SRC, filter: ['==', ['get', 'kind'], 'sector'], paint: {
      'line-color': ['case', ['get', 'cam'], '#c792ea', '#4cc2ff'], 'line-width': ['case', ['get', 'sel'], 2, 0.7],
      'line-opacity': ['case', ['get', 'on'], 0.8, 0.3] } });
    map.addLayer({ id: 'set-path', type: 'line', source: SRC, filter: ['==', ['get', 'kind'], 'path'], paint: {
      'line-color': ['case', ['get', 'sel'], '#ffd166', ['get', 'on'], '#7ae582', '#5b6574'], 'line-width': ['case', ['get', 'sel'], 3.5, 2] } });
    map.addLayer({ id: 'set-gate', type: 'fill', source: SRC, filter: ['==', ['get', 'kind'], 'gate'], paint: {
      'fill-color': '#ff7ab6', 'fill-opacity': ['case', ['get', 'sel'], 0.55, ['get', 'on'], 0.3, 0.1] } });
    for (const id of LAYER_IDS)
      map.setLayoutProperty(id, 'visibility', 'none');
  }

  /** Traffic lane under the click (a few pixels of tolerance). */
  function laneAt(point) {
    const box = [[point.x - 6, point.y - 6], [point.x + 6, point.y + 6]];
    const src = S.lanes || { features: [] };
    const ids = new Set(map.queryRenderedFeatures(box, { layers: ['set-simlanes-hit'] }).map(f => Number(f.properties.id)));
    // nearest centre line in screen space (rendered features are clipped to tiles, so use the source geometry)
    let best = null, bestD = Infinity;
    for (const f of src.features || []) {
      if (!ids.has(f.properties.id)) continue;
      const pts = f.geometry.coordinates.map(c => map.project(c));
      for (let i = 1; i < pts.length; i++) {
        const a = pts[i - 1], b = pts[i], dx = b.x - a.x, dy = b.y - a.y, L = dx * dx + dy * dy;
        const t = L ? Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / L)) : 0;
        const d = Math.hypot(a.x + t * dx - point.x, a.y + t * dy - point.y);
        if (d < bestD) { bestD = d; best = f; }
      }
    }
    return best ? { id: best.properties.id, next: best.properties.next, prev: best.properties.prev, start: best.geometry.coordinates[0],
                    z: best.properties.z0 } : null;
  }

  function lanePopup(e, lane) {
    const v = S.files.vehicles;
    const div = document.createElement('div');
    div.className = 'set-popup';
    div.innerHTML = `<b>lanelet ${lane.id}</b><br>next: ${lane.next.join(', ') || '–'}<br>previous: ${lane.prev.join(', ') || '–'}`;
    const bar = document.createElement('div'); bar.className = 'bar';
    if (S.tab === 'vehicles' && v) {
      const sp = (v.spawn_points || []).find(s => s.lanelet === lane.id);
      bar.appendChild(button(sp ? 'Edit spawn point' : 'Spawn vehicles here', () => {
        if (!sp) addSpawn(lane); else select('spawn', sp);
        popup.remove();
      }, 'small'));
      if (lane.next.length > 1) {
        const br = (v.branches || []).find(b => b.from_lanelet === lane.id);
        bar.appendChild(button(br ? 'Edit branch ratios' : 'Set branch ratios', () => {
          if (br) select('branch', br);
          else {
            const b = { from_lanelet: lane.id, to: lane.next.map(n => ({ lanelet: n, ratio: Math.round(100 / lane.next.length) / 100 })) };
            (v.branches = v.branches || []).push(b); changed('vehicles'); select('branch', b);
          }
          popup.remove();
        }, 'small'));
      }
    }
    div.appendChild(bar);
    if (popup) popup.remove();
    popup = new maplibregl.Popup({ closeButton: true, maxWidth: '260px' }).setLngLat(e.lngLat).setDOMContent(div).addTo(map);
  }

  function addSpawn(lane) {
    const f = S.files.vehicles;
    const [x, y] = Geo.toMap(lane.start[0], lane.start[1]);
    const s = { name: 'lanelet' + lane.id, lanelet: lane.id, vehicles_per_minute: 2, position: { x, y, z: lane.z } };
    (f.spawn_points = f.spawn_points || []).push(s); setTool(null); select('spawn', s); changed('vehicles');
  }

  function onMapClick(e) {
    if (!active) return;
    if (!S.tool) {
      if (!LANE_TABS.includes(S.tab) || e.originalEvent.defaultPrevented) return;
      const lane = laneAt(e.point);
      if (lane) lanePopup(e, lane);
      return;
    }
    const [x, y] = Geo.toMap(e.lngLat.lng, e.lngLat.lat);
    const sel = S.sel && S.sel.item;
    if (S.tool === 'place-sensor' && S.sel && S.sel.kind === 'rsu') {
      const r = sel, used = new Set((r.sensors || []).map(s => s.sensor_id));
      let id = 1; while (used.has(id)) id++;
      const s = { name: `${r.name}_sensor${id}`, type: (types()[0] || {}).name || '', sensor_id: id, enabled: true,
                  pose: { x, y, z: 25, heading_deg: 0, pitch_deg: -10, roll_deg: 0 } };
      (r.sensors = r.sensors || []).push(s); setTool(null); select('sensor', s, r); changed('rsus'); return;
    }
    if (S.tool === 'add-spawn') {
      const lane = laneAt(e.point);
      if (!lane) { setStatus('Click on a lane line (zoom in if needed).', 'error'); return; }
      addSpawn(lane); return;
    }
    if (S.tool === 'pick-lanes' && S.sel && S.sel.kind === 'gate') {
      const lane = laneAt(e.point);
      if (!lane) return;
      const lm = sel.lane_midpoint, i = lm.lanelets.indexOf(lane.id);
      if (i >= 0) lm.lanelets.splice(i, 1); else lm.lanelets.push(lane.id);
      changed('teleports'); renderPanel(); return;
    }
    if (S.tool === 'add-waypoint' && S.sel && S.sel.kind === 'path') {
      const prev = sel.waypoints[sel.waypoints.length - 1];
      sel.waypoints.push({ x, y, z: prev ? prev.z : 20 }); changed('pedestrians'); renderPanel(); return;
    }
    if (S.tool === 'add-gate') {
      const f = S.files.teleports, g = { name: 'Gate' + ((f.teleports || []).length + 1), enabled: true,
        trigger: { x, y, z: 20, heading_deg: 0, pitch_deg: 0, roll_deg: 0 }, trigger_size: { x: 10, y: 3, z: 4 },
        strategy: 'random_point', clearance_radius_m: 15, delay_s: 0.5, random_delay_s: [0, 0], random_points: [] };
      (f.teleports = f.teleports || []).push(g); setTool(null); select('gate', g); changed('teleports'); return;
    }
    if (S.tool === 'add-target' && S.sel && S.sel.kind === 'gate') {
      (sel.random_points = sel.random_points || []).push({ x, y, z: sel.trigger.z, heading_deg: 0, pitch_deg: 0, roll_deg: 0 });
      changed('teleports'); renderPanel(); return;
    }
    if (S.tool === 'add-raw') {
      const rawTypes = ((S.files.sensor_types || {}).raw_sensor_types || []);
      const t = rawTypes[0] || { name: '', kind: 'lidar' };
      const s = { name: 'new_' + t.name + '_' + ((S.files.raw_sensors.raw_sensors || []).length + 1), kind: t.kind, mount: 'world', type: t.name,
                  enabled: true, pose: { x, y, z: 25, heading_deg: 0, pitch_deg: 0, roll_deg: 0 }, hz: 10, frame_id: 'world',
                  point_cloud_topics: t.kind === 'lidar' ? [{ topic: '/rsu/new/pointcloud', format: 'Pcl24', publish: true }] : undefined };
      S.files.raw_sensors.raw_sensors.push(s); setTool(null); select('raw', s); changed('raw_sensors');
    }
  }

  // -------------------------------------------------------------------------------------------- mode
  function show(on) {
    active = on;
    document.body.classList.toggle('settings-mode', on);
    for (const id of LAYER_IDS)
      if (map.getLayer(id)) map.setLayoutProperty(id, 'visibility', on ? 'visible' : 'none');
    if (!on) { clearMarkers(); setTool(null); if (popup) popup.remove(); }
    Overlays.setHidden(on);
    for (const topic of state.addedLayers) for (const id of topicLayerIds(topic)) map.setLayoutProperty(id, 'visibility', on ? 'none' : 'visible');
    if (!on) applyVisibility();
    setTimeout(() => map.resize(), 0);
    if (on) redrawMap();
  }

  async function init(config) {
    Geo.init(config.utm_epsg || 32654, config.map_to_utm_offset || [400000, 3900000]);
    addLayers();
    map.on('click', onMapClick);
    fetch('/api/settings/lanes').then(r => r.json()).then(d => {
      S.lanes = d;
      map.getSource('sim-lanes').setData(d);
      if (d.missing) S.lanesMissing = true;
    }).catch(() => { S.lanesMissing = true; });
    document.addEventListener('keydown', e => { if (e.key === 'Escape' && S.tool) setTool(null); });
    document.querySelectorAll('.mode-tab').forEach(b => b.addEventListener('click', () => {
      document.querySelectorAll('.mode-tab').forEach(x => x.classList.toggle('active', x === b));
      show(b.dataset.mode === 'settings');
    }));
    const tabs = document.getElementById('set-tabs');
    for (const [key, label] of TABS) {
      const t = document.createElement('div'); t.className = 'tab'; t.dataset.tab = key; t.textContent = label;
      t.addEventListener('click', () => { S.tab = key; S.sel = null; setTool(null); renderPanel(); redrawMap(); });
      tabs.appendChild(t);
    }
    document.getElementById('set-save').addEventListener('click', save);
    document.getElementById('set-discard').addEventListener('click', () => load(S.scenario));
    document.getElementById('set-reset').addEventListener('click', reset);
    document.getElementById('set-new').addEventListener('click', newScenario);
    document.getElementById('set-default').addEventListener('click', makeDefault);
    document.getElementById('set-scenario').addEventListener('change', async e => {
      if (dirtyKeys().length && !confirm('Discard unsaved changes?')) { fillScenarios(); return; }
      await load(e.target.value);
    });
    window.addEventListener('beforeunload', e => { if (dirtyKeys().length) { e.preventDefault(); e.returnValue = ''; } });
    try {
      const idx = await api('/api/settings');
      S.scenarios = idx.scenarios; S.defaultScenario = idx.default_scenario;
      if (S.scenarios.length) { S.scenario = idx.default_scenario || S.scenarios[0]; fillScenarios(); await load(S.scenario); }
      else panel.querySelector('#set-body').innerHTML = `<p class="muted">No scenarios in ${esc(idx.config_root)}.</p>`;
    } catch (e) {
      panel.querySelector('#set-body').innerHTML = `<p class="muted">Settings unavailable: ${esc(e.message)}</p>`;
    }
  }

  return { init };
})();
