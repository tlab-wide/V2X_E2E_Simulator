/* Signal heads (config/traffic_lights.yaml) and RSU sensors (config/sensors.yaml) on the map.
 * Signal state: CooL4 DM signal info first (it has the remaining time), else V2X / Autoware traffic signals. */
'use strict';

const Overlays = (() => {
  // CooL4 MainLightIndication -> [bulb, flashing]; pedestrians: 5 walk, 7 walk flashing (青点滅), 3/2 stop
  const DM_VEHICLE = { 5: ['g', false], 7: ['y', false], 9: ['y', true], 3: ['r', false], 2: ['r', true] };
  const DM_PEDESTRIAN = { 5: ['walk', false], 7: ['walk', true], 3: ['stop', false], 2: ['stop', true] };
  const V2X_BULB = { green: 'g', yellow: 'y', red: 'r' };
  const DM_ARROWS = [[0x40, '←'], [0x10, '↑'], [0x04, '→']];
  const V2X_ARROWS = { left: '←', up: '↑', right: '→', 'up-left': '↖', 'up-right': '↗', down: '↓', 'down-left': '↙', 'down-right': '↘' };
  const FAR_ZOOM = 15.8;      // below: no heads
  const COMPACT_ZOOM = 17.3;  // below: bulbs only
  const SENSOR_COLOR = { RSU_CAMERA: '#ff9f43', RSU_LIDAR: '#5ce1e6' };
  const STAND = '<svg viewBox="0 0 10 14"><circle cx="5" cy="2" r="1.7"/><rect x="3.3" y="4.1" width="3.4" height="5.4" rx="1"/>' +
                '<rect x="3.5" y="9" width="1.3" height="4.8"/><rect x="5.2" y="9" width="1.3" height="4.8"/></svg>';
  const WALK = '<svg viewBox="0 0 10 14"><circle cx="5.6" cy="2" r="1.7"/><path d="M4.6 4.1h2.2l-.4 4.6 2 4.6-1.3.5-2.1-4.2-1.4 4.2-1.3-.4 1.7-5z"/></svg>';

  let heads = [];              // {el, marker, inter, app, kind, cfg, dm, v2x}
  let sensors = [];            // {cfg, el, marker}
  let dmIndex = new Map();     // `${crp}|${signalId}` -> {signal, topic, live}
  let v2xIndex = new Map();    // lanelet id -> {signal, topic, live}
  let prefs;
  let hidden = false;          // settings mode hides the live overlays

  // ---------------------------------------------------------------------------------------------- init
  function init(config) {
    prefs = state.prefs.overlays = Object.assign({ heads: true, sensors: true, coverage: true, disabled: false },
                                                state.prefs.overlays || {});
    buildToggles();
    for (const inter of config.intersections) {
      for (const app of inter.approaches) {
        for (const kind of ['vehicle', 'pedestrian']) {
          const cfg = app[kind];
          if (!cfg) continue;
          for (const h of cfg.heads || []) heads.push(makeHead(inter, app, kind, cfg, h));
        }
      }
    }
    sensors = (config.sensors || []).map(makeSensor);
    map.addSource('sensor-coverage', { type: 'geojson', data: coverageGeoJson() });
    map.addLayer({ id: 'sensor-coverage-fill', type: 'fill', source: 'sensor-coverage',
                   paint: { 'fill-color': ['get', 'color'], 'fill-opacity': ['case', ['get', 'enabled'], 0.035, 0.015] } }, 'lanes');
    map.addLayer({ id: 'sensor-coverage-line', type: 'line', source: 'sensor-coverage',
                   paint: { 'line-color': ['get', 'color'], 'line-width': 1, 'line-opacity': ['case', ['get', 'enabled'], 0.4, 0.2],
                            'line-dasharray': [2, 2] } }, 'stop-lines');
    map.on('zoom', zoomClasses);
    map.on('moveend', declutter);
    zoomClasses();
    onVisibility();
    requestAnimationFrame(declutter);
  }

  function buildToggles() {
    const box = document.createElement('div');
    box.className = 'overlay-toggles';
    const items = [['heads', 'Signal heads'], ['sensors', 'Sensors'], ['coverage', 'Sensor coverage'], ['disabled', 'Disabled sensors']];
    box.innerHTML = items.map(([k, label]) =>
      `<label class="toggle small"><input type="checkbox" data-overlay="${k}"${prefs[k] ? ' checked' : ''}> ${label}</label>`).join('');
    box.addEventListener('change', e => {
      const k = e.target.dataset.overlay;
      if (!k) return;
      prefs[k] = e.target.checked;
      savePrefs();
      if (k === 'disabled') map.getSource('sensor-coverage').setData(coverageGeoJson());
      onVisibility();
      declutter();
    });
    document.querySelector('.legend').after(box);
  }

  // Screen-space decluttering: markers whose boxes overlap are nudged down until they are free
  // (a camera and a lidar on one pole, several heads on one corner, ...). Runs after every view change.
  function declutter() {
    if (map.getZoom() < COMPACT_ZOOM) {          // zoomed out: keep everything at its real position
      for (const item of [...heads, ...sensors]) item.marker.setOffset([0, 0]);
      return;
    }
    const placed = [];
    const items = [...heads, ...sensors].filter(i => i.el.style.display !== 'none' && i.el.offsetWidth);
    items.sort((a, b) => a.marker.getLngLat().lat - b.marker.getLngLat().lat || a.marker.getLngLat().lng - b.marker.getLngLat().lng);
    for (const item of items) {
      const p = map.project(item.marker.getLngLat());
      const w = item.el.offsetWidth, h = item.el.offsetHeight;
      let dy = 0;
      for (let step = 0; step < 6; step++) {
        const box = { x0: p.x - w / 2, x1: p.x + w / 2, y0: p.y - h / 2 + dy, y1: p.y + h / 2 + dy };
        const hit = placed.find(o => box.x0 < o.x1 && box.x1 > o.x0 && box.y0 < o.y1 && box.y1 > o.y0);
        if (!hit) break;
        dy = hit.y1 - (p.y - h / 2) + 2;
      }
      item.marker.setOffset([0, dy]);
      placed.push({ x0: p.x - w / 2, x1: p.x + w / 2, y0: p.y - h / 2 + dy, y1: p.y + h / 2 + dy });
    }
  }

  function zoomClasses() {
    const z = map.getZoom();
    const el = document.getElementById('map');
    el.classList.toggle('heads-far', z < FAR_ZOOM);
    el.classList.toggle('heads-compact', z >= FAR_ZOOM && z < COMPACT_ZOOM);
    el.classList.toggle('sensor-labels', z >= 18);
  }

  // ---------------------------------------------------------------------------------------------- signal heads
  function makeHead(inter, app, kind, cfg, h) {
    const el = document.createElement('div');
    el.className = `head ${kind === 'vehicle' ? 'veh' : 'ped'} nodata`;
    el.innerHTML = kind === 'vehicle'
      ? '<div class="bulbs"><i class="b g"></i><i class="b y"></i><i class="b r"></i></div><div class="arr"></div><div class="t"></div>'
      : `<div class="figs"><i class="f stop">${STAND}</i><i class="f walk">${WALK}</i></div><div class="t"></div>`;
    const head = { el, inter, app, kind, cfg, dm: null, v2x: null };
    el.addEventListener('click', ev => { ev.stopPropagation(); inspectHead(head); });
    head.marker = new maplibregl.Marker({ element: el }).setLngLat([h.lon, h.lat]).addTo(map);
    return head;
  }

  function enabledTopics(layerKind) {
    const rows = [...state.topics.values()].filter(r => isEnabled(r) && state.layers.get(r.name)?.layer === layerKind);
    return rows.sort((a, b) => a.name.localeCompare(b.name));
  }

  function rebuildIndexes() {
    dmIndex = new Map();
    for (const row of enabledTopics('dm_signals')) {
      for (const sig of state.layers.get(row.name).signals || []) {
        for (const id of sig.signal_ids) {
          const key = `${sig.crp_id}|${id}`;
          if (!dmIndex.has(key)) dmIndex.set(key, { signal: sig, topic: row.name, live: row.status === 'live' });
        }
      }
    }
    v2xIndex = new Map();
    for (const row of enabledTopics('traffic_signals')) {
      for (const sig of state.layers.get(row.name).signals || []) {
        if (!v2xIndex.has(sig.id)) v2xIndex.set(sig.id, { signal: sig, topic: row.name, live: row.status === 'live' });
      }
    }
  }

  function resolve(head) {
    head.dm = null;
    for (const id of head.cfg.dm_signal_ids || []) {
      const hit = dmIndex.get(`${head.inter.crp_id}|${id}`);
      if (hit) { head.dm = hit; break; }
    }
    head.v2x = null;
    for (const id of [...(head.cfg.lanelet_regulatory_elements || []), ...(head.cfg.lanelet_light_ways || [])]) {
      const hit = v2xIndex.get(id);
      if (hit) { head.v2x = hit; break; }
    }
  }

  function render(head) {
    const el = head.el;
    let lit = null, flashing = false, arrows = '', text = '', live = false, source = '';
    if (head.dm) {
      const s = head.dm.signal;
      [lit, flashing] = (head.kind === 'vehicle' ? DM_VEHICLE : DM_PEDESTRIAN)[s.main_light] || [null, false];
      arrows = DM_ARROWS.filter(([bit]) => s.arrow_bits & bit).map(([, a]) => a).join('');
      text = s.remaining_s !== null ? `${Math.ceil(s.remaining_s)}s` : (s.countdown_stopped ? '—' : '');
      live = head.dm.live; source = 'DM';
    } else if (head.v2x && head.kind === 'vehicle') {
      const s = head.v2x.signal;
      lit = V2X_BULB[s.color] || null;
      flashing = s.flashing;
      arrows = s.arrows.map(a => V2X_ARROWS[a] || '').join('');
      live = head.v2x.live; source = 'V2X';
    }
    el.classList.toggle('nodata', !source);
    el.classList.toggle('stale', !!source && !live);
    el.querySelectorAll('.b, .f').forEach(b => {
      const on = b.classList.contains(lit);
      b.classList.toggle('on', on);
      b.classList.toggle('blink', on && flashing);
    });
    const arr = el.querySelector('.arr');
    if (arr) arr.textContent = arrows;
    el.querySelector('.t').textContent = text;
    el.title = `${head.inter.name} 方路${head.app.id} (${head.app.direction}) ${head.kind}` + (source ? ` — ${source}` : ' — no data');
  }

  function inspectHead(head) {
    state.inspecting = null;
    const ids = (head.cfg.dm_signal_ids || []).map(i => '0x' + i.toString(16).padStart(2, '0')).join(' ');
    let html = '<h4>Signal head</h4>' + table({
      intersection: head.inter.name, crp_id: `${head.inter.crp_id} (0x${head.inter.crp_id.toString(16).padStart(8, '0')})`,
      approach: `方路${head.app.id} (${head.app.direction}, ${head.app.bearing_deg}°)`, kind: head.kind,
      'DM signal IDs': ids, 'Lanelet2 regulatory elements': (head.cfg.lanelet_regulatory_elements || []).join(' '),
      'Lanelet2 light ways': (head.cfg.lanelet_light_ways || []).join(' ') });
    if (head.dm) {
      const s = head.dm.signal;
      html += `<h4>${escapeHtml(head.dm.topic)} (CooL4 DM)</h4>` + table({
        'signal group': s.signal_ids.map(i => '0x' + i.toString(16).padStart(2, '0')).join(' '),
        light: s.light, 'remaining (s)': s.remaining_s === null ? 'unknown' : s.remaining_s,
        'countdown stopped': s.countdown_stopped, arrows: s.arrow_bits || '',
        schedule: s.schedule.map(x => `${x.light} ${x.min_s === null ? '?' : x.min_s + 's'}`).join(' → '),
        status: head.dm.live ? 'live' : 'not live' });
    } else {
      html += '<h4>CooL4 DM</h4><p class="muted">No DM signal info for this head (tick a DM signal topic).</p>';
    }
    if (head.kind === 'vehicle') {
      if (head.v2x) {
        const s = head.v2x.signal;
        html += `<h4>${escapeHtml(head.v2x.topic)} (V2X / Autoware)</h4>` + table({
          'lanelet id': s.id, color: s.color, elements: s.elements, 'remaining (s)': 'not in this message type',
          status: head.v2x.live ? 'live' : 'not live' });
      } else {
        html += '<h4>V2X / Autoware</h4><p class="muted">No V2X/Autoware signal for these Lanelet2 IDs.</p>';
      }
    }
    showInspector(`${head.inter.name} 方路${head.app.id} ${head.kind}`, html);
  }

  // ---------------------------------------------------------------------------------------------- sensors
  function makeSensor(cfg) {
    const el = document.createElement('div');
    el.className = 'sensor' + (cfg.enabled ? '' : ' disabled');
    el.style.setProperty('--c', SENSOR_COLOR[cfg.type] || '#c792ea');
    el.innerHTML = `<span class="ic">${cfg.type === 'RSU_LIDAR' ? 'L' : 'C'}</span><span class="nm">${escapeHtml(cfg.name)}</span>`;
    const sensor = { cfg, el };
    el.addEventListener('click', ev => { ev.stopPropagation(); inspectSensor(sensor); });
    sensor.marker = new maplibregl.Marker({ element: el }).setLngLat([cfg.position.lon, cfg.position.lat]).addTo(map);
    return sensor;
  }

  function destinationPoint(lon, lat, bearingDeg, distanceM) {
    const R = 6378137, b = bearingDeg * Math.PI / 180;
    return [lon + (distanceM * Math.sin(b) / (R * Math.cos(lat * Math.PI / 180))) * 180 / Math.PI,
            lat + (distanceM * Math.cos(b) / R) * 180 / Math.PI];
  }

  function coverageGeoJson() {
    const features = [];
    for (const s of state.config.sensors || []) {
      if (!s.enabled && !prefs.disabled) continue;
      const { lon, lat } = s.position;
      const half = Math.min(180, s.h_fov_deg);
      const ring = half >= 180 ? [] : [[lon, lat]];
      for (let a = -half; a <= half + 1e-6; a += Math.max(1, half / 24)) ring.push(destinationPoint(lon, lat, s.heading_deg + a, s.range_m));
      ring.push(ring[0]);
      features.push({ type: 'Feature', properties: { name: s.name, enabled: !!s.enabled, color: SENSOR_COLOR[s.type] || '#c792ea' },
                      geometry: { type: 'Polygon', coordinates: [ring] } });
    }
    return { type: 'FeatureCollection', features };
  }

  function sensorLive(cfg) {
    return (cfg.topics || []).some(t => state.topics.get(t)?.status === 'live');
  }

  function inspectSensor(sensor) {
    state.inspecting = null;
    const c = sensor.cfg;
    let html = '<h4>Sensor</h4>' + table({
      name: c.name, type: c.type, enabled: c.enabled, near: c.near, 'sensor id': c.sensor_id,
      'RSU ids': (c.rsu_ids || []).join(' '), position: `${c.position.lat}, ${c.position.lon}`, 'height z (m)': c.position.z_m,
      'heading (°)': c.heading_deg, 'pitch (°)': c.pitch_deg,
      'horizontal FOV': `±${c.h_fov_deg}° (${Math.min(360, 2 * c.h_fov_deg)}° wide)`, 'vertical FOV': `±${c.v_fov_deg}°`,
      'range (m)': c.range_m, 'scene object': c.scene_object });
    if ((c.topics || []).length) {
      html += '<h4>Topics</h4><ul class="topic-list">' + c.topics.map(t => {
        const r = state.topics.get(t);
        const status = r ? r.status : 'offline';
        const st = r ? `${r.status}${r.status === 'live' ? ` · ${r.hz} Hz` : ''}` : 'not published';
        return `<li><i class="s ${status}"></i><span>${escapeHtml(t)}</span><em>${escapeHtml(st)}</em></li>`;
      }).join('') + '</ul>';
    }
    showInspector(c.name, html);
  }

  // ---------------------------------------------------------------------------------------------- hooks
  function onLayers() {
    rebuildIndexes();
    for (const h of heads) { resolve(h); render(h); }
  }

  function onHealth() {
    onLayers();   // live / stale status may have changed
    for (const s of sensors) s.el.classList.toggle('live', s.cfg.enabled && sensorLive(s.cfg));
  }

  function onVisibility() {
    if (!prefs) return;
    for (const h of heads) h.el.style.display = prefs.heads && !hidden ? '' : 'none';
    for (const s of sensors) s.el.style.display = prefs.sensors && !hidden && (s.cfg.enabled || prefs.disabled) ? '' : 'none';
    for (const id of ['sensor-coverage-fill', 'sensor-coverage-line'])
      if (map.getLayer(id)) map.setLayoutProperty(id, 'visibility', prefs.coverage && !hidden ? 'visible' : 'none');
  }

  function stats() {
    return { total: heads.length, withData: heads.filter(h => h.dm || h.v2x).length,
             sensors: sensors.filter(s => s.cfg.enabled).length, sensorsLive: sensors.filter(s => s.cfg.enabled && sensorLive(s.cfg)).length };
  }

  function setHidden(on) { hidden = on; onVisibility(); }

  return { init, onLayers, onHealth, onVisibility, setHidden, stats };
})();
