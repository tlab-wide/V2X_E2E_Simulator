/* V2X E2E Simulator viewer: topic health + map of everything the simulator publishes (except raw sensor data). */
'use strict';

const PALETTE = ['#4cc2ff', '#ff7ab6', '#ffd166', '#7ae582', '#c792ea', '#ff9f43', '#5ce1e6', '#f78fb3',
                 '#a3de83', '#ffb4a2', '#89b4fa', '#e5c07b'];
const POLYGON_LAYERS = new Set(['dm_objects', 'autoware_objects', 'poses']);
const STORAGE_KEY = 'v2x-viewer:v1';

const state = {
  topics: new Map(),        // name -> health row
  layers: new Map(),        // name -> {layer, features, summary}
  addedLayers: new Set(),   // topics with map sources
  selected: null,
  ws: null,
  prefs: loadPrefs(),
  config: null,
  decodeErrors: {},
};

function loadPrefs() {
  try { return Object.assign({ enabled: {}, collapsed: {}, showRaw: false, basemap: false },
                             JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}')); }
  catch (e) { return { enabled: {}, collapsed: {}, showRaw: false, basemap: false }; }
}
function savePrefs() { try { localStorage.setItem(STORAGE_KEY, JSON.stringify(state.prefs)); } catch (e) { /* ignore */ } }

function colorOf(name) {
  let h = 0;
  for (const c of name) h = (h * 31 + c.charCodeAt(0)) >>> 0;
  return PALETTE[h % PALETTE.length];
}
function isEnabled(row) {
  if (!row.renderable) return false;
  if (row.name in state.prefs.enabled) return state.prefs.enabled[row.name];
  if (/\/(DSRC|Cellular)$/.test(row.name)) return false;          // delayed copies are off by default
  return true;
}
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// ------------------------------------------------------------------------------------------------ map
const map = new maplibregl.Map({
  container: 'map',
  attributionControl: false,
  style: {
    version: 8,
    sources: {
      basemap: { type: 'raster', tileSize: 256, maxzoom: 18,
                 tiles: ['https://cyberjapandata.gsi.go.jp/xyz/pale/{z}/{x}/{y}.png'],
                 attribution: '地理院タイル (GSI)' },
      lanelet: { type: 'geojson', data: '/map/lanelet2.geojson' },
    },
    layers: [
      { id: 'bg', type: 'background', paint: { 'background-color': '#0f1218' } },
      { id: 'basemap', type: 'raster', source: 'basemap', layout: { visibility: 'none' }, paint: { 'raster-opacity': 0.45 } },
      { id: 'lanes', type: 'fill', source: 'lanelet', filter: ['==', ['get', 'kind'], 'lane'],
        paint: { 'fill-color': '#262c37', 'fill-opacity': 0.9 } },
      { id: 'crosswalks', type: 'fill', source: 'lanelet', filter: ['==', ['get', 'kind'], 'crosswalk'],
        paint: { 'fill-color': '#4a5568', 'fill-opacity': 0.55 } },
      { id: 'borders', type: 'line', source: 'lanelet', filter: ['all', ['==', ['get', 'kind'], 'border'], ['!', ['get', 'dashed']]],
        paint: { 'line-color': '#6b778a', 'line-width': ['interpolate', ['linear'], ['zoom'], 14, 0.4, 19, 1.4] } },
      { id: 'borders-dashed', type: 'line', source: 'lanelet', filter: ['all', ['==', ['get', 'kind'], 'border'], ['get', 'dashed']],
        paint: { 'line-color': '#6b778a', 'line-width': 1, 'line-dasharray': [3, 3] } },
      { id: 'stop-lines', type: 'line', source: 'lanelet', filter: ['==', ['get', 'kind'], 'stop_line'],
        paint: { 'line-color': '#e8edf3', 'line-width': ['interpolate', ['linear'], ['zoom'], 15, 0.8, 19, 3] } },
      { id: 'map-lights', type: 'line', source: 'lanelet', filter: ['==', ['get', 'kind'], 'traffic_light'],
        paint: { 'line-color': '#c9a227', 'line-width': 2 } },
    ],
  },
  center: [139.944, 35.8975],
  zoom: 15.3,
  maxPitch: 0,
});
map.addControl(new maplibregl.NavigationControl({ showCompass: true }), 'top-right');
map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');
map.addControl(new maplibregl.AttributionControl({ compact: true }), 'bottom-right');

function sourceId(topic) { return 't:' + topic; }

function ensureTopicLayer(topic, kind) {
  if (state.addedLayers.has(topic) || kind === 'dm_signals') return;
  const src = sourceId(topic);
  const color = colorOf(topic);
  map.addSource(src, { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  if (POLYGON_LAYERS.has(kind)) {
    const fillColor = kind === 'poses' ? ['case', ['==', ['get', 'kind'], 'ego'], '#00e5ff', color] : color;
    map.addLayer({ id: src + ':fill', type: 'fill', source: src, paint: { 'fill-color': fillColor, 'fill-opacity': 0.35 } });
    map.addLayer({ id: src + ':line', type: 'line', source: src, paint: { 'line-color': fillColor, 'line-width': 1.5 } });
    bindClick(src + ':fill', topic);
  } else {
    map.addLayer({ id: src + ':circle', type: 'circle', source: src, paint: {
      'circle-radius': ['interpolate', ['linear'], ['zoom'], 14, 3, 19, 7],
      'circle-color': ['match', ['get', 'color'], 'red', '#ff4d4f', 'yellow', '#ffc53d', 'green', '#36d07a', '#5b6574'],
      'circle-stroke-color': color, 'circle-stroke-width': 1.5 } });
    bindClick(src + ':circle', topic);
  }
  state.addedLayers.add(topic);
}

function topicLayerIds(topic) {
  const src = sourceId(topic);
  return [src + ':fill', src + ':line', src + ':circle'].filter(id => map.getLayer(id));
}

function bindClick(layerId, topic) {
  map.on('click', layerId, e => { if (e.features.length) inspectFeature(topic, e.features[0].properties); });
  map.on('mouseenter', layerId, () => { map.getCanvas().style.cursor = 'pointer'; });
  map.on('mouseleave', layerId, () => { map.getCanvas().style.cursor = ''; });
}

function applyVisibility() {
  for (const topic of state.addedLayers) {
    const row = state.topics.get(topic);
    const visible = row ? isEnabled(row) : false;
    const dim = row && row.status !== 'live';
    for (const id of topicLayerIds(topic)) {
      map.setLayoutProperty(id, 'visibility', visible ? 'visible' : 'none');
      if (id.endsWith(':fill')) map.setPaintProperty(id, 'fill-opacity', dim ? 0.1 : 0.35);
      if (id.endsWith(':line')) map.setPaintProperty(id, 'line-opacity', dim ? 0.3 : 1);
      if (id.endsWith(':circle')) map.setPaintProperty(id, 'circle-opacity', dim ? 0.3 : 1);
    }
  }
  Overlays.onVisibility();
}

function addIntersectionLabels() {
  for (const inter of state.config.intersections) {
    const el = document.createElement('div');
    el.className = 'inter-label';
    el.textContent = inter.name;
    new maplibregl.Marker({ element: el, offset: [0, -34] }).setLngLat([inter.center.lon, inter.center.lat]).addTo(map);
  }
  const lons = state.config.intersections.map(i => i.center.lon);
  const lats = state.config.intersections.map(i => i.center.lat);
  map.fitBounds([[Math.min(...lons), Math.min(...lats)], [Math.max(...lons), Math.max(...lats)]], { padding: 90, duration: 0 });
}

// ------------------------------------------------------------------------------------------------ sidebar

// The topic tree and the problems summary live in sidebar.js.
function renderTopics() { Sidebar.render(); }

document.getElementById('filter').addEventListener('input', renderTopics);
const showRaw = document.getElementById('show-raw');
showRaw.checked = state.prefs.showRaw;
showRaw.addEventListener('change', () => { state.prefs.showRaw = showRaw.checked; savePrefs(); renderTopics(); });
const basemap = document.getElementById('basemap');
basemap.checked = state.prefs.basemap;
basemap.addEventListener('change', () => {
  state.prefs.basemap = basemap.checked; savePrefs();
  map.setLayoutProperty('basemap', 'visibility', basemap.checked ? 'visible' : 'none');
});

function renderStats() {
  const rows = [...state.topics.values()].filter(r => r.status !== 'raw');
  const live = rows.filter(r => r.status === 'live').length;
  let objects = 0;
  for (const [topic, l] of state.layers) {
    const row = state.topics.get(topic);
    if (!row || !isEnabled(row) || row.status !== 'live') continue;
    if (l.layer !== 'poses' && l.layer !== 'traffic_signals' && l.layer !== 'dm_signals') objects += l.features.length;
  }
  const heads = Overlays.stats();
  document.getElementById('stats').innerHTML =
    `<span><b>${live}</b>/${rows.length} topics live</span><span><b>${objects}</b> objects shown</span>` +
    `<span><b>${heads.withData}</b>/${heads.total} signal heads with data</span><span><b>${heads.sensorsLive}</b>/${heads.sensors} sensors live</span>`;
}

// ------------------------------------------------------------------------------------------------ inspector
const inspector = document.getElementById('inspector');
document.getElementById('insp-close').addEventListener('click', () => {
  inspector.classList.add('hidden'); state.selected = null; state.inspecting = null; renderTopics(); map.resize();
});

function table(obj) {
  let html = '<table>';
  for (const [k, v] of Object.entries(obj)) {
    if (v === null || v === undefined || v === '') continue;
    html += `<tr><td>${escapeHtml(k)}</td><td>${escapeHtml(typeof v === 'object' ? JSON.stringify(v) : v)}</td></tr>`;
  }
  return html + '</table>';
}

function showInspector(title, html) {
  document.getElementById('insp-title').textContent = title;
  document.getElementById('insp-body').innerHTML = html;
  if (inspector.classList.contains('hidden')) { inspector.classList.remove('hidden'); map.resize(); }
}

function inspectTopic(name) {
  state.inspecting = { topic: name };
  const r = state.topics.get(name);
  if (!r) return;
  const layer = state.layers.get(name);
  let html = '<h4>Topic</h4>' + table({ name: r.name, type: r.type, status: r.status, publishers: r.publishers,
    'rate (Hz)': r.hz, 'last message (s ago)': r.age_s, messages: r.count, 'kB/s': r.kbps, error: r.error,
    'decode error': state.decodeErrors[name] });
  if (layer) html += '<h4>Last message</h4>' + table(layer.summary);
  if (r.renderable && !isEnabled(r)) html += '<p style="padding:0 12px;color:var(--muted)">Tick the checkbox to show it on the map.</p>';
  showInspector(name, html);
}

function inspectFeature(topic, props) {
  state.inspecting = null;
  const p = {};
  for (const [k, v] of Object.entries(props)) {
    if (typeof v === 'string' && /^[\[{]/.test(v)) { try { p[k] = JSON.parse(v); continue; } catch (e) { /* keep string */ } }
    p[k] = v;
  }
  if (p.its_time_ms) p['time (JST epoch → local)'] = new Date(p.its_time_ms + 1072882800000).toLocaleTimeString();
  delete p.label;
  showInspector(p.intersection ? `${p.intersection} 方路${p.approach}` : (p.kind || 'feature'),
                `<h4>${escapeHtml(topic)}</h4>` + table(p));
}

// ------------------------------------------------------------------------------------------------ connection
function sendEnabled() {
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
  const topics = [...state.topics.values()].filter(isEnabled).map(r => r.name);
  state.ws.send(JSON.stringify({ type: 'enable', topics }));
}

function setConn(ok, text) {
  const el = document.getElementById('conn');
  el.className = 'conn ' + (ok ? 'ok' : 'bad');
  el.querySelector('.text').textContent = text;
}

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  state.ws = ws;
  ws.onopen = () => { setConn(true, 'connected'); sendEnabled(); };
  ws.onclose = () => { setConn(false, 'disconnected – retrying'); setTimeout(connect, 2000); };
  ws.onmessage = ev => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'health') {
      const before = new Set([...state.topics.values()].filter(isEnabled).map(r => r.name));
      state.topics = new Map(msg.topics.map(r => [r.name, r]));
      state.decodeErrors = msg.decode_errors || {};
      const after = [...state.topics.values()].filter(isEnabled).map(r => r.name);
      if (after.length !== before.size || after.some(n => !before.has(n))) sendEnabled();
      renderTopics(); Overlays.onHealth(); renderStats(); applyVisibility();
      if (state.inspecting) inspectTopic(state.inspecting.topic);
    } else if (msg.type === 'layers') {
      if (!map.isStyleLoaded()) return;
      for (const [topic, data] of Object.entries(msg.layers)) {
        state.layers.set(topic, data);
        if (data.layer === 'dm_signals') continue;          // drawn on the signal heads (overlays.js)
        ensureTopicLayer(topic, data.layer);
        map.getSource(sourceId(topic)).setData({ type: 'FeatureCollection', features: data.features });
      }
      Overlays.onLayers(); applyVisibility(); renderStats();
    }
  };
}

map.on('load', async () => {
  map.setLayoutProperty('basemap', 'visibility', state.prefs.basemap ? 'visible' : 'none');
  state.config = await (await fetch('/api/config')).json();
  addIntersectionLabels();
  Overlays.init(state.config);
  Settings.init(state.config);
  connect();
});
