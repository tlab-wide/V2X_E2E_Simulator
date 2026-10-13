/* Sidebar: problems summary on top, then the topics as a tree (category -> topic namespace). */
'use strict';

const Sidebar = (() => {
  const CATEGORY_ORDER = ['DM signal info', 'DM object info', 'Traffic signals', 'Autoware objects', 'Ego vehicle',
                          'Control', 'Clock', 'Other', 'Raw sensors'];
  // Problems shown above the tree, most serious first.
  const PROBLEMS = [
    { status: 'unsupported', title: 'Message type missing', cls: 'bad', open: true,
      hint: 'These message types are not in the sourced message workspace, so the topics cannot be received or decoded. ' +
            'Rebuild v2x_e2e_msgs with them, or point MSGS_SETUP to a workspace that has them.',
      detail: r => `type ${r.type}` },
    { status: 'waiting', title: 'Publisher but no data', cls: 'warn', open: true,
      hint: 'A publisher exists but nothing has arrived yet: the publisher may be idle, or its QoS is incompatible.',
      detail: r => `${r.publishers} publisher(s), ${r.type}` },
    { status: 'stale', title: 'Stopped publishing', cls: 'warn', open: true,
      hint: 'Messages arrived before but stopped: the simulator is paused, or the publisher stopped.',
      detail: r => `last message ${r.age_s}s ago` },
    { status: 'offline', title: 'Offline (no publisher)', cls: 'muted', open: false,
      hint: 'No publisher any more: the simulator is stopped, or the publisher is disabled in the scene.',
      detail: r => (r.age_s !== null ? `last message ${r.age_s}s ago` : 'never received') },
  ];
  const LEAF_COLLAPSE_LIMIT = 6;   // namespaces with more topics than this start collapsed

  const problemsEl = document.getElementById('problems');
  const topicsEl = document.getElementById('topics');
  const tree = () => (state.prefs.tree = state.prefs.tree || {});
  const problemsOpen = () => (state.prefs.problemsOpen = state.prefs.problemsOpen || {});

  function metaText(r) {
    if (r.status === 'live') return `${r.hz.toFixed(1)} Hz`;
    if (r.status === 'stale') return `stopped ${r.age_s}s ago`;
    if (r.status === 'waiting') return 'no data yet';
    if (r.status === 'unsupported') return 'type missing';
    if (r.status === 'raw') return 'raw sensor';
    return 'offline';
  }

  // ---------------------------------------------------------------------------------------------- problems
  function renderProblems(rows) {
    const monitored = rows.filter(r => r.status !== 'raw');
    let html = '';
    if (monitored.length && monitored.every(r => r.status === 'offline')) {
      html = `<div class="problem-banner muted">Simulator not publishing: all ${monitored.length} topics are offline. ` +
             'Start the simulator (Play) — topics come back automatically.</div>';
    } else {
      for (const p of PROBLEMS) {
        const list = monitored.filter(r => r.status === p.status).sort((a, b) => a.name.localeCompare(b.name));
        if (!list.length) continue;
        const open = p.status in problemsOpen() ? problemsOpen()[p.status] : p.open;
        html += `<div class="problem ${p.cls}${open ? ' open' : ''}" data-problem="${p.status}">` +
                `<div class="problem-head"><span class="chev">▾</span><span class="title">${escapeHtml(p.title)}</span>` +
                `<span class="count">${list.length}</span></div><div class="problem-body">` +
                `<div class="problem-hint">${escapeHtml(p.hint)}</div>`;
        for (const r of list) {
          html += `<div class="problem-item" data-topic="${escapeHtml(r.name)}"><div class="pname">${escapeHtml(r.name)}</div>` +
                  `<div class="phint">${escapeHtml(p.detail(r))}</div></div>`;
        }
        html += '</div></div>';
      }
    }
    problemsEl.innerHTML = html;
    problemsEl.style.display = html ? '' : 'none';
  }

  problemsEl.addEventListener('click', e => {
    const item = e.target.closest('.problem-item');
    if (item) { state.selected = item.dataset.topic; inspectTopic(item.dataset.topic); render(); return; }
    const head = e.target.closest('.problem-head');
    if (head) {
      const box = head.parentElement;
      problemsOpen()[box.dataset.problem] = !box.classList.contains('open');
      savePrefs(); render();
    }
  });

  // ---------------------------------------------------------------------------------------------- tree
  function newNode(name, path) { return { name, path, children: new Map(), topic: null }; }

  function buildTree(category, rows) {
    const root = newNode(category, 'cat:' + category);
    for (const r of rows) {
      let node = root;
      for (const seg of r.name.split('/').filter(Boolean)) {
        if (!node.children.has(seg)) node.children.set(seg, newNode(seg, node.path + '/' + seg));
        node = node.children.get(seg);
      }
      node.topic = r;
    }
    // Merge namespace chains with a single child: /v2x/rsu12020002 instead of /v2x -> rsu12020002
    const compress = node => {
      for (const [key, child] of [...node.children]) {
        let c = child;
        while (!c.topic && c.children.size === 1) {
          const only = [...c.children.values()][0];
          only.name = c.name + '/' + only.name;
          c = only;
        }
        node.children.delete(key);
        node.children.set(c.name, c);
        compress(c);
      }
    };
    compress(root);
    return root;
  }

  function topicsUnder(node, out = []) {
    if (node.topic) out.push(node.topic);
    for (const c of node.children.values()) topicsUnder(c, out);
    return out;
  }

  function isCollapsed(node, filtering) {
    if (filtering) return false;
    if (node.path in tree()) return tree()[node.path];
    if (node.path.startsWith('cat:') && !node.path.includes('/')) return false;   // categories start open
    if (node.topic) return true;                                                   // a topic's delayed copies start closed
    return topicsUnder(node).length > LEAF_COLLAPSE_LIMIT;
  }

  function aggregate(rows) {
    const live = rows.filter(r => r.status === 'live').length;
    const counts = {};
    for (const r of rows) counts[r.status] = (counts[r.status] || 0) + 1;
    const dot = live === rows.length ? 'live' : live ? 'partial' : counts.unsupported ? 'unsupported'
              : counts.stale ? 'stale' : counts.waiting ? 'waiting' : rows.every(r => r.status === 'raw') ? 'raw' : 'offline';
    const badges = [['unsupported', 'type missing', 'bad'], ['waiting', 'no data', 'warn'], ['stale', 'stopped', 'warn']]
      .filter(([s]) => counts[s]).map(([s, label, cls]) => `<span class="badge ${cls}">${counts[s]} ${label}</span>`).join('');
    return { live, total: rows.length, dot, badges };
  }

  function sortChildren(node) {
    return [...node.children.values()].sort((a, b) => a.name.localeCompare(b.name, undefined, { numeric: true }));
  }

  function renderNode(node, depth, filtering) {
    const pad = `style="padding-left:${8 + depth * 14}px"`;
    const hasChildren = node.children.size > 0;
    const collapsed = hasChildren && isCollapsed(node, filtering);
    const chev = hasChildren ? `<span class="chev${collapsed ? ' closed' : ''}" data-toggle="${escapeHtml(node.path)}">▾</span>`
                             : '<span class="chev-space"></span>';
    let html = '';
    if (node.topic) {
      const r = node.topic;
      const sel = state.selected === r.name ? ' selected' : '';
      const sw = r.renderable ? `<span class="swatch" style="background:${colorOf(r.name)}"></span>` : '';
      html += `<div class="row leaf${sel} st-${r.status}" data-topic="${escapeHtml(r.name)}" title="${escapeHtml(r.name + '\n' + r.type)}" ${pad}>` +
              `${chev}<input type="checkbox"${isEnabled(r) ? ' checked' : ''}${r.renderable ? '' : ' disabled'}>` +
              `<i class="s ${r.status}"></i><span class="tname">${escapeHtml(node.name)}</span>` +
              `<span class="meta">${sw}${escapeHtml(metaText(r))}</span></div>`;
    } else {
      const rows = topicsUnder(node);
      const agg = aggregate(rows);
      const renderable = rows.filter(r => r.renderable);
      const on = renderable.filter(isEnabled).length;
      const cb = renderable.length
        ? `<input type="checkbox" data-branch="${escapeHtml(node.path)}"${on === renderable.length ? ' checked' : ''}${on && on < renderable.length ? ' data-indeterminate="1"' : ''}>`
        : '<span class="cb-space"></span>';
      const cls = depth === 0 ? 'node category' : 'node';
      html += `<div class="row ${cls}" data-node="${escapeHtml(node.path)}" ${pad}>${chev}${cb}<i class="s ${agg.dot}"></i>` +
              `<span class="tname">${escapeHtml(node.name)}</span><span class="meta">${agg.badges}<span class="cnt">${agg.live}/${agg.total}</span></span></div>`;
    }
    if (hasChildren && !collapsed) {
      for (const c of sortChildren(node)) html += renderNode(c, depth + 1, filtering);
    }
    return html;
  }

  let roots = new Map();   // path -> node (for branch checkboxes)

  function render() {
    const all = [...state.topics.values()];
    renderProblems(all);
    const filter = document.getElementById('filter').value.trim().toLowerCase();
    const groups = new Map();
    for (const r of all) {
      if (r.status === 'raw' && !state.prefs.showRaw) continue;
      if (filter && !r.name.toLowerCase().includes(filter) && !r.type.toLowerCase().includes(filter)) continue;
      if (!groups.has(r.category)) groups.set(r.category, []);
      groups.get(r.category).push(r);
    }
    const cats = [...groups.keys()].sort((a, b) => CATEGORY_ORDER.indexOf(a) - CATEGORY_ORDER.indexOf(b));
    roots = new Map();
    let html = '';
    for (const c of cats) {
      const root = buildTree(c, groups.get(c));
      const index = n => { roots.set(n.path, n); n.children.forEach(index); };
      index(root);
      html += renderNode(root, 0, !!filter);
    }
    const scroll = topicsEl.scrollTop;
    topicsEl.innerHTML = html || '<div class="empty">No topics yet. Is the simulator running?</div>';
    topicsEl.querySelectorAll('[data-indeterminate]').forEach(cb => { cb.indeterminate = true; });
    topicsEl.scrollTop = scroll;
  }

  topicsEl.addEventListener('click', e => {
    const toggle = e.target.closest('[data-toggle]');
    if (toggle) {
      const node = roots.get(toggle.dataset.toggle);
      tree()[toggle.dataset.toggle] = !isCollapsed(node, false);
      savePrefs(); render(); return;
    }
    const branch = e.target.closest('input[data-branch]');
    if (branch) {
      for (const r of topicsUnder(roots.get(branch.dataset.branch)))
        if (r.renderable) state.prefs.enabled[r.name] = branch.checked;
      savePrefs(); sendEnabled(); applyVisibility(); render(); return;
    }
    const nodeRow = e.target.closest('.row.node');
    if (nodeRow) {
      const node = roots.get(nodeRow.dataset.node);
      tree()[node.path] = !isCollapsed(node, false);
      savePrefs(); render(); return;
    }
    const leaf = e.target.closest('.row.leaf');
    if (!leaf) return;
    const name = leaf.dataset.topic;
    if (e.target.matches('input[type=checkbox]')) {
      state.prefs.enabled[name] = e.target.checked;
      savePrefs(); sendEnabled(); applyVisibility(); render(); return;
    }
    state.selected = name;
    inspectTopic(name);
    render();
  });

  return { render };
})();
