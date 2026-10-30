// Robot-mounted tablet page: shows the current task in large text and a manual confirm button if the camera does not recognize it.
// Uses the same /ws/ui/ as the control page (snapshot / state / cargo / log messages).

const $ = (sel) => document.querySelector(sel);
const S = { robot: null, task: null, cargo: null, yolo: null, lastLog: '', stateKey: '', stateSince: Date.now(), ws: null,
            view: null,         // currently drawn view (action the button will use)
            dismissed: null };  // YOLO mismatch question closed with "reload/unload again" (not asked again for the same text)

// show the manual button if the camera has not recognized within this time (s)
const MANUAL_AFTER = 20;

// if a detection label is a color name, show it in that color (same as app.js)
const CARGO_COLORS = { red: '#ef4444', green: '#22c55e', blue: '#3b82f6', yellow: '#eab308', white: '#e5e7eb',
                       빨강: '#ef4444', 초록: '#22c55e', 파랑: '#3b82f6', 노랑: '#eab308' };
const cargoColor = (label) => CARGO_COLORS[(label || '').toLowerCase()] || '#94a3b8';

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  // Robot-mounted screen: robot name in the URL, e.g. /robot?robot=tb3_1 (server default tb3 if omitted)
  const name = new URLSearchParams(location.search).get('robot');
  const ws = new WebSocket(`${proto}://${location.host}/ws/ui/${name ? `?robot=${encodeURIComponent(name)}` : ''}`);
  S.ws = ws;
  ws.onopen = () => { $('#conn').textContent = '연결됨'; $('#conn').className = 'conn on'; };
  ws.onclose = () => {
    $('#conn').textContent = '연결 안 됨';
    $('#conn').className = 'conn off';
    setTimeout(connect, 2000);
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    switch (msg.type) {
      case 'snapshot':
        S.robot = msg.robot;
        S.task = msg.task;
        S.cargo = msg.cargo;
        S.yolo = msg.yolo;
        S.lastLog = msg.logs && msg.logs.length ? msg.logs[0].message : '';
        break;
      case 'state':
        S.robot = msg.robot;
        S.task = msg.task;
        break;
      case 'cargo':
        S.cargo = msg.label;
        break;
      case 'yolo':
        S.yolo = msg;
        break;
      case 'log':
        S.lastLog = msg.log.message;
        break;
      case 'error':
        S.lastLog = msg.message;
        break;
      default:
        return;
    }
    render();
  };
}

function send(action, extra = {}) {
  if (S.ws && S.ws.readyState === WebSocket.OPEN) S.ws.send(JSON.stringify({ action, ...extra }));
}

function currentStep() {
  if (!S.task || S.task.status !== 'running') return null;
  return S.task.steps.find((s) => s.status !== 'done' && s.status !== 'skipped') || null;
}

// ---------------------------------------------------------------- orders (multiple items)
const hasItems = (s) => !!(s && s.items && Object.keys(s.items).length);
const itemsText = (items) => Object.entries(items).map(([k, v]) => `${k} ${v}개`).join(', ');

// YOLO counts text "DHT11: 0 / led_red: 1 / ..." -> {led_red: 1, ...}
function parseCounts(text) {
  const out = {};
  for (const part of (text || '').split('/')) {
    const [name, n] = part.split(':').map((x) => (x || '').trim());
    if (name && /^\d+$/.test(n || '')) out[name.toLowerCase()] = Number(n);
  }
  return out;
}

// counts to check at this step (same as server expected_counts): load = this section items only, receive = all ordered items at 0
function expected(step) {
  if (step.action === 'unload') return Object.fromEntries(Object.keys(step.items).map((k) => [k, 0]));
  return { ...step.items };
}

function renderCounts(step) {
  const table = $('#counts');
  if (!hasItems(step) || step.status !== 'arrived') { table.hidden = true; return; }
  const want = expected(step);
  const fresh = S.yolo && S.yolo.counts;
  const have = parseCounts(fresh);
  table.hidden = false;
  table.innerHTML = '';
  for (const [label, n] of Object.entries(want)) {
    const tr = document.createElement('tr');
    const h = fresh ? (have[label.toLowerCase()] || 0) : null;
    tr.className = h === null ? '' : h === n ? 'ok' : 'bad';
    const mark = h === null ? '?' : h === n ? '✓' : '✗';
    const cells = step.action === 'unload'
      ? [label, h === null ? '-' : `${h}개 남음`, mark]
      : [label, `${h === null ? '-' : h} / ${n}`, mark];
    for (const c of cells) {
      const td = document.createElement('td');
      td.textContent = c;
      tr.appendChild(td);
    }
    table.appendChild(tr);
  }
}

function orderView(state, step, r) {
  if (state === 'WAITING_LOAD' || state === 'WAITING_UNLOAD') {
    const load = state === 'WAITING_LOAD';
    const base = load
      ? { key: 'LOAD', title: `${itemsText(step.items)} 실어 주세요`, sub: `${step.name} · 다 실으면 "적재 완료"` }
      : { key: 'UNLOAD', title: '물건을 꺼내 주세요', sub: `${itemsText(step.items)} · 다 꺼내면 "꺼냈어요"` };
    // Button pressed but the server found a YOLO mismatch (wrong load / not fully unloaded / no YOLO counts) -> ask the person
    const warn = /^(적재 잘못|꺼내기 덜 됨|YOLO)/.test(r.message || '') ? r.message : '';
    if (warn && S.dismissed !== warn) {
      return { ...base, ask: true, warn: `${warn} — 직접 확인하셨으면 보내 주세요`,
               manual: { label: load ? '✓ 확인했어요, 보내기' : '✓ 확인했어요, 홈으로', action: 'force_next', extra: { confirmed: true }, now: true },
               force: { label: load ? '↺ 다시 싣기' : '↺ 다시 꺼내기', local: 'dismiss', key: warn } };
    }
    return { ...base,
             manual: { label: load ? '✓ 적재 완료' : '✓ 꺼냈어요', action: load ? 'confirm_load' : 'confirm_unload', now: true, check: true },
             force: { label: load ? '다음 경유지로 (확인 없이)' : '홈으로 보내기 (확인 없이)', action: 'force_next', small: true,
                      ask: load ? 'YOLO 확인 없이 다음 경유지로 보낼까요?' : 'YOLO 확인 없이 홈으로 보낼까요?' } };
  }
  if (state === 'MOVING' && step.action === 'load') {
    return { key: 'GO_LOAD', title: `${step.name}(으)로 가는 중`, sub: `${itemsText(step.items)} 실으러 가요` };
  }
  if (state === 'MOVING' && step.action === 'unload') {
    return { key: 'DELIVER', title: '배달 중', sub: `${step.name}(으)로 가고 있어요` };
  }
  return null;
}

// content to show now {key, title, sub, led, manual, force, warn}
function view() {
  const r = S.robot;
  const state = r ? r.state : 'OFFLINE';
  const step = currentStep();
  const cargo = step && step.cargo;
  if (state === 'OFFLINE') return { key: 'OFFLINE', title: '로봇 연결 안 됨', sub: '로봇(브리지)이 꺼져 있거나 응답이 없습니다' };
  if (state === 'ERROR') return { key: 'ERROR', title: '문제가 생겼어요', sub: r.message || '관제 화면을 확인해 주세요' };
  if (hasItems(step)) {
    const v = orderView(state, step, r);
    if (v) return v;
  }
  if (state === 'WAITING_LOAD') {
    return cargo
      ? { key: 'LOAD', title: `${cargo} 을(를) 실어 주세요`, sub: '실으면 카메라가 확인하고 바로 출발합니다', led: cargo,
          manual: { label: '✓ 실었어요', action: 'confirm_load' } }
      : { key: 'LOAD', title: '물건을 실어 주세요', sub: r.message, manual: { label: '✓ 적재 완료', action: 'confirm_load', now: true } };
  }
  if (state === 'WAITING_UNLOAD') {
    return { key: 'UNLOAD', title: `${cargo || '물건'} 을(를) 꺼내 주세요`, sub: '꺼내면 카메라가 확인하고 돌아갑니다', led: cargo,
             manual: { label: '✓ 꺼냈어요', action: 'confirm_unload' } };
  }
  if (state === 'MOVING' && step) {
    if (step.action === 'load' && cargo) return { key: 'GO_LOAD', title: `${cargo} 가지러 가는 중`, sub: `${step.name}(으)로 이동 중`, led: cargo };
    if (step.action === 'unload') return { key: 'DELIVER', title: '배달 중', sub: `${cargo} 을(를) 요청한 자리로 가고 있어요`, led: cargo };
    if (step.action === 'move') return { key: 'RETURN', title: `${step.name}(으)로 가는 중`, sub: '' };
    return { key: 'MOVING', title: '이동 중', sub: r.message };
  }
  if (state === 'MOVING') return { key: 'MOVING', title: '이동 중', sub: r.message };
  return { key: 'IDLE', title: '대기 중', sub: '관제 화면의 "주문" 으로 부를 수 있어요' };
}

function render() {
  const v = view();
  if (v.key !== S.stateKey) {
    S.stateKey = v.key;
    S.stateSince = Date.now();
  }
  document.body.className = v.key;
  $('#title').textContent = v.title;
  $('#sub').textContent = v.sub || '';
  const led = $('#led');
  led.hidden = !v.led;
  if (v.led) {
    led.style.background = cargoColor(v.led);
    led.textContent = v.led;
  }
  // Recent notices, e.g. camera saw a different item (only while waiting to load/receive)
  $('#note').textContent = v.warn || ((v.key === 'LOAD' || v.key === 'UNLOAD') && S.lastLog.startsWith('카메라') ? S.lastLog : '');
  renderCounts(currentStep());
  S.view = v;
  const force = $('#btn-force');
  force.hidden = !v.force;
  if (v.force) {
    force.textContent = v.force.label;
    force.classList.toggle('small', !!v.force.small);
  }
  document.body.classList.toggle('ask', !!v.ask);
  const btn = $('#btn-manual');
  const waited = (Date.now() - S.stateSince) / 1000;
  if (v.manual && (v.manual.now || waited >= MANUAL_AFTER || S.cargo === null || S.cargo === undefined)) {
    btn.hidden = false;
    btn.textContent = v.manual.label;
    btn.dataset.action = v.manual.action;
  } else {
    btn.hidden = true;
  }
  const r = S.robot;
  $('#battery').textContent = r && r.battery_percentage != null ? `배터리 ${Math.round(r.battery_percentage)}%` : '';
  $('#camera').textContent = S.cargo === null || S.cargo === undefined ? '카메라: 인식 노드 없음'
    : `카메라: ${S.cargo || '비어 있음'}`;
  const step = currentStep();
  $('#step').textContent = S.task && S.task.status === 'running'
    ? S.task.steps.map((s) => (s === step ? `[${s.name}]` : s.name)).join(' → ') : '';
}

$('#btn-manual').addEventListener('click', () => {
  const m = S.view && S.view.manual;
  if (m && m.check) S.dismissed = null;   // re-check requested: ask again if it still differs
  if (m && m.extra) send(m.action, m.extra);
  else send($('#btn-manual').dataset.action);
});
$('#btn-force').addEventListener('click', () => {
  const f = S.view && S.view.force;
  if (!f) return;
  if (f.local === 'dismiss') { S.dismissed = f.key; render(); return; }
  if (!f.ask || confirm(f.ask)) send(f.action);
});

setInterval(render, 1000);  // refresh when the manual button appears
connect();
