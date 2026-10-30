// Orderer page: pick the delivery location (person location) and item counts, place the order and watch its progress.
// Uses the same /ws/ui/ as the control page (snapshot / state / locations / order_preview / error messages).
// Adding ?place=<location id> to the URL preselects that location (handy as a per-seat bookmark/QR code).

const $ = (sel) => document.querySelector(sel);
const S = {
  ws: null, robot: null, task: null, locations: [],
  place: null,        // selected person location id
  counts: {},         // label -> count
  watching: null,     // order (task) id watched on this page: to show 'done' after it ends
  sent: false,        // order sent, waiting for the reply (state)
};

const COLORS = { red: '#ef4444', green: '#22c55e', blue: '#3b82f6', yellow: '#eab308', white: '#e5e7eb' };
const color = (label) => COLORS[label.toLowerCase().replace(/^led_/, '')] || '#94a3b8';
const STATE_TEXT = { OFFLINE: '로봇 꺼짐', IDLE: '주문 가능', MOVING: '이동 중', WAITING_LOAD: '싣는 중',
                     WAITING_UNLOAD: '받는 중', ERROR: '문제 발생' };

function store(key, value) {
  try { value === null ? localStorage.removeItem(key) : localStorage.setItem(key, value); } catch (e) { /* fine if it cannot be stored */ }
}
function load(key) {
  try { return localStorage.getItem(key); } catch (e) { return null; }
}

const params = new URLSearchParams(location.search);
S.place = Number(params.get('place') || load('order.place')) || null;

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const name = params.get('robot');
  const ws = new WebSocket(`${proto}://${location.host}/ws/ui/${name ? `?robot=${encodeURIComponent(name)}` : ''}`);
  S.ws = ws;
  ws.onclose = () => {
    S.robot = null;
    render();
    setTimeout(connect, 2000);
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    switch (msg.type) {
      case 'snapshot':
        S.robot = msg.robot;
        S.task = msg.task;
        S.locations = msg.locations || [];
        if (isOrder(S.task) && S.task.status === 'running') S.watching = S.task.id;
        break;
      case 'state':
        S.robot = msg.robot;
        S.task = msg.task;
        if (S.sent && isOrder(S.task) && S.task.status === 'running') { S.watching = S.task.id; S.sent = false; }
        break;
      case 'locations':
        S.locations = msg.locations || [];
        break;
      case 'order_preview':
        $('#preview').textContent = `가는 순서: ${msg.names.join(' → ')} · 받기까지 약 ${fmtSec(msg.total)}`;
        return;
      case 'error':
        S.sent = false;
        showError(msg.message);
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

let errorTimer = null;
function showError(text) {
  const el = $('#error');
  el.textContent = text;
  el.hidden = false;
  clearTimeout(errorTimer);
  errorTimer = setTimeout(() => { el.hidden = true; }, 6000);
}

function fmtSec(sec) {
  sec = Math.max(0, Math.round(sec));
  return sec >= 60 ? `${Math.floor(sec / 60)}분 ${sec % 60}초` : `${sec}초`;
}

const isOrder = (task) => !!(task && task.steps.some((s) => s.items && Object.keys(s.items).length));
const itemsText = (items) => Object.entries(items).map(([k, v]) => `${k} ${v}개`).join(', ');

// ---------------------------------------------------------------- order form
function places() { return S.locations.filter((l) => l.kind === 'dropoff'); }
function labels() {
  const all = S.locations.filter((l) => l.kind === 'pickup')
    .flatMap((l) => (l.cargo || '').split(',').map((x) => x.trim()).filter(Boolean));
  return [...new Set(all)];
}
function chosenItems() {
  const items = {};
  for (const label of labels()) if (S.counts[label] > 0) items[label] = S.counts[label];
  return Object.keys(items).length ? items : null;
}

function renderForm() {
  const box = $('#places');
  const list = places();
  if (S.place && !list.some((l) => l.id === S.place)) S.place = null;
  box.innerHTML = '';
  if (!list.length) box.innerHTML = '<p class="muted">등록된 받을 위치가 없습니다 (관리자에게 문의)</p>';
  for (const loc of list) {
    const b = document.createElement('button');
    b.className = 'place' + (loc.id === S.place ? ' active' : '');
    b.innerHTML = '<span class="radio"></span><span class="pname"></span>';
    b.querySelector('.pname').textContent = loc.name;
    b.addEventListener('click', () => {
      S.place = loc.id;
      store('order.place', String(loc.id));
      render();
      preview();
    });
    box.appendChild(b);
  }

  const itemsBox = $('#items');
  itemsBox.innerHTML = '';
  const ls = labels();
  if (!ls.length) itemsBox.innerHTML = '<p class="muted">주문할 수 있는 물건이 없습니다 (관리자에게 문의)</p>';
  for (const label of ls) {
    const row = document.createElement('div');
    row.className = 'item';
    const n = S.counts[label] || 0;
    row.className = 'item' + (n ? ' chosen' : '');
    row.innerHTML = `<i style="background:${color(label)}"></i><span class="name"></span>
      <span class="stepper"><button data-d="-1" aria-label="빼기">−</button><span class="count">${n}</span><button data-d="1" aria-label="더하기">+</button></span>`;
    row.querySelector('.name').textContent = label;
    row.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
      S.counts[label] = Math.min(9, Math.max(0, (S.counts[label] || 0) + Number(b.dataset.d)));
      render();
      preview();
    }));
    row.querySelector('[data-d="-1"]').disabled = n === 0;
    itemsBox.appendChild(row);
  }

  const items = chosenItems();
  const place = list.find((l) => l.id === S.place);
  $('#summary').textContent = place && items ? `${place.name} 으로 ${itemsText(items)}`
    : !place ? '받을 위치를 고르세요' : '물건 개수를 정하세요';
  if (!(place && items)) $('#preview').textContent = '';
  const state = S.robot ? S.robot.state : 'OFFLINE';
  const busy = (S.task && S.task.status === 'running') || state !== 'IDLE';
  $('#busy').textContent = !S.robot ? '서버에 연결 중...' : state === 'OFFLINE' ? '로봇이 꺼져 있어 주문할 수 없습니다'
    : busy ? '로봇이 다른 일을 하는 중입니다. 끝나면 주문할 수 있어요' : '';
  $('#btn-order').disabled = !(place && items) || busy || S.sent;
  $('#btn-order').textContent = S.sent ? '주문 보내는 중...' : '주문하기';
}

let previewTimer = null;
function preview() {
  clearTimeout(previewTimer);
  const items = chosenItems();
  if (!S.place || !items) return;
  previewTimer = setTimeout(() => send('preview_order', { dropoff_id: S.place, items }), 300);
}

$('#btn-order').addEventListener('click', () => {
  const items = chosenItems();
  const place = places().find((l) => l.id === S.place);
  if (!items || !place) return;
  if (!confirm(`${place.name} 으로 ${itemsText(items)} 주문할까요?`)) return;
  S.sent = true;
  send('start_order', { dropoff_id: S.place, items });
  render();
});

// ---------------------------------------------------------------- progress
function stepLine(s, now, state) {
  const what = s.action === 'load' ? `${itemsText(s.items)} 싣기`
    : s.action === 'unload' ? `받기 (${itemsText(s.items)})` : '로봇 복귀';
  let detail = '';
  if (s === now) {
    if (s.status === 'moving') {
      detail = '가는 중';
      if (s.predicted_sec != null && s.started_at) {
        const left = s.predicted_sec - (Date.now() - new Date(s.started_at).getTime()) / 1000;
        detail += left > 0 ? ` · 약 ${fmtSec(left)}` : ' · 조금 늦어지고 있어요';
      }
    } else if (s.status === 'arrived') {
      detail = s.action === 'unload' ? '도착' : '싣는 중';
    } else if (s.status === 'failed' || state === 'ERROR') {
      detail = '멈춤 (관리자 확인 중)';
    }
  }
  const mark = s.status === 'done' ? '✓' : s.status === 'skipped' ? '–' : s === now ? '▶' : '·';
  const cls = s.status === 'done' || s.status === 'skipped' ? 'done' : s === now ? 'now' : '';
  return { mark, cls, name: s.name, what: detail ? `${what} · ${detail}` : what };
}

function renderStatus() {
  const t = S.task;
  const r = S.robot;
  const state = r ? r.state : 'OFFLINE';
  const running = t.status === 'running';
  const now = running ? t.steps.find((s) => s.status !== 'done' && s.status !== 'skipped') : null;
  const dropoff = t.steps.find((s) => s.action === 'unload');
  if (running) {
    const atMe = now && now.action === 'unload' && now.status === 'arrived';
    const after = dropoff && (dropoff.status === 'done' || dropoff.status === 'skipped');
    $('#status-title').textContent = atMe ? '로봇이 도착했어요' : after ? '받기 완료 · 로봇 복귀 중'
      : state === 'ERROR' ? '로봇이 멈췄어요' : '주문 진행 중';
    $('#status-sub').textContent = atMe ? '물건을 꺼내고 로봇 화면에서 "꺼냈어요" 를 눌러 주세요'
      : dropoff ? `${dropoff.name} · ${itemsText(dropoff.items)}` : '';
    $('.hero').classList.toggle('arrived', !!atMe);
  } else {
    $('.hero').classList.remove('arrived');
    $('#status-title').textContent = { done: '배달 완료', canceled: '주문 취소됨', failed: '주문 실패' }[t.status] || t.status;
    $('#status-sub').textContent = dropoff ? `${dropoff.name} · ${itemsText(dropoff.items)}` : '';
  }
  // big ETA while the robot heads to the person's location
  const eta = $('#status-eta');
  eta.hidden = true;
  if (running && now && now.status === 'moving' && now.predicted_sec != null && now.started_at) {
    const left = now.predicted_sec - (Date.now() - new Date(now.started_at).getTime()) / 1000;
    eta.textContent = `${now.name}까지 ${left > 0 ? `약 ${fmtSec(left)}` : '곧 도착'}`;
    eta.hidden = false;
  }
  const ol = $('#steps');
  ol.innerHTML = '';
  for (const s of t.steps) {
    const l = stepLine(s, now, state);
    const li = document.createElement('li');
    li.className = l.cls;
    li.innerHTML = '<span class="mark"></span><span class="body"><b></b><span class="what"></span></span>';
    li.querySelector('.mark').textContent = l.mark;
    li.querySelector('b').textContent = l.name;
    li.querySelector('.what').textContent = l.what;
    ol.appendChild(li);
  }
  // Server notices, e.g. YOLO count mismatch at a section
  $('#status-note').textContent = running && /^(적재 잘못|꺼내기 덜 됨|YOLO)/.test((r && r.message) || '') ? r.message : '';
  $('#btn-cancel').hidden = !running;
  $('#btn-new').hidden = running;
}

$('#btn-cancel').addEventListener('click', () => {
  if (confirm('주문을 취소할까요? 로봇은 그 자리에 멈춥니다.')) send('cancel_task');
});
$('#btn-new').addEventListener('click', () => {
  S.watching = null;
  S.counts = {};
  render();
});

// ---------------------------------------------------------------- view
function render() {
  const state = S.robot ? S.robot.state : 'OFFLINE';
  const pill = $('#robot-state');
  pill.textContent = S.robot ? STATE_TEXT[state] || state : '연결 중';
  pill.className = `pill ${state}`;
  // Show progress if an order is running or the one watched here just finished
  const showStatus = isOrder(S.task) && (S.task.status === 'running' || S.task.id === S.watching);
  $('#form-view').hidden = showStatus;
  $('#status-view').hidden = !showStatus;
  if (showStatus) renderStatus();
  else renderForm();
}

setInterval(() => { if (!$('#status-view').hidden) render(); }, 1000);   // refresh remaining time
render();
connect();
