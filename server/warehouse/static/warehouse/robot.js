// 로봇 위 태블릿 화면: 지금 할 일을 크게 보여주고, 카메라가 인식하지 못하면 수동 확인 버튼을 띄운다.
// 관제 화면과 같은 /ws/ui/ 를 쓴다 (snapshot / state / cargo / log 메시지).

const $ = (sel) => document.querySelector(sel);
const S = { robot: null, task: null, cargo: null, yolo: null, lastLog: '', stateKey: '', stateSince: Date.now(), ws: null };

// 카메라가 이 시간 안에 인식하지 못하면 수동 버튼을 보여준다 (초)
const MANUAL_AFTER = 20;

// 인식 라벨이 색 이름이면 그 색으로 표시 (관제 화면 app.js 와 같음)
const CARGO_COLORS = { red: '#ef4444', green: '#22c55e', blue: '#3b82f6', yellow: '#eab308', white: '#e5e7eb',
                       빨강: '#ef4444', 초록: '#22c55e', 파랑: '#3b82f6', 노랑: '#eab308' };
const cargoColor = (label) => CARGO_COLORS[(label || '').toLowerCase()] || '#94a3b8';

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  // 로봇에 달린 화면: /robot?robot=tb3_1 처럼 그 로봇 이름을 주소에 (없으면 서버 기본 tb3)
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

// ---------------------------------------------------------------- 주문 (여러 물건)
const hasItems = (s) => !!(s && s.items && Object.keys(s.items).length);
const itemsText = (items) => Object.entries(items).map(([k, v]) => `${k} ${v}개`).join(', ');

// YOLO 개수 글자 "DHT11: 0 / led_red: 1 / ..." → {led_red: 1, ...}
function parseCounts(text) {
  const out = {};
  for (const part of (text || '').split('/')) {
    const [name, n] = part.split(':').map((x) => (x || '').trim());
    if (name && /^\d+$/.test(n || '')) out[name.toLowerCase()] = Number(n);
  }
  return out;
}

// 이 단계에서 적재함에 보여야 할 개수 (서버 expected_counts 와 같음): 적재는 이 단계까지 누적, 수령은 모두 0
function expected(step) {
  if (step.action === 'unload') return Object.fromEntries(Object.keys(step.items).map((k) => [k, 0]));
  const total = {};
  for (const s of S.task.steps) {
    if (s.action !== 'load' || s.order > step.order) continue;
    for (const [k, v] of Object.entries(s.items || {})) total[k] = (total[k] || 0) + v;
  }
  return total;
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
    tr.className = (h === null ? '' : h === n ? 'ok' : 'bad') + (step.items[label] && step.action === 'load' ? ' now' : '');
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
  // 서버가 남긴 판단 결과 (적재 잘못 / 꺼내기 덜 됨 / YOLO 개수 없음) 는 note 로
  const warn = /^(적재 잘못|꺼내기 덜 됨|YOLO)/.test(r.message || '') ? r.message : '';
  if (state === 'WAITING_LOAD') {
    return { key: 'LOAD', title: `${itemsText(step.items)} 실어 주세요`, sub: `${step.name} · 다 실으면 "적재 완료"`, warn,
             manual: { label: '✓ 적재 완료', action: 'confirm_load', now: true },
             force: { label: '⏭ 다음 경유지로', action: 'force_next', ask: 'YOLO 확인 없이 다음 경유지로 보낼까요?' } };
  }
  if (state === 'WAITING_UNLOAD') {
    return { key: 'UNLOAD', title: '물건을 꺼내 주세요', sub: `${itemsText(step.items)} · 다 꺼내면 "꺼냈어요"`, warn,
             manual: { label: '✓ 꺼냈어요', action: 'confirm_unload', now: true },
             force: { label: '⏭ 강제 보내기', action: 'force_next', ask: 'YOLO 확인 없이 홈으로 보낼까요?' } };
  }
  if (state === 'MOVING' && step.action === 'load') {
    return { key: 'GO_LOAD', title: `${step.name}(으)로 가는 중`, sub: `${itemsText(step.items)} 실으러 가요` };
  }
  if (state === 'MOVING' && step.action === 'unload') {
    return { key: 'DELIVER', title: '배달 중', sub: `${step.name}(으)로 가고 있어요` };
  }
  return null;
}

// 지금 화면에 보여줄 내용 {key, title, sub, led, manual, force, warn}
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
  return { key: 'IDLE', title: '대기 중', sub: '관제 화면의 "주문" 또는 "LED 요청" 으로 부를 수 있어요' };
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
  // 카메라가 다른 물건을 봤다는 등 최근 안내 (적재/수령 대기 중일 때만)
  $('#note').textContent = v.warn || ((v.key === 'LOAD' || v.key === 'UNLOAD') && S.lastLog.startsWith('카메라') ? S.lastLog : '');
  renderCounts(currentStep());
  const force = $('#btn-force');
  force.hidden = !v.force;
  if (v.force) {
    force.textContent = v.force.label;
    force.dataset.action = v.force.action;
    force.dataset.ask = v.force.ask;
  }
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
  const action = $('#btn-manual').dataset.action;
  if (action) send(action);
});
$('#btn-force').addEventListener('click', () => {
  const b = $('#btn-force');
  if (b.dataset.action && confirm(b.dataset.ask)) send(b.dataset.action);
});
$('#btn-stop').addEventListener('click', () => {
  if (confirm('로봇을 멈추고 진행 중인 요청을 취소할까요?')) send('estop');
});

setInterval(render, 1000);  // 수동 버튼이 나타나는 시점 갱신
connect();
