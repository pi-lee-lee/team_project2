'use strict';

// ------------------------------------------------------------------ 상태
const S = {
  ws: null,
  robot: null,
  task: null,
  map: null,          // {resolution, origin:[x,y], width, height, image_url}
  mapImg: null,
  locations: [],
  zones: [],          // 금지 구역 [{id, name, shape: rect|line, x1, y1, x2, y2}] - A* 가 피함
  pose: null,         // {x, y, yaw}
  path: [],           // [[x, y], ...]
  selected: new Set(),
  mode: 'view',       // view | goto | add | initpose | deliver | zone | wall
  view: { scale: 1, ox: 0, oy: 0 },  // scale = 맵 한 칸의 캔버스 픽셀 수, (ox, oy) = 돌린 좌표 (0, 0) 의 캔버스 위치
  rot: 0,             // 화면 회전 (rad, 반시계): 맵의 view_yaw. 새 SLAM 을 시작할 때 로봇 정면이 화면 위쪽이 되게 서버가 정함
  drag: null,
};

const $ = (sel) => document.querySelector(sel);
const canvas = $('#map');
const ctx = canvas.getContext('2d');

// ------------------------------------------------------------------ WebSocket
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws/ui/`);
  S.ws = ws;
  ws.onopen = () => {
    setWsStatus(true);
    ws.send(JSON.stringify({ action: 'travel_model' }));  // 이동 시간 모델 요약
  };
  ws.onclose = () => {
    setWsStatus(false);
    // 서버가 끊기면 로봇 상태를 알 수 없으므로 화면에서 지운다
    clearPose();
    S.health = { bridge: false };
    renderHealth();
    setTimeout(connect, 2000);
  };
  ws.onmessage = (ev) => handle(JSON.parse(ev.data));
}

function send(action, data = {}) {
  if (!S.ws || S.ws.readyState !== WebSocket.OPEN) {
    toast('서버에 연결되어 있지 않습니다.');
    return;
  }
  S.ws.send(JSON.stringify({ action, ...data }));
}

function handle(msg) {
  switch (msg.type) {
    case 'snapshot':
      S.robot = msg.robot;
      S.task = msg.task;
      S.locations = msg.locations;
      S.zones = msg.zones || [];
      renderZones();
      S.pose = msg.pose;
      S.path = msg.path || [];
      S.poseAt = msg.pose ? Date.now() : 0;
      S.health = msg.health || { bridge: false };
      renderHealth();
      if (msg.runner) setRunner(msg.runner);
      setMap(msg.map);
      S.cargo = msg.cargo;
      renderCargo();
      S.memory = msg.memory;
      S.obstacles = msg.obstacles || [];
      S.obstaclesAt = Date.now();
      $('#logs').innerHTML = '';
      msg.logs.slice().reverse().forEach(addLog);
      renderPanel();
      break;
    case 'state':
      S.robot = msg.robot;
      S.task = msg.task;
      renderPanel();
      break;
    case 'map':
      setMap(msg.map);
      break;
    case 'view_yaw':
      // 새 SLAM 실행: 로봇 정면이 화면 위쪽이 되는 각도. 맵이 오기 전 격자 화면도 바로 돌린다
      S.slamViewYaw = msg.view_yaw;
      if (S.map && S.map.virtual) useVirtualMap();
      break;
    case 'cargo':
      S.cargo = msg.label;
      renderCargo();
      break;
    case 'zones':
      S.zones = msg.zones || [];
      renderZones();
      draw();
      break;
    case 'obstacles':
      S.obstacles = msg.obstacles || [];
      S.obstaclesAt = Date.now();
      draw();
      break;
    case 'memory':
      S.memory = msg;
      draw();
      break;
    case 'locations':
      S.locations = msg.locations;
      for (const id of [...S.selected]) {
        if (!S.locations.some((l) => l.id === id)) S.selected.delete(id);
      }
      renderPanel();  // 홈 유무에 따라 버튼 상태가 바뀜
      break;
    case 'pose':
      if (msg.lost) {
        clearPose();
        break;
      }
      S.poseAt = Date.now();
      // 가상 맵인데 로봇이 가장자리(중심에서 3m 이상)로 가면 로봇 중심으로 다시 잡는다
      if (S.map && S.map.virtual) {
        const c = [S.map.origin[0] + VIRTUAL_SIZE_M / 2, S.map.origin[1] + VIRTUAL_SIZE_M / 2];
        if (!S.pose || Math.hypot(msg.x - c[0], msg.y - c[1]) > 3) {
          S.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
          useVirtualMap();
        }
      }
      if (!S.pose) {
        S.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
        renderPanel();  // 위치를 처음 알게 되면 "현재 위치를 홈으로" 활성화
      }
      S.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
      draw();
      break;
    case 'health':
      S.health = msg;
      renderHealth();
      break;
    case 'runner':
      if (msg.state === 'started') {
        S.runnerBusy = msg.job;
        toast(`${msg.job} 시작...`);
      } else {
        S.runnerBusy = null;
        if (msg.mode) S.runnerMode = msg.mode;
        toast(`${msg.job} ${msg.ok ? '완료' : '실패'}: ${msg.message}`);
      }
      renderRunner();
      break;
    case 'plan_order':
      showPlan(msg);
      break;
    case 'travel_model':
      showModel(msg);
      break;
    case 'runner_info':
      setRunner(msg);
      break;
    case 'reset_done':
      toast((msg.ok ? '상태 초기화 완료: ' : '상태 초기화 실패: ') + msg.message);
      break;
    case 'map_saved':
      toast((msg.ok ? '맵 저장 완료: ' : '맵 저장 실패: ') + msg.message);
      break;
    case 'path':
      S.path = msg.points;
      draw();
      break;
    case 'log':
      addLog(msg.log);
      break;
    case 'error':
      toast(msg.message);
      break;
  }
}

// 로봇이 사라지면(브리지 종료, Pi 꺼짐, 위치 TF 끊김) 화면에서 로봇과 경로를 지운다
function clearPose() {
  const had = !!S.pose;
  S.pose = null;
  S.path = [];
  S.obstacles = [];
  draw();
  if (had) renderPanel();
}

// 2초 넘게 위치가 안 오면 사라진 것으로 처리 (서버 연결이 끊긴 경우 포함)
setInterval(() => {
  if (S.pose && Date.now() - (S.poseAt || 0) > 2000) clearPose();
}, 500);

// 이동 중이면 "약 N초 남음" 을 갱신
setInterval(() => {
  if (S.task && S.task.status === 'running' && S.task.steps.some((st) => st.status === 'moving')) renderPanel();
}, 2000);

function renderHealth() {
  const h = S.health || {};
  if (document.getElementById('runner-card')) renderRunner();
  const slam = h.bridge && h.mode === 'slam';
  const modeEl = $('#run-mode');
  modeEl.hidden = !h.bridge;
  modeEl.textContent = slam ? '맵 만들기 (SLAM)' : '맵 사용 (AMCL)';
  modeEl.className = 'badge ' + (slam ? 'WAITING_LOAD' : 'MOVING');
  $('#btn-save-map').hidden = !slam;
  const initBtn = document.querySelector('button.mode[data-mode="initpose"]');
  initBtn.disabled = slam;
  initBtn.title = slam ? 'SLAM 모드에서는 SLAM이 위치를 잡으므로 필요 없습니다' : '';
  if (slam && S.mode === 'initpose') setMode('view');
  const item = (label, ok) => `<span class="chip ${ok ? 'ok' : 'bad'}"><span class="dot ${ok ? 'ok' : 'bad'}"></span>${label}</span>`;
  $('#health').innerHTML = [
    item('브리지', h.bridge),
    item('LiDAR', h.bridge && h.scan_ok),
    item('오도메트리', h.bridge && h.odom_ok),
    item('위치 추정', h.bridge && h.localized),
    item(h.tipped ? `넘어짐 (${h.tilt_deg}°)` : '자세', h.bridge && !h.tipped),
  ].join(' ');
}

// ------------------------------------------------------------------ 실행 모드
const ROBOT_LABEL = { sim: '시뮬레이션', real: '실물' };

function setRunner(info) {
  S.runnerMode = info.mode || {};
  S.runnerBusy = info.busy || null;
  const sel = $('#runner-map');
  const keep = sel.value;
  sel.innerHTML = '';
  const def = document.createElement('option');
  def.value = '';
  def.textContent = '(시뮬레이션 기본 맵: turtlebot3_world)';
  sel.appendChild(def);
  for (const m of info.maps || []) {
    const o = document.createElement('option');
    o.value = m.path;
    o.textContent = m.path;
    sel.appendChild(o);
  }
  // 지금 쓰는 맵을 기본 선택
  const cur = (S.runnerMode.map || '').split('/maps/').pop();
  sel.value = keep || ([...sel.options].some((o) => o.value === cur) ? cur : '');
  if (S.runnerMode.robot) $('#runner-robot').value = S.runnerMode.robot;
  S.runnerMaps = info.maps || [];
  const ws = $('#runner-world');
  const keepWorld = ws.value;
  ws.innerHTML = '';
  for (const w of info.worlds || []) {
    const o = document.createElement('option');
    o.value = w.name;
    o.textContent = w.label === w.name ? w.name : `${w.label} · ${w.name}`;
    ws.appendChild(o);
  }
  // 지금 실행 중인 월드를 기본 선택
  ws.value = keepWorld || S.runnerMode.world || 'turtlebot3_world';
  renderRunner();
}

// 선택한 맵을 만든 월드 (시뮬레이션에서 저장한 맵만 기록되어 있음)
function selectedMapWorld() {
  const path = $('#runner-map').value;
  if (!path) return 'turtlebot3_world';  // 시뮬레이션 기본 맵 = TurtleBot3 기본 월드 맵
  const m = (S.runnerMaps || []).find((x) => x.path === path);
  return m ? m.world : '';
}

// 맵을 고르면 그 맵을 만든 월드로 맞춘다 (다른 월드에서 AMCL 을 쓰면 좌표가 맞지 않음)
function syncWorldToMap() {
  const w = selectedMapWorld();
  if (w && [...$('#runner-world').options].some((o) => o.value === w)) $('#runner-world').value = w;
  renderRunner();
}
$('#runner-map').addEventListener('change', syncWorldToMap);
$('#runner-world').addEventListener('change', renderRunner);
$('#runner-robot').addEventListener('change', renderRunner);

function renderRunner() {
  const m = S.runnerMode || {};
  const h = S.health || {};
  const running = h.bridge;
  const world = m.robot === 'sim' && m.world ? ` (${m.world})` : '';
  $('#runner-current').textContent = running
    ? `현재: ${ROBOT_LABEL[m.robot] || m.robot || '?'}${world} / ${m.mode === 'slam' ? '맵 만들기 (SLAM)' : `위치 추정 (AMCL) · ${(m.map || '').split('/').pop() || '-'}${(m.amcl || '').includes('amcl_hall') ? ' · 강당 설정' : ''}`}`
    : `현재: 실행 중 아님${m.robot ? ` (마지막: ${ROBOT_LABEL[m.robot]} / ${m.mode === 'slam' ? 'SLAM' : 'AMCL'})` : ''}`;
  const sim = $('#runner-robot').value === 'sim';
  $('#runner-world-row').hidden = !sim;
  const mw = selectedMapWorld(), cw = $('#runner-world').value;
  $('#runner-world-hint').textContent = !sim ? ''
    : mw && mw !== cw ? `주의: 선택한 맵은 '${mw}' 월드에서 만든 맵입니다. AMCL 은 그 월드에서만 맞습니다 (맵 만들기에는 상관없음).`
    : mw ? `선택한 맵을 만든 월드: ${mw}` : '';
  const busy = !!S.runnerBusy;
  $('#runner-busy').textContent = busy ? `진행 중: ${S.runnerBusy} (수십 초 걸립니다)` : '';
  for (const id of ['#btn-run-amcl', '#btn-run-slam', '#btn-run-stop']) $(id).disabled = busy;
  $('#btn-auto-localize').disabled = busy || !(h.bridge && h.mode === 'map');
  const exploring = S.runnerBusy === '자동 탐색';
  $('#btn-explore').hidden = !(h.bridge && h.mode === 'slam') || exploring;
  $('#btn-explore').disabled = busy;
  $('#btn-explore-stop').hidden = !exploring;
}

function runnerStart(mode) {
  const robot = $('#runner-robot').value;
  const map = $('#runner-map').value;
  if (mode === 'map' && robot === 'real' && !map) { toast('실물 로봇은 맵을 선택하세요.'); return; }
  const world = robot === 'sim' ? $('#runner-world').value : '';
  const mw = selectedMapWorld();
  if (mode === 'map' && robot === 'sim' && mw && mw !== world
      && !confirm(`선택한 맵은 '${mw}' 월드에서 만든 맵인데 '${world}' 월드로 실행합니다.\n위치 추정이 맞지 않을 수 있습니다. 계속할까요?`)) return;
  const what = mode === 'slam' ? '맵 만들기 (SLAM)' : `위치 추정 (AMCL) · ${map || '기본 맵'}`;
  if (!confirm(`${ROBOT_LABEL[robot]}${world ? ` (${world})` : ''} / ${what} 으로 다시 시작할까요?\n(지금 실행 중인 노드는 정지됩니다)`)) return;
  send('runner_start', {
    robot, mode, map, world, amcl: $('#runner-amcl-hall').checked ? 'hall' : 'default',
    scan_noise: robot === 'sim' && $('#runner-scan-noise').checked,
  });
}

$('#btn-run-amcl').addEventListener('click', () => runnerStart('map'));
$('#btn-run-slam').addEventListener('click', () => runnerStart('slam'));
$('#btn-run-stop').addEventListener('click', () => {
  if (confirm('ROS 노드와 시뮬레이션을 모두 정지할까요? (웹 서버는 유지)')) send('runner_stop');
});
$('#btn-explore').addEventListener('click', () => {
  if (confirm('미탐색 경계를 따라 자동으로 이동하며 맵을 만들까요? (최대 30분, "탐색 중지"로 멈춤)')) send('explore_start');
});
$('#btn-explore-stop').addEventListener('click', () => send('explore_stop'));
$('#btn-auto-localize').addEventListener('click', () => {
  if (confirm('맵 전체에서 로봇 위치를 찾아 초기 위치로 지정할까요? (10~30초)')) send('auto_localize');
});

function setWsStatus(on) {
  const el = $('#ws-status');
  el.textContent = on ? '서버 연결됨' : '서버 연결 안 됨';
  el.className = 'badge ' + (on ? 'on' : 'off');
}

// ------------------------------------------------------------------ 좌표 변환
// 월드(m) -> S.rot 만큼 돌린 좌표 -> 캔버스 픽셀 (캔버스 y 는 아래로). 좌표 자체는 그대로, 보기만 돌린다
function rotXY(x, y, a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c * x - s * y, s * x + c * y];
}
function pxPerMeter() { return S.view.scale / S.map.resolution; }
function worldToCanvas(x, y) {
  const v = S.view, k = pxPerMeter();
  const [u, w] = rotXY(x, y, S.rot);
  return [u * k + v.ox, -w * k + v.oy];
}
function canvasToWorld(cx, cy) {
  const v = S.view, k = pxPerMeter();
  return rotXY((cx - v.ox) / k, -(cy - v.oy) / k, -S.rot);
}

// ------------------------------------------------------------------ 맵
// 맵이 아직 없을 때(SLAM 시작 직후 등): 로봇 주변 8m 를 격자로 보여주는 가상 맵.
// 좌표 변환은 진짜 맵과 똑같이 동작하므로 로봇/경로 표시와 "여기로 이동" 클릭을 그대로 쓸 수 있다
const VIRTUAL_SIZE_M = 8;
function useVirtualMap() {
  const p = S.pose || { x: 0, y: 0 };
  if (S.slamViewYaw !== undefined) S.rot = S.slamViewYaw;   // SLAM 시작 직후: 서버가 정한 각도 (로봇 정면이 위)
  const res = 0.05, n = VIRTUAL_SIZE_M / res;
  S.map = { virtual: true, name: '', resolution: res, origin: [p.x - VIRTUAL_SIZE_M / 2, p.y - VIRTUAL_SIZE_M / 2],
            width: n, height: n };
  S.mapImg = null;
  S.mapName = null;
  S.wbox = { x0: S.map.origin[0], y0: S.map.origin[1],
             x1: S.map.origin[0] + VIRTUAL_SIZE_M, y1: S.map.origin[1] + VIRTUAL_SIZE_M };
  $('#no-map').hidden = false;
  fitView();
}

function setMap(map) {
  if (!map || !map.width || !map.height) {
    useVirtualMap();
    return;
  }
  const wasVirtual = S.map && S.map.virtual;
  $('#no-map').hidden = true;
  const img = new Image();
  img.onload = () => {
    // 이미지가 준비된 뒤에 맵을 바꾼다 (원점이 바뀐 새 맵과 이전 이미지가 섞여 그려지지 않게)
    S.map = map;
    const rotChanged = Math.abs((map.view_yaw || 0) - S.rot) > 1e-6;
    S.rot = map.view_yaw || 0;
    const changed = !S.mapImg || S.mapName !== map.name || rotChanged;
    S.mapImg = img;
    S.mapName = map.name;
    const b = knownBounds(img, map);
    // 화면 범위는 월드 좌표로 기억한다. SLAM 맵은 커질 때 원점이 바뀌므로 픽셀로 기억하면
    // 갱신(3초)마다 화면 전체가 밀려 로봇/위치가 흔들려 보였다
    const w = S.wbox;
    const inside = w && b.x0 >= w.x0 && b.y0 >= w.y0 && b.x1 <= w.x1 && b.y1 <= w.y1;
    if (wasVirtual && inside) {
      fitView();  // SLAM 시작 때의 격자 화면(로봇 주변 8m)을 그대로 이어 쓴다
    } else if (changed && !wasVirtual || !inside) {
      // SLAM 으로 커지는 중이면 0.5m 여유를 두어 자주 바뀌지 않게
      const g = changed && !wasVirtual ? 0 : 0.5;
      const u = changed || !w ? b : { x0: Math.min(b.x0, w.x0), y0: Math.min(b.y0, w.y0),
                                      x1: Math.max(b.x1, w.x1), y1: Math.max(b.y1, w.y1) };
      S.wbox = { x0: u.x0 - g, y0: u.y0 - g, x1: u.x1 + g, y1: u.y1 + g };
      fitView();
    } else {
      fitView();  // 범위는 그대로, 새 원점에 맞춰 변환만 다시 계산 (화면상 위치는 변하지 않음)
    }
  };
  img.src = map.image_url;
  renderLocations();
}

// 미탐색(회색 205)이 아닌 영역의 경계 상자 = 실제 방 범위 (월드 좌표, m)
function knownBounds(img, map) {
  const c = document.createElement('canvas');
  c.width = img.width;
  c.height = img.height;
  const g = c.getContext('2d');
  g.drawImage(img, 0, 0);
  const d = g.getImageData(0, 0, img.width, img.height).data;
  let x0 = img.width, y0 = img.height, x1 = -1, y1 = -1;
  for (let y = 0; y < img.height; y++) {
    for (let x = 0; x < img.width; x++) {
      if (d[(y * img.width + x) * 4] !== 205) {
        if (x < x0) x0 = x;
        if (x > x1) x1 = x;
        if (y < y0) y0 = y;
        if (y > y1) y1 = y;
      }
    }
  }
  if (x1 < 0) { x0 = 0; y0 = 0; x1 = img.width - 1; y1 = img.height - 1; }
  const m = 4;  // 여백(칸)
  const r = map.resolution, [ox, oy] = map.origin;
  // 이미지 y 는 아래로, 월드 y 는 위로
  return { x0: ox + (x0 - m) * r, x1: ox + (x1 + 1 + m) * r,
           y0: oy + (img.height - (y1 + 1 + m)) * r, y1: oy + (img.height - (y0 - m)) * r };
}

// 화면을 방 범위에 맞춰 고정 (확대/축소/이동 없음)
function fitView() {
  if (!S.map) return;
  // 월드 범위(S.wbox)의 네 모서리를 화면 각도(S.rot)로 돌린 범위에 맞춘다
  const m = S.map, r = m.resolution;
  const wb = S.wbox || { x0: m.origin[0], y0: m.origin[1],
                         x1: m.origin[0] + m.width * r, y1: m.origin[1] + m.height * r };
  const pts = [[wb.x0, wb.y0], [wb.x1, wb.y0], [wb.x0, wb.y1], [wb.x1, wb.y1]].map(([x, y]) => rotXY(x, y, S.rot));
  const u0 = Math.min(...pts.map((p) => p[0])), u1 = Math.max(...pts.map((p) => p[0]));
  const v0 = Math.min(...pts.map((p) => p[1])), v1 = Math.max(...pts.map((p) => p[1]));
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const k = Math.min(w / (u1 - u0), h / (v1 - v0));   // 1m 당 캔버스 픽셀
  S.view = { scale: k * r, ox: (w - (u1 - u0) * k) / 2 - u0 * k, oy: (h - (v1 - v0) * k) / 2 + v1 * k };
  draw();
}

// 맵 이미지 픽셀 (px, py) -> 캔버스: 월드 (ox + px r, oy + (H - py) r) 를 돌려서 그린다
function drawMapImage() {
  const m = S.map, r = m.resolution, k = pxPerMeter(), v = S.view;
  const c = Math.cos(S.rot), s = Math.sin(S.rot);
  const top = m.origin[1] + m.height * r;
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.transform(k * r * c, -k * r * s, k * r * s, k * r * c,
                k * (c * m.origin[0] - s * top) + v.ox, -k * (s * m.origin[0] + c * top) + v.oy);
  ctx.drawImage(S.mapImg, 0, 0);
  ctx.restore();
}

// 월드 좌표 다각형 채우기/선 (돌린 화면에서도 맞게)
function worldPolygon(points) {
  ctx.beginPath();
  points.forEach(([x, y], i) => {
    const [cx, cy] = worldToCanvas(x, y);
    i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy);
  });
  ctx.closePath();
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  fitView();
  draw();
}

// ------------------------------------------------------------------ 그리기
function draw() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);
  if (!S.map) return;
  const v = S.view;
  if (S.map.virtual) {
    drawGrid();
  } else {
    if (!S.mapImg) return;
    drawMapImage();
  }

  const pxPerM = v.scale / S.map.resolution;

  // 맵 원점 (0, 0) = SLAM을 시작한 위치. 빨간 축 = +x(0°), 초록 축 = +y(90°)
  {
    const [cx, cy] = worldToCanvas(0, 0);
    const len = Math.max(15, 0.3 * pxPerM) / pxPerM;   // m
    const [xx, xy] = worldToCanvas(len, 0), [yx, yy] = worldToCanvas(0, len);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#ef4444';
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(xx, xy); ctx.stroke();
    ctx.strokeStyle = '#22c55e';
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(yx, yy); ctx.stroke();
    ctx.fillStyle = '#111';
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText('원점(0,0)', cx + 4, cy + 14);
  }

  // 금지 구역 (A* 가 피함): 사각형 = 빗금, 선 = 굵은 점선 (가상 벽)
  for (const z of S.zones) drawZone(z, z.shape, false);
  if (S.drag && S.drag.kind === 'zone') {
    const d = S.drag;
    drawZone({ ...canvasPair(d) }, S.mode === 'wall' ? 'line' : 'rect', true);
  }

  // A* 가 기억 중인 장애물: 맵에 없는 LiDAR 장애물 칸(빨간 점, 20초) + 부딪힌 자리(✕, 120초)
  if (S.memory) {
    const cell = Math.max(2, 0.05 * pxPerM);
    ctx.fillStyle = 'rgba(220, 38, 38, 0.55)';
    for (const [x, y] of S.memory.remembered || []) {
      const [cx, cy] = worldToCanvas(x, y);
      ctx.fillRect(cx - cell / 2, cy - cell / 2, cell, cell);
    }
    ctx.strokeStyle = '#b91c1c';
    ctx.lineWidth = 3;
    const k = Math.max(6, 0.12 * pxPerM);
    for (const [x, y] of S.memory.bumps || []) {
      const [cx, cy] = worldToCanvas(x, y);
      ctx.beginPath();
      ctx.moveTo(cx - k, cy - k); ctx.lineTo(cx + k, cy + k);
      ctx.moveTo(cx + k, cy - k); ctx.lineTo(cx - k, cy + k);
      ctx.stroke();
    }
  }

  // 추적 중인 장애물 (LiDAR 군집 + 칼만필터): 주황 원 + 1초 뒤 위치까지 화살표
  if (S.obstacles && S.obstacles.length && Date.now() - (S.obstaclesAt || 0) < 2000) {
    for (const o of S.obstacles) {
      const [cx, cy] = worldToCanvas(o.x, o.y);
      // 움직이는 물체: 주황 + 1초 뒤 위치 화살표, 지도에 없는 멈춘 물체: 회색
      const moving = o.moving === undefined ? Math.hypot(o.vx, o.vy) > 0.08 : o.moving;
      ctx.beginPath();
      ctx.arc(cx, cy, Math.max(4, o.r * pxPerM), 0, 2 * Math.PI);
      ctx.fillStyle = moving ? 'rgba(249, 115, 22, 0.35)' : 'rgba(100, 116, 139, 0.35)';
      ctx.fill();
      ctx.strokeStyle = moving ? '#ea580c' : '#475569';
      ctx.lineWidth = 2;
      ctx.stroke();
      if (moving) {
        const [ex, ey] = worldToCanvas(o.x + o.vx, o.y + o.vy);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
        ctx.beginPath(); ctx.arc(ex, ey, 3, 0, 2 * Math.PI); ctx.fillStyle = '#ea580c'; ctx.fill();
      }
    }
  }

  // 경로
  if (S.path.length > 1) {
    ctx.strokeStyle = '#16a34a';
    ctx.lineWidth = 3;
    ctx.beginPath();
    S.path.forEach(([x, y], i) => {
      const [cx, cy] = worldToCanvas(x, y);
      i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy);
    });
    ctx.stroke();
  }

  // 위치
  const current = currentStep();
  for (const loc of S.locations) {
    const [cx, cy] = worldToCanvas(loc.x, loc.y);
    const isHome = loc.kind === 'home';
    const isTarget = current && current.x === loc.x && current.y === loc.y;
    const color = isHome ? '#7c3aed' : S.selected.has(loc.id) ? '#d97706' : '#2563eb';
    drawArrow(cx, cy, loc.yaw, Math.max(8, 0.25 * pxPerM), color, isTarget);
    ctx.fillStyle = '#111';
    ctx.font = '12px system-ui, sans-serif';
    ctx.fillText((isHome ? '⌂ ' : '') + loc.name, cx + 9, cy - 9);
  }

  // 등록된 위치가 아닌 목표(여기로 이동)
  if (current && !S.locations.some((l) => l.x === current.x && l.y === current.y)) {
    const [cx, cy] = worldToCanvas(current.x, current.y);
    drawArrow(cx, cy, current.yaw, Math.max(8, 0.25 * pxPerM), '#ea580c', true);
  }

  // 로봇 (Waffle Pi 반경 약 0.22m)
  if (S.pose) {
    const [cx, cy] = worldToCanvas(S.pose.x, S.pose.y);
    const r = Math.max(6, 0.22 * pxPerM);
    ctx.fillStyle = 'rgba(220, 38, 38, 0.25)';
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(S.pose.yaw + S.rot) * r * 1.6, cy - Math.sin(S.pose.yaw + S.rot) * r * 1.6);
    ctx.stroke();
  }

  // 드래그 중인 방향 표시
  if (S.drag && S.drag.kind === 'pose') {
    const { sx, sy, cx, cy } = S.drag;
    ctx.strokeStyle = S.mode === 'initpose' ? '#dc2626' : '#2563eb';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(sx, sy, 6, 0, Math.PI * 2);
    ctx.moveTo(sx, sy);
    ctx.lineTo(cx, cy);
    ctx.stroke();
  }
}

function canvasPair(d) {
  const [x1, y1] = canvasToWorld(d.sx, d.sy);
  const [x2, y2] = canvasToWorld(d.cx, d.cy);
  return { x1, y1, x2, y2 };
}

function drawZone(z, shape, preview) {
  const [ax, ay] = worldToCanvas(z.x1, z.y1);
  const [bx, by] = worldToCanvas(z.x2, z.y2);
  ctx.save();
  ctx.strokeStyle = '#b91c1c';
  ctx.setLineDash(preview ? [4, 3] : []);
  if (shape === 'line') {
    const pxPerM = S.view.scale / S.map.resolution;
    ctx.lineWidth = Math.max(3, 0.1 * pxPerM);
    ctx.lineCap = 'round';
    ctx.setLineDash([Math.max(4, 0.15 * pxPerM), Math.max(3, 0.08 * pxPerM)]);
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  } else {
    // 구역은 맵 축 기준 사각형 → 돌린 화면에서는 네 모서리를 이어서 그림
    const corners = [[z.x1, z.y1], [z.x2, z.y1], [z.x2, z.y2], [z.x1, z.y2]];
    const cs = corners.map(([x, y]) => worldToCanvas(x, y));
    const x = Math.min(...cs.map((p) => p[0])), y = Math.min(...cs.map((p) => p[1]));
    const w = Math.max(...cs.map((p) => p[0])) - x, h = Math.max(...cs.map((p) => p[1])) - y;
    ctx.fillStyle = 'rgba(220, 38, 38, 0.13)';
    worldPolygon(corners);
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.stroke();
    // 빗금
    worldPolygon(corners);
    ctx.clip();
    ctx.beginPath();
    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(185, 28, 28, 0.45)';
    for (let k = -h; k < w; k += 10) { ctx.moveTo(x + k, y + h); ctx.lineTo(x + k + h, y); }
    ctx.stroke();
  }
  ctx.restore();
  if (!preview && z.name) {
    ctx.fillStyle = '#991b1b';
    ctx.font = '11px system-ui, sans-serif';
    const [lx, ly] = worldToCanvas(Math.min(z.x1, z.x2), Math.max(z.y1, z.y2));
    ctx.fillText(z.name, lx + 3, ly - 3);
  }
}

// 가상 맵: 1m 격자
function drawGrid() {
  const m = S.map;
  const [x0w, y0w] = m.origin, x1w = x0w + VIRTUAL_SIZE_M, y1w = y0w + VIRTUAL_SIZE_M;
  ctx.fillStyle = '#e5e7eb';
  worldPolygon([[x0w, y0w], [x1w, y0w], [x1w, y1w], [x0w, y1w]]);
  ctx.fill();
  ctx.strokeStyle = '#cbd5e1';
  ctx.lineWidth = 1;
  for (let x = Math.ceil(x0w); x <= x1w; x++) {
    const [a, b] = worldToCanvas(x, y0w), [c, d] = worldToCanvas(x, y1w);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
  }
  for (let y = Math.ceil(y0w); y <= y1w; y++) {
    const [a, b] = worldToCanvas(x0w, y), [c, d] = worldToCanvas(x1w, y);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
  }
}

function drawArrow(cx, cy, yaw, len, color, highlight) {
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  ctx.lineWidth = highlight ? 4 : 2;
  ctx.beginPath();
  ctx.arc(cx, cy, highlight ? 7 : 5, 0, Math.PI * 2);
  ctx.fill();
  const ex = cx + Math.cos(yaw + S.rot) * len, ey = cy - Math.sin(yaw + S.rot) * len;   // yaw: 월드 방향
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
}

// ------------------------------------------------------------------ 마우스 / 터치
function eventPos(ev) {
  const r = canvas.getBoundingClientRect();
  return [ev.clientX - r.left, ev.clientY - r.top];
}

canvas.addEventListener('pointerdown', (ev) => {
  if (!S.map) return;
  canvas.setPointerCapture(ev.pointerId);
  if (S.mode === 'view') return;  // 보기 모드에서는 지도 고정
  const [x, y] = eventPos(ev);
  const kind = S.mode === 'zone' || S.mode === 'wall' ? 'zone' : 'pose';
  S.drag = { kind, sx: x, sy: y, cx: x, cy: y };
});

canvas.addEventListener('pointermove', (ev) => {
  if (!S.map) return;
  const [x, y] = eventPos(ev);
  const [wx, wy] = canvasToWorld(x, y);
  $('#cursor-pos').textContent = `x: ${wx.toFixed(2)} m, y: ${wy.toFixed(2)} m`;
  if (!S.drag) return;
  S.drag.cx = x;
  S.drag.cy = y;
  draw();
});

canvas.addEventListener('pointerup', () => {
  const d = S.drag;
  S.drag = null;
  if (d && d.kind === 'zone') {
    // 끌어서 그린 사각형/선을 금지 구역으로 저장 (모드는 그대로 두어 이어서 그릴 수 있게)
    if (Math.hypot(d.cx - d.sx, d.cy - d.sy) > 5) {
      send('save_zone', { zone: { shape: S.mode === 'wall' ? 'line' : 'rect', ...canvasPair(d) } });
      notifyExploreZones();
    }
    draw();
    return;
  }
  if (!d || d.kind !== 'pose') { draw(); return; }
  const [wx, wy] = canvasToWorld(d.sx, d.sy);
  const dx = d.cx - d.sx, dy = d.cy - d.sy;
  // 드래그 없이 클릭만 하면: 이동은 로봇에서 목표를 향하는 방향, 나머지는 0°(맵의 +x 방향)
  let yaw = 0;
  if (Math.hypot(dx, dy) > 5) yaw = Math.atan2(-dy, dx) - S.rot;   // 화면 방향 → 월드 방향
  else if (S.mode === 'goto' && S.pose) yaw = Math.atan2(wy - S.pose.y, wx - S.pose.x);
  // 마우스가 두 번 눌리는 경우 같은 명령이 연달아 가지 않도록 무시
  const now = Date.now();
  if (S.mode !== 'add' && now - (S.lastCommandAt || 0) < 800) { draw(); return; }
  if (S.mode === 'initpose') {
    S.lastCommandAt = now;
    send('set_initial_pose', { x: wx, y: wy, yaw });
    setMode('view');
  } else if (S.mode === 'goto') {
    S.lastCommandAt = now;
    send('goto_point', { x: wx, y: wy, yaw });
    setMode('view');
  } else if (S.mode === 'deliver' && S.deliveryLoc) {
    S.lastCommandAt = now;
    send('start_delivery', { location_id: S.deliveryLoc.id, x: wx, y: wy, yaw });
    setMode('view');
  } else if (S.mode === 'add') {
    openLocationForm({ x: wx, y: wy, yaw });
  }
  draw();
});

const HINTS = {
  view: '아래 버튼으로 모드를 고르세요',
  goto: '갈 지점을 클릭하세요 (드래그하면 도착 후 바라볼 방향 지정). A* 경로로 이동합니다',
  add: '위치를 클릭하고, 바라볼 방향으로 드래그하세요',
  initpose: '로봇의 실제 위치를 클릭하고, 로봇이 바라보는 방향으로 드래그하세요',
  deliver: 'LED 를 받을 자리를 클릭하세요 (드래그하면 도착 후 바라볼 방향)',
  zone: '로봇이 들어가면 안 되는 곳(책상 밑, 좌석 줄)을 끌어서 사각형으로 그리세요. 오른쪽 "금지 구역" 목록에서 삭제',
  wall: '지나가면 안 되는 선(가상 벽)을 끌어서 그리세요. 오른쪽 "금지 구역" 목록에서 삭제',
};

function setMode(mode) {
  S.mode = mode;
  document.querySelectorAll('button.mode').forEach((b) => b.classList.toggle('active', b.dataset.mode === mode));
  $('#mode-hint').textContent = mode === 'deliver' && S.deliveryLoc
    ? `${S.deliveryLoc.cargo} — ${HINTS.deliver}` : HINTS[mode];
  if (mode !== 'deliver') S.deliveryLoc = null;
  renderDeliveryButtons();
}

document.querySelectorAll('button.mode').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));

// ------------------------------------------------------------------ 패널
const STATE_LABEL = {
  OFFLINE: '오프라인', IDLE: '대기', MOVING: '이동 중', WAITING_LOAD: '적재 대기', WAITING_UNLOAD: '수령 대기', ERROR: '오류',
};

// ------------------------------------------------------------------ LED 요청 (배달)
// 인식 라벨이 색 이름이면 그 색으로 칩을 칠한다
const CARGO_COLORS = { red: '#ef4444', green: '#22c55e', blue: '#3b82f6', yellow: '#eab308', white: '#e5e7eb',
                       빨강: '#ef4444', 초록: '#22c55e', 파랑: '#3b82f6', 노랑: '#eab308' };
function cargoColor(label) { return CARGO_COLORS[(label || '').toLowerCase()] || '#64748b'; }

function renderDeliveryButtons() {
  const box = $('#delivery-buttons');
  if (!box) return;
  box.innerHTML = '';
  const busy = !!(S.task && S.task.status === 'running');
  const online = S.robot && S.robot.state !== 'OFFLINE';
  const items = (S.locations || []).filter((l) => l.kind !== 'home');
  if (!items.length) { box.textContent = '등록된 물류 위치가 없습니다.'; return; }
  for (const loc of items) {
    const b = document.createElement('button');
    b.className = 'cargo-btn' + (S.deliveryLoc && S.deliveryLoc.id === loc.id ? ' active' : '');
    b.innerHTML = `<i style="background:${cargoColor(loc.cargo)}"></i>${loc.cargo}` +
                  (loc.cargo !== loc.name ? ` <small>(${loc.name})</small>` : '');
    b.disabled = busy || !online;
    b.title = busy ? '다른 요청을 처리 중입니다' : `${loc.cargo} 요청: 지도에서 받을 자리를 클릭`;
    b.addEventListener('click', () => {
      if (S.cargo) { toast(`로봇에 이미 ${S.cargo} 이(가) 실려 있습니다. 먼저 꺼내 주세요.`); return; }
      S.deliveryLoc = loc;
      setMode('deliver');
    });
    box.appendChild(b);
  }
}

function renderCargo() {
  const el = $('#cargo-status');
  if (!el) return;
  el.textContent = S.cargo === undefined || S.cargo === null ? '카메라 인식: 인식 노드 없음'
    : `카메라 인식: ${S.cargo || '비어 있음'}`;
  el.className = 'cargo-status' + (S.cargo ? ' loaded' : '');
}
const STEP_LABEL = {
  pending: '대기', moving: '이동 중', arrived: '도착(적재 대기)', done: '완료', failed: '실패', skipped: '건너뜀',
};

function currentStep() {
  if (!S.task || S.task.status !== 'running') return null;
  return S.task.steps.find((s) => s.status !== 'done' && s.status !== 'skipped') || null;
}

function renderPanel() {
  const r = S.robot;
  const state = r ? r.state : 'OFFLINE';
  $('#robot-name').textContent = r ? r.name : '';
  const badge = $('#robot-state');
  badge.textContent = STATE_LABEL[state] || state;
  badge.className = 'badge ' + state;
  $('#robot-message').textContent = r ? r.message : '';
  const bat = $('#battery');
  bat.textContent = r && r.battery_voltage != null && state !== 'OFFLINE'
    ? `배터리 ${r.battery_voltage.toFixed(1)} V` + (r.battery_percentage != null ? ` (${Math.round(r.battery_percentage)}%)` : '')
    : '';
  bat.className = r && r.battery_percentage != null && r.battery_percentage < 20 ? 'low' : '';
  bat.title = bat.className ? '배터리가 부족합니다. 충전하세요.' : '';
  // 배터리 막대: 값이 없으면(시뮬레이션 등) 줄 자체를 숨김
  const pct = r && r.battery_percentage != null && state !== 'OFFLINE' ? r.battery_percentage : null;
  $('#battery-row').classList.toggle('empty', pct == null && !bat.textContent);
  const fill = $('#battery-fill');
  fill.style.width = `${Math.max(0, Math.min(100, pct || 0))}%`;
  fill.className = pct == null ? '' : pct < 20 ? 'low' : pct < 50 ? 'mid' : '';

  const online = state !== 'OFFLINE';
  const running = S.task && S.task.status === 'running';
  const step = currentStep();
  const unload = step && step.action === 'unload';
  $('#btn-confirm').textContent = unload ? '✓ 수령 완료' : '✓ 적재 완료';
  $('#btn-confirm').disabled = !(step && step.status === 'arrived');
  renderDeliveryButtons();
  $('#btn-retry').disabled = !(online && step && (step.status === 'failed' || state === 'ERROR'));
  $('#btn-skip').disabled = !(online && step);
  $('#btn-cancel').disabled = !running;
  const hasHome = S.locations.some((l) => l.kind === 'home');
  $('#btn-home').disabled = !online || !hasHome;
  $('#btn-home').title = hasHome ? '' : '홈 위치가 없습니다. 로봇을 홈에 두고 "현재 위치를 홈으로"를 누르세요.';
  $('#btn-set-home').disabled = !S.pose || running;
  $('#btn-reset').disabled = !(state === 'ERROR' && !running);
  $('#btn-start').disabled = !(online && !running);

  const ol = $('#task-steps');
  ol.innerHTML = '';
  if (!S.task) {
    ol.innerHTML = '<li class="muted">없음</li>';
  } else {
    for (const s of S.task.steps) {
      const li = document.createElement('li');
      li.className = s.status;
      let eta = '';
      if (s.status === 'moving' && s.predicted_sec != null && s.started_at) {
        const left = s.predicted_sec - (Date.now() - new Date(s.started_at).getTime()) / 1000;
        eta = left > 0 ? ` · 약 ${fmtSec(left)} 남음` : ' · 예상보다 지연';
      }
      li.textContent = `${s.name} - ${STEP_LABEL[s.status] || s.status}${eta}`;
      ol.appendChild(li);
    }
    if (S.task.status !== 'running') {
      const li = document.createElement('li');
      li.className = 'muted';
      li.style.listStyle = 'none';
      li.textContent = `(작업 #${S.task.id} ${{ done: '완료', canceled: '취소됨', failed: '실패' }[S.task.status] || S.task.status})`;
      ol.appendChild(li);
    }
  }
  renderLocations();
  draw();
}

function renderLocations() {
  const ul = $('#locations');
  ul.innerHTML = '';
  if (!S.locations.length) {
    ul.innerHTML = '<li class="muted">"위치 추가" 모드에서 맵을 클릭해 등록하세요</li>';
    return;
  }
  const online = S.robot && S.robot.state !== 'OFFLINE';
  const running = S.task && S.task.status === 'running';
  for (const loc of S.locations) {
    const li = document.createElement('li');
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = S.selected.has(loc.id);
    cb.disabled = loc.kind === 'home';
    cb.addEventListener('change', () => {
      cb.checked ? S.selected.add(loc.id) : S.selected.delete(loc.id);
      draw();
    });
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = (loc.kind === 'home' ? '⌂ ' : '') + loc.name;
    name.title = '클릭해서 수정';
    name.addEventListener('click', () => openLocationForm(loc));
    const coord = document.createElement('span');
    coord.className = 'coord';
    coord.textContent = `${loc.x.toFixed(2)}, ${loc.y.toFixed(2)}`;
    const go = document.createElement('button');
    go.className = 'small';
    go.textContent = '이동';
    go.disabled = !online || running;
    go.addEventListener('click', () => send('goto_location', { location_id: loc.id }));
    const del = document.createElement('button');
    del.className = 'small';
    del.textContent = '삭제';
    del.addEventListener('click', () => {
      if (confirm(`"${loc.name}" 위치를 삭제할까요?`)) send('delete_location', { location_id: loc.id });
    });
    li.append(cb, name, coord, go, del);
    ul.appendChild(li);
  }
}

// 자동 탐색 중에 금지 구역을 바꾸면 탐색기(tools/explore.py)가 받아서 목표를 다시 고른다
function notifyExploreZones() {
  if (S.runnerBusy === '자동 탐색') toast('자동 탐색 중: 바뀐 금지 구역을 반영해 탐색 목표를 다시 고릅니다 (구역 안/뒤로는 가지 않음)');
}

function renderZones() {
  const ul = $('#zones');
  if (!ul) return;
  ul.innerHTML = '';
  if (!S.zones.length) {
    ul.innerHTML = '<li class="muted">"금지 구역"/"가상 벽" 모드에서 지도를 끌어서 그리세요</li>';
    return;
  }
  for (const z of S.zones) {
    const li = document.createElement('li');
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = (z.shape === 'line' ? '┃ ' : '▨ ') + z.name;
    const coord = document.createElement('span');
    coord.className = 'coord';
    coord.textContent = `${z.x1.toFixed(1)}, ${z.y1.toFixed(1)} ~ ${z.x2.toFixed(1)}, ${z.y2.toFixed(1)}`;
    const del = document.createElement('button');
    del.className = 'small';
    del.textContent = '삭제';
    del.addEventListener('click', () => {
      if (confirm(`"${z.name}" 을(를) 삭제할까요?`)) {
        send('delete_zone', { zone_id: z.id });
        notifyExploreZones();
      }
    });
    li.append(name, coord, del);
    ul.appendChild(li);
  }
}

function addLog(entry) {
  const li = document.createElement('li');
  const t = document.createElement('time');
  t.textContent = new Date(entry.created_at).toLocaleTimeString('ko-KR', { hour12: false });
  li.append(t, entry.message);
  const ul = $('#logs');
  ul.prepend(li);
  while (ul.children.length > 100) ul.lastChild.remove();
}

let toastTimer = null;
function toast(text) {
  const el = $('#toast');
  el.textContent = text;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, 3500);
}

// ------------------------------------------------------------------ 위치 폼
const form = $('#loc-form');

function openLocationForm(loc) {
  $('#loc-form-card').hidden = false;
  $('#loc-form-title').textContent = loc.id ? '위치 수정' : '위치 추가';
  form.elements.id.value = loc.id || '';
  form.elements.name.value = loc.name || '';
  form.elements.kind.value = loc.kind || 'pickup';
  form.elements.x.value = loc.x.toFixed(2);
  form.elements.y.value = loc.y.toFixed(2);
  form.elements.yaw_deg.value = Math.round((loc.yaw || 0) * 180 / Math.PI);
  form.elements.cargo.value = loc.cargo && loc.cargo !== loc.name ? loc.cargo : '';
  form.elements.name.focus();
}

form.addEventListener('submit', (ev) => {
  ev.preventDefault();
  const f = form.elements;
  send('save_location', {
    location: {
      id: f.id.value ? Number(f.id.value) : null,
      name: f.name.value,
      kind: f.kind.value,
      x: Number(f.x.value),
      y: Number(f.y.value),
      yaw: Number(f.yaw_deg.value || 0) * Math.PI / 180,
      cargo: f.cargo.value,
    },
  });
  $('#loc-form-card').hidden = true;
});
$('#loc-form-cancel').addEventListener('click', () => { $('#loc-form-card').hidden = true; });

// ------------------------------------------------------------------ 버튼
$('#btn-confirm').addEventListener('click', () => {
  const step = currentStep();
  send(step && step.action === 'unload' ? 'confirm_unload' : 'confirm_load');
});
$('#btn-retry').addEventListener('click', () => send('retry_step'));
$('#btn-skip').addEventListener('click', () => send('skip_step'));
$('#btn-cancel').addEventListener('click', () => send('cancel_task'));
$('#btn-home').addEventListener('click', () => send('go_home'));
$('#btn-save-map').addEventListener('click', () => {
  const name = prompt('저장할 맵 이름 (영문/숫자/_/-, 예: office_1f)');
  if (!name) return;
  if (!/^[A-Za-z0-9_-]{1,50}$/.test(name)) { toast('맵 이름은 영문/숫자/_/- 만 쓸 수 있습니다.'); return; }
  send('save_map', { name });
  toast('맵 저장 중... (10초 정도 걸립니다)');
});
$('#btn-set-home').addEventListener('click', () => {
  if (confirm('로봇의 현재 위치를 홈으로 저장할까요?')) send('set_home_here');
});
$('#btn-reset').addEventListener('click', () => send('reset_error'));
$('#btn-estop').addEventListener('click', () => send('estop'));
$('#btn-reset-robot').addEventListener('click', () => {
  if (confirm('작업을 취소하고 로봇 상태를 초기화할까요?\n(시뮬레이션이면 로봇이 출발 위치로 돌아갑니다)')) send('reset_robot');
});
// 선택한 위치 id (목록 순서). 같은 위치들로 "순서 최적화" 를 했으면 그 순서
function selectedIds() {
  const ids = S.locations.filter((l) => S.selected.has(l.id)).map((l) => l.id);
  const p = S.plan;
  if (p && p.withHome === $('#with-home').checked && p.order.length === ids.length
      && p.order.every((id) => S.selected.has(id))) return p.order;
  return ids;
}

function fmtSec(sec) {
  if (sec == null) return '-';
  sec = Math.round(sec);
  return sec >= 60 ? `${Math.floor(sec / 60)}분 ${sec % 60}초` : `${sec}초`;
}

function clearPlan() {
  S.plan = null;
  $('#plan-result').hidden = true;
}

$('#btn-start').addEventListener('click', () => {
  const ids = selectedIds();
  if (!ids.length) { toast('이동할 위치를 선택하세요.'); return; }
  send('start_task', { location_ids: ids, with_home: $('#with-home').checked });
  S.selected.clear();
  clearPlan();
});

$('#btn-plan').addEventListener('click', () => {
  const ids = S.locations.filter((l) => S.selected.has(l.id)).map((l) => l.id);
  if (!ids.length) { toast('최적화할 위치를 선택하세요.'); return; }
  send('plan_order', { location_ids: ids, with_home: $('#with-home').checked });
});
$('#with-home').addEventListener('change', clearPlan);

function showPlan(msg) {
  S.plan = { order: msg.order, withHome: $('#with-home').checked };
  const saved = msg.original_total != null ? msg.original_total - msg.total : null;
  $('#plan-route').textContent = msg.names.join(' → ');
  $('#plan-time').textContent = `예상 ${fmtSec(msg.total)}`
    + (saved != null && saved > 0.5 ? ` · 선택 순서(${fmtSec(msg.original_total)})보다 ${fmtSec(saved)} 단축` : ' · 선택 순서가 이미 최적');
  $('#plan-legs').textContent = msg.legs.map((l) => `${l.length}m ${fmtSec(l.sec)}`).join(' / ');
  $('#plan-result').hidden = false;
  showModel(msg.model);
}

function showModel(m) {
  $('#travel-model').textContent = m.learned
    ? `이동 시간 모델: ${m.summary} · 기록 ${m.n}개${m.outliers ? ` (이상치 ${m.outliers}개 제외)` : ''}, 평균 오차 ${m.mae.toFixed(1)}초`
    : `이동 시간 모델: 기본값 (${m.summary}) · 학습할 이동 기록이 아직 부족합니다`;
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();
renderPanel();
renderHealth();
connect();

// ------------------------------------------------------------------ 수동 조작 (teleop)
// 누르고 있는 동안 0.1초마다 {v, w} 를 보낸다. 브리지는 0.4초 넘게 안 오면 멈춘다 (손을 떼거나 창을 벗어나면 정지)
const TELEOP = { v: 0, w: 0, timer: null };
function teleopSend() { send('teleop', { v: TELEOP.v, w: TELEOP.w }); }
function teleopStart(dv, dw) {
  const speed = Number($('#teleop-speed').value);
  TELEOP.v = dv >= 0 ? dv * speed : dv * Math.min(speed, 0.1);   // 뒤로는 최대 0.1 m/s
  TELEOP.w = dw * (speed <= 0.08 ? 0.6 : 1.0);
  $('#teleop-status').textContent = `이동 중: ${TELEOP.v.toFixed(2)} m/s, ${TELEOP.w.toFixed(1)} rad/s`;
  if (!TELEOP.timer) { teleopSend(); TELEOP.timer = setInterval(teleopSend, 100); }
}
function teleopStop() {
  if (TELEOP.timer) { clearInterval(TELEOP.timer); TELEOP.timer = null; }
  TELEOP.v = 0; TELEOP.w = 0;
  send('teleop', { v: 0, w: 0 });
  $('#teleop-status').textContent = '';
}
document.querySelectorAll('.teleop-pad button[data-tv]').forEach((b) => {
  const go = (ev) => { ev.preventDefault(); teleopStart(Number(b.dataset.tv), Number(b.dataset.tw)); };
  b.addEventListener('pointerdown', go);
  b.addEventListener('pointerup', teleopStop);
  b.addEventListener('pointerleave', () => { if (TELEOP.timer) teleopStop(); });
  b.addEventListener('pointercancel', teleopStop);
  b.addEventListener('contextmenu', (ev) => ev.preventDefault());
});
$('#teleop-stop').addEventListener('click', teleopStop);
window.addEventListener('blur', () => { if (TELEOP.timer) teleopStop(); });

const TELEOP_KEYS = { ArrowUp: [1, 0], w: [1, 0], ArrowDown: [-1, 0], s: [-1, 0],
                      ArrowLeft: [0, 1], a: [0, 1], ArrowRight: [0, -1], d: [0, -1] };
const keysDown = new Set();
function teleopFromKeys() {
  let v = 0, w = 0;
  for (const k of keysDown) { v += TELEOP_KEYS[k][0]; w += TELEOP_KEYS[k][1]; }
  if (!keysDown.size) { teleopStop(); return; }
  teleopStart(Math.max(-1, Math.min(1, v)), Math.max(-1, Math.min(1, w)) * (v ? 0.6 : 1));
}
window.addEventListener('keydown', (ev) => {
  if (!$('#teleop-keys').checked || !(ev.key in TELEOP_KEYS) || ev.target.closest('input, select, textarea')) return;
  ev.preventDefault();
  if (!keysDown.has(ev.key)) { keysDown.add(ev.key); teleopFromKeys(); }
});
window.addEventListener('keyup', (ev) => {
  if (!(ev.key in TELEOP_KEYS) || !keysDown.has(ev.key)) return;
  keysDown.delete(ev.key);
  teleopFromKeys();
});
$('#teleop-keys').addEventListener('change', () => { keysDown.clear(); if (TELEOP.timer) teleopStop(); });
