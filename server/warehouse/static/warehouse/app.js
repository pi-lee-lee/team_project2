'use strict';

// ------------------------------------------------------------------ state
const S = {
  ws: null,
  robot: null,
  task: null,
  map: null,          // {resolution, origin:[x,y], width, height, image_url}
  mapImg: null,
  locations: [],
  zones: [],          // keep-out zones [{id, name, shape: rect|line, x1, y1, x2, y2}] - avoided by A*
  pose: null,         // {x, y, yaw}
  path: [],           // [[x, y], ...]
  mode: 'view',       // view | goto | add | initpose | zone | wall
  view: { scale: 1, ox: 0, oy: 0 },  // scale = canvas pixels per map cell, (ox, oy) = canvas position of rotated coords (0, 0)
  rot: Math.PI / 2,   // view rotation (rad, CCW): map x axis points up, y axis left (robot front = x shown upward, ROS convention)
  drag: null,
  robotName: initialRobotName(),   // robot controlled from this page (bridge name, e.g. tb3, tb3_1)
  robots: [],         // robot selection list [{name, state, message, bridge}]
};

// ?robot= from the URL, else the last selected robot, else the server default (tb3)
function initialRobotName() {
  const q = new URLSearchParams(location.search).get('robot');
  if (q) return q;
  try { return localStorage.getItem('robotName') || ''; } catch (e) { return ''; }
}

const $ = (sel) => document.querySelector(sel);
const canvas = $('#map');
const ctx = canvas.getContext('2d');

// ------------------------------------------------------------------ WebSocket
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const q = S.robotName ? `?robot=${encodeURIComponent(S.robotName)}` : '';
  const ws = new WebSocket(`${proto}://${location.host}/ws/ui/${q}`);
  S.ws = ws;
  ws.onopen = () => {
    setWsStatus(true);
  };
  ws.onclose = () => {
    setWsStatus(false);
    // If the server disconnects the robot state is unknown, so clear it from the view
    clearPose();
    S.health = { bridge: false };
    renderHealth();
    renderYolo();
    renderParams();
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
      S.robotName = msg.robot.name;
      S.robots = msg.robots || [];
      renderRobotSelect();
      S.scan = null;
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
      S.yolo = msg.yolo;
      renderYolo();
      S.params = msg.params;
      renderParams();
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
      updateRobotEntry(msg.robot);
      renderPanel();
      break;
    case 'robot_summary':   // state change of an unselected robot (list display only)
      updateRobotEntry(msg.robot);
      break;
    case 'robots':
      S.robots = msg.robots || [];
      renderRobotSelect();
      break;
    case 'map':
      setMap(msg.map);
      break;
    case 'yolo':
      // time the counts text last changed (small bridge age = new result)
      msg.countsAt = msg.counts && msg.age !== null && msg.age < 0.3 ? Date.now()
        : (S.yolo && msg.counts ? S.yolo.countsAt : null);
      S.yolo = msg;
      renderYolo();
      break;
    case 'zones':
      S.zones = msg.zones || [];
      renderZones();
      draw();
      break;
    case 'scan':
      S.scan = msg;
      S.scanAt = Date.now();
      draw();
      break;
    case 'obstacles':
      S.obstacles = msg.obstacles || [];
      S.obstaclesAt = Date.now();
      draw();
      break;
    case 'local_plan':
      S.localPlan = msg;
      S.localPlanAt = Date.now();
      draw();
      break;
    case 'memory':
      S.memory = msg;
      draw();
      break;
    case 'locations':
      S.locations = msg.locations;
      renderPanel();  // button state depends on whether home exists
      break;
    case 'pose':
      if (msg.lost) {
        clearPose();
        break;
      }
      S.poseAt = Date.now();
      // On a virtual map, re-center on the robot when it nears the edge (3 m+ from center)
      if (S.map && S.map.virtual) {
        const c = [S.map.origin[0] + VIRTUAL_SIZE_M / 2, S.map.origin[1] + VIRTUAL_SIZE_M / 2];
        if (!S.pose || Math.hypot(msg.x - c[0], msg.y - c[1]) > 3) {
          S.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
          useVirtualMap();
        }
      }
      if (!S.pose) {
        S.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
        renderPanel();  // enable "set current position as home" once the pose is first known
      }
      S.pose = { x: msg.x, y: msg.y, yaw: msg.yaw };
      draw();
      break;
    case 'health':
      S.health = msg;
      renderHealth();
      renderYolo();
      renderParams();
      break;
    case 'params':
      S.params = msg.items;
      renderParams();
      break;
    case 'param_result':
      paramResult(msg);
      break;
    case 'runner':
      if (msg.state === 'progress') {
        runProgress(msg);
        break;
      }
      if (msg.state === 'started') {
        S.runnerBusy = msg.job;
        if (runOverlayJob(msg.job)) runOpen(msg.job, 0);
        else toast(`${msg.job} 시작...`);
      } else {
        S.runnerBusy = null;
        if (msg.mode) S.runnerMode = msg.mode;
        if (S.run && S.run.job === msg.job) runDone(msg);
        else toast(`${msg.job} ${msg.ok ? '완료' : '실패'}: ${msg.message}`);
      }
      renderRunner();
      break;
    case 'order_preview':
      $('#order-preview').textContent = `순서: ${msg.names.join(' → ')} (사람 위치까지 약 ${fmtSec(msg.total)})`;
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

// When the robot disappears (bridge stopped, Pi off, pose TF lost), remove the robot and path from the view
// ------------------------------------------------------------------ robot selection (multiple robots)
const ROBOT_STATE_KO = { OFFLINE: '연결 안 됨', IDLE: '대기', MOVING: '이동 중', WAITING_LOAD: '적재 대기',
                         WAITING_UNLOAD: '수령 대기', ERROR: '오류' };
function updateRobotEntry(robot) {
  const r = S.robots.find((x) => x.name === robot.name);
  if (r) Object.assign(r, { state: robot.state, message: robot.message });
  else S.robots.push({ name: robot.name, state: robot.state, message: robot.message, bridge: false });
  renderRobotSelect();
}
// Show the selector in the top bar only with two or more robots (single robot: no selector)
function renderRobotSelect() {
  const sel = $('#robot-select');
  const names = S.robots.map((r) => r.name);
  if (S.robotName && !names.includes(S.robotName)) names.push(S.robotName);
  sel.hidden = names.length < 2;
  sel.replaceChildren(...names.sort().map((name) => {
    const r = S.robots.find((x) => x.name === name);
    const opt = document.createElement('option');
    opt.value = name;
    opt.textContent = r ? `${name} · ${ROBOT_STATE_KO[r.state] || r.state}` : name;
    return opt;
  }));
  sel.value = S.robotName;
}
$('#robot-select').addEventListener('change', (e) => {
  S.robotName = e.target.value;
  try { localStorage.setItem('robotName', S.robotName); } catch (err) { /* works without storage */ }
  // Clear the previous robot pose/path now; the server resends the full state (snapshot) of the new robot
  clearPose();
  S.scan = null;
  send('select_robot', { robot_name: S.robotName });
});

function clearPose() {
  const had = !!S.pose;
  S.pose = null;
  S.path = [];
  S.obstacles = [];
  S.localPlan = null;
  draw();
  if (had) renderPanel();
}

// Treat the robot as gone if no pose for over 2 s (incl. lost server connection)
setInterval(() => {
  if (S.pose && Date.now() - (S.poseAt || 0) > 2000) clearPose();
}, 500);

// while moving, refresh "about N s left"
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

// ------------------------------------------------------------------ run mode
const ROBOT_LABEL = { sim: '시뮬레이션', real: '실물' };

// ---------------------------------------------------------- run-mode switch waiting screen
// Build map / use map / stop all: the map and cards are unused for tens of seconds, so cover the page and show the steps and elapsed time.
// Auto exploration and auto localization are not covered, since one waits while watching the map
function runOverlayJob(job) {
  return job && job !== '자동 탐색' && job !== '위치 자동 찾기';
}
function runKey(job) {   // last duration (stored in the browser only): keyed by job kind, without world name etc.
  return 'runTime:' + job.replace(/\s*\(.*?\)/, '');
}
function runOpen(job, elapsed) {
  clearInterval(S.runTimer);
  S.run = { job, t0: performance.now() - elapsed * 1000, steps: [], hidden: false };
  let prev = null;
  try { prev = localStorage.getItem(runKey(job)); } catch (e) { /* no storage */ }
  $('#run-title').textContent = `${job} 준비 중`;
  $('#run-prev').textContent = prev ? `지난번 ${prev}초` : '';
  $('#run-steps').innerHTML = '';
  $('#run-line').textContent = elapsed ? '진행 중인 작업에 다시 연결했습니다' : '';
  $('#run-result').hidden = true;
  $('#run-spinner').className = 'spinner big';
  $('#run-hide').hidden = false;
  $('#run-close').hidden = true;
  $('#run-overlay').hidden = false;
  S.runTimer = setInterval(runTick, 100);
  runTick();
}
function runTick() {
  if (!S.run) return;
  const t = (performance.now() - S.run.t0) / 1000;
  $('#run-elapsed').textContent = t.toFixed(1);
  const cur = $('#run-steps li.cur .t');
  const last = S.run.steps[S.run.steps.length - 1];
  if (cur && last) cur.textContent = `${(t - last.at).toFixed(1)}초`;
}
function runProgress(msg) {
  if (!S.run || S.run.job !== msg.job) {
    if (!runOverlayJob(msg.job)) return;
    runOpen(msg.job, msg.elapsed || 0);   // job started from another page
  }
  if (S.run.hidden) return;
  $('#run-line').textContent = msg.line;
  if (!msg.step || msg.line.startsWith('== 완료')) return;
  if (msg.line.startsWith('==')) {   // first line of run.sh: robot / mode / domain summary
    $('#run-line').textContent = msg.line.replace(/^==\s*/, '');
    return;
  }
  const t = (performance.now() - S.run.t0) / 1000;
  const ol = $('#run-steps');
  const prev = ol.querySelector('li.cur');
  const last = S.run.steps[S.run.steps.length - 1];
  if (prev) {
    prev.classList.remove('cur');
    prev.querySelector('.run-ico').textContent = '✓';
    prev.querySelector('.t').textContent = `${(t - last.at).toFixed(1)}초`;
  }
  S.run.steps.push({ text: msg.line, at: t });
  const li = document.createElement('li');
  li.className = 'cur';
  const ico = document.createElement('span');
  ico.className = 'run-ico';
  ico.textContent = '…';
  const tt = document.createElement('span');
  tt.className = 't';
  li.append(ico, msg.line.replace(/^\d\)\s*/, ''), tt);
  ol.appendChild(li);
}
function runDone(msg) {
  clearInterval(S.runTimer);
  const t = msg.elapsed != null ? msg.elapsed : (performance.now() - S.run.t0) / 1000;
  $('#run-elapsed').textContent = t.toFixed(1);
  const cur = $('#run-steps li.cur');
  if (cur) {
    cur.classList.remove('cur');
    cur.querySelector('.run-ico').textContent = msg.ok ? '✓' : '✕';
    const last = S.run.steps[S.run.steps.length - 1];
    if (last) cur.querySelector('.t').textContent = `${(t - last.at).toFixed(1)}초`;
  }
  if (msg.ok) {
    try { localStorage.setItem(runKey(msg.job), t.toFixed(0)); } catch (e) { /* no storage */ }
  }
  $('#run-title').textContent = `${msg.job} ${msg.ok ? '완료' : '실패'}`;
  $('#run-spinner').className = 'spinner big ' + (msg.ok ? 'done' : 'fail');
  const res = $('#run-result');
  res.textContent = `${msg.ok ? '✓' : '✕'} ${t.toFixed(1)}초 · ${msg.message || ''}`;
  res.className = 'run-result' + (msg.ok ? '' : ' fail');
  res.hidden = false;
  $('#run-line').textContent = '';
  $('#run-hide').hidden = true;
  $('#run-close').hidden = false;
  const job = S.run.job;
  S.run.finished = true;
  if (S.run.hidden) toast(`${job} ${msg.ok ? '완료' : '실패'} (${t.toFixed(1)}초): ${msg.message}`);
  if (msg.ok) setTimeout(() => { if (S.run && S.run.finished && S.run.job === job) runClose(); }, 3000);
}
function runClose() {
  clearInterval(S.runTimer);
  S.run = null;
  $('#run-overlay').hidden = true;
}
$('#run-close').addEventListener('click', runClose);
$('#run-hide').addEventListener('click', () => {
  // Job keeps running, only the overlay closes; a notice is shown when it ends
  S.run.hidden = true;
  $('#run-overlay').hidden = true;
});

function setRunner(info) {
  S.runnerMode = info.mode || {};
  S.runnerBusy = info.busy || null;
  // If a switch is in progress when the page opens, resume the waiting screen (incl. elapsed time)
  if (info.busy && runOverlayJob(info.busy) && !S.run) runOpen(info.busy, info.elapsed || 0);
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
  // preselect the map in use
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
  // preselect the running world
  ws.value = keepWorld || S.runnerMode.world || 'turtlebot3_world';
  renderRunner();
}

// world the selected map was built in (recorded only for maps saved in simulation)
function selectedMapWorld() {
  const path = $('#runner-map').value;
  if (!path) return 'turtlebot3_world';  // default sim map = TurtleBot3 default world map
  const m = (S.runnerMaps || []).find((x) => x.path === path);
  return m ? m.world : '';
}

// Selecting a map selects the world it was built in (AMCL in another world would not match)
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
    ? `현재: ${ROBOT_LABEL[m.robot] || m.robot || '?'}${world} / ${m.mode === 'slam' ? '맵 만들기 (SLAM)' : `맵 사용 (AMCL) · ${(m.map || '').split('/').pop() || '-'}${(m.amcl || '').includes('amcl_hall') ? ' · 강당 설정' : ''}`}`
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
  const what = mode === 'slam' ? '맵 만들기 (SLAM)' : `맵 사용 (AMCL) · ${map || '기본 맵'}`;
  if (!confirm(`${ROBOT_LABEL[robot]}${world ? ` (${world})` : ''} / ${what} 으로 다시 시작할까요?\n(지금 실행 중인 노드는 정지됩니다)`)) return;
  send('runner_start', {
    robot, mode, map, world, amcl: $('#runner-amcl-hall').checked ? 'hall' : 'default',
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
  if (confirm('맵 전체에서 로봇 위치를 찾아 초기 위치로 지정할까요? (1~3분)\n'
              + '로봇이 제자리에서 한 바퀴 돌고, 위치가 애매하면 트인 쪽으로 최대 0.8m 씩 2번까지 움직입니다.\n'
              + '주변 0.3m 안에 물체가 있으면 돌지 않고, 앞이 0.35m 안으로 막히면 멈춥니다.')) send('auto_localize');
});

function setWsStatus(on) {
  const el = $('#ws-status');
  el.textContent = on ? '서버 연결됨' : '서버 연결 안 됨';
  el.className = 'badge ' + (on ? 'on' : 'off');
}

// ------------------------------------------------------------------ coordinate transform
// world (m) -> coords rotated by S.rot -> canvas pixels (canvas y points down). Only the view is rotated, not the coordinates
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

// ------------------------------------------------------------------ map
// When there is no map yet (e.g. right after SLAM starts): a virtual map showing an 8 m grid around the robot.
// The coordinate transform works as with a real map, so robot/path display and "go here" clicks still work
const VIRTUAL_SIZE_M = 8;
function useVirtualMap() {
  const p = S.pose || { x: 0, y: 0 };
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
    // Switch the map only after the image is loaded (so a new origin is not drawn with the old image)
    S.map = map;
    const changed = !S.mapImg || S.mapName !== map.name;
    S.mapImg = img;
    S.mapName = map.name;
    const b = knownBounds(img, map);
    // View bounds are kept in world coordinates. A growing SLAM map changes its origin, so pixel bounds
    // would shift the whole view on every refresh and make the robot/locations jitter
    const w = S.wbox;
    const inside = w && b.x0 >= w.x0 && b.y0 >= w.y0 && b.x1 <= w.x1 && b.y1 <= w.y1;
    if (wasVirtual && inside) {
      fitView();  // keep the grid view from SLAM start (8 m around the robot)
    } else if (changed && !wasVirtual || !inside) {
      // while SLAM is growing the map, add 0.5 m slack so the view does not change often
      const g = changed && !wasVirtual ? 0 : 0.5;
      const u = changed || !w ? b : { x0: Math.min(b.x0, w.x0), y0: Math.min(b.y0, w.y0),
                                      x1: Math.max(b.x1, w.x1), y1: Math.max(b.y1, w.y1) };
      S.wbox = { x0: u.x0 - g, y0: u.y0 - g, x1: u.x1 + g, y1: u.y1 + g };
      fitView();
    } else {
      fitView();  // same bounds, only recompute the transform for the new origin (on-screen positions unchanged)
    }
  };
  img.src = map.image_url;
  renderLocations();
}

// bounding box of non-unknown (gray 205) cells = actual room extent (world coords, m)
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
  const m = 4;  // margin (cells)
  const r = map.resolution, [ox, oy] = map.origin;
  // image y points down, world y points up
  return { x0: ox + (x0 - m) * r, x1: ox + (x1 + 1 + m) * r,
           y0: oy + (img.height - (y1 + 1 + m)) * r, y1: oy + (img.height - (y0 - m)) * r };
}

// fix the view to the room extent (no zoom/pan)
function fitView() {
  if (!S.map) return;
  // fit the four corners of the world bounds (S.wbox) rotated by the view angle (S.rot)
  const m = S.map, r = m.resolution;
  const wb = S.wbox || { x0: m.origin[0], y0: m.origin[1],
                         x1: m.origin[0] + m.width * r, y1: m.origin[1] + m.height * r };
  const pts = [[wb.x0, wb.y0], [wb.x1, wb.y0], [wb.x0, wb.y1], [wb.x1, wb.y1]].map(([x, y]) => rotXY(x, y, S.rot));
  const u0 = Math.min(...pts.map((p) => p[0])), u1 = Math.max(...pts.map((p) => p[0]));
  const v0 = Math.min(...pts.map((p) => p[1])), v1 = Math.max(...pts.map((p) => p[1]));
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const k = Math.min(w / (u1 - u0), h / (v1 - v0));   // canvas pixels per meter
  S.view = { scale: k * r, ox: (w - (u1 - u0) * k) / 2 - u0 * k, oy: (h - (v1 - v0) * k) / 2 + v1 * k };
  draw();
}

// map image pixel (px, py) -> canvas: draw world (ox + px r, oy + (H - py) r) rotated
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

// fill/stroke a world-coordinate polygon (correct in the rotated view)
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

// ------------------------------------------------------------------ drawing
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

  // map origin (0, 0) = where SLAM started. Red axis = +x (0°), green axis = +y (90°)
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

  // keep-out zones (avoided by A*): rect = hatched, line = thick dashed line (virtual wall)
  for (const z of S.zones) drawZone(z, z.shape, false);
  if (S.drag && S.drag.kind === 'zone') {
    const d = S.drag;
    drawZone({ ...canvasPair(d) }, S.mode === 'wall' ? 'line' : 'rect', true);
  }

  // live LiDAR: connect neighboring beam points when close (walls as lines, isolated chair legs etc. as dots)
  if (S.scan && Date.now() - (S.scanAt || 0) < 1500) drawScan(S.scan, pxPerM);

  // obstacles remembered by A*: LiDAR obstacle cells not in the map (red dots, 20 s) + bump points (✕, 120 s)
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

  // tracked obstacles (LiDAR clusters + Kalman filter): orange circle + arrow to the position 1 s ahead
  if (S.obstacles && S.obstacles.length && Date.now() - (S.obstaclesAt || 0) < 2000) {
    for (const o of S.obstacles) {
      const [cx, cy] = worldToCanvas(o.x, o.y);
      // moving object: orange + arrow to position 1 s ahead; stationary object not in the map: gray
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

  // path
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

  // trajectory chosen by the Frenet local planner (next 4 s): thick magenta line + end point, with command speed/state. Hidden if not received for over 1 s
  // (magenta so it stands apart from the green A* path)
  if (S.localPlan && Date.now() - (S.localPlanAt || 0) < 1000) {
    const t = S.localPlan.traj || [];
    const color = '#c026d3';
    let lx, ly;
    if (t.length > 1) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 4;
      ctx.beginPath();
      t.forEach(([x, y], i) => {
        const [cx, cy] = worldToCanvas(x, y);
        i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy);
      });
      ctx.stroke();
      [lx, ly] = worldToCanvas(...t[t.length - 1]);
      ctx.beginPath(); ctx.arc(lx, ly, 4, 0, 2 * Math.PI); ctx.fillStyle = color; ctx.fill();
    } else if (S.pose) {
      [lx, ly] = worldToCanvas(S.pose.x, S.pose.y);
    }
    if (lx !== undefined) {
      const name = { driving: '주행', blocked: '막힘', rotating: '회전', done: '도착' }[S.localPlan.state] || S.localPlan.state;
      ctx.font = '11px system-ui, sans-serif';
      ctx.fillStyle = '#a21caf';
      ctx.fillText(`Frenet ${name} v=${(+S.localPlan.v).toFixed(2)} w=${(+S.localPlan.w).toFixed(2)}`, lx + 8, ly - 8);
    }
  }

  // locations
  const current = currentStep();
  for (const loc of S.locations) {
    const [cx, cy] = worldToCanvas(loc.x, loc.y);
    const isHome = loc.kind === 'home';
    const isTarget = current && current.x === loc.x && current.y === loc.y;
    const color = isHome ? '#7c3aed' : loc.kind === 'dropoff' ? '#db2777' : '#2563eb';
    drawArrow(cx, cy, loc.yaw, Math.max(8, 0.25 * pxPerM), color, isTarget);
    ctx.fillStyle = '#111';
    ctx.font = '12px system-ui, sans-serif';
    ctx.fillText(locPrefix(loc) + loc.name, cx + 9, cy - 9);
  }

  // goal that is not a registered location (go here)
  if (current && !S.locations.some((l) => l.x === current.x && l.y === current.y)) {
    const [cx, cy] = worldToCanvas(current.x, current.y);
    drawArrow(cx, cy, current.yaw, Math.max(8, 0.25 * pxPerM), '#ea580c', true);
  }

  // robot: actual outline rectangles (body + rear attachment, footprint from bridge health). Circle if unavailable
  if (S.pose) {
    const [cx, cy] = worldToCanvas(S.pose.x, S.pose.y);
    const r = Math.max(6, 0.22 * pxPerM);
    ctx.fillStyle = 'rgba(220, 38, 38, 0.25)';
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 2;
    const rects = (S.health && S.health.footprint) || null;
    if (rects && rects.length && 0.3 * pxPerM >= 8) {
      const a = S.pose.yaw + S.rot;
      const ca = Math.cos(a);
      const sa = Math.sin(a);
      for (const [x0, x1, y0, y1] of rects) {   // robot frame (x forward, y left) -> screen (y positive down)
        ctx.beginPath();
        [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].forEach(([px, py], i) => {
          const sx = cx + (px * ca - py * sa) * pxPerM;
          const sy = cy - (px * sa + py * ca) * pxPerM;
          if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
        });
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      }
    } else {   // when drawn too small / no footprint info
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(S.pose.yaw + S.rot) * r * 1.6, cy - Math.sin(S.pose.yaw + S.rot) * r * 1.6);
    ctx.stroke();
  }

  // heading being dragged
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

// max distance (m) to join neighboring beam points: base + multiple of the beam spacing at that range, since beams spread with distance (3 m: about 0.3 m)
// isolated objects like chair legs (0.4 m+ apart) are not joined
const SCAN_JOIN_BASE = 0.1, SCAN_JOIN_BEAMS = 2.5;

function drawScan(scan, pxPerM) {
  const pts = scan.points || [], n = pts.length;
  if (!n) return;
  const [ox, oy] = scan.origin || [0, 0];
  const inc = scan.inc || (2 * Math.PI / n);
  const joined = new Array(n).fill(false);
  ctx.save();
  ctx.strokeStyle = 'rgba(8, 145, 178, 0.9)';
  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.beginPath();
  for (let i = 0; i < n; i++) {
    const a = pts[i], b = pts[(i + 1) % n];
    if (!a || !b) continue;
    const r = Math.hypot(a[0] - ox, a[1] - oy);
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) > SCAN_JOIN_BASE + SCAN_JOIN_BEAMS * r * inc) continue;
    const [ax, ay] = worldToCanvas(a[0], a[1]), [bx, by] = worldToCanvas(b[0], b[1]);
    ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
    joined[i] = joined[(i + 1) % n] = true;
  }
  ctx.stroke();
  ctx.fillStyle = 'rgba(8, 145, 178, 0.9)';
  const dot = Math.max(1.5, 0.025 * pxPerM);
  for (let i = 0; i < n; i++) {
    if (!pts[i] || joined[i]) continue;
    const [cx, cy] = worldToCanvas(pts[i][0], pts[i][1]);
    ctx.beginPath(); ctx.arc(cx, cy, dot, 0, 2 * Math.PI); ctx.fill();
  }
  ctx.restore();
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
    // zones are map-axis rectangles -> draw by joining the four corners in the rotated view
    const corners = [[z.x1, z.y1], [z.x2, z.y1], [z.x2, z.y2], [z.x1, z.y2]];
    const cs = corners.map(([x, y]) => worldToCanvas(x, y));
    const x = Math.min(...cs.map((p) => p[0])), y = Math.min(...cs.map((p) => p[1]));
    const w = Math.max(...cs.map((p) => p[0])) - x, h = Math.max(...cs.map((p) => p[1])) - y;
    ctx.fillStyle = 'rgba(220, 38, 38, 0.13)';
    worldPolygon(corners);
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.stroke();
    // hatching
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

// virtual map: 1 m grid
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
  const ex = cx + Math.cos(yaw + S.rot) * len, ey = cy - Math.sin(yaw + S.rot) * len;   // yaw: world heading
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
}

// ------------------------------------------------------------------ mouse / touch
function eventPos(ev) {
  const r = canvas.getBoundingClientRect();
  return [ev.clientX - r.left, ev.clientY - r.top];
}

canvas.addEventListener('pointerdown', (ev) => {
  if (!S.map) return;
  canvas.setPointerCapture(ev.pointerId);
  if (S.mode === 'view') return;  // map is fixed in view mode
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
    // save the dragged rect/line as a keep-out zone (mode kept so more can be drawn)
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
  // plain click without drag: goto uses the robot-to-goal direction, others use 0° (map +x)
  let yaw = 0;
  if (Math.hypot(dx, dy) > 5) yaw = Math.atan2(-dy, dx) - S.rot;   // screen direction -> world direction
  else if (S.mode === 'goto' && S.pose) yaw = Math.atan2(wy - S.pose.y, wx - S.pose.x);
  // ignore mouse double-presses so the same command is not sent twice in a row
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
  zone: '로봇이 들어가면 안 되는 곳(책상 밑, 좌석 줄)을 끌어서 사각형으로 그리세요. 오른쪽 "금지 구역" 목록에서 삭제',
  wall: '지나가면 안 되는 선(가상 벽)을 끌어서 그리세요. 오른쪽 "금지 구역" 목록에서 삭제',
};

function setMode(mode) {
  S.mode = mode;
  // in view mode, swiping on the map scrolls the page (other modes tap/drag on the map)
  canvas.classList.toggle('pan', mode === 'view');
  document.querySelectorAll('button.mode').forEach((b) => b.classList.toggle('active', b.dataset.mode === mode));
  $('#mode-hint').textContent = HINTS[mode];
}

document.querySelectorAll('button.mode').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));

// ------------------------------------------------------------------ panel
const STATE_LABEL = {
  OFFLINE: '오프라인', IDLE: '대기', MOVING: '이동 중', WAITING_LOAD: '적재 대기', WAITING_UNLOAD: '수령 대기', ERROR: '오류',
};

// if a detection label is a color name, color the chip with it (order card)
const CARGO_COLORS = { red: '#ef4444', green: '#22c55e', blue: '#3b82f6', yellow: '#eab308', white: '#e5e7eb',
                       빨강: '#ef4444', 초록: '#22c55e', 파랑: '#3b82f6', 노랑: '#eab308' };
function cargoColor(label) { return CARGO_COLORS[(label || '').toLowerCase()] || '#64748b'; }

// ------------------------------------------------------------------ orders (multiple items)
// destination = locations of kind person, items = detection labels of storage locations (sections; comma-separated)
function locPrefix(loc) { return loc.kind === 'home' ? '⌂ ' : loc.kind === 'dropoff' ? '👤 ' : ''; }
S.orderCounts = {};   // label -> entered count (kept across list redraws)
function renderOrder() {
  const sel = $('#order-dropoff');
  if (!sel) return;
  const drops = (S.locations || []).filter((l) => l.kind === 'dropoff');
  const keep = sel.value;
  sel.innerHTML = drops.length ? '' : '<option value="">(위치 종류 "사람 위치"를 먼저 등록하세요)</option>';
  for (const l of drops) sel.add(new Option(l.name, l.id));
  if (drops.some((l) => String(l.id) === keep)) sel.value = keep;
  const labels = [...new Set((S.locations || []).filter((l) => l.kind === 'pickup')
    .flatMap((l) => l.cargo.split(',').map((x) => x.trim()).filter(Boolean)))];
  const box = $('#order-items');
  const sig = labels.join('|');
  if (box.dataset.sig !== sig) {
    box.dataset.sig = sig;
    box.innerHTML = labels.length ? '' : '<span class="muted small">물류 위치(인식 라벨)가 없습니다.</span>';
    for (const label of labels) {
      const lab = document.createElement('label');
      lab.innerHTML = `<i style="background:${cargoColor(label.replace(/^led_/, ''))}"></i><span></span>`;
      lab.querySelector('span').textContent = label;
      const inp = document.createElement('input');
      inp.type = 'number';
      inp.min = '0';
      inp.max = '9';
      inp.value = S.orderCounts[label] || 0;
      inp.addEventListener('input', () => { S.orderCounts[label] = Math.max(0, parseInt(inp.value, 10) || 0); previewOrder(); });
      lab.appendChild(inp);
      box.appendChild(lab);
    }
  }
  const busy = !!(S.task && S.task.status === 'running');
  const online = S.robot && S.robot.state !== 'OFFLINE';
  $('#btn-order').disabled = busy || !online || !drops.length || !orderItems();
  $('#btn-order').title = busy ? '다른 작업을 처리 중입니다' : '';
}
function orderItems() {
  const items = {};
  for (const [k, v] of Object.entries(S.orderCounts)) if (v > 0) items[k] = v;
  return Object.keys(items).length ? items : null;
}
let orderPreviewTimer = null;
function previewOrder() {
  renderOrder();
  clearTimeout(orderPreviewTimer);
  const items = orderItems();
  const dropoff = $('#order-dropoff').value;
  if (!items || !dropoff) { $('#order-preview').textContent = ''; return; }
  orderPreviewTimer = setTimeout(() => send('preview_order', { dropoff_id: Number(dropoff), items }), 300);
}
$('#order-dropoff').addEventListener('change', previewOrder);
$('#btn-order').addEventListener('click', () => {
  const items = orderItems();
  const dropoff = $('#order-dropoff').value;
  if (!items || !dropoff) return;
  const text = Object.entries(items).map(([k, v]) => `${k} ${v}개`).join(', ');
  if (!confirm(`${$('#order-dropoff').selectedOptions[0].text} 로 ${text} 주문할까요?`)) return;
  send('start_order', { dropoff_id: Number(dropoff), items });
});

// Drive parameters: the list sent by the bridge (params: name, current value, startup value) as grouped input fields.
// A change sends param_set -> the bridge sets the ROS parameter and replies with the result (param_result) and the new list
const PARAM_GROUPS = [['frenet.', 'Frenet 로컬 플래너'], ['tracking.', '움직이는 물체 추적'], ['', '안전 / 회복']];
const paramGroup = (name) => PARAM_GROUPS.find(([p]) => name.startsWith(p));
S.paramRows = new Map();      // name -> {row, input, item}
S.paramPending = new Set();   // names sent from this page awaiting a reply
function renderParams() {
  const box = $('#params-list');
  const items = S.health && S.health.bridge ? S.params : null;
  if (!items || !items.length) {
    box.innerHTML = '<p class="muted small">브리지가 연결되어 있지 않습니다.</p>';
    S.paramRows.clear();
    return;
  }
  const names = items.map((it) => it.name).join(',');
  if (box.dataset.names !== names) {   // rebuild only when the list changes (protects fields being edited)
    box.dataset.names = names;
    box.innerHTML = '';
    S.paramRows.clear();
    for (const [prefix, title] of PARAM_GROUPS) {
      const group = items.filter((it) => paramGroup(it.name)[0] === prefix);
      if (!group.length) continue;
      const h = document.createElement('div');
      h.className = 'params-group';
      h.textContent = title;
      box.appendChild(h);
      for (const it of group) box.appendChild(paramRow(it, prefix));
    }
  }
  for (const it of items) {
    const r = S.paramRows.get(it.name);
    if (!r) continue;
    r.item = it;
    if (document.activeElement !== r.input) {
      if (r.input.type === 'checkbox') r.input.checked = !!it.value;
      else r.input.value = String(it.value);
    }
    const changed = it.value !== it.start;
    r.row.classList.toggle('changed', changed);
    r.reset.hidden = !changed;
    r.reset.title = `시작 값 ${it.start} 으로 되돌림`;
  }
}
function paramRow(it, prefix) {
  const row = document.createElement('div');
  row.className = 'param-row';
  const label = document.createElement('label');
  label.textContent = it.name.slice(prefix.length);
  label.title = it.name;
  const input = document.createElement('input');
  if (typeof it.value === 'boolean') {
    input.type = 'checkbox';
  } else {
    input.type = 'number';
    input.step = 'any';
  }
  const reset = document.createElement('button');
  reset.className = 'ghost small';
  reset.textContent = '↺';
  const r = { row, input, reset, item: it };
  const submit = (value) => {
    if (value === r.item.value) return;
    S.paramPending.add(it.name);
    send('param_set', { name: it.name, value });
  };
  input.addEventListener('change', () => {
    if (input.type === 'checkbox') return submit(input.checked);
    const v = Number(input.value);
    if (input.value.trim() === '' || !Number.isFinite(v)) {
      input.value = String(r.item.value);
      return toast(`${it.name}: 숫자를 넣으세요.`);
    }
    submit(v);
  });
  input.addEventListener('keydown', (e) => { if (e.key === 'Enter') input.blur(); });
  reset.addEventListener('click', () => submit(r.item.start));
  row.append(label, input, reset);
  S.paramRows.set(it.name, r);
  return row;
}
function paramResult(msg) {
  if (!S.paramPending.delete(msg.name)) return;   // changed by another page
  const r = S.paramRows.get(msg.name);
  if (!msg.ok) {
    toast(`${msg.name} 변경 실패: ${msg.reason || '알 수 없는 이유'}`);
    if (r) {
      if (r.input.type === 'checkbox') r.input.checked = !!r.item.value;
      else r.input.value = String(r.item.value);
    }
    return;
  }
  if (r) {
    r.row.classList.add('saved');
    setTimeout(() => r.row.classList.remove('saved'), 1000);
  }
}

let YOLO_NAMES = [];   // class names last seen in /yolo/counts (cell order)
// YOLO detector: state badge, on/off buttons, /yolo/counts text ("DHT11: 0 / led_red: 1 / ...")
function renderYolo() {
  const y = S.yolo;
  const bridge = S.health && S.health.bridge;
  const badge = $('#yolo-state');
  let label = '브리지 없음';
  let cls = 'badge off';
  if (bridge && y && !y.available) label = '검출기 없음';
  else if (bridge && y && y.enabled) { label = y.counts ? '실행 중' : '실행 중 · 영상 대기'; cls = 'badge on'; }
  else if (bridge && y && y.enabled === false) label = '정지';
  badge.textContent = label;
  badge.className = cls;
  const on = !!(bridge && y && y.available);
  $('#btn-yolo-on').disabled = !on || y.enabled === true;
  $('#btn-yolo-off').disabled = !on || y.enabled === false;
  $('#btn-yolo-capture').disabled = !on || !y.enabled;
  // the bridge only sends counts received within 3 s, so show them even if enabled could not be read
  const counts = (bridge && y && y.counts) || '';
  const items = counts.split('/').map((s) => s.trim().split(':')).filter((p) => p.length === 2)
    .map(([name, n]) => [name.trim(), parseInt(n, 10) || 0]);
  const found = items.filter(([, n]) => n > 0);
  const total = found.reduce((a, [, n]) => a + n, 0);
  if (items.length) YOLO_NAMES = items.map(([name]) => name);
  renderYoloClasses(items);
  $('#yolo-summary').textContent = counts
    ? (total ? `검출 ${total}개 · ${found.map(([name, n]) => `${name} ${n}`).join(', ')}` : '검출 없음 (0개)')
    : !on ? (bridge && y ? '검출기 노드 없음 (브리지와 같이 실행됨: logs/bridge.log 확인)' : '')
    : !y.enabled ? 'YOLO 정지됨 (추론 안 함)'
    : '카메라 영상을 기다리는 중';
  $('#yolo-counts').textContent = counts;
}
// per-class count cells above the buttons. When no counts arrive, the last seen classes stay as '-' so the layout does not jump
function renderYoloClasses(items) {
  const box = $('#yolo-classes');
  const map = new Map(items);
  box.hidden = !YOLO_NAMES.length;
  box.replaceChildren(...YOLO_NAMES.map((name) => {
    const n = map.get(name);
    const tile = document.createElement('div');
    tile.className = 'yolo-class' + (n > 0 ? ' hit' : '');
    const label = document.createElement('span');
    label.textContent = name;
    label.title = name;
    const num = document.createElement('b');
    num.textContent = n === undefined ? '-' : String(n);
    tile.append(label, num);
    return tile;
  }));
}
// time from button press to result (loading indicator). On: set reply -> first detection, off: set reply, capture: next detection
const YOLO_JOB = {
  on: { title: 'YOLO 시작 중', limit: 20000 },
  off: { title: 'YOLO 정지 중', limit: 10000 },
  capture: { title: '장면 캡처 중', limit: 15000 },
};
function yoloStart(kind, data) {
  if (S.yoloJob) return;
  S.yoloJob = { kind, t0: performance.now(), ack: null, countsAt: S.yolo && S.yolo.countsAt };
  $('#yolo-loading-title').textContent = YOLO_JOB[kind].title;
  $('#yolo-loading').hidden = false;
  S.yoloTimer = setInterval(yoloTick, 100);
  send('yolo_set', data);
}
function yoloFinish(text, ok = true) {
  clearInterval(S.yoloTimer);
  S.yoloJob = null;
  $('#yolo-loading').hidden = true;
  $('#yolo-timing').textContent = text;
  addLog({ created_at: new Date().toISOString(), message: `YOLO ${text}${ok ? '' : ' ⚠'}` });
}
function yoloTick() {
  const j = S.yoloJob;
  if (!j) return;
  const now = performance.now();
  const sec = (ms) => (ms / 1000).toFixed(1);
  const y = S.yolo || {};
  if (j.ack === null && y.available && ((j.kind === 'on' && y.enabled === true) || (j.kind === 'off' && y.enabled === false)
      || j.kind === 'capture')) j.ack = now;
  const fresh = y.countsAt && y.countsAt !== j.countsAt;   // new detection result since the button press
  $('#yolo-loading-time').textContent = `${sec(now - j.t0)}초` + (j.ack !== null && j.kind === 'on' ? ` · 설정 ${sec(j.ack - j.t0)}초, 첫 결과 기다리는 중` : '');
  if (j.kind === 'off' && j.ack !== null) return yoloFinish(`정지 완료: ${sec(j.ack - j.t0)}초`);
  if (j.kind !== 'off' && j.ack !== null && fresh) {
    return yoloFinish(j.kind === 'on' ? `시작 완료: 설정 ${sec(j.ack - j.t0)}초, 첫 검출 결과 ${sec(now - j.t0)}초`
      : `캡처 완료: ${sec(now - j.t0)}초 (~/camera_dataset/captures)`);
  }
  if (now - j.t0 > YOLO_JOB[j.kind].limit) {
    yoloFinish(`${YOLO_JOB[j.kind].title.replace(' 중', '')} 시간 초과 (${sec(now - j.t0)}초)`
      + (j.ack === null ? ': 검출기 응답 없음' : ': 검출 결과 없음, 카메라 영상(/csi/image_raw/compressed) 확인'), false);
  }
}
$('#btn-yolo-on').addEventListener('click', () => yoloStart('on', { enabled: true }));
$('#btn-yolo-off').addEventListener('click', () => yoloStart('off', { enabled: false }));
$('#btn-yolo-capture').addEventListener('click', () => yoloStart('capture', { capture: true }));

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
  // phone: robot state and battery in the top bar that stays visible while scrolling
  const top = $('#top-robot');
  top.textContent = (STATE_LABEL[state] || state)
    + (r && r.battery_percentage != null && state !== 'OFFLINE' ? ` · ${Math.round(r.battery_percentage)}%` : '');
  top.className = 'badge top-robot ' + state;
  const bat = $('#battery');
  bat.textContent = r && r.battery_voltage != null && state !== 'OFFLINE'
    ? `배터리 ${r.battery_voltage.toFixed(1)} V` + (r.battery_percentage != null ? ` (${Math.round(r.battery_percentage)}%)` : '')
    : '';
  bat.className = r && r.battery_percentage != null && r.battery_percentage < 20 ? 'low' : '';
  bat.title = bat.className ? '배터리가 부족합니다. 충전하세요.' : '';
  // battery bar: hide the whole row when there is no value (e.g. simulation)
  const pct = r && r.battery_percentage != null && state !== 'OFFLINE' ? r.battery_percentage : null;
  $('#battery-row').classList.toggle('empty', pct == null && !bat.textContent);
  const fill = $('#battery-fill');
  fill.style.width = `${Math.max(0, Math.min(100, pct || 0))}%`;
  fill.className = pct == null ? '' : pct < 20 ? 'low' : pct < 50 ? 'mid' : '';

  const online = state !== 'OFFLINE';
  const running = S.task && S.task.status === 'running';
  const step = currentStep();
  const unload = step && step.action === 'unload';
  // arrival actions only while waiting at a stop; retry/skip only while a step is active
  const arrived = !!(step && step.status === 'arrived');
  $('#wait-actions').hidden = !arrived;
  $('#btn-confirm').textContent = unload ? '✓ 수령 완료' : '✓ 적재 완료';
  const orderWait = arrived && !!(step.items && Object.keys(step.items).length);
  $('#btn-force').hidden = !orderWait;
  $('#btn-force').textContent = unload ? '⏭ 강제 보내기 (확인 없이 홈으로)' : '⏭ 다음 경유지로 (확인 없이)';
  renderOrder();
  $('#step-actions').hidden = !(online && step);
  $('#btn-retry').disabled = !(online && step && (step.status === 'failed' || state === 'ERROR'));
  $('#btn-skip').disabled = !(online && step);
  $('#btn-cancel').disabled = !running;
  const hasHome = S.locations.some((l) => l.kind === 'home');
  $('#btn-home').disabled = !online || !hasHome;
  $('#btn-home').title = hasHome ? '' : '홈 위치가 없습니다. 로봇을 홈에 두고 "현재 위치를 홈으로"를 누르세요.';
  $('#btn-set-home').disabled = !S.pose || running;
  $('#btn-reset').hidden = !(state === 'ERROR' && !running);
  $('#task-status').textContent = S.task ? `#${S.task.id} ${{ running: '진행 중', done: '완료', canceled: '취소됨', failed: '실패' }[S.task.status] || S.task.status}` : '';

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
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = locPrefix(loc) + loc.name;
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
    li.append(name, coord, go, del);
    ul.appendChild(li);
  }
}

// changing keep-out zones during auto exploration makes the explorer (tools/explore.py) re-pick its goal
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
  // with several robots, prefix the robot name to each entry
  const who = S.robots.length > 1 && entry.robot_name ? `[${entry.robot_name}] ` : '';
  li.append(t, who + entry.message);
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

// ------------------------------------------------------------------ location form
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

// ------------------------------------------------------------------ buttons
$('#btn-confirm').addEventListener('click', () => {
  const step = currentStep();
  send(step && step.action === 'unload' ? 'confirm_unload' : 'confirm_load');
});
$('#btn-retry').addEventListener('click', () => send('retry_step'));
$('#btn-skip').addEventListener('click', () => send('skip_step'));
$('#btn-force').addEventListener('click', () => send('force_next'));
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
$('#btn-legend').addEventListener('click', () => {
  const open = $('#canvas-wrap').classList.toggle('legend-open');
  $('#btn-legend').setAttribute('aria-expanded', String(open));
});
$('#btn-reset-robot').addEventListener('click', () => {
  if (confirm('작업을 취소하고 로봇 상태를 초기화할까요?\n(시뮬레이션이면 로봇이 출발 위치로 돌아갑니다)')) send('reset_robot');
});
function fmtSec(sec) {
  if (sec == null) return '-';
  sec = Math.round(sec);
  return sec >= 60 ? `${Math.floor(sec / 60)}분 ${sec % 60}초` : `${sec}초`;
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();
renderPanel();
renderHealth();
connect();

// ------------------------------------------------------------------ manual control (teleop)
// While held, send {v, w} every 0.1 s. The bridge stops if nothing arrives for over 0.4 s (stops on release or leaving the window)
const TELEOP = { v: 0, w: 0, timer: null };
function teleopSend() { send('teleop', { v: TELEOP.v, w: TELEOP.w }); }
function teleopStart(dv, dw) {
  const speed = Number($('#teleop-speed').value);
  TELEOP.v = dv >= 0 ? dv * speed : dv * Math.min(speed, 0.1);   // reverse at most 0.1 m/s
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
