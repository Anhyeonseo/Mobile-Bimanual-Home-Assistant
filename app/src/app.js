import { renderDialog } from './ui/dialogs.js';
import { renderRequest } from './ui/screens/request.js';
import { renderTask } from './ui/screens/task.js';
import { renderMap } from './ui/screens/map.js';
import { renderRoutines } from './ui/screens/routines.js';
import { renderActivity } from './ui/screens/activity.js';
import { renderSettings } from './ui/screens/settings.js';
import { renderSetup } from './ui/screens/setup.js';
import { html } from './ui/html.js';
import { renderShell } from './ui/shell.js';
import { renderHome } from './ui/home.js';
import {
  initNativeMotion,
  setComposerOpen,
  consumeHandleClick,
  isWideLayout,
  isComposerDragging,
} from './ui/motion.js';
import {
  captureMotionFrame,
  animateRenderedFrame,
  captureSelections,
  animateSelections,
} from './ui/transitions.js';
import { scenarios } from './data/scenarios.js';
import { REQUEST_EXAMPLES } from './data/requests.js';
import { initDeliveryMotion, syncDeliveryScene } from './ui/delivery-motion.js';
import { OBSERVATIONS } from './slam.js';
import {
  KEY,
  PLACES,
  ITEMS,
  initialState,
  loadState,
  saveState,
  parseRequest,
  makeTask,
  tickTask,
  taskGuard,
  cancelTask,
  dayLabel,
  escapeHTML as e,
  toward,
  isBlocked,
  findRoute,
} from './core.js';
import { icon as i, floorplan } from './art.js';

let state = loadState(localStorage);
let previousPage = 'home';
let composerExpanded = false;
let draftPrompt = '';
let page = ['home', 'request', 'map', 'routines', 'activity', 'settings', 'setup', 'task'].includes(
  location.hash.slice(1),
)
  ? location.hash.slice(1)
  : 'home';
let modal = null,
  modalData = {},
  drawer = false,
  demo = null,
  setupStep = 0,
  selectedPlace = null,
  request = null,
  activityFilter = 'all',
  zoom = 1,
  selectedObservation = null;
let routineDraft = null,
  toastTimer = null,
  priorFocus = null;
const $ = (s) => document.querySelector(s);
const names = {
  home: '홈',
  request: '부탁하기',
  map: '우리 집 지도',
  routines: '나의 루틴',
  activity: '활동 기록',
  settings: '설정',
  setup: '처음 연결하기',
  task: '진행 중인 부탁',
};
const mascot = (cls = '') => html`
  <img
    class="aloha-mascot ${cls}"
    src="assets/aloha-mini-opal.png"
    alt="은은한 블루와 라벤더 빛의 로미 캐릭터"
    width="1188"
    height="1324"
  />
`;
const activeTask = () => state.task && ['running', 'paused', 'blocked'].includes(state.task.status);
const place = (id) => PLACES.find((p) => p.id === id) || PLACES[0];
const item = (id) => state.items.find((x) => x.id === id);
const button = (label, action, cls = 'btn-secondary', ic = '', extra = '') => html`
  <button type="button" class="btn ${cls}" data-action="${action}" ${extra}>
    ${ic ? i(ic, 16) : ''}${label}
  </button>
`;
const arrowLink = (label, action, extra = '') => html`
  <button class="text-btn" data-action="${action}" ${extra}>${label}${i('chevron', 13)}</button>
`;
const roomIcon = (id) =>
  ({ living: 'home', bedroom: 'bed', study: 'desk', kitchen: 'kitchen', dock: 'battery' })[id] ||
  'home';
const selected = (a, b) => (a === b ? ' selected' : '');
function persist() {
  if (!saveState(localStorage, state)) toast('저장 공간이 부족해 변경 내용을 저장하지 못했어요.');
}
function toast(msg) {
  clearTimeout(toastTimer);
  $('#toast-root').innerHTML = html`
    <div class="toast">
      ${i('check', 16)}
      <span>${e(msg)}</span>
    </div>
  `;
  toastTimer = setTimeout(() => ($('#toast-root').innerHTML = ''), 3600);
}
function go(next) {
  previousPage = page;
  page = next;
  if (location.hash !== `#${next}`) history.replaceState(null, '', `#${next}`);
  render();
  window.scrollTo({ top: 0, behavior: 'instant' });
}
function heading(title, description = '', actions = '') {
  return html`
    <div class="page-heading">
      <div>
        <h1>${title}</h1>
        ${description
          ? html`
              <p>${description}</p>
            `
          : ''}
      </div>
      ${actions}
    </div>
  `;
}

function shell(content) {
  return renderShell(content, {
    page,
    previousPage,
    unread: state.notifications.some((notification) => !notification.read),
    activeTask: !!activeTask(),
    emergency: state.robot.emergency,
    demoBar: demo ? demoBar() : '',
  });
}

function liveBanner() {
  if (!activeTask()) return '';
  return html`
    <div class="live-task-banner">
      <div class="row">
        ${i(state.task.status === 'running' ? 'activity' : 'pause', 19)}
        <div>
          <strong>${e(state.task.title)}</strong>
          <p>
            ${state.task.status === 'running'
              ? e(state.task.stages[state.task.stage])
              : '잠시 멈춰 있어요'}
            · ${state.task.progress}%
          </p>
        </div>
      </div>
      ${button('진행 보기', 'nav', 'btn-outline', 'chevron', 'data-page="task"')}
    </div>
  `;
}
function render() {
  const motionFrame = captureMotionFrame();
  const context = {
    state,
    request: request || { text: draftPrompt },
    setupStep,
    selectedObservation,
    selectedPlace,
    activityFilter,
    zoom,
    heading,
    button,
    arrowLink,
    selected,
    item,
    place,
    activeTask,
    liveBanner,
    mascot,
    roomIcon,
  };
  const screens = {
    home: homeView,
    request: () => renderRequest(context),
    map: () => renderMap(context),
    routines: () => renderRoutines(context),
    activity: () => renderActivity(context),
    settings: () => renderSettings(context),
    setup: () => renderSetup(context),
    task: () => renderTask(context),
  };
  $('#app').innerHTML = shell((screens[page] || homeView)());
  $('.main').classList.toggle('native-task-main', !!$('.delivery-experience'));
  syncDeliveryScene(state);
  animateRenderedFrame(motionFrame);
  renderOverlay();
}

function homeView() {
  return renderHome(state, {
    hasTask: !!activeTask(),
    requestText: request?.text || draftPrompt,
    composerOpen: isWideLayout() || composerExpanded,
  });
}

function demoBar() {
  const s = scenarios[demo.id];
  return html`
    <div class="demo-running">
      <span class="assistant-orb" style="width:30px;height:30px;border-radius:9px">
        ${i('play', 14)}
      </span>
      <div style="flex:1">
        <div class="demo-step">
          SCENARIO ${String(demo.id + 1).padStart(2, '0')} · ${demo.step + 1}/${s.steps.length}
        </div>
        <strong>${s.steps[demo.step]}</strong>
      </div>
      ${button(
        demo.step === s.steps.length - 1 ? '시연 마치기' : '다음 안내',
        'demo-next',
        'btn-primary',
      )}
      <button class="icon-btn" data-action="demo-close" aria-label="시연 안내 닫기">
        ${i('close', 15)}
      </button>
    </div>
  `;
}
function demoDrawer() {
  return html`
    <div class="drawer-scrim" data-action="close-drawer"></div>
    <aside class="demo-panel" role="dialog" aria-modal="true" aria-labelledby="demo-title">
      <div class="section-top">
        <div>
          <div class="eyebrow">MEET YOUR NEW EVERYDAY</div>
          <h2 id="demo-title">사용 흐름별 시연</h2>
        </div>
        <button class="icon-btn" data-action="close-drawer" aria-label="닫기">
          ${i('close', 21)}
        </button>
      </div>
      <p>
        처음 연결부터 SLAM 지도 이동까지 확인하세요.
        <br />
        시작하면 화면 아래에 진행 안내가 나타나요.
      </p>
      <div class="note blue mb">
        ${i('info', 17)} 예시 데이터로 동작하는 인터랙티브 앱이에요. 실제 로봇 제어·계정 연동·예약
        실행은 연결 전 단계예요.
      </div>
      ${scenarios
        .map(
          (s, k) => html`
            <article class="scenario">
              <div class="scenario-number">SCENARIO ${String(k + 1).padStart(2, '0')}</div>
              <h3>${s.title}</h3>
              <p>${s.desc}</p>
              <div class="scenario-meta">
                <span>${s.subtitle}</span>
                <span>${s.time}</span>
              </div>
              ${button('이 시나리오 시작', 'demo-start', 'btn-primary', 'play', `data-id="${k}"`)}
            </article>
          `,
        )
        .join('')}
      <div class="note">
        ${i('shield', 16)} 시연을 다시 시작해도 저장한 이름·가구·루틴은 유지해요. 전체 초기화는
        설정에서 할 수 있어요.
      </div>
      <a class="download-link mt" href="guide.html" target="_blank" rel="noopener">
        ${i('book', 16)} 사용자 시나리오와 기능 가이드
      </a>
    </aside>
  `;
}

function showModal(type, data = {}) {
  priorFocus = document.activeElement;
  modal = type;
  modalData = data;
  drawer = false;
  renderOverlay();
  requestAnimationFrame(() => $('#overlay-root [role="dialog"] input')?.focus());
}
function closeModal() {
  modal = null;
  modalData = {};
  renderOverlay();
  priorFocus?.isConnected && priorFocus.focus();
}
function renderOverlay() {
  const root = $('#overlay-root');
  const previousTitle = root.querySelector('.modal-header h2')?.textContent;
  const previousSelections = captureSelections(root);
  let out = '';
  if (drawer) out = demoDrawer();
  else if (modal)
    out = renderDialog(modal, {
      state,
      modalData,
      routineDraft,
      button,
      item,
      place,
      roomIcon,
      mascot,
      selected,
    });
  root.innerHTML = out;
  const dialog = root.querySelector('.modal');
  if (dialog && dialog.querySelector('.modal-header h2')?.textContent === previousTitle) {
    dialog.style.animation = 'none';
    animateSelections(previousSelections, root);
  }
  document.body.style.overflow = modal || drawer ? 'hidden' : '';
}
function prepareTask(q) {
  const error = taskGuard(state, q.kind);
  if (error) {
    toast(error);
    if (!state.robot.connected) showModal('device');
    return;
  }
  if (q.stopTarget && isBlocked(q.stopTarget.x, q.stopTarget.y, state.map.zone)) {
    toast('선택한 접근 위치는 이동할 수 없어요. 금지 구역을 확인해 주세요.');
    return;
  }
  if (q.kind === 'transport' && !item(q.itemId)) {
    toast('옮길 물건을 먼저 선택해 주세요.');
    return;
  }
  showModal('review', { ...q });
}
function openRoutine(r) {
  routineDraft = structuredClone(
    r || {
      id: null,
      name: '',
      time: '09:00',
      days: [1, 2, 3, 4, 5],
      kind: 'transport',
      item: 'water',
      dest: 'study',
      enabled: true,
      icon: 'sun',
    },
  );
  routineDraft.kind ||= 'transport';
  showModal('routine');
}
function newMapping() {
  if (activeTask()) {
    toast('진행 중인 부탁을 마친 뒤 지도를 만들 수 있어요.');
    return;
  }
  if (!state.robot.connected) {
    toast('로미와 먼저 연결해 주세요.');
    return;
  }
  if (state.robot.emergency) {
    toast('긴급 정지를 먼저 해제해 주세요.');
    return;
  }
  if (!state.map.draft) {
    state.map.scan = 0;
    state.map.scanning = false;
  }
  showModal('mapping');
}
function scanStart() {
  if (activeTask() || state.robot.emergency || !state.robot.connected) {
    toast('로봇 연결과 진행 중인 작업을 확인해 주세요.');
    return;
  }
  state.robot.charging = false;
  state.map.scanning = true;
  state.map.draft = true;
  if (state.map.scan === 100) state.map.scan = 0;
  persist();
  render();
}
function scanSave() {
  state.map.localization = 'ready';
  state.map.localizeProgress = 100;
  state.map.scanning = false;
  state.map.draft = false;
  state.map.ready = true;
  state.map.scan = 100;
  state.map.name = state.profile.home;
  state.map.savedAt = '방금';
  state.history.unshift({
    id: 'map-' + Date.now(),
    title: '우리 집 지도를 새로 기억했어요',
    subtitle: '주방 · 소파·작업 공간 · 침실',
    status: 'done',
    time: '방금',
    duration: '15초 · 체험',
    icon: 'map',
    kind: 'map',
  });
  persist();
}
function captureRoutine() {
  if (!$('#routine-form')) return;
  const f = new FormData($('#routine-form'));
  Object.assign(routineDraft, {
    name: String(f.get('name') || ''),
    time: String(f.get('time') || '09:00'),
    kind: String(f.get('kind') || 'transport'),
    item: String(f.get('item') || 'water'),
    dest: String(f.get('dest') || 'study'),
  });
}
function moveRobot(dir) {
  if (!state.robot.connected || state.robot.emergency || activeTask() || state.map.scanning) {
    toast('로봇이 대기 상태일 때 수동 조작할 수 있어요.');
    return;
  }
  if (state.robot.battery <= 15) {
    toast('배터리가 부족해요. 먼저 충전해 주세요.');
    return;
  }
  const delta = { up: [0, -12], down: [0, 12], left: [-12, 0], right: [12, 0] }[dir];
  if (!delta) return;
  const before = state.robot.position,
    next = { x: before.x + delta[0], y: before.y + delta[1] };
  const blocked = (x, y) => isBlocked(x, y, state.map.zone);
  for (let k = 1; k <= 4; k++) {
    if (blocked(before.x + (delta[0] * k) / 4, before.y + (delta[1] * k) / 4)) {
      toast('앞에 벽이나 가구가 있어요. 다른 방향으로 이동해 주세요.');
      return;
    }
  }
  state.robot.position = next;
  state.robot.charging = false;
  state.robot.room = next.y > 520 ? '침실' : next.y < 390 ? '주방' : '소파·작업 공간';
  persist();
  renderOverlay();
}
document.addEventListener('click', (event) => {
  const b = event.target.closest('[data-action]');
  if (!b || b.disabled) return;
  const a = b.dataset.action,
    id = b.dataset.id;
  switch (a) {
    case 'map-view': {
      const mode = b.dataset.value;
      if (!['home', 'slam'].includes(mode) || mode === state.settings.mapView) return;
      state.settings.mapView = mode;
      persist();
      render();
      $(`.main [data-action="map-view"][data-value="${mode}"]`)?.focus({ preventScroll: true });
      break;
    }
    case 'slam-demo':
      if (activeTask()) {
        toast('진행 중인 부탁을 마친 뒤 시작해 주세요.');
        return;
      }
      demo = { id: 4, step: 0 };
      selectedObservation = null;
      go('map');
      break;
    case 'load-slam':
      if (activeTask() || state.map.scanning) {
        toast('진행 중인 작업이나 탐색을 먼저 마쳐 주세요.');
        return;
      }
      state.map.ready = true;
      state.map.localization = 'unconfirmed';
      state.map.localizeProgress = 0;
      selectedObservation = null;
      persist();
      render();
      toast('저장 지도를 불러왔어요. 로봇의 현재 위치를 맞춰 주세요.');
      break;
    case 'localize':
      if (activeTask()) {
        toast('진행 중인 부탁을 마친 뒤 위치를 맞춰 주세요.');
        return;
      }
      showModal('localize', { pose: 'desk' });
      break;
    case 'pose-select':
      modalData.pose = id;
      renderOverlay();
      break;
    case 'start-localize': {
      const o = OBSERVATIONS.find((o) => o.id === modalData.pose);
      if (!o) return;
      state.robot.position = { ...o.stop };
      state.robot.room = o.room;
      state.map.localization = 'localizing';
      state.map.localizeProgress = 0;
      persist();
      closeModal();
      render();
      break;
    }
    case 'observation': {
      if (modal) return;
      const o = OBSERVATIONS.find((o) => o.id === id);
      if (!o) return;
      selectedObservation = id;
      selectedPlace = o.place;
      if (page === 'request' && request) {
        request.dest = o.place;
        render();
      } else go('map');
      break;
    }
    case 'composer-open':
      setComposerOpen(true, { focus: true });
      break;
    case 'composer-toggle':
      if (!consumeHandleClick()) setComposerOpen(!composerExpanded);
      break;
    case 'nav':
      if (page === 'setup' && state.map.scanning) {
        state.map.scanning = false;
        state.map.draft = true;
        persist();
      }
      go(b.dataset.page);
      break;
    case 'close-modal':
      closeModal();
      break;
    case 'backdrop':
      if (event.target === b) closeModal();
      break;
    case 'demo':
      drawer = true;
      modal = null;
      renderOverlay();
      break;
    case 'close-drawer':
      drawer = false;
      renderOverlay();
      break;
    case 'demo-start': {
      if (activeTask()) {
        toast('진행 중인 부탁을 마치거나 취소한 뒤 시연을 시작해 주세요.');
        return;
      }
      drawer = false;
      demo = { id: Number(id), step: 0 };
      const scenario = scenarios[Number(id)];
      if (scenario.screen === 'map') selectedObservation = null;
      if (scenario.screen === 'setup') setupStep = 0;
      if (scenario.prompt) request = { text: scenario.prompt };
      go(scenario.screen);
      break;
    }
    case 'demo-next':
      if (demo.step < scenarios[demo.id].steps.length - 1) demo.step++;
      else {
        demo = null;
        toast('시연을 마쳤어요. 다른 시나리오도 이어서 체험해 보세요.');
      }
      render();
      break;
    case 'demo-close':
      demo = null;
      render();
      break;
    case 'setup-start':
      if (activeTask()) {
        toast('진행 중인 부탁을 마친 뒤 연결 체험을 시작해 주세요.');
        return;
      }
      setupStep = 0;
      go('setup');
      break;
    case 'setup-back':
      if (state.map.scanning) {
        state.map.scanning = false;
        persist();
      }
      if (setupStep > 0) {
        setupStep--;
        render();
      } else go('home');
      break;
    case 'setup-pair': {
      const name = $('#setup-robot').value.trim();
      if (!name) {
        $('#setup-robot').focus();
        toast('로봇 이름을 입력해 주세요.');
        return;
      }
      state.robot.name = name;
      setupStep = 3;
      persist();
      render();
      break;
    }
    case 'pair-retry':
      toast('가까이에 있는 ALOHA Mini · HK-2026-001을 다시 찾았어요.');
      break;
    case 'wifi-fail':
      $('#wifi-error').innerHTML =
        '연결하지 못했어요. 비밀번호와 2.4GHz 네트워크를 확인하고 다시 시도해 주세요.';
      break;
    case 'setup-map-done':
      scanSave();
      setupStep = 5;
      state.setupComplete = true;
      persist();
      render();
      break;
    case 'setup-finish':
      request = { text: '거실 테이블에 있는 검은 머그컵을 침실 협탁으로 옮겨줘' };
      state.setupComplete = true;
      persist();
      go('request');
      break;
    case 'setup-home':
      state.setupComplete = true;
      persist();
      go('home');
      break;
    case 'request-example': {
      request = { text: (REQUEST_EXAMPLES[id] || REQUEST_EXAMPLES.cup).text };
      draftPrompt = request.text;
      if (page === 'home') {
        $('#request-text').value = request.text;
        setComposerOpen(true, { focus: true });
      } else {
        go('request');
        requestAnimationFrame(() => $('#request-text')?.focus());
      }
      break;
    }
    case 'request-reset':
      request = null;
      draftPrompt = '';
      render();
      break;
    case 'new-request':
      request = null;
      draftPrompt = '';
      go('request');
      break;
    case 'choose-item':
      request.itemId = id;
      render();
      break;
    case 'choose-dest':
      request.dest = id;
      render();
      break;
    case 'confirm-clarify':
      if (request?.dest && (request.kind !== 'transport' || request.itemId)) {
        request.needsItem = false;
        request.needsDest = false;
        prepareTask(request);
      }
      break;
    case 'come':
      showModal('come', { dest: null });
      break;
    case 'come-select':
      modalData.dest = id;
      renderOverlay();
      break;
    case 'come-confirm':
      if (modalData.dest) prepareTask({ kind: 'navigate', dest: modalData.dest });
      break;
    case 'patrol':
      prepareTask({ kind: 'patrol', dest: 'living' });
      break;
    case 'charge':
      prepareTask({ kind: 'charge', dest: 'dock' });
      break;
    case 'start-task': {
      const q = { ...modalData };
      const err = taskGuard(state, q.kind);
      if (err) {
        toast(err);
        return;
      }
      state.task = makeTask(q, state);
      state.task.angle = q.angle || 0;
      state.robot.charging = false;
      persist();
      closeModal();
      go('task');
      break;
    }
    case 'pause-task':
      if (state.task?.status === 'running') {
        state.task.status = 'paused';
        state.task.reason = '잠시 멈췄어요. 준비되면 이어서 진행하세요.';
        persist();
        render();
      }
      break;
    case 'resume-task':
      if (!state.robot.connected) {
        toast('다시 연결한 뒤 이어서 진행해 주세요.');
        showModal('device');
        return;
      }
      if (state.robot.emergency) {
        toast('긴급 정지를 먼저 해제해 주세요.');
        return;
      }
      if (state.task && ['paused', 'blocked'].includes(state.task.status)) {
        state.task.status = 'running';
        state.task.reason = '';
        state.task.obstacle = false;
        persist();
        render();
        toast('주변을 다시 확인하고 이어서 진행해요.');
      }
      break;
    case 'cancel-task':
      showModal('cancel');
      break;
    case 'confirm-cancel':
      cancelTask(state);
      persist();
      closeModal();
      render();
      break;
    case 'missing-item':
      if (activeTask()) {
        state.task.status = 'blocked';
        state.task.reason =
          '기억한 위치에서 물건을 찾지 못했어요. 물건을 원래 위치에 놓고 다시 확인하거나, 작업을 취소해 주세요.';
        persist();
        render();
      }
      break;
    case 'simulate-obstacle':
      if (activeTask()) {
        state.task.status = 'blocked';
        state.task.obstacle = true;
        state.task.reason =
          '이동 경로에 장애물이 있어요. 주변 물건을 정리한 뒤 이어서 진행해 주세요.';
        persist();
        render();
        toast('장애물을 발견해 안전하게 멈췄어요.');
      }
      break;
    case 'emergency':
      state.robot.emergency = true;
      state.robot.charging = false;
      if (activeTask()) {
        state.task.status = 'paused';
        state.task.reason = '긴급 정지했어요. 주변을 확인하고 정지를 해제해 주세요.';
      }
      state.map.scanning = false;
      persist();
      closeModal();
      render();
      toast('긴급 정지했어요. 로미가 현재 위치에서 멈췄어요.');
      break;
    case 'release-stop':
      showModal('release');
      break;
    case 'confirm-release':
      state.robot.emergency = false;
      persist();
      closeModal();
      render();
      toast('정지를 해제했어요. 진행하던 작업은 일시 정지 상태예요.');
      break;
    case 'disconnect':
      state.robot.connected = false;
      state.map.scanning = false;
      if (activeTask()) {
        state.task.status = 'paused';
        state.task.reason = '로봇과 연결이 끊겨 멈췄어요. 다시 연결한 뒤 이어서 진행하세요.';
      }
      persist();
      closeModal();
      render();
      toast('연결 끊김 상황을 체험 중이에요.');
      break;
    case 'reconnect':
      state.robot.connected = true;
      persist();
      closeModal();
      render();
      toast('로미와 다시 연결했어요. 진행하던 작업은 직접 재개해 주세요.');
      break;
    case 'feedback':
      if (!state.task) return;
      state.task.feedback = b.dataset.value;
      persist();
      render();
      if (b.dataset.value === 'issue') showModal('feedback');
      else toast('마음을 전해주셔서 고마워요.');
      break;
    case 'room':
      if (modal) return;
      if (page === 'request' && request) {
        request.dest = id;
        render();
      } else {
        selectedPlace = id;
        go('map');
      }
      break;
    case 'navigate-selected': {
      const o = OBSERVATIONS.find((o) => o.id === selectedObservation);
      if (o)
        prepareTask({
          kind: 'navigate',
          dest: o.place,
          stopTarget: o.stop,
          stopLabel: (state.furniture.find((f) => f.id === o.id)?.name || o.name) + ' 앞',
          angle: $('#arrival-angle')?.value || 0,
        });
      break;
    }
    case 'locate':
      selectedObservation = 'desk';
      selectedPlace = PLACES.find((p) => p.name === state.robot.room)?.id || 'living';
      go('map');
      toast(`${state.robot.name}는 지금 ${state.robot.room}에 있어요.`);
      break;
    case 'zoom-in':
      zoom = Math.min(1.65, zoom + 0.15);
      render();
      break;
    case 'zoom-out':
      zoom = Math.max(0.85, zoom - 0.15);
      render();
      break;
    case 'zoom-reset':
      zoom = 1;
      render();
      break;
    case 'mapping-open':
      newMapping();
      break;
    case 'start-mapping':
      scanStart();
      break;
    case 'pause-mapping':
      state.map.scanning = false;
      state.map.draft = true;
      persist();
      render();
      break;
    case 'restart-mapping':
      state.map.scan = 0;
      scanStart();
      break;
    case 'save-draft':
      state.map.scanning = false;
      state.map.draft = true;
      persist();
      closeModal();
      render();
      toast('탐색한 위치까지 저장했어요. 나중에 이어서 만들 수 있어요.');
      break;
    case 'finish-mapping':
      scanSave();
      closeModal();
      render();
      toast('우리 집 지도를 저장했어요.');
      break;
    case 'save-map':
      if (!state.map.ready) {
        toast('먼저 집 지도를 만들어 주세요.');
        return;
      }
      state.map.savedAt = '방금';
      persist();
      render();
      toast('지도와 공간 설정을 저장했어요.');
      break;
    case 'zone':
      showModal('zone');
      break;
    case 'confirm-zone':
      state.map.zone = !state.map.zone;
      persist();
      closeModal();
      render();
      toast(state.map.zone ? '진입 금지 구역을 설정했어요.' : '진입 금지 구역을 해제했어요.');
      break;
    case 'furniture-focus':
      go('map');
      requestAnimationFrame(() =>
        $('#furniture-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' }),
      );
      break;
    case 'edit-furniture':
      showModal('furniture', { id });
      break;
    case 'add-furniture':
      showModal('furniture');
      break;
    case 'delete-furniture':
      state.furniture = state.furniture.filter((f) => f.id !== modalData.id);
      persist();
      closeModal();
      render();
      toast('가구 이름을 삭제했어요.');
      break;
    case 'add-routine':
      openRoutine();
      break;
    case 'edit-routine':
      openRoutine(state.routines.find((r) => r.id === id));
      break;
    case 'routine-template':
      openRoutine({
        id: null,
        name: id === 'morning' ? '업무 시작 전, 물 한 병' : '잠들기 전, 책 한 권',
        time: id === 'morning' ? '09:00' : '21:00',
        days: id === 'morning' ? [1, 2, 3, 4, 5] : [0, 1, 2, 3, 4, 5, 6],
        kind: 'transport',
        item: id === 'morning' ? 'water' : 'book',
        dest: id === 'morning' ? 'study' : 'bedroom',
        icon: id === 'morning' ? 'sun' : 'book',
        enabled: true,
      });
      break;
    case 'routine-day': {
      captureRoutine();
      const d = Number(id);
      routineDraft.days = routineDraft.days.includes(d)
        ? routineDraft.days.filter((x) => x !== d)
        : [...routineDraft.days, d].sort();
      renderOverlay();
      break;
    }
    case 'delete-routine':
      state.routines = state.routines.filter((r) => r.id !== routineDraft.id);
      persist();
      closeModal();
      render();
      toast('루틴을 삭제했어요.');
      break;
    case 'toggle-routine': {
      const r = state.routines.find((x) => x.id === id);
      r.enabled = !r.enabled;
      persist();
      render();
      toast(r.enabled ? '루틴을 켰어요.' : '루틴을 껐어요.');
      break;
    }
    case 'run-routine': {
      const r = state.routines.find((x) => x.id === id);
      prepareTask({
        kind: r.kind || 'transport',
        itemId: r.item,
        dest: r.kind === 'charge' ? 'dock' : r.dest,
      });
      break;
    }
    case 'filter-history':
      activityFilter = id;
      render();
      break;
    case 'history-detail':
      showModal('history', { id });
      break;
    case 'history-map':
      closeModal();
      go('map');
      break;
    case 'repeat-task': {
      const h = state.history.find((x) => x.id === modalData.id);
      prepareTask({
        kind: h.kind || 'transport',
        itemId: h.itemId || 'black-cup',
        dest: h.dest || 'bedroom',
        stopTarget: h.stopTarget,
        stopLabel: h.stopLabel,
        angle: h.angle,
      });
      break;
    }
    case 'export-history': {
      const blob = new Blob(
        [
          JSON.stringify(
            {
              app: 'HOMETEKHU',
              mode: 'simulation',
              exportedAt: new Date().toISOString(),
              history: state.history,
            },
            null,
            2,
          ),
        ],
        { type: 'application/json' },
      );
      const url = URL.createObjectURL(blob),
        a = document.createElement('a');
      a.href = url;
      a.download = 'HOMETEKHU-activity.json';
      document.body.append(a);
      a.click();
      a.remove();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      toast('활동 기록을 파일로 내보냈어요.');
      break;
    }
    case 'device':
      showModal('device');
      break;
    case 'remote': {
      const err = taskGuard(state, 'navigate');
      if (err) {
        toast(err);
        return;
      }
      state.robot.charging = false;
      showModal('remote');
      break;
    }
    case 'drive':
      moveRobot(b.dataset.dir);
      break;
    case 'remote-stop':
      toast('정지했어요. 방향 버튼을 누를 때만 움직여요.');
      break;
    case 'profile':
      showModal('profile');
      break;
    case 'home-settings':
      showModal('home');
      break;
    case 'wifi':
      showModal('wifi');
      break;
    case 'firmware':
      showModal('firmware');
      break;
    case 'check-firmware':
      toast('확인했어요. 체험 기기는 최신 소프트웨어예요.');
      break;
    case 'family':
      showModal('family');
      break;
    case 'remove-member':
      state.family.splice(Number(id), 1);
      persist();
      renderOverlay();
      toast('체험 멤버를 삭제했어요.');
      break;
    case 'help':
      showModal('help');
      break;
    case 'privacy':
      showModal('privacy');
      break;
    case 'notifications':
      showModal('notifications');
      break;
    case 'read-notifications':
      state.notifications.forEach((n) => (n.read = true));
      persist();
      closeModal();
      render();
      break;
    case 'setting-toggle':
      state.settings[id] = !state.settings[id];
      persist();
      render();
      toast('설정을 저장했어요.');
      break;
    case 'reset':
      showModal('reset');
      break;
    case 'confirm-reset':
      state = initialState();
      request = null;
      demo = null;
      selectedPlace = null;
      setupStep = 0;
      persist();
      closeModal();
      go('home');
      toast('처음의 체험 데이터로 돌아왔어요.');
      break;
  }
});
document.addEventListener('submit', (event) => {
  event.preventDefault();
  const form = event.target,
    f = new FormData(form);
  switch (form.id) {
    case 'request-form': {
      const parsed = parseRequest(String(f.get('prompt') || ''), state);
      if (parsed.error) {
        $('#request-error').textContent = parsed.error;
        $('#request-text').focus();
        return;
      }
      request = parsed;
      if (parsed.needsItem || parsed.needsDest) go('request');
      else prepareTask(parsed);
      break;
    }
    case 'setup-profile': {
      const name = String(f.get('name')).trim(),
        home = String(f.get('home')).trim();
      if (!name || !home) {
        toast('이름과 집 이름을 입력해 주세요.');
        return;
      }
      state.profile.name = name;
      state.profile.home = home;
      state.family[0].name = name;
      setupStep = 1;
      persist();
      render();
      break;
    }
    case 'setup-permissions':
      state.settings.notifications = f.get('notify') === 'on';
      setupStep = 2;
      persist();
      render();
      break;
    case 'setup-wifi':
      state.robot.connected = true;
      state.map.scan = 0;
      state.map.scanning = false;
      state.map.draft = false;
      setupStep = 4;
      persist();
      render();
      toast('예시 Wi-Fi에 연결했어요.');
      break;
    case 'device-form': {
      const name = String(f.get('name')).trim();
      if (!name) {
        toast('로봇 이름을 입력해 주세요.');
        return;
      }
      state.robot.name = name;
      persist();
      closeModal();
      render();
      toast('로봇 이름을 저장했어요.');
      break;
    }
    case 'profile-form': {
      const name = String(f.get('name')).trim();
      if (!name) {
        toast('이름을 입력해 주세요.');
        return;
      }
      state.profile.name = name;
      state.family[0].name = name;
      persist();
      closeModal();
      render();
      toast('프로필을 저장했어요.');
      break;
    }
    case 'home-form': {
      const name = String(f.get('name')).trim();
      if (!name) {
        toast('집 이름을 입력해 주세요.');
        return;
      }
      state.profile.home = name;
      state.map.name = name;
      persist();
      closeModal();
      render();
      toast('우리 집 이름을 저장했어요.');
      break;
    }
    case 'wifi-form':
      state.robot.connected = true;
      persist();
      closeModal();
      render();
      toast('예시 Wi-Fi에 연결했어요.');
      break;
    case 'furniture-form': {
      const name = String(f.get('name')).trim();
      if (!name) {
        toast('가구 이름을 입력해 주세요.');
        return;
      }
      const data = { name, room: String(f.get('room')), alias: String(f.get('alias')).trim() };
      const roomPlace = PLACES.find((p) => p.name === data.room);
      if (roomPlace) data.place = roomPlace.id;
      if (modalData.id)
        Object.assign(
          state.furniture.find((x) => x.id === modalData.id),
          data,
        );
      else
        state.furniture.push({
          id: 'f-' + Date.now(),
          icon: 'table',
          place: PLACES.find((p) => p.name === data.room)?.id || 'living',
          ...data,
        });
      persist();
      closeModal();
      render();
      toast('가구 이름과 별칭을 기억했어요.');
      break;
    }
    case 'routine-form': {
      captureRoutine();
      if (!routineDraft.days.length) {
        $('#days-error').textContent = '반복할 요일을 하나 이상 선택해 주세요.';
        $('#days-error').style.color = 'var(--danger)';
        return;
      }
      if (!routineDraft.name.trim()) {
        toast('루틴 이름을 입력해 주세요.');
        return;
      }
      routineDraft.name = routineDraft.name.trim();
      routineDraft.icon =
        routineDraft.kind === 'charge' ? 'moon' : routineDraft.item === 'book' ? 'book' : 'sun';
      if (routineDraft.id) {
        const idx = state.routines.findIndex((r) => r.id === routineDraft.id);
        state.routines[idx] = structuredClone(routineDraft);
      } else state.routines.push({ ...structuredClone(routineDraft), id: 'r-' + Date.now() });
      persist();
      closeModal();
      render();
      toast('새로운 일상 루틴을 저장했어요.');
      break;
    }
    case 'family-form': {
      const name = String(f.get('name')).trim();
      if (!name) {
        toast('멤버 이름을 입력해 주세요.');
        return;
      }
      if (state.family.some((x) => x.name === name)) {
        toast('이미 등록된 이름이에요.');
        return;
      }
      state.family.push({ name, role: '가족 · 체험 멤버' });
      persist();
      renderOverlay();
      toast('체험 멤버를 추가했어요.');
      break;
    }
    case 'feedback-form': {
      const feedback = String(f.get('feedback')).trim();
      if (!feedback) {
        toast('확인할 내용을 입력해 주세요.');
        return;
      }
      state.task.feedbackText = feedback;
      const h = state.history.find((x) => x.id === state.task.id);
      if (h) h.feedback = feedback;
      persist();
      closeModal();
      toast('확인할 내용을 기록에 저장했어요.');
      break;
    }
  }
});
document.addEventListener('change', (event) => {
  const el = event.target;
  if (el.id === 'observation-select') {
    selectedObservation = el.value || null;
    selectedPlace = OBSERVATIONS.find((o) => o.id === el.value)?.place || null;
    render();
  }
  if (el.id === 'robot-speed') {
    state.settings.speed = el.value;
    persist();
    toast('이동 속도 설정을 저장했어요.');
  }
  if (el.id === 'routine-kind') {
    captureRoutine();
    renderOverlay();
  }
});
document.addEventListener('input', (event) => {
  if (event.target.id === 'request-text') {
    draftPrompt = event.target.value;
    if (request) request.text = draftPrompt;
  }
  if (event.target.id === 'arrival-angle') $('#angle-value').textContent = event.target.value + '°';
});
document.addEventListener('keydown', (event) => {
  if (event.key === 'Escape') {
    if (modal) closeModal();
    else if (drawer) {
      drawer = false;
      renderOverlay();
    }
    return;
  }
  if (['Enter', ' '].includes(event.key) && event.target.matches('g[data-action]')) {
    event.preventDefault();
    event.target.dispatchEvent(new MouseEvent('click', { bubbles: true }));
  }
  if (event.key === 'Tab' && (modal || drawer)) {
    const dialog = $('#overlay-root [role="dialog"]'),
      els = [
        ...dialog.querySelectorAll(
          'button:not([disabled]),a[href],input,select,textarea,[tabindex="0"]',
        ),
      ].filter((el) => el.getClientRects().length);
    if (!els.length) return;
    const first = els[0],
      last = els[els.length - 1];
    if (
      event.shiftKey &&
      (document.activeElement === first || !dialog.contains(document.activeElement))
    ) {
      event.preventDefault();
      last.focus();
    } else if (
      !event.shiftKey &&
      (document.activeElement === last || !dialog.contains(document.activeElement))
    ) {
      event.preventDefault();
      first.focus();
    }
  }
});
window.addEventListener('composer-change', (event) => {
  composerExpanded = event.detail;
});
initNativeMotion();
initDeliveryMotion();
window.addEventListener('hashchange', () => {
  const next = location.hash.slice(1);
  if (names[next]) {
    page = next;
    render();
  }
});
let tick = 0;
setInterval(() => {
  tick++;
  let dirty = false,
    changedPage = false;
  if (tickTask(state)) {
    dirty = true;
    changedPage = page === 'task' || page === 'home';
    if (
      state.task.status === 'done' &&
      state.settings.notifications &&
      !(page === 'task' && state.task.kind === 'transport')
    )
      toast(`${state.task.title} 완료!`);
  }
  if (state.map.scanning) {
    state.map.scan = Math.min(100, state.map.scan + 7);
    if (state.map.scan === 100) {
      state.map.scanning = false;
      toast('우리 집을 모두 둘러봤어요. 지도를 저장해 주세요.');
    }
    dirty = true;
    changedPage = page === 'setup' || page === 'map';
  }
  if (state.map.localization === 'localizing') {
    state.map.localizeProgress = Math.min(100, state.map.localizeProgress + 25);
    if (state.map.localizeProgress === 100) {
      state.map.localization = 'ready';
      toast('예시 현재 위치를 지도에 맞췄어요.');
    }
    dirty = true;
    changedPage = page === 'map';
  }
  if (
    state.robot.charging &&
    state.robot.connected &&
    state.robot.battery < 100 &&
    tick % 3 === 0
  ) {
    state.robot.battery = Math.min(100, state.robot.battery + 1);
    dirty = true;
    changedPage = page === 'home';
  }
  if (dirty) {
    persist();
    const deliveryUpdated =
      page === 'task' && state.task?.kind === 'transport' && syncDeliveryScene(state);
    if (
      changedPage &&
      !deliveryUpdated &&
      !modal &&
      !drawer &&
      !isComposerDragging() &&
      !document.activeElement?.matches('textarea,input')
    )
      render();
    else if (modal === 'mapping') renderOverlay();
  }
}, 1000);
render();
