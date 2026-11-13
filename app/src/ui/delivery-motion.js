import { transportFrame } from '../transport.js';
import { deliveryCopy } from './screens/delivery.js';
import { icon } from '../art.js';

const FULL_MAP = { x: 360, y: 240, width: 360, height: 475 };
const reduceMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;
let scene = null;
let frameRequest = 0;

const action = (label, name, glyph, secondary = false) =>
  `<button class="btn ${secondary ? 'btn-outline' : 'btn-primary'}" data-action="${name}">${icon(glyph, 18)}${label}</button>`;

function controls(state, mode) {
  if (mode === 'done') {
    const primary =
      state.task.feedback === 'good'
        ? action('새로운 부탁', 'new-request', 'plus')
        : '<button class="btn btn-primary" data-action="feedback" data-value="good">' +
          icon('check', 18) +
          '잘 받았어요</button>';
    return (
      primary +
      '<button class="btn btn-outline" data-action="nav" data-page="activity">' +
      icon('activity', 18) +
      '기록 보기</button>'
    );
  }
  if (mode === 'cancelled') return action('새로운 부탁', 'new-request', 'plus');
  if (state.robot.emergency) return action('긴급 정지 해제', 'release-stop', 'shield');
  if (!state.robot.connected) return action('다시 연결하기', 'reconnect', 'wifi');
  return mode === 'running'
    ? action('잠시 멈추기', 'pause-task', 'pause') + action('긴급 정지', 'emergency', 'stop', true)
    : action('이어서 진행', 'resume-task', 'play') +
        action('부탁 취소', 'cancel-task', 'close', true);
}

function modeFor(task, frame) {
  if (['paused', 'blocked', 'cancelled'].includes(task.status)) return task.status;
  return task.status === 'done' && frame.progress >= 100 ? 'done' : 'running';
}

function overviewFor(task) {
  const points = [...(task.sourceRoute || []), ...(task.route || [])];
  if (!points.length) return { ...FULL_MAP };
  const xs = points.map((point) => point.x),
    ys = points.map((point) => point.y);
  const left = Math.min(...xs),
    right = Math.max(...xs);
  const top = Math.min(...ys),
    bottom = Math.max(...ys);
  const width = Math.max(260, right - left + 130);
  const height = Math.max(330, bottom - top + 140);
  return {
    x: (left + right) / 2 - width / 2 + 8,
    y: (top + bottom) / 2 - height / 2 - 12,
    width,
    height,
  };
}

function cameraFor(frame) {
  if (!scene.follow || reduceMotion()) return FULL_MAP;
  const focus = ['checking', 'picking'].includes(frame.phase)
    ? frame.source
    : ['placing', 'delivered'].includes(frame.phase)
      ? frame.destination
      : null;
  return focus ? { x: focus.x - 86, y: focus.y - 126, width: 190, height: 235 } : scene.overview;
}

function setText(element, text) {
  if (element.textContent !== text) element.textContent = text;
}

function syncCameraControl() {
  const button = scene.root.querySelector('[data-delivery-camera]');
  button.setAttribute('aria-pressed', String(!scene.follow));
  button.setAttribute('aria-label', scene.follow ? '전체 지도 보기' : '로미 따라가기');
  setText(
    scene.root.querySelector('.delivery-camera-label'),
    scene.follow ? '움직임을 따라가는 지도' : '우리 집 전체 지도',
  );
}

function paintStatus(frame) {
  const { root, refs, state } = scene;
  const task = state.task,
    mode = modeFor(task, frame);
  root.dataset.mode = mode;
  root.dataset.phase = frame.phase;
  root.dataset.cargo = frame.attached ? 'carried' : frame.released ? 'placed' : 'waiting';
  const emergencyButton = document.querySelector('.app-header .emergency-pill');
  if (emergencyButton)
    emergencyButton.style.display = ['done', 'cancelled'].includes(mode) ? 'none' : '';
  const signature = [mode, frame.phase, task.reason].join('|');
  if (signature !== scene.copyKey) {
    const [title, detail] = deliveryCopy(task, frame);
    setText(refs.title, title);
    setText(refs.detail, detail);
    if (scene.copyKey && !reduceMotion())
      refs.copy.animate(
        [
          { opacity: 0.25, translate: '0 7px' },
          { opacity: 1, translate: '0 0' },
        ],
        { duration: 360, easing: 'cubic-bezier(.2,.9,.3,1)' },
      );
    scene.copyKey = signature;
  }
  const controlKey = [mode, state.robot.connected, state.robot.emergency, task.feedback].join('|');
  if (controlKey !== scene.controlKey) {
    refs.actions.innerHTML = controls(state, mode);
    scene.controlKey = controlKey;
  }
  refs.options.hidden = mode === 'done' || mode === 'cancelled';
  refs.feedback.hidden = mode !== 'done';
  refs.missingExample.hidden = frame.progress >= 52;
  const paused = mode === 'paused' || mode === 'blocked';
  setText(
    refs.time,
    mode === 'done'
      ? '전달 완료'
      : mode === 'cancelled'
        ? '중단됨'
        : paused
          ? '일시 정지'
          : `약 ${Math.max(1, Math.ceil((100 - frame.progress) / 4))}초 남음`,
  );
  setText(
    refs.cargoStatus,
    mode === 'cancelled'
      ? '중단됨'
      : frame.released
        ? '내려놓음'
        : frame.attached
          ? '운반 중'
          : frame.phase === 'picking'
            ? '집는 중'
            : '찾는 중',
  );
  refs.milestones.forEach((element, index) => {
    const complete = frame.segments[index] >= 1;
    element.classList.toggle('is-complete', complete);
    setText(element.querySelector('.delivery-step-label b'), complete ? '✓' : String(index + 1));
    element.classList.toggle('is-current', index === frame.milestone && mode !== 'done');
    if (index === frame.milestone && mode !== 'done') element.setAttribute('aria-current', 'step');
    else element.removeAttribute('aria-current');
    element.querySelector('.delivery-segment span').style.transform =
      `scaleX(${frame.segments[index]})`;
  });
}

function paint(now) {
  frameRequest = 0;
  if (!scene || !scene.root.isConnected) return;
  const { refs, state } = scene;
  const task = state.task;
  const moving = task.status === 'running' || task.status === 'done';
  const fraction = reduceMotion() ? 1 : Math.min(1, (now - scene.startedAt) / 880);
  if (moving)
    scene.progress = scene.fromProgress + (scene.toProgress - scene.fromProgress) * fraction;
  const frame = transportFrame(task, scene.progress);
  const robot = frame.robot,
    cargo = frame.cargo;
  refs.robot.setAttribute('transform', `translate(${robot.x} ${robot.y})`);
  refs.cargo.setAttribute(
    'transform',
    `translate(${cargo.x} ${cargo.y}) scale(${frame.cargoScale})`,
  );
  refs.grip.setAttribute('transform', `translate(${cargo.x} ${cargo.y})`);
  refs.grip.style.opacity = frame.gripping ? '1' : '0';
  refs.gripLeft.setAttribute('transform', `translate(${-frame.gripperOpen * 8} 0)`);
  refs.gripRight.setAttribute('transform', `translate(${frame.gripperOpen * 8} 0)`);
  refs.sourceRoute.style.strokeDasharray = `${frame.approach * 100} 100`;
  refs.carryRoute.style.strokeDasharray = `${frame.carry * 100} 100`;
  refs.sourceRoute.style.opacity = frame.progress >= 52 ? '.25' : '1';
  if (scene.lastRobot) {
    const dx = robot.x - scene.lastRobot.x,
      dy = robot.y - scene.lastRobot.y;
    if (Math.hypot(dx, dy) > 0.015)
      refs.heading.setAttribute(
        'transform',
        `rotate(${(Math.atan2(dy, dx) * 180) / Math.PI + 90})`,
      );
  }
  scene.lastRobot = robot;

  const target = cameraFor(frame);
  const cameraMoving = moving || scene.cameraChanged;
  let cameraDistance = 0;
  for (const key of ['x', 'y', 'width', 'height']) {
    const difference = target[key] - scene.camera[key];
    cameraDistance += Math.abs(difference);
    if (cameraMoving) scene.camera[key] += difference * (reduceMotion() ? 1 : 0.075);
  }
  refs.map.setAttribute(
    'viewBox',
    `${scene.camera.x} ${scene.camera.y} ${scene.camera.width} ${scene.camera.height}`,
  );
  paintStatus(frame);
  if (cameraDistance < 0.15) scene.cameraChanged = false;
  if (
    (!reduceMotion() &&
      moving &&
      (task.status === 'running' || fraction < 1 || cameraDistance > 0.15)) ||
    scene.cameraChanged
  ) {
    frameRequest = requestAnimationFrame(paint);
  }
}

/** Called after a screen render and on state ticks; the map DOM survives ticks. */
export function syncDeliveryScene(state) {
  const root = document.querySelector('[data-delivery-task]');
  if (!root || state.task?.kind !== 'transport') {
    cancelAnimationFrame(frameRequest);
    frameRequest = 0;
    scene = null;
    return false;
  }
  const sameTask = scene?.taskId === state.task.id;
  const existing = sameTask ? scene : null;
  if (!scene || scene.root !== root) {
    cancelAnimationFrame(frameRequest);
    const find = (selector) => root.querySelector(selector);
    scene = {
      root,
      taskId: state.task.id,
      progress: existing?.progress ?? state.task.progress,
      camera: existing?.camera ?? overviewFor(state.task),
      overview: overviewFor(state.task),
      follow: existing?.follow ?? true,
      cameraChanged: false,
      copyKey: '',
      controlKey: '',
      refs: {
        map: find('.floorplan'),
        robot: find('[data-delivery-robot]'),
        heading: find('[data-delivery-robot] > path'),
        cargo: find('[data-delivery-cargo]'),
        grip: find('[data-delivery-grip]'),
        gripLeft: find('[data-grip-left]'),
        gripRight: find('[data-grip-right]'),
        sourceRoute: find('[data-source-route]'),
        carryRoute: find('[data-carry-route]'),
        title: find('[data-delivery-title]'),
        detail: find('[data-delivery-detail]'),
        copy: find('.delivery-status-copy'),
        time: find('[data-delivery-time]'),
        cargoStatus: find('[data-cargo-status]'),
        actions: find('[data-delivery-actions]'),
        milestones: [...root.querySelectorAll('[data-milestone]')],
        options: find('.delivery-options'),
        feedback: find('[data-delivery-feedback]'),
        missingExample: find('[data-missing-example]'),
      },
    };
  }
  const object = state.items.find((item) => item.id === state.task.itemId);
  scene.state = {
    robot: { ...state.robot },
    task: { ...state.task, itemName: state.task.itemName || object?.name || '물건' },
  };
  scene.fromProgress = scene.progress;
  scene.toProgress = state.task.progress;
  scene.startedAt = performance.now();
  syncCameraControl();
  cancelAnimationFrame(frameRequest);
  paint(scene.startedAt);
  return true;
}

export function initDeliveryMotion() {
  document.addEventListener('click', (event) => {
    const button = event.target.closest('[data-delivery-camera]');
    if (!button || !scene) return;
    scene.follow = !scene.follow;
    scene.cameraChanged = true;
    syncCameraControl();
    cancelAnimationFrame(frameRequest);
    paint(performance.now());
  });
}
