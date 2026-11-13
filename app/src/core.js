import { OBSERVATIONS, insideMap } from './slam.js';
import { isBlocked, findRoute, routePoint } from './navigation.js';
import { transportFrame } from './transport.js';
export { isBlocked, findRoute, routePoint } from './navigation.js';
export const KEY = 'hometekhu-life-v1';
export const escapeHTML = (value) =>
  String(value ?? '').replace(
    /[&<>"']/g,
    (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c],
  );
export const PLACES = [
  { id: 'living', name: '거실', x: 563, y: 409 },
  { id: 'bedroom', name: '침실', x: 505, y: 568 },
  { id: 'study', name: '작업 공간', x: 561, y: 471 },
  { id: 'kitchen', name: '주방', x: 510, y: 360 },
  { id: 'dock', name: '충전 스테이션', x: 455, y: 620 },
];
export function toward(word) {
  const c = word.charCodeAt(word.length - 1);
  const final = (c - 44032) % 28;
  return word + (c >= 44032 && c <= 55203 && final !== 0 && final !== 8 ? '으로' : '로');
}
export const ITEMS = [
  {
    id: 'black-cup',
    name: '검은 머그컵',
    icon: 'cup',
    place: 'living',
    surface: '거실 테이블',
    color: 'black',
  },
  {
    id: 'white-cup',
    name: '흰 머그컵',
    icon: 'cup',
    place: 'kitchen',
    surface: '주방 조리대',
    color: 'white',
  },
  { id: 'water', name: '물병', icon: 'bottle', place: 'kitchen', surface: '주방 조리대' },
  { id: 'remote', name: '리모컨', icon: 'remote', place: 'living', surface: '소파 옆 테이블' },
  { id: 'book', name: '읽던 책', icon: 'book', place: 'study', surface: '작업 책상' },
];
export function initialState() {
  return {
    version: 1,
    profile: { name: '지우', home: '우리 집' },
    robot: {
      name: '로미',
      connected: true,
      battery: 86,
      room: '거실',
      position: { x: 561, y: 471 },
      charging: false,
      emergency: false,
    },
    map: {
      reference: 'slam-reference-v1',
      localization: 'ready',
      localizeProgress: 100,
      ready: true,
      name: '우리 집',
      zone: false,
      scan: 0,
      scanning: false,
      draft: false,
      savedAt: '오늘 09:12',
    },
    furniture: OBSERVATIONS.map((o) => ({ ...o })),
    items: ITEMS.map((i) => ({ ...i })),
    routines: [
      {
        id: 'morning',
        name: '가볍게 시작하는 아침',
        time: '08:00',
        days: [1, 2, 3, 4, 5],
        item: 'water',
        dest: 'study',
        enabled: true,
        icon: 'sun',
      },
      {
        id: 'evening',
        name: '나를 위한 독서 시간',
        time: '21:00',
        days: [0, 1, 2, 3, 4, 5, 6],
        item: 'book',
        dest: 'bedroom',
        enabled: true,
        icon: 'book',
      },
      {
        id: 'night',
        name: '로미도 쉬어갈 시간',
        time: '23:00',
        days: [0, 1, 2, 3, 4, 5, 6],
        kind: 'charge',
        enabled: false,
        icon: 'moon',
      },
    ],
    history: [
      {
        id: 'sample1',
        title: '물병을 책상으로 옮겼어요',
        subtitle: '주방 → 서재',
        status: 'done',
        time: '오늘 10:24',
        duration: '1분 42초',
        icon: 'bottle',
        kind: 'transport',
        itemId: 'water',
        dest: 'study',
      },
      {
        id: 'sample2',
        title: '우리 집을 한 바퀴 둘러봤어요',
        subtitle: '거실 · 주방 · 침실 · 서재',
        status: 'done',
        time: '오늘 09:30',
        duration: '3분 18초',
        icon: 'scan',
        kind: 'patrol',
      },
      {
        id: 'sample3',
        title: '우리 집 지도가 업데이트됐어요',
        subtitle: '4개 공간 · 가구 12개',
        status: 'done',
        time: '오늘 09:12',
        duration: '2분 06초',
        icon: 'map',
        kind: 'map',
      },
    ],
    notifications: [
      {
        id: 'welcome',
        title: '로미가 함께할 준비를 마쳤어요',
        text: '첫 부탁을 건네보세요. 우리 집의 일상이 조금 더 편해져요.',
        read: false,
      },
    ],
    settings: {
      mapView: 'home',
      notifications: true,
      sound: true,
      photos: false,
      avoidPets: true,
      speed: '보통',
      firmware: '1.4.2',
    },
    family: [{ name: '지우', role: '소유자' }],
    task: null,
    setupComplete: true,
  };
}
export function loadState(storage) {
  try {
    const raw = JSON.parse(storage.getItem(KEY));
    if (!raw || raw.version !== 1) return initialState();
    const base = initialState();
    const s = {
      ...base,
      ...raw,
      robot: { ...base.robot, ...raw.robot },
      map: { ...base.map, ...raw.map },
      settings: { ...base.settings, ...raw.settings },
    };
    if (s.task?.status === 'running') {
      s.task.status = 'paused';
      s.task.reason = '앱을 다시 열었어요. 주변을 확인하고 이어서 진행하세요.';
    }
    s.map.scanning = false;
    if (!insideMap(s.robot.position.x, s.robot.position.y)) s.robot.position = { x: 561, y: 471 };
    if (!raw.map?.reference) {
      s.map.reference = 'slam-reference-v1';
      s.furniture = OBSERVATIONS.map((o) => ({ ...o }));
      s.task = null;
    }
    if (s.map.localization === 'localizing') s.map.localization = 'unconfirmed';
    return s;
  } catch {
    return initialState();
  }
}
export function saveState(storage, state) {
  try {
    storage.setItem(KEY, JSON.stringify(state));
    return true;
  } catch {
    return false;
  }
}
export function parseRequest(text, state) {
  const t = text.trim();
  if (!t) return { error: '로미에게 부탁할 내용을 입력해 주세요.' };
  let kind = 'transport';
  if (/충전|스테이션|쉬어/.test(t)) kind = 'charge';
  else if (/둘러|순찰|살펴/.test(t)) kind = 'patrol';
  else if (!/옮겨|가져|갖다|놓아|물병|머그|컵|리모컨|책/.test(t) && /와|이동|가줘/.test(t))
    kind = 'navigate';
  let item;
  if (/검은|검정|블랙/.test(t) && /컵/.test(t))
    item = state.items.find((i) => i.id === 'black-cup');
  else if (/흰|하얀|화이트/.test(t) && /컵/.test(t))
    item = state.items.find((i) => i.id === 'white-cup');
  else if (/물병|생수/.test(t)) item = state.items.find((i) => i.id === 'water');
  else if (/리모컨/.test(t)) item = state.items.find((i) => i.id === 'remote');
  else if (/읽던|책을|책 좀|책 가져/.test(t)) item = state.items.find((i) => i.id === 'book');
  let dest = null;
  let last = -1;
  for (const p of PLACES.filter((p) => p.id !== 'dock')) {
    const idx = t.lastIndexOf(p.name);
    if (idx > last) {
      last = idx;
      dest = p.id;
    }
  }
  if (/협탁|침대/.test(t)) dest = 'bedroom';
  if (/서재|작업 공간/.test(t)) dest = 'study';
  if (/책상|작업대/.test(t) && t.lastIndexOf('책상') > last) dest = 'study';
  if (/소파/.test(t) && t.lastIndexOf('소파') > last) dest = 'living';
  if (/조리대/.test(t) && t.lastIndexOf('조리대') > last) dest = 'kitchen';
  for (const f of state.furniture) {
    for (const alias of f.alias
      .split(',')
      .map((s) => s.trim())
      .filter(Boolean)) {
      const idx = t.lastIndexOf(alias);
      if (idx > last) {
        last = idx;
        dest = f.place || PLACES.find((p) => p.name === f.room)?.id || dest;
      }
    }
  }
  if (kind === 'charge') dest = 'dock';
  if (kind === 'patrol') dest = 'living';
  return {
    text: t,
    kind,
    itemId: item?.id || null,
    dest,
    needsItem: kind === 'transport' && !item,
    needsDest: kind !== 'charge' && kind !== 'patrol' && !dest,
    ambiguous: /컵/.test(t) && !item,
  };
}
export function taskGuard(state, kind = 'transport') {
  if (state.robot.emergency) return '긴급 정지를 해제한 뒤 다시 시작해 주세요.';
  if (!state.robot.connected) return '로봇 연결을 먼저 확인해 주세요.';
  if (state.task && ['running', 'paused', 'blocked'].includes(state.task.status))
    return '진행 중인 작업을 마치거나 취소한 뒤 시작해 주세요.';
  if (state.map.localization !== 'ready' && kind !== 'charge')
    return '저장된 지도에서 로봇의 현재 위치를 먼저 맞춰 주세요.';
  if (state.map.scanning) return '지도 만들기를 마친 뒤 시작해 주세요.';
  if (!state.map.ready && kind !== 'charge') return '우리 집 지도를 먼저 만들어 주세요.';
  if (state.robot.battery <= 15 && kind !== 'charge')
    return '배터리가 부족해요. 먼저 충전해 주세요.';
  return null;
}
export function makeTask(request, state) {
  const item = state.items.find((i) => i.id === request.itemId);
  const target = {
    ...(PLACES.find((p) => p.id === request.dest) || PLACES[0]),
    ...(request.stopTarget || {}),
  };
  const kind = request.kind || 'transport';
  const title =
    kind === 'charge'
      ? '충전 스테이션으로 돌아가기'
      : kind === 'patrol'
        ? '우리 집 한 바퀴 둘러보기'
        : kind === 'navigate'
          ? `${request.stopLabel ? request.stopLabel + '으로' : toward(target.name)} 이동하기`
          : `${item?.name || '물건'} 옮기기`;
  const stages =
    kind === 'transport'
      ? [
          '이동 경로 확인',
          '물건이 있는 곳으로 이동',
          '물건 다시 확인',
          '안전하게 집기',
          `${toward(target.name)} 이동`,
          '내려놓고 결과 확인',
        ]
      : kind === 'charge'
        ? ['충전 경로 확인', '스테이션으로 이동', '충전 시작']
        : kind === 'patrol'
          ? ['거실 확인', '주방 확인', '침실 확인', '서재 확인']
          : ['이동 경로 확인', `${toward(target.name)} 이동`, '목적지 도착'];
  const sourcePlace = PLACES.find((p) => p.id === item?.place) || PLACES[0];
  const sourceRoute = findRoute(state.robot.position, sourcePlace, state.map.zone);
  let route = findRoute(
    kind === 'transport' ? sourcePlace : state.robot.position,
    target,
    state.map.zone,
  );
  if (kind === 'patrol') {
    let from = state.robot.position;
    route = [];
    for (const p of [PLACES[0], PLACES[3], PLACES[1], PLACES[2], PLACES[0]]) {
      route.push(...findRoute(from, p, state.map.zone));
      from = p;
    }
  }
  return {
    id: 'task-' + Date.now(),
    stopTarget: request.stopTarget || null,
    angle: Number.isFinite(Number(request.angle)) ? Number(request.angle) % 360 : 0,
    route,
    sourceRoute,
    title,
    kind,
    itemId: request.itemId,
    dest: target.id,
    destinationLabel: request.stopLabel || target.name,
    source: item?.surface || state.robot.room,
    itemName: item?.name || '물건',
    itemIcon: item?.icon || 'box',
    status: 'running',
    progress: 0,
    stage: 0,
    stages,
    reason: '',
    createdAt: Date.now(),
    obstacle: false,
    feedback: null,
  };
}
export function tickTask(state) {
  const t = state.task;
  if (!t || t.status !== 'running' || !state.robot.connected || state.robot.emergency) return false;
  t.progress = Math.min(100, t.progress + 4);
  const transport = t.kind === 'transport' ? transportFrame(t) : null;
  t.stage = transport
    ? transport.stage
    : Math.min(t.stages.length - 1, Math.floor((t.progress / 100) * t.stages.length));
  const dest = PLACES.find((p) => p.id === t.dest) || PLACES[0];
  const p = t.progress;
  state.robot.position = transport
    ? transport.robot
    : routePoint(t.route, t.kind === 'patrol' ? p / 100 : (p - 20) / 60);
  if (t.progress === 100) {
    t.status = 'done';
    state.robot.room = dest.name;
    state.robot.charging = t.kind === 'charge';
    state.robot.battery = Math.max(0, state.robot.battery - 2);
    if (t.itemId) {
      const i = state.items.find((i) => i.id === t.itemId);
      if (i) {
        i.place = t.dest;
        i.surface =
          dest.name === '침실'
            ? '침실 협탁'
            : t.dest === 'study'
              ? '작업 책상'
              : dest.name === '주방'
                ? '주방 조리대'
                : '거실 테이블';
      }
    }
    const time = new Intl.DateTimeFormat('ko-KR', {
      hour: '2-digit',
      minute: '2-digit',
      hour12: false,
      timeZone: 'Asia/Seoul',
    }).format(new Date());
    state.history.unshift({
      id: t.id,
      title: t.title + ' 완료',
      subtitle:
        t.kind === 'transport' ? `${t.source} → ${dest.name}` : t.destinationLabel || dest.name,
      status: 'done',
      time: '오늘 ' + time,
      duration: '25초 · 체험',
      icon: t.kind === 'charge' ? 'battery' : t.kind === 'patrol' ? 'scan' : 'check',
      kind: t.kind,
      itemId: t.itemId,
      dest: t.dest,
      stopTarget: t.stopTarget,
      stopLabel: t.destinationLabel,
      angle: t.angle,
    });
    state.notifications.unshift({
      id: t.id,
      title: t.title + ' 완료',
      text: '활동 기록에서 결과를 확인할 수 있어요.',
      read: false,
    });
  }
  return true;
}
export function cancelTask(state, reason = '사용자가 취소했어요') {
  if (!state.task || !['running', 'paused', 'blocked'].includes(state.task.status)) return false;
  const t = state.task;
  t.status = 'cancelled';
  t.reason = reason;
  state.history.unshift({
    id: t.id,
    title: t.title,
    subtitle: reason,
    status: 'cancelled',
    time: '방금',
    duration: '중단됨',
    icon: 'stop',
    kind: t.kind,
    itemId: t.itemId,
    dest: t.dest,
    stopTarget: t.stopTarget,
    stopLabel: t.destinationLabel,
    angle: t.angle,
  });
  return true;
}
export function dayLabel(days) {
  if (days.length === 7) return '매일';
  if (days.join(',') === '1,2,3,4,5') return '평일';
  if (days.length === 2 && days.includes(0) && days.includes(6)) return '주말';
  return days.map((d) => ['일', '월', '화', '수', '목', '금', '토'][d]).join(' · ');
}
