import test from 'node:test';
import assert from 'node:assert/strict';
import {
  initialState,
  parseRequest,
  makeTask,
  tickTask,
  cancelTask,
  taskGuard,
  PLACES,
  findRoute,
  isBlocked,
  loadState,
  saveState,
  toward,
  escapeHTML,
} from '../src/core.js';

test('an ambiguous cup requires an explicit object choice', () => {
  const q = parseRequest('컵을 침실로 가져다줘', initialState());
  assert.equal(q.needsItem, true);
  assert.equal(q.ambiguous, true);
  assert.equal(q.dest, 'bedroom');
});
test('specific item, source and destination produce the intended request', () => {
  const q = parseRequest('거실 테이블에 있는 검은 머그컵을 침실 협탁으로 옮겨줘', initialState());
  assert.equal(q.kind, 'transport');
  assert.equal(q.itemId, 'black-cup');
  assert.equal(q.dest, 'bedroom');
  assert.equal(q.needsDest, false);
});
test('saved household aliases participate in destination resolution', () => {
  const s = initialState();
  s.furniture.find((f) => f.id === 'desk').alias = '집중하는 곳';
  assert.equal(parseRequest('물병을 집중하는 곳으로 옮겨줘', s).dest, 'study');
});
test('unsupported and empty requests are not executed without clarification', () => {
  assert.ok(parseRequest(' ', initialState()).error);
  const q = parseRequest('저녁을 요리해줘', initialState());
  assert.equal(q.needsItem, true);
  assert.equal(q.needsDest, true);
});
test('every room can be reached while avoiding mapped obstacles and no-go areas', () => {
  for (const zone of [false, true])
    for (const a of PLACES)
      for (const b of PLACES) {
        const route = findRoute(a, b, zone);
        assert.ok(route.length, `${a.id} → ${b.id}, zone ${zone}`);
        for (const p of route) assert.equal(isBlocked(p.x, p.y, zone), false, JSON.stringify(p));
        for (let j = 1; j < route.length; j++)
          assert.ok(
            Math.abs(route[j].x - route[j - 1].x) + Math.abs(route[j].y - route[j - 1].y) <= 8,
          );
      }
});
test('transport completion updates object memory and history exactly once', () => {
  const s = initialState();
  s.task = makeTask({ kind: 'transport', itemId: 'black-cup', dest: 'bedroom' }, s);
  for (let j = 0; j < 40; j++) tickTask(s);
  assert.equal(s.task.status, 'done');
  assert.equal(s.items[0].place, 'bedroom');
  assert.equal(s.items[0].surface, '침실 협탁');
  assert.equal(s.history.filter((h) => h.id === s.task.id).length, 1);
  assert.equal(s.robot.room, '침실');
});
test('pause, offline and emergency stop prevent movement', () => {
  for (const flag of ['pause', 'offline', 'emergency']) {
    const s = initialState();
    s.task = makeTask({ kind: 'navigate', dest: 'study' }, s);
    if (flag === 'pause') s.task.status = 'paused';
    if (flag === 'offline') s.robot.connected = false;
    if (flag === 'emergency') s.robot.emergency = true;
    const before = structuredClone(s.robot.position);
    tickTask(s);
    assert.equal(s.task.progress, 0);
    assert.deepEqual(s.robot.position, before);
  }
});
test('cancelled jobs preserve object position and log a single cancellation', () => {
  const s = initialState();
  s.task = makeTask({ kind: 'transport', itemId: 'black-cup', dest: 'bedroom' }, s);
  assert.equal(cancelTask(s), true);
  assert.equal(cancelTask(s), false);
  tickTask(s);
  assert.equal(s.items[0].place, 'living');
  assert.equal(s.history[0].status, 'cancelled');
});
test('busy, offline, emergency, scanning and low-battery starts are guarded', () => {
  for (const setup of [
    (s) => (s.robot.connected = false),
    (s) => (s.robot.emergency = true),
    (s) => (s.robot.battery = 8),
    (s) => (s.map.scanning = true),
    (s) => (s.map.ready = false),
    (s) => (s.task = makeTask({ kind: 'navigate', dest: 'study' }, s)),
  ]) {
    const s = initialState();
    setup(s);
    assert.ok(taskGuard(s));
  }
  const s = initialState();
  s.robot.battery = 8;
  assert.equal(taskGuard(s, 'charge'), null);
});
test('charging finishes at the station and changes device state', () => {
  const s = initialState();
  s.task = makeTask({ kind: 'charge', dest: 'dock' }, s);
  for (let j = 0; j < 25; j++) tickTask(s);
  assert.equal(s.robot.charging, true);
  assert.equal(s.robot.room, '충전 스테이션');
});
test('restoring state preserves preferences but pauses moving work', () => {
  const s = initialState();
  s.profile.name = '민서';
  s.task = makeTask({ kind: 'navigate', dest: 'study' }, s);
  tickTask(s);
  let raw;
  const storage = { setItem: (k, v) => (raw = v), getItem: () => raw };
  assert.equal(saveState(storage, s), true);
  const restored = loadState(storage);
  assert.equal(restored.profile.name, '민서');
  assert.equal(restored.task.status, 'paused');
  assert.equal(restored.task.progress, 4);
  assert.equal(loadState({ getItem: () => '{broken' }).version, 1);
});
test('Korean direction labels and user strings render safely', () => {
  assert.equal(toward('침실'), '침실로');
  assert.equal(toward('주방'), '주방으로');
  assert.equal(toward('서재'), '서재로');
  assert.equal(escapeHTML('<img onerror="bad">'), '&lt;img onerror=&quot;bad&quot;&gt;');
});

test('SLAM localization must finish before navigation and survives reload safely', () => {
  const s = initialState();
  for (const status of ['unconfirmed', 'localizing']) {
    s.map.localization = status;
    assert.ok(taskGuard(s, 'navigate'));
  }
  let raw;
  const storage = { setItem: (k, v) => (raw = v), getItem: () => raw };
  saveState(storage, s);
  assert.equal(loadState(storage).map.localization, 'unconfirmed');
  s.map.localization = 'ready';
  assert.equal(taskGuard(s, 'navigate'), null);
});

test('SLAM approach point and arrival angle remain available in completed history', () => {
  const s = initialState();
  const stop = { x: 625, y: 398 };
  s.task = makeTask(
    { kind: 'navigate', dest: 'living', stopTarget: stop, stopLabel: '소파 앞', angle: '90' },
    s,
  );
  for (let j = 0; j < 25; j++) tickTask(s);
  assert.equal(s.task.status, 'done');
  assert.equal(s.task.angle, 90);
  assert.ok(Math.hypot(s.robot.position.x - stop.x, s.robot.position.y - stop.y) < 5);
  assert.equal(s.history[0].stopLabel, '소파 앞');
  assert.deepEqual(s.history[0].stopTarget, stop);
  assert.equal(s.history[0].angle, 90);
});
