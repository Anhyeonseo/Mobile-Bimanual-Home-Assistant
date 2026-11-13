import test from 'node:test';
import assert from 'node:assert/strict';
import {
  initialState,
  makeTask,
  tickTask,
  cancelTask,
  loadState,
  saveState,
} from '../src/core.js';
import { transportFrame } from '../src/transport.js';

const delivery = () => {
  const state = initialState();
  state.task = makeTask({ kind: 'transport', itemId: 'remote', dest: 'bedroom' }, state);
  return state;
};
const near = (a, b) => assert.ok(Math.hypot(a.x - b.x, a.y - b.y) < 0.001);

test('arrival, inspection and pickup share one stationary source position', () => {
  const { task } = delivery();
  const first = transportFrame(task, 8),
    arrived = transportFrame(task, 32);
  assert.notDeepEqual(first.robot, arrived.robot);
  for (const p of [32, 36, 40, 44, 48, 52]) near(transportFrame(task, p).robot, arrived.source);
  assert.equal(transportFrame(task, 36).phase, 'checking');
  assert.equal(transportFrame(task, 44).phase, 'picking');
  assert.equal(transportFrame(task, 52).phase, 'carrying');
});

test('the remote stays on its surface, attaches to the robot, then detaches at arrival', () => {
  const { task } = delivery();
  const start = transportFrame(task, 0);
  near(transportFrame(task, 39).cargo, start.sourceItem);
  for (const p of [52, 64, 80]) {
    const frame = transportFrame(task, p);
    assert.equal(frame.attached, true);
    near(frame.cargo, { x: frame.robot.x + 18, y: frame.robot.y - 20 });
  }
  const released = transportFrame(task, 96);
  assert.equal(released.attached, false);
  assert.equal(released.released, true);
  near(released.cargo, released.destinationItem);
  assert.equal(transportFrame(task, 100).phase, 'delivered');
});

test('pausing and restoring during pickup preserve the handoff instead of replaying it', () => {
  const state = delivery();
  for (let count = 0; count < 11; count++) tickTask(state);
  state.task.status = 'paused';
  const before = transportFrame(state.task);
  for (let count = 0; count < 5; count++) assert.equal(tickTask(state), false);
  let raw;
  const storage = { setItem: (_, value) => (raw = value), getItem: () => raw };
  saveState(storage, state);
  const restored = loadState(storage);
  assert.deepEqual(transportFrame(restored.task), before);
  restored.task.status = 'running';
  tickTask(restored);
  assert.equal(restored.task.progress, 48);
  assert.equal(transportFrame(restored.task).phase, 'picking');
});

test('cancelling while carrying never creates a delivered item or completion record', () => {
  const state = delivery();
  for (let count = 0; count < 18; count++) tickTask(state);
  const before = transportFrame(state.task);
  assert.equal(before.attached, true);
  cancelTask(state);
  for (let count = 0; count < 10; count++) tickTask(state);
  near(transportFrame(state.task).cargo, before.cargo);
  assert.equal(state.items.find((item) => item.id === 'remote').place, 'living');
  assert.equal(state.history[0].status, 'cancelled');
});

test('delivery completion updates memory once while preserving the original pickup anchor', () => {
  const state = delivery(),
    source = transportFrame(state.task).source;
  for (let count = 0; count < 25; count++) tickTask(state);
  near(transportFrame(state.task).source, source);
  near(state.robot.position, transportFrame(state.task).destination);
  assert.equal(state.items.find((item) => item.id === 'remote').surface, '침실 협탁');
  tickTask(state);
  assert.equal(state.history.filter((entry) => entry.id === state.task.id).length, 1);
});
