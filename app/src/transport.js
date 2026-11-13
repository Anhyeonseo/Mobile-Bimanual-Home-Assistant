import { routePoint } from './navigation.js';

// One timeline drives simulation state, map motion and the object handoff.
// Progress is still persisted by core.js; rendering never advances a task.
export const TRANSPORT_PHASES = [
  { id: 'planning', from: 0, to: 8 },
  { id: 'approaching', from: 8, to: 32 },
  { id: 'checking', from: 32, to: 40 },
  { id: 'picking', from: 40, to: 52 },
  { id: 'carrying', from: 52, to: 84 },
  { id: 'placing', from: 84, to: 100 },
];

const clamp = (value) => Math.max(0, Math.min(1, value));
const between = (progress, from, to) => clamp((progress - from) / (to - from));
const ease = (value) => value * value * (3 - 2 * value);
const offset = (point, x, y) => ({ x: point.x + x, y: point.y + y });
const mix = (a, b, fraction) => ({
  x: a.x + (b.x - a.x) * fraction,
  y: a.y + (b.y - a.y) * fraction,
});

export function transportFrame(task, progress = task.progress) {
  const p = Math.max(0, Math.min(100, progress));
  const stage = Math.max(
    0,
    TRANSPORT_PHASES.findLastIndex((phase) => p >= phase.from),
  );
  const phase = p >= 100 ? 'delivered' : TRANSPORT_PHASES[stage].id;
  const source = routePoint(task.sourceRoute, 1);
  const destination = routePoint(task.route, 1);
  const approach = between(p, 8, 32);
  const carry = between(p, 52, 84);
  const robot = p < 52 ? routePoint(task.sourceRoute, approach) : routePoint(task.route, carry);
  const sourceItem = offset(source, 28, -30);
  const destinationItem = offset(destination, 28, -30);
  const carriedItem = offset(robot, 18, -20);
  const pickup = ease(between(p, 40, 52));
  const putdown = ease(between(p, 84, 96));
  let cargo = sourceItem;
  let cargoScale = 1;

  if (p >= 40 && p < 52) {
    cargo = mix(sourceItem, carriedItem, pickup);
    cargo.y -= Math.sin(pickup * Math.PI) * 12;
    cargoScale = 1 - pickup * 0.2;
  } else if (p >= 52 && p < 84) {
    cargo = carriedItem;
    cargoScale = 0.8;
  } else if (p >= 84) {
    cargo = mix(carriedItem, destinationItem, putdown);
    cargoScale = 0.8 + putdown * 0.2;
  }

  return {
    progress: p,
    phase,
    stage,
    robot,
    cargo,
    cargoScale,
    source,
    destination,
    sourceItem,
    destinationItem,
    approach,
    carry,
    pickup,
    putdown,
    gripping: phase === 'picking' || (phase === 'placing' && p < 96),
    gripperOpen: phase === 'picking' ? 1 - between(p, 40, 46) : between(p, 84, 92),
    attached: p >= 52 && p < 84,
    released: p >= 96,
    milestone: p < 32 ? 0 : p < 52 ? 1 : 2,
    segments: [between(p, 0, 32), between(p, 32, 52), between(p, 52, 100)],
  };
}
