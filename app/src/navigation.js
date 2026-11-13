import { slamBlocked } from './slam.js';

// Coordinates are demonstration map units, not calibrated hardware coordinates.
const GRID_STEP = 4;
const GRID_BOUNDS = { minX: 384, maxX: 684, minY: 264, maxY: 676 };
const NEIGHBORS = [
  [GRID_STEP, 0],
  [-GRID_STEP, 0],
  [0, GRID_STEP],
  [0, -GRID_STEP],
];
const cellKey = ({ x, y }) => `${x},${y}`;

export const isBlocked = slamBlocked;

function nearestFreeCell(position, noGoZone) {
  let nearest = null;
  let shortestDistance = Infinity;
  for (let x = GRID_BOUNDS.minX; x <= GRID_BOUNDS.maxX; x += GRID_STEP) {
    for (let y = GRID_BOUNDS.minY; y <= GRID_BOUNDS.maxY; y += GRID_STEP) {
      const squaredDistance = (x - position.x) ** 2 + (y - position.y) ** 2;
      if (squaredDistance < shortestDistance && !isBlocked(x, y, noGoZone)) {
        nearest = { x, y };
        shortestDistance = squaredDistance;
      }
    }
  }
  return nearest;
}

function reconstructRoute(parents, destinationKey) {
  const route = [];
  let key = destinationKey;
  while (key) {
    const [x, y] = key.split(',').map(Number);
    route.push({ x, y });
    key = parents.get(key);
  }
  return route.reverse();
}

/** Four-way breadth-first search over free cells in the demonstration map. */
export function findRoute(from, to, noGoZone = false) {
  const origin = nearestFreeCell(from, noGoZone);
  const destination = nearestFreeCell(to, noGoZone);
  if (!origin || !destination) return [];
  const destinationKey = cellKey(destination);
  const queue = [origin];
  const parents = new Map([[cellKey(origin), null]]);

  for (let cursor = 0; cursor < queue.length; cursor++) {
    const current = queue[cursor];
    if (cellKey(current) === destinationKey) return reconstructRoute(parents, destinationKey);

    for (const [dx, dy] of NEIGHBORS) {
      const next = { x: current.x + dx, y: current.y + dy };
      const nextKey = cellKey(next);
      // Check the segment midpoint as well, so an edge cannot cut through furniture.
      const blocked =
        isBlocked(next.x, next.y, noGoZone) ||
        isBlocked((current.x + next.x) / 2, (current.y + next.y) / 2, noGoZone);
      if (parents.has(nextKey) || blocked) continue;
      parents.set(nextKey, cellKey(current));
      queue.push(next);
    }
  }
  return [];
}

/** Interpolate position for the progress animation without changing the saved route. */
export function routePoint(route, progress) {
  if (!route?.length) return { x: 561, y: 471 };
  const offset = Math.min(1, Math.max(0, progress)) * (route.length - 1);
  const previous = route[Math.floor(offset)];
  const next = route[Math.min(route.length - 1, Math.floor(offset) + 1)];
  const fraction = offset % 1;
  return {
    x: previous.x + (next.x - previous.x) * fraction,
    y: previous.y + (next.y - previous.y) * fraction,
  };
}
