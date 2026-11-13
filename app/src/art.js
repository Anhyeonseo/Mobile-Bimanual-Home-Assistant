import { OBSERVATIONS, MAP_OUTLINE } from './slam.js';
import { escapeHTML as e } from './core.js';
import { homeMap } from './home-map.js';
const paths = {
  home: 'M3 10 12 3l9 7v10a1 1 0 0 1-1 1h-5v-7H9v7H4a1 1 0 0 1-1-1Z',
  map: 'm3 5 6-2 6 2 6-2v16l-6 2-6-2-6 2Zm6-2v16m6-14v16',
  spark: 'm12 3 2.5 6.5L21 12l-6.5 2.5L12 21l-2.5-6.5L3 12l6.5-2.5Zm7-2v4m-2-2h4',
  clock: 'M12 8v5l3 2M22 12a10 10 0 1 1-20 0 10 10 0 0 1 20 0',
  activity: 'M3 12h4l3-8 4 16 3-8h4',
  settings:
    'M9 3h6l1 3 3 1 2 5-2 5-3 1-1 3H9l-1-3-3-1-2-5 2-5 3-1ZM16 12a4 4 0 1 1-8 0 4 4 0 0 1 8 0',
  bell: 'M18 8a6 6 0 0 0-12 0c0 7-3 7-3 9h18c0-2-3-2-3-9M10 21h4',
  chevron: 'm9 5 7 7-7 7',
  down: 'm6 9 6 6 6-6',
  arrow: 'M4 12h16m-6-6 6 6-6 6',
  back: 'M20 12H4m6-6-6 6 6 6',
  plus: 'M12 4v16M4 12h16',
  close: 'm6 6 12 12M18 6 6 18',
  check: 'm5 12 4 4L19 6',
  play: 'm8 4 12 8-12 8Z',
  pause: 'M8 5v14M16 5v14',
  stop: 'M6 6h12v12H6Z',
  battery: 'M3 6h16v12H3ZM22 10v4M7 9v6m4-6v6m4-6v6',
  wifi: 'M2 8a16 16 0 0 1 20 0M5 12a11 11 0 0 1 14 0M8 16a6 6 0 0 1 8 0M12 20h.01',
  robot: 'M5 7h14v11H5Zm7-5v5M9 12h.01M15 12h.01M9 16h6M2 10v5m20-5v5',
  cup: 'M4 4h12v12a4 4 0 0 1-4 4H8a4 4 0 0 1-4-4Zm12 2h2a4 4 0 0 1 0 8h-2',
  bottle: 'M9 3h6v4l3 4v10H6V11l3-4ZM9 7h6M6 14h12',
  remote: 'M8 2h8v20H8ZM12 6h.01M11 10h2m-2 4h2m-2 4h2',
  book: 'M3 3h7a3 3 0 0 1 2 2 3 3 0 0 1 2-2h7v17h-7a3 3 0 0 0-2 1 3 3 0 0 0-2-1H3Zm9 2v16',
  scan: 'M8 3H3v5m13-5h5v5M3 16v5h5m8 0h5v-5M3 12h18',
  locate: 'M12 2v4m0 12v4M2 12h4m12 0h4M18 12a6 6 0 1 1-12 0 6 6 0 0 1 12 0',
  mic: 'M9 5a3 3 0 0 1 6 0v7a3 3 0 0 1-6 0ZM5 10v2a7 7 0 0 0 14 0v-2M12 19v3m-4 0h8',
  send: 'm3 3 19 9-19 9 4-9Zm4 9h15',
  sun: 'M12 2v2m0 16v2M2 12h2m16 0h2M5 5l2 2m10 10 2 2M5 19l2-2M17 7l2-2M17 12a5 5 0 1 1-10 0 5 5 0 0 1 10 0',
  moon: 'M20 15A9 9 0 0 1 9 4 9 9 0 1 0 20 15',
  help: 'M9 9a3 3 0 1 1 5 2c-2 1-2 2-2 3m0 3h.01M22 12a10 10 0 1 1-20 0 10 10 0 0 1 20 0',
  shield: 'm12 2 9 4v6c0 5-9 10-9 10S3 17 3 12V6Zm-5 9 3 3 7-7',
  edit: 'm14 5 5 5M3 21l5-1L21 7l-5-5L3 15Z',
  trash: 'M3 6h18M9 6V3h6v3M5 6l1 15h12l1-15M10 10v7m4-7v7',
  user: 'M16 7a4 4 0 1 1-8 0 4 4 0 0 1 8 0M4 21v-2a8 8 0 0 1 16 0v2',
  users:
    'M14 7a4 4 0 1 1-8 0 4 4 0 0 1 8 0M2 21v-2a8 8 0 0 1 16 0v2M18 3a4 4 0 0 1 0 8m2 4a6 6 0 0 1 2 6',
  link: 'M10 13a5 5 0 0 0 7 0l4-4a5 5 0 0 0-7-7l-3 3m2 6a5 5 0 0 0-7 0l-4 4a5 5 0 0 0 7 7l3-3',
  logout: 'M9 3H3v18h6m6-15 6 6-6 6M9 12h12',
  refresh: 'M20 4v6h-6M4 20v-6h6M5 7a8 8 0 0 1 14-1M19 17a8 8 0 0 1-14 1',
  alert: 'm12 3 10 18H2ZM12 9v5m0 3h.01',
  image: 'M3 3h18v18H3Zm0 14 6-6 4 4 3-3 5 5M16 7h.01',
  download: 'M12 3v12m-5-5 5 5 5-5M3 16v5h18v-5',
  table: 'M3 7h18v5H3Zm3 5v9m12-9v9',
  bed: 'M3 19V6h18v13M3 15h18M7 6v5h10V6',
  desk: 'M3 12h18M5 12v9m14-9v9M7 3h10v6H7Zm5 6v3',
  kitchen: 'M3 8h18v13H3Zm6 0v13m6-13v13M6 3v5m-3-5h6',
  volume: 'M3 9h4l5-5v16l-5-5H3Zm13-2a7 7 0 0 1 0 10m3-13a11 11 0 0 1 0 16',
  info: 'M12 10v7m0-10h.01M22 12a10 10 0 1 1-20 0 10 10 0 0 1 20 0',
  star: 'm12 2 3 6 7 1-5 5 1 8-6-4-6 4 1-8-5-5 7-1Z',
  box: 'm3 7 9-5 9 5v10l-9 5-9-5Zm0 0 9 5 9-5m-9 5v10',
  bluetooth: 'm7 7 10 10-5 5V2l5 5L7 17',
  cloud: 'M6 19a5 5 0 0 1-1-10 7 7 0 0 1 13-2 6 6 0 0 1 0 12Z',
  camera: 'M3 7h4l2-4h6l2 4h4v14H3ZM16 13a4 4 0 1 1-8 0 4 4 0 0 1 8 0',
};
export function icon(name, size = 22) {
  return `<svg class="icon" width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.65" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="${paths[name] || paths.spark}"/></svg>`;
}
let n = 0;
let pointCloud = '';
function cloud() {
  if (pointCloud) return pointCloud;
  let seed = 39;
  const rand = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 4294967296;
  };
  const put = (x, y) => {
    const a = Math.round(x / 2) * 2,
      b = Math.round(y / 2) * 2;
    pointCloud += `M${a} ${b}h2v2h-2z`;
  };
  for (let j = 0; j < MAP_OUTLINE.length; j++) {
    const a = MAP_OUTLINE[j],
      b = MAP_OUTLINE[(j + 1) % MAP_OUTLINE.length],
      d = Math.hypot(b[0] - a[0], b[1] - a[1]);
    for (let k = 0; k < d; k += 1.7) {
      const f = k / d,
        x = a[0] + (b[0] - a[0]) * f,
        y = a[1] + (b[1] - a[1]) * f;
      for (let z = 0; z < 3; z++) put(x + (rand() - 0.5) * 8, y + (rand() - 0.5) * 8);
      if (rand() > 0.6) put(x + (rand() - 0.5) * 17, y + (rand() - 0.5) * 17);
    }
  }
  for (const o of OBSERVATIONS) {
    for (let j = 0; j < 35; j++) {
      const a = rand() * Math.PI * 2,
        r = 8 + rand() * 6;
      put(o.x + Math.cos(a) * r, o.y + Math.sin(a) * r);
    }
  }
  return pointCloud;
}
export function floorplan(
  state,
  {
    large = false,
    path = false,
    scan = false,
    observation = null,
    routeOverride = null,
    showRobot = true,
    interactive = true,
    overlay = '',
  } = {},
) {
  const id = 'slam' + ++n,
    p = state.robot.position,
    ob = OBSERVATIONS.find((o) => o.id === observation),
    route = routeOverride || state.task?.route,
    home = state.settings.mapView !== 'slam';
  const description = home
    ? 'SLAM 지도와 같은 좌표에 바닥과 가구를 표현한 우리 집 예시. 진한 원은 로미의 위치입니다.'
    : 'SLAM 예시 점유 지도. 회색은 관측된 벽, 흰색은 내부 공간, 진한 원은 로미의 위치입니다.';
  return `<svg class="floorplan ${home ? 'home-map' : 'slam-map'} ${large ? 'large' : ''} ${state.task?.status === 'running' ? 'is-moving' : ''}" data-map-view="${home ? 'home' : 'slam'}" viewBox="360 240 360 475" role="group" aria-label="${description}">
 <defs><pattern id="${id}zone" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><path d="M0 0v7" stroke="#bd7775" stroke-width="2"/></pattern></defs>
 <rect class="map-background" x="360" y="240" width="360" height="475" fill="#eceef0"/>
 ${home ? homeMap(id) : `<g class="map-base-layer slam-map-layer"><path class="map-outline" d="M${MAP_OUTLINE.map((p) => p.join(' ')).join(' L')}Z" fill="#fff" stroke="#c4c8cc" stroke-width="4"/><path class="slam-points" d="${cloud()}" fill="#93969a" opacity=".9"/></g>`}
 ${state.map.zone ? `<rect x="480" y="622" width="57" height="45" rx="2" fill="url(#${id}zone)" stroke="#ad6a68" stroke-dasharray="4 3" opacity=".8"/>` : ''}
 ${path && route?.length ? `<path d="M${route.map((p) => `${p.x} ${p.y}`).join(' L')}" stroke="#5e6165" stroke-width="2.6" stroke-dasharray="5 4" fill="none" class="route-line"/>` : ''}
 <g class="map-observation-layer">${OBSERVATIONS.map((o) => observationMarker(o, ob, { home, interactive })).join('')}</g>
 ${ob ? `<g pointer-events="none"><path d="M${ob.x} ${ob.y} ${ob.stop.x} ${ob.stop.y}" fill="none" stroke="#96999d" stroke-dasharray="2 3"/><circle cx="${ob.stop.x}" cy="${ob.stop.y}" r="8" fill="#f1f4f8" stroke="#5e6165" stroke-width="1.7"/><path d="M${ob.stop.x - 3} ${ob.stop.y}h6m-3-3v6" stroke="#5e6165" stroke-width="1.2"/></g>` : ''}
 ${scan ? `<rect x="360" y="${257 + (442 * state.map.scan) / 100}" width="360" height="${458 * (1 - state.map.scan / 100)}" fill="#eceef0" opacity=".93"/><path d="M375 ${257 + (442 * state.map.scan) / 100}h320" stroke="#919498" stroke-width="1.5"/>` : ''}
 ${showRobot ? `<g class="robot-marker" transform="translate(${p.x} ${p.y})" pointer-events="none"><circle r="20" fill="#7f8286" opacity=".12"/><circle r="10" fill="#5e6165" stroke="white" stroke-width="2.5"/><path d="m-3 2 3-6 3 6Z" fill="white" transform="rotate(${state.task?.status === 'done' ? state.task.angle || 0 : 0})"/></g>` : ''}${overlay}</svg>`;
}

function observationMarker(observation, selected, { home, interactive }) {
  const { id, x, y, name } = observation;
  const active = selected?.id === id;
  const showLabel = active || (!home && ['counter', 'table', 'desk', 'bed', 'vanity'].includes(id));
  const attributes = interactive
    ? `data-action="observation" data-id="${id}" tabindex="0" role="button" aria-label="${e(name)} 관측점 선택"`
    : '';
  return `<g class="slam-observation ${home ? 'home-observation' : ''}" ${attributes}>
    <circle class="observation-hit" cx="${x}" cy="${y}" r="${home ? 22 : 15}" fill="transparent" stroke="${active ? '#303743' : 'transparent'}" stroke-width="1.5"/>
    ${!home || active ? `<circle cx="${x}" cy="${y}" r="3.7" fill="#5e6165" stroke="white" stroke-width="1.4"/>` : ''}
    <title>${e(name)} · 가구 관측</title>
    ${showLabel ? `<text x="${x}" y="${y - (home ? 28 : 17)}" text-anchor="middle" font-size="9" font-family="Pretendard,sans-serif" fill="#454b53" paint-order="stroke" stroke="#fff" stroke-width="3" stroke-linejoin="round">${e(name)}</text>` : ''}
  </g>`;
}
