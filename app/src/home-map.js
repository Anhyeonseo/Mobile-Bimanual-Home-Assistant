import { MAP_OUTLINE, OBSERVATIONS } from './slam.js';

/** Display geometry only. Navigation always uses the original SLAM coordinates. */
const FURNITURE = {
  counter: {
    angle: -24,
    body: `<rect x="-11" y="-30" width="22" height="60" rx="3" fill="#d0c7b9"/>
      <rect x="-11" y="-33" width="22" height="60" rx="3" fill="#f5f2eb" stroke="#c1b9ae"/>
      <rect x="-7" y="-25" width="14" height="18" rx="3" fill="#b8bdbe" stroke="#a3aaac"/>
      <rect x="-5" y="-23" width="10" height="14" rx="2" fill="#dce0df"/>
      <path d="M0-27v-5h4v4" fill="none" stroke="#858c8e" stroke-width="1.8"/>
      <rect x="-7" y="6" width="14" height="16" rx="2" fill="#555b5e"/>
      <circle cx="-3" cy="10" r="2.5" fill="none" stroke="#b9bec0"/>
      <circle cx="3" cy="17" r="3.2" fill="none" stroke="#b9bec0"/>`,
  },
  fridge: {
    angle: 30,
    body: `<rect x="-13" y="-13" width="26" height="30" rx="3" fill="#a8adaf"/>
      <rect x="-13" y="-17" width="26" height="30" rx="3" fill="#e6e9e8" stroke="#aab0b2"/>
      <path d="M-12 2h24M8-10v6M8 6v3" stroke="#a4abad" stroke-width="1.4"/>`,
  },
  table: {
    angle: 30,
    body: `<rect x="-23" y="-11" width="46" height="27" rx="12" fill="#b4a18c"/>
      <rect x="-23" y="-14" width="46" height="26" rx="12" fill="url(#IDwood)" stroke="#c4b29d"/>
      <rect x="-11" y="-7" width="13" height="11" rx="1" fill="#eeeae2" transform="rotate(-12)"/>
      <path d="M-9-3h8M-9 0h6" stroke="#b8b3ab" stroke-width=".7"/>
      <circle cx="11" cy="1" r="4" fill="#f7f4ee" stroke="#b6a58f"/>`,
  },
  sofa: {
    angle: 30,
    body: `<rect x="-17" y="-29" width="34" height="61" rx="7" fill="#a4a7a5"/>
      <rect x="-17" y="-33" width="34" height="61" rx="7" fill="url(#IDfabric)" stroke="#aeb2af"/>
      <rect x="-14" y="-26" width="22" height="24" rx="5" fill="#e3e5df" stroke="#bfc4bd"/>
      <rect x="-14" y="0" width="22" height="24" rx="5" fill="#e3e5df" stroke="#bfc4bd"/>
      <rect x="8" y="-28" width="7" height="52" rx="3.5" fill="#d1d6cf"/>
      <rect x="-16" y="-33" width="32" height="7" rx="3.5" fill="#d9dcd6"/>
      <rect x="-16" y="22" width="32" height="7" rx="3.5" fill="#d9dcd6"/>
      <rect x="-8" y="-23" width="13" height="13" rx="4" fill="#f4f1e8" transform="rotate(8)"/>`,
  },
  cabinet: {
    angle: 30,
    body: `<rect x="-11" y="-9" width="22" height="24" rx="2" fill="#aa9580"/>
      <rect x="-11" y="-13" width="22" height="24" rx="2" fill="url(#IDwood)" stroke="#ad9e8a"/>
      <path d="M-8-2h16M-2 6h4" stroke="#b4a18c"/>`,
  },
  desk: {
    angle: 30,
    body: `<rect x="-22" y="-10" width="44" height="24" rx="3" fill="#ab9881"/>
      <rect x="-22" y="-14" width="44" height="24" rx="3" fill="url(#IDwood)" stroke="#b2a18b"/>
      <rect x="-12" y="-11" width="18" height="11" rx="1.5" fill="#686f73" stroke="#4a5155"/>
      <rect x="-10" y="-9" width="14" height="7" rx=".5" fill="#d9e0e1"/>
      <path d="M-3 0v3M-7 3h8" stroke="#666d70" stroke-width="1.5"/>
      <rect x="-10" y="5" width="15" height="3" rx="1" fill="#e8e9e6"/>
      <circle cx="14" cy="-5" r="3.5" fill="#ece8df" stroke="#b1a18c"/>`,
  },
  chair: {
    angle: 30,
    body: `<path d="M0 0v17m-8-2 8-3 8 3" stroke="#8e9394" stroke-width="2" fill="none"/>
      <rect x="-9" y="-7" width="18" height="19" rx="6" fill="#b6bcbf" stroke="#959d9f"/>
      <rect x="-10" y="-12" width="20" height="7" rx="3" fill="#d7dbda" stroke="#aab2b3"/>`,
  },
  bedside: {
    angle: 30,
    body: `<rect x="-11" y="-9" width="22" height="23" rx="3" fill="#b3a08a"/>
      <rect x="-11" y="-13" width="22" height="23" rx="3" fill="url(#IDwood)" stroke="#baa78e"/>
      <circle cx="0" cy="-2" r="6" fill="#e6dfd1" stroke="#c9beaa"/>
      <circle cx="0" cy="-3" r="4" fill="#faf7ed"/>`,
  },
  bed: {
    angle: 30,
    body: `<rect x="-22" y="-36" width="44" height="74" rx="4" fill="#b6a38e"/>
      <rect x="-22" y="-39" width="44" height="9" rx="3" fill="#cbbda8"/>
      <rect x="-20" y="-33" width="40" height="66" rx="6" fill="#fbf9f4" stroke="#d0cac1"/>
      <rect x="-17" y="-29" width="15" height="14" rx="4" fill="#eeece6" stroke="#dad7ce"/>
      <rect x="2" y="-29" width="15" height="14" rx="4" fill="#eeece6" stroke="#dad7ce"/>
      <path d="M-20-12h40v39q0 6-6 6h-28q-6 0-6-6Z" fill="url(#IDduvet)" stroke="#b9bec0"/>
      <path d="M-19-8q18 4 38 0M-15-2v28M15 0v26" fill="none" stroke="#dbe0df" stroke-width="1.2"/>
      <path d="M-20 18h40v9q0 6-6 6h-28q-6 0-6-6Z" fill="#ded7ca" opacity=".9"/>
      <path d="M-17 21h34M-17 24h34" stroke="#c6bba9" stroke-width=".7"/>`,
  },
  vanity: {
    angle: 30,
    body: `<rect x="-18" y="-7" width="36" height="18" rx="3" fill="#b8a58e"/>
      <rect x="-18" y="-11" width="36" height="18" rx="3" fill="url(#IDwood)" stroke="#bdad96"/>
      <ellipse cx="0" cy="-8" rx="10" ry="6" fill="#d5dddc" stroke="#f9f7f1" stroke-width="2"/>
      <path d="m-5-9 7-3M0-7l5-2" stroke="#f9faf7"/>
      <rect x="11" y="-4" width="3" height="6" rx="1" fill="#b7a48f"/>`,
  },
  stool: {
    angle: 0,
    body: `<ellipse cy="3" rx="8" ry="7" fill="#ac9f8c"/>
      <circle r="8" fill="#e1d9cb" stroke="#bcb09c"/>`,
  },
  fitness: {
    angle: 30,
    body: `<rect x="-11" y="-23" width="22" height="45" rx="4" fill="#dedbd4" stroke="#c4bfb5"/>
      <rect x="-7" y="-18" width="14" height="33" rx="3" fill="#b2b4b0"/>
      <path d="M-8-15v-9H8v9" fill="none" stroke="#626b6e" stroke-width="3"/>
      <rect x="-6" y="-26" width="12" height="7" rx="2" fill="#7e878a"/>`,
  },
};

/** A furnished interpretation of the observations, not a photo or a new navigation map. */
export function homeMap(id) {
  const outline = `M${MAP_OUTLINE.map((point) => point.join(' ')).join(' L')}Z`;
  const furnishing = OBSERVATIONS.map((observation) => {
    const furniture = FURNITURE[observation.id];
    if (!furniture) return '';
    return `<g class="home-furniture" data-furniture="${observation.id}"
      transform="translate(${observation.x} ${observation.y}) rotate(${furniture.angle})">
      ${furniture.body.replaceAll('ID', id)}</g>`;
  }).join('');

  return `<defs>
    <clipPath id="${id}floor-clip"><path d="${outline}"/></clipPath>
    <linearGradient id="${id}floor" x1="0" y1="0" x2="1" y2="1">
      <stop stop-color="#f1ebe1"/><stop offset="1" stop-color="#ded4c4"/>
    </linearGradient>
    <linearGradient id="${id}wood" x1="0" y1="0" x2=".8" y2="1">
      <stop stop-color="#e3d7c4"/><stop offset="1" stop-color="#cdbb9e"/>
    </linearGradient>
    <linearGradient id="${id}fabric" x1="0" y1="0" x2="1" y2="1">
      <stop stop-color="#e9ece5"/><stop offset="1" stop-color="#bec5be"/>
    </linearGradient>
    <linearGradient id="${id}duvet" x1="0" y1="0" x2="1" y2="1">
      <stop stop-color="#d7ddde"/><stop offset="1" stop-color="#b7c1c3"/>
    </linearGradient>
    <pattern id="${id}planks" width="31" height="110" patternUnits="userSpaceOnUse" patternTransform="rotate(30)">
      <path d="M0 0v110M15.5 0v110M0 32h15.5M15.5 83H31" fill="none" stroke="#c2b79f" stroke-width=".6" opacity=".45"/>
      <path d="M4 0v30m6 25v52m12-100v55m5 6v42" stroke="#fff" stroke-width=".5" opacity=".4"/>
    </pattern>
    <filter id="${id}furniture-shadow" x="-40%" y="-40%" width="180%" height="190%">
      <feDropShadow dx="1" dy="3" stdDeviation="2" flood-color="#635b4e" flood-opacity=".2"/>
    </filter>
  </defs>
  <g class="map-base-layer home-map-layer" aria-hidden="true">
    <path d="${outline}" transform="translate(0 7)" fill="#b8b5ad" stroke="#b8b5ad" stroke-width="7" stroke-linejoin="round"/>
    <path class="map-outline" d="${outline}" fill="url(#${id}floor)" stroke="#bbb9b1" stroke-width="7" stroke-linejoin="round"/>
    <g clip-path="url(#${id}floor-clip)">
      <path d="${outline}" fill="url(#${id}planks)"/>
      <path d="M430 270 548 298 538 403 472 412Z" fill="#e6e5de" opacity=".7"/>
      <path d="M460 290 529 330M468 313 525 350M476 338 522 367M483 362 518 384" stroke="#f9f9f4" opacity=".65"/>
      <rect x="551" y="356" width="96" height="87" rx="10" transform="rotate(30 599 400)" fill="#f8f6ef" stroke="#d5cec1"/>
      <rect x="556" y="361" width="86" height="77" rx="7" transform="rotate(30 599 400)" fill="none" stroke="#e6e0d5" stroke-width="2"/>
      <rect x="497" y="529" width="61" height="93" rx="6" transform="rotate(30 527 575)" fill="#ebe6db" opacity=".75"/>
      <g filter="url(#${id}furniture-shadow)">${furnishing}</g>
    </g>
    <path d="${outline}" fill="none" stroke="#faf9f6" stroke-width="5" stroke-linejoin="round"/>
    <path d="M503 283 540 303 577 322 616 348" fill="none" stroke="#d2d6d2" stroke-width="3"/>
    <path d="M507 285 539 303M550 310 580 327M589 332 612 346" fill="none" stroke="#fcfdfa" stroke-width="2"/>
    <g transform="translate(455 620) rotate(30)">
      <rect x="-14" y="-8" width="28" height="17" rx="5" fill="#e9e9e4" stroke="#c2c4bf"/>
      <rect x="-10" y="-6" width="20" height="4" rx="2" fill="#8b9493"/>
    </g>
    <g class="home-room-labels">
      <text x="540" y="353" text-anchor="middle">주방</text>
      <text x="563" y="450" text-anchor="middle">거실 · 작업 공간</text>
      <text x="476" y="605" text-anchor="middle">침실</text>
    </g>
  </g>`;
}
