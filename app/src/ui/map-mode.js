import { html } from './html.js';
import { icon } from '../art.js';

export const mapView = (state) => (state.settings.mapView === 'slam' ? 'slam' : 'home');

/** A shared preference: changing the presentation never changes the map or task. */
export function mapModeSwitch(state) {
  const mode = mapView(state);
  return html`
    <div class="map-view-switch" role="group" aria-label="지도 보기 방식" data-mode="${mode}">
      <span class="map-view-thumb" aria-hidden="true"></span>
      <button data-action="map-view" data-value="home" aria-pressed="${mode === 'home'}">
        ${icon('home', 15)}우리 집
      </button>
      <button data-action="map-view" data-value="slam" aria-pressed="${mode === 'slam'}">
        ${icon('scan', 15)}SLAM
      </button>
    </div>
  `;
}
