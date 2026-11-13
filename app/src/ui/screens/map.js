import { html } from '../html.js';
import { icon as i, floorplan } from '../../art.js';
import { escapeHTML as e, isBlocked, findRoute } from '../../core.js';
import { OBSERVATIONS } from '../../slam.js';
import { mapModeSwitch, mapView } from '../map-mode.js';

/** Render the map screen from a read-only snapshot. */
export function renderMap(context) {
  const { state, selectedObservation, zoom, heading, button, arrowLink, selected } = context;
  const scanning = state.map.scanning || state.map.draft;
  const ob = OBSERVATIONS.find((o) => o.id === selectedObservation);
  const ready = state.map.localization === 'ready';
  const route =
    ob && ready && !isBlocked(ob.stop.x, ob.stop.y, state.map.zone)
      ? findRoute(state.robot.position, ob.stop, state.map.zone)
      : null;
  return html`
    ${heading(
      '우리 집 지도',
      '익숙한 집 모습과 로봇의 지도를 함께 확인하세요.',
      button('SLAM 시연', 'slam-demo', 'btn-primary', 'play'),
    )}
    <div class="map-page-grid">
      <section class="card map-page">
        <div class="section-top">
          <div>
            <h2>${e(state.map.name)}</h2>
            <p class="map-caption">SLAM 예시 지도 · ${state.furniture.length}개 가구 관측</p>
          </div>
          <span class="map-version">MAP 01</span>
        </div>
        <div class="map-view-toolbar">
          ${mapModeSwitch(state)}
          <span>${mapView(state) === 'home' ? '가구 배치로 보기' : '관측 데이터로 보기'}</span>
        </div>
        <div class="map-wrap slam-wrap">
          <div style="transform:scale(${zoom});transform-origin:50% 50%;transition:transform .2s">
            ${floorplan(state, {
              large: true,
              path: !!route,
              scan: scanning,
              observation: selectedObservation,
              routeOverride: route,
            })}
          </div>
          <div class="map-type-badge">
            ${scanning
              ? `탐색 ${state.map.scan}%`
              : mapView(state) === 'home'
                ? '우리 집 · 재구성 예시'
                : 'SLAM · 관측 예시'}
          </div>
          <div class="map-controls">
            <button data-action="zoom-in" aria-label="지도 확대">${i('plus', 15)}</button>
            <button data-action="zoom-out" aria-label="지도 축소">−</button>
            <button data-action="zoom-reset" aria-label="지도 크기 초기화">
              ${i('locate', 14)}
            </button>
          </div>
        </div>
        <div class="map-toolbar">
          ${button('저장 지도 불러오기', 'load-slam', 'btn-outline', 'map')}${button(
            scanning ? '탐색 이어보기' : '지도 새로 만들기',
            'mapping-open',
            'btn-outline',
            'scan',
          )}${button(
            state.map.zone ? '금지 구역 해제' : '진입 금지 구역',
            'zone',
            'btn-outline',
            'shield',
          )}
        </div>
        <div class="map-legend">
          <span>
            <span class="legend-dot"></span>
            로봇 위치
          </span>
          <span>
            <span class="legend-dot zone"></span>
            ${mapView(state) === 'home' ? '가구를 눌러 선택' : '가구 관측'}
          </span>
          <span>
            <span class="legend-ring"></span>
            접근 후보
          </span>
        </div>
      </section>
      <div class="slam-side-stack">
        <section class="card localization-card">
          <div class="row-between">
            <h3>로봇 위치 확인</h3>
            <span class="status-tag">
              ${ready
                ? '확인됨'
                : state.map.localization === 'localizing'
                  ? '확인 중'
                  : '확인 필요'}
            </span>
          </div>
          <p>
            ${ready
              ? '저장한 지도와 예시 현재 위치를 맞췄어요.'
              : '지도에서 로봇의 시작 위치를 먼저 맞춰 주세요.'}
          </p>
          ${state.map.localization === 'localizing'
            ? html`
                <div class="progress-track">
                  <div class="progress-fill" style="width:${state.map.localizeProgress}%"></div>
                </div>
              `
            : button(
                ready ? '위치 다시 맞추기' : '현재 위치 맞추기',
                'localize',
                'btn-outline full',
                'locate',
              )}
        </section>
        <section class="card destination-card">
          <h2>이동할 곳</h2>
          <p class="map-caption">지도 위의 가구나 관측점, 목록에서 선택하세요.</p>
          <div class="field mt">
            <label class="form-label" for="observation-select">가구 관측 지점</label>
            <select id="observation-select">
              <option value="">목적지를 선택하세요</option>
              ${OBSERVATIONS.map(
                (o) => html`
                  <option value="${o.id}" ${selected(selectedObservation, o.id)}>
                    ${e(state.furniture.find((f) => f.id === o.id)?.name || o.name)}
                  </option>
                `,
              ).join('')}
            </select>
          </div>
          ${ob
            ? html`
                <div class="stop-candidate">
                  <span class="stop-candidate-icon">${i('locate', 20)}</span>
                  <div>
                    <strong>
                      ${e(state.furniture.find((f) => f.id === ob.id)?.name || ob.name)} 앞
                    </strong>
                    <small>관측점에서 떨어진 접근 후보 · 예시</small>
                  </div>
                  ${i('check', 16)}
                </div>
              `
            : '<div class="destination-empty">목적지를 고르면<br>접근 위치와 경로가 표시돼요.</div>'}
          <div class="field mt">
            <label class="form-label" for="arrival-angle">
              도착 방향
              <span id="angle-value">0°</span>
            </label>
            <input id="arrival-angle" type="range" min="0" max="360" step="45" value="0" />
          </div>
          ${button(
            '경로 확인하고 이동',
            'navigate-selected',
            'btn-primary full',
            'arrow',
            !ob || !ready || !route ? 'disabled' : '',
          )}
          <p class="destination-note">
            가구 관측점은 로봇의 이동 목표가 아니에요. 실제 이동 전 접근 위치를 확인합니다.
          </p>
        </section>
      </div>
    </div>
    <section id="furniture-section" class="card furniture-section">
      <div class="section-top">
        <div>
          <h2>관측한 가구와 이름</h2>
          <p class="map-caption">익숙한 이름과 별칭을 등록해 보세요.</p>
        </div>
        ${arrowLink('이름 추가', 'add-furniture')}
      </div>
      <div class="furniture-list">
        ${state.furniture
          .map(
            (f) => html`
              <button class="furniture-list-row" data-action="edit-furniture" data-id="${e(f.id)}">
                <span class="furniture-label">
                  <span class="observation-dot"></span>
                  <strong>${e(f.name)}</strong>
                </span>
                <span class="furniture-room">${e(f.room)}</span>
                <span class="furniture-alias">${e(f.alias) || '별칭 없음'}</span>
                ${i('edit', 15)}
              </button>
            `,
          )
          .join('')}
      </div>
    </section>
  `;
}
