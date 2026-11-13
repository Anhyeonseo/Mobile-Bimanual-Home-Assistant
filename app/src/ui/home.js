import { escapeHTML as escape } from '../core.js';
import { floorplan, icon } from '../art.js';
import { html } from './html.js';
import { mapModeSwitch, mapView } from './map-mode.js';

/** A single spatial canvas, with a draggable request sheet over the map. */
export function renderHome(state, { hasTask, requestText, composerOpen }) {
  const status = state.robot.emergency
    ? '잠시 멈춰 있어요'
    : !state.robot.connected
      ? '연결을 기다려요'
      : state.robot.charging
        ? '에너지를 채우고 있어요'
        : hasTask
          ? '부탁한 일을 하고 있어요'
          : '로미가 대기 중이에요';
  const nextRoutine = state.routines.find((routine) => routine.enabled);

  return html`
    <section class="home-space" aria-label="우리 집 공간과 로봇">
      <div class="space-heading">
        <div>
          <button class="space-name" data-action="home-settings">
            <h1>${escape(state.profile.home)}</h1>
            ${icon('down', 16)}
          </button>
          <p>${escape(status)}</p>
        </div>
        <button class="robot-presence" data-action="device" aria-label="로미 기기 정보">
          <img src="assets/aloha-mini-opal.png" alt="로미 캐릭터" width="1188" height="1324" />
          <span>
            <strong>${escape(state.robot.name)}</strong>
            <small>${icon('battery', 13)}${state.robot.battery}%</small>
          </span>
        </button>
      </div>

      <div class="map-studio">
        <div class="home-map-switch">${mapModeSwitch(state)}</div>
        <div class="studio-aura" aria-hidden="true"></div>
        <div class="native-map">${floorplan(state, { path: hasTask })}</div>
        <span class="map-floating-label">
          ${icon('locate', 12)}${state.map.localization === 'ready'
            ? '위치 확인됨'
            : '위치를 맞춰주세요'}
        </span>
        <div class="scene-tools">
          <button class="glass-icon" data-action="locate" aria-label="로봇 위치 확인">
            ${icon('locate', 20)}
          </button>
          <button class="glass-icon" data-action="nav" data-page="map" aria-label="지도 관리">
            ${icon('map', 20)}
          </button>
        </div>
        <div class="scene-caption">
          <span class="map-signal">
            <i></i>
            ${mapView(state) === 'home' ? '가구 배치로 보는 우리 집' : '로봇이 관측한 지도'}
          </span>
          <button data-action="nav" data-page="map">
            ${state.furniture.length}개의 관측 지점 ${icon('arrow', 14)}
          </button>
        </div>
        <div class="space-actions">
          <button class="glass-action" data-action="remote">${icon('locate', 18)}직접 조작</button>
          <button class="glass-action" data-action="charge">${icon('battery', 18)}충전 복귀</button>
          ${hasTask
            ? html`
                <button class="glass-action task-live" data-action="nav" data-page="task">
                  ${icon('activity', 18)}진행 보기 · ${state.task.progress}%
                </button>
              `
            : ''}
        </div>
      </div>

      <section
        class="native-composer ${composerOpen ? 'is-open' : ''}"
        aria-label="로미에게 부탁하기"
      >
        <button
          class="sheet-handle"
          data-action="composer-toggle"
          aria-label="부탁 패널 ${composerOpen ? '접기' : '펼치기'}"
          aria-expanded="${composerOpen}"
        >
          <span></span>
        </button>
        <button class="composer-teaser" data-action="composer-open">
          <span class="liquid-orb">${icon('spark', 23)}</span>
          <span>
            <strong>로미에게 부탁해요</strong>
            <small>물건 옮기기 · 이동 · 루틴</small>
          </span>
          ${icon('plus', 20)}
        </button>
        <div class="composer-body" ${composerOpen ? '' : 'inert'}>
          <div class="composer-title">
            <h2>로미에게 부탁하기</h2>
          </div>
          <form id="request-form">
            <label class="sr-only" for="request-text">로미에게 부탁할 내용</label>
            <textarea
              id="request-text"
              name="prompt"
              maxlength="300"
              placeholder="거실에 있는 리모컨을
침실로 가져다줘"
            >
${escape(requestText)}</textarea
            >
            <div id="request-error" class="field-error" role="alert"></div>
            <div class="composer-submit-row">
              <span>${icon('shield', 13)}시작하기 전에 확인해요</span>
              <button class="send-orb" type="submit" aria-label="부탁 확인하기">
                <span>부탁하기</span>
                ${icon('arrow', 18)}
              </button>
            </div>
          </form>
          <div class="composer-shortcuts">
            <button data-action="request-example" data-id="remote">
              ${icon('remote', 21)}
              <span>리모컨 가져오기</span>
            </button>
            <button data-action="come">
              ${icon('locate', 21)}
              <span>여기로 와줘</span>
            </button>
            <button data-action="patrol">
              ${icon('scan', 21)}
              <span>둘러보기</span>
            </button>
          </div>
          ${nextRoutine
            ? html`
                <button class="next-routine" data-action="nav" data-page="routines">
                  <span class="next-time">${escape(nextRoutine.time)}</span>
                  <span>
                    <small>나의 루틴</small>
                    <strong>${escape(nextRoutine.name)}</strong>
                  </span>
                  ${icon('chevron', 15)}
                </button>
              `
            : html`
                <button class="next-routine" data-action="setup-start">
                  ${icon('link', 18)}새 로봇 연결하기 ${icon('arrow', 16)}
                </button>
              `}
        </div>
      </section>
    </section>
  `;
}
