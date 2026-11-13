import { html } from '../html.js';
import { icon as i, floorplan } from '../../art.js';
import { escapeHTML as e } from '../../core.js';

/** Render the setup screen from a read-only snapshot. */
export function renderSetup(context) {
  const { state, setupStep, button, mascot } = context;
  let body = '';
  if (setupStep === 0)
    body = html`
      <div class="welcome-mascot">${mascot()}</div>
      <div class="eyebrow">HELLO, NEW EVERYDAY</div>
      <h1>
        우리 집의 새로운 식구,
        <br />
        로미를 만나보세요.
      </h1>
      <p>
        처음 연결부터 첫 부탁까지 약 2분.
        <br />
        로미와의 일상을 차근차근 시작해요.
      </p>
      <form id="setup-profile">
        <div class="field">
          <label class="form-label" for="setup-name">어떻게 불러드릴까요?</label>
          <input
            id="setup-name"
            name="name"
            maxlength="15"
            value="${e(state.profile.name)}"
            placeholder="이름"
            required
          />
        </div>
        <div class="field">
          <label class="form-label" for="setup-home">우리 집 이름</label>
          <input
            id="setup-home"
            name="home"
            maxlength="24"
            value="${e(state.profile.home)}"
            required
          />
        </div>
        <button class="btn btn-primary full" type="submit">함께 시작하기 ${i('arrow', 17)}</button>
      </form>
      <p class="storage-notice">
        체험용 연결이에요. 기존 지도와 활동 기록은 유지되며, 실제 계정이나 기기 권한은 요청하지
        않아요.
      </p>
    `;
  if (setupStep === 1)
    body = html`
      <div class="eyebrow">STEP 01 · GET READY</div>
      <h1>연결할 준비를 해볼까요?</h1>
      <p>
        로봇 전원을 켜고, 가까운 곳에 놓아주세요.
        <br />
        편리한 연결을 위해 필요한 항목이에요.
      </p>
      <form id="setup-permissions">
        <label class="permission-row">
          ${i('bluetooth', 23)}
          <div>
            <strong>주변 기기 찾기</strong>
            <p>가까이 있는 로미를 찾는 데 사용해요.</p>
          </div>
          <input type="checkbox" class="check-box" name="bluetooth" checked required />
        </label>
        <label class="permission-row">
          ${i('wifi', 23)}
          <div>
            <strong>로컬 네트워크 연결</strong>
            <p>같은 Wi-Fi에서 로봇과 연결해요.</p>
          </div>
          <input type="checkbox" class="check-box" name="network" checked required />
        </label>
        <label class="permission-row">
          ${i('bell', 23)}
          <div>
            <strong>작업 알림 (선택)</strong>
            <p>작업이 끝나면 앱에서 알려드려요.</p>
          </div>
          <input type="checkbox" class="check-box" name="notify" checked />
        </label>
        <button class="btn btn-primary full mt" type="submit">로미 찾기 ${i('arrow', 17)}</button>
      </form>
      <p class="storage-notice">
        현재는 연결 화면을 체험하는 단계로, 휴대폰의 실제 권한을 변경하지 않아요.
      </p>
    `;
  if (setupStep === 2)
    body = html`
      <div class="eyebrow">STEP 02 · NICE TO MEET YOU</div>
      <h1>로미를 찾았어요.</h1>
      <p>
        로봇 뒷면의 기기 번호와 확인해 주세요.
        <br />
        이 로봇이 맞다면 연결을 시작해요.
      </p>
      <div class="connect-device">
        ${i('robot', 32)}
        <div>
          <strong>HOMETEKHU · ALOHA Mini</strong>
          <small>HK-2026-001 · 체험 기기</small>
        </div>
        ${i('check', 19)}
      </div>
      <div class="field">
        <label class="form-label" for="setup-robot">로봇에게 이름을 지어주세요</label>
        <input id="setup-robot" maxlength="15" value="${e(state.robot.name)}" placeholder="로미" />
      </div>
      ${button('이 로봇 연결하기', 'setup-pair', 'btn-primary full', 'link')}
      <button class="text-btn full mt" data-action="pair-retry">
        ${i('refresh', 14)} 기기를 다시 찾아볼게요
      </button>
    `;
  if (setupStep === 3)
    body = html`
      <div class="eyebrow">STEP 03 · STAY CONNECTED</div>
      <h1>우리 집 Wi-Fi와 연결해요.</h1>
      <p>
        로미가 집 안 어디서든 연결될 수 있도록
        <br />
        2.4GHz 네트워크를 선택해 주세요.
      </p>
      <form id="setup-wifi">
        <div class="field">
          <label class="form-label" for="wifi-name">네트워크</label>
          <select id="wifi-name">
            <option>HOMETEKHU_Home · 2.4GHz</option>
            <option>HOMETEKHU_Guest · 2.4GHz</option>
          </select>
        </div>
        <div class="field">
          <label class="form-label" for="wifi-pass">체험용 비밀번호</label>
          <input
            id="wifi-pass"
            type="password"
            placeholder="12345678"
            value="12345678"
            minlength="8"
            autocomplete="off"
            required
          />
          <small>예시 비밀번호예요. 실제 비밀번호를 입력하지 않아도 돼요.</small>
        </div>
        <div id="wifi-error" class="field-error" role="alert"></div>
        <button type="submit" class="btn btn-primary full">
          연결하고 다음으로 ${i('arrow', 17)}
        </button>
      </form>
      <button class="text-btn full mt" data-action="wifi-fail">연결 실패 상황 체험하기</button>
    `;
  if (setupStep === 4)
    body = html`
      <div class="eyebrow">STEP 04 · GET TO KNOW HOME</div>
      <h1>우리 집을 알려주세요.</h1>
      <p>
        방문을 열고 바닥의 물건을 잠깐 정리해 주세요.
        <br />
        로미가 갈 수 있는 곳을 천천히 둘러볼게요.
      </p>
      <div class="map-wrap">${floorplan(state, { scan: true })}</div>
      <div class="row-between tiny muted mt">
        <span>
          ${state.map.scan === 100
            ? '12개의 가구 후보를 찾았어요'
            : state.map.scanning
              ? '집 안을 둘러보고 있어요'
              : '탐색 준비가 됐어요'}
        </span>
        <strong>${state.map.scan}%</strong>
      </div>
      <div class="progress-track">
        <div class="progress-fill" style="width:${state.map.scan}%"></div>
      </div>
      ${state.map.scan === 100
        ? button('지도 저장하고 완료', 'setup-map-done', 'btn-primary full', 'check')
        : state.map.scanning
          ? button('잠시 멈추기', 'pause-mapping', 'btn-secondary full', 'pause')
          : button(
              state.map.scan > 0 ? '이어서 둘러보기' : '집 지도 만들기',
              'start-mapping',
              'btn-primary full',
              'scan',
            )}
      <p class="storage-notice">
        체험 속도로 약 15초 동안 탐색해요. 실제 집 구조를 스캔하지는 않아요.
      </p>
    `;
  if (setupStep === 5)
    body = html`
      <div class="success-orb">${i('check', 34)}</div>
      <div class="eyebrow center">ALL SET. WELCOME HOME.</div>
      <h1 class="center">이제, 함께할 준비가 됐어요.</h1>
      <p class="center">
        ${e(state.robot.name)}가 ${e(state.profile.home)}의 지도와 12개 관측 지점을 기억했어요.
        <br />
        첫 부탁을 건네볼까요?
      </p>
      <div class="note blue mb">${i('spark', 18)} “거실의 검은 머그컵을 침실 협탁으로 옮겨줘.”</div>
      ${button('첫 부탁 하러 가기', 'setup-finish', 'btn-primary full', 'arrow')}${button(
        '홈으로 가기',
        'setup-home',
        'btn-outline full mt',
        'home',
      )}
    `;
  return html`
    <div class="setup-shell">
      <div class="setup-top">
        <button class="text-btn" data-action="setup-back">
          ${i('back', 15)}${setupStep > 0 ? '이전 단계' : '홈으로'}
        </button>
        <div class="setup-progress" aria-label="${setupStep + 1}/6 단계">
          ${Array.from(
            { length: 6 },
            (_, n) => html`
              <span class="${n <= setupStep ? 'done' : ''}"></span>
            `,
          ).join('')}
        </div>
        <span class="demo-badge">연결 체험</span>
      </div>
      <div class="setup-layout">
        <section class="setup-form">${body}</section>
      </div>
    </div>
  `;
}
