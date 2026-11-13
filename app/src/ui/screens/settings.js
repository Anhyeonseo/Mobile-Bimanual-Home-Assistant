import { html } from '../html.js';
import { icon as i } from '../../art.js';
import { escapeHTML as e } from '../../core.js';

/** Render the settings screen from a read-only snapshot. */
export function renderSettings(context) {
  const { state, heading, button, selected } = context;
  const toggleRow = (key, title, desc) => html`
    <div class="toggle-row">
      <div>
        <strong>${title}</strong>
        <small>${desc}</small>
      </div>
      <button
        class="toggle ${state.settings[key] ? 'on' : ''}"
        role="switch"
        aria-checked="${state.settings[key]}"
        aria-label="${title}"
        data-action="setting-toggle"
        data-id="${key}"
      ></button>
    </div>
  `;
  return html`
    ${heading('우리 집에 맞게', '로미와 함께하는 방식을 편하게 조절해 보세요.')}
    <div class="settings-grid">
      <section class="card settings-section">
        <h2>${i('robot', 20)} 나의 로봇</h2>
        <div class="settings-device">
          <div>
            <h3>${e(state.robot.name)}</h3>
            <p>ALOHA Mini · HK-2026-001</p>
            <span class="online-tag mt" style="margin-top:8px">
              ${state.robot.connected ? '연결됨' : '연결 끊김'}
            </span>
          </div>
        </div>
        <button class="setting-link" data-action="device">
          ${i('settings', 18)}
          <span>로봇 이름·기기 정보</span>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="wifi">
          ${i('wifi', 18)}
          <span>Wi-Fi 연결</span>
          <small>${state.robot.connected ? 'HOMETEKHU_Home' : '연결 필요'}</small>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="firmware">
          ${i('download', 18)}
          <span>소프트웨어 업데이트</span>
          <small>v${e(state.settings.firmware)}</small>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="remote">
          ${i('locate', 18)}
          <span>수동 조작</span>
          ${i('chevron', 15)}
        </button>
      </section>
      <section class="card settings-section">
        <h2>${i('user', 20)} 나와 우리 집</h2>
        <button class="setting-link" data-action="profile">
          <span class="avatar">${e(state.profile.name.slice(0, 1))}</span>
          <span>${e(state.profile.name)} 님</span>
          <small>프로필 편집</small>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="home-settings">
          ${i('home', 18)}
          <span>집 이름</span>
          <small>${e(state.profile.home)}</small>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="family">
          ${i('users', 18)}
          <span>가족 함께 사용</span>
          <small>${state.family.length}명</small>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="setup-start">
          ${i('plus', 18)}
          <span>새 로봇 연결 체험</span>
          ${i('chevron', 15)}
        </button>
        <div class="note blue mt">
          ${i('info', 17)} 개인 설정은 이 브라우저에 저장돼요. 계정 가입 없이 기능을 체험할 수
          있어요.
        </div>
      </section>
      <section class="card settings-section">
        <h2>${i('bell', 20)} 알림과 사용 환경</h2>
        ${toggleRow(
          'notifications',
          '작업 알림',
          '작업 완료와 확인이 필요한 순간을 앱에서 알려줘요.',
        )}${toggleRow(
          'sound',
          '로봇 소리',
          '작업 시작과 종료 때 소리로 알려주는 설정이에요.',
        )}${toggleRow('avoidPets', '반려동물 배려 모드', '이동 속도를 줄이고 여유 있게 지나가요.')}
        <div class="field mt">
          <label class="form-label" for="robot-speed">이동 속도</label>
          <select id="robot-speed">
            <option ${selected(state.settings.speed, '천천히')}>천천히</option>
            <option ${selected(state.settings.speed, '보통')}>보통</option>
          </select>
        </div>
      </section>
      <section class="card settings-section">
        <h2>${i('shield', 20)} 개인정보와 도움말</h2>
        ${toggleRow(
          'photos',
          '작업 사진 저장',
          '실제 로봇 연결 후 촬영하는 사진의 저장 설정이에요.',
        )}
        <button class="setting-link" data-action="privacy">
          ${i('shield', 18)}
          <span>데이터와 개인정보 안내</span>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="help">
          ${i('help', 18)}
          <span>자주 묻는 질문</span>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="demo">
          ${i('play', 18)}
          <span>시연 가이드</span>
          ${i('chevron', 15)}
        </button>
        <button class="setting-link" data-action="reset" style="color:var(--danger)">
          ${i('refresh', 18)}
          <span>체험 데이터 초기화</span>
          ${i('chevron', 15)}
        </button>
      </section>
    </div>
  `;
}
