import { icon } from '../art.js';
import { html } from './html.js';

const TABS = [
  { page: 'home', icon: 'home', label: '홈' },
  { page: 'request', icon: 'spark', label: '부탁' },
  { page: 'map', icon: 'map', label: '공간' },
  { page: 'routines', icon: 'clock', label: '루틴' },
  { page: 'activity', icon: 'activity', label: '기록' },
];

/** The same floating navigation is used on phones and tablets. */
export function renderShell(
  content,
  { page, previousPage, unread, activeTask, emergency, demoBar },
) {
  const current = page === 'task' ? 'request' : page;
  const activeIndex = TABS.findIndex((tab) => tab.page === current);
  const previousIndex = Math.max(
    0,
    TABS.findIndex((tab) => tab.page === previousPage),
  );

  return html`
    <div class="app-shell" data-page="${page}">
      <header class="app-header">
        <button class="app-brand" data-action="nav" data-page="home" aria-label="HOMETEKHU 홈">
          <img src="assets/hometekhu-logo.svg" alt="HOMETEKHU" width="150" height="20" />
        </button>
        <div class="app-header-actions">
          <button class="preview-pill" data-action="demo" aria-label="시연 가이드">
            ${icon('play', 12)}
            <span>체험</span>
          </button>
          ${activeTask
            ? html`
                <button class="emergency-pill" data-action="emergency" aria-label="긴급 정지">
                  ${icon('stop', 16)}
                </button>
              `
            : ''}
          <button
            class="glass-icon"
            data-action="notifications"
            aria-label="알림${unread ? ', 읽지 않은 알림 있음' : ''}"
          >
            ${icon('bell', 21)}${unread ? '<i class="notification-dot"></i>' : ''}
          </button>
          <button
            class="glass-icon ${page === 'settings' ? 'selected' : ''}"
            data-action="nav"
            data-page="settings"
            aria-label="설정"
          >
            ${icon('settings', 21)}
          </button>
        </div>
      </header>

      ${emergency
        ? html`
            <div class="emergency-banner">
              <span>${icon('alert', 17)}긴급 정지 상태예요.</span>
              <button class="btn btn-outline" data-action="release-stop">정지 해제</button>
            </div>
          `
        : ''}

      <main class="main ${page === 'home' ? 'native-home-main' : ''}">${content}</main>

      <nav
        class="app-tabbar"
        aria-label="주요 메뉴"
        style="--tab-index:${activeIndex};--previous-tab:${previousIndex}"
      >
        ${activeIndex >= 0 ? '<span class="tab-lens" aria-hidden="true"></span>' : ''}
        ${TABS.map(
          (tab) => html`
            <button
              class="app-tab ${current === tab.page ? 'active' : ''}"
              data-action="nav"
              data-page="${tab.page}"
              aria-label="${tab.label}"
              ${current === tab.page ? 'aria-current="page"' : ''}
            >
              ${icon(tab.icon, 23)}
              <span>${tab.label}</span>
            </button>
          `,
        ).join('')}
      </nav>
      ${demoBar}
    </div>
  `;
}
