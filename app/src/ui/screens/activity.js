import { html } from '../html.js';
import { icon as i } from '../../art.js';
import { escapeHTML as e } from '../../core.js';

/** Render the activity screen from a read-only snapshot. */
export function renderActivity(context) {
  const { state, activityFilter, heading, button, selected } = context;
  const hs = state.history.filter((h) => activityFilter === 'all' || h.status === activityFilter);
  const count = state.history.filter((h) => h.status === 'done').length;
  return html`
    ${heading(
      '활동 기록',
      '로미가 도운 일과 우리 집의 변화를 확인하세요.',
      button('기록 내보내기', 'export-history', 'btn-outline', 'download'),
    )}
    <div class="activity-summary">
      <div class="card stat-card">
        <div>
          <small>완료한 부탁</small>
          <strong>
            ${count}
            <span style="font-size:13px">회</span>
          </strong>
        </div>
        ${i('check', 26)}
      </div>
      <div class="card stat-card">
        <div>
          <small>등록한 공간</small>
          <strong>
            ${state.map.ready ? 4 : 0}
            <span style="font-size:13px">개</span>
          </strong>
        </div>
        ${i('map', 26)}
      </div>
      <div class="card stat-card">
        <div>
          <small>함께하는 루틴</small>
          <strong>
            ${state.routines.filter((r) => r.enabled).length}
            <span style="font-size:13px">개</span>
          </strong>
        </div>
        ${i('clock', 26)}
      </div>
    </div>
    <div class="tabs" role="tablist" aria-label="기록 필터">
      ${[
        ['all', '전체 기록'],
        ['done', '완료'],
        ['cancelled', '중단'],
      ]
        .map(
          ([id, label]) => html`
            <button
              role="tab"
              aria-selected="${activityFilter === id}"
              class="tab ${activityFilter === id ? 'active' : ''}"
              data-action="filter-history"
              data-id="${id}"
            >
              ${label}
            </button>
          `,
        )
        .join('')}
    </div>
    <section class="card activity-list">
      ${hs
        .map(
          (h) => html`
            <button class="activity-row" data-action="history-detail" data-id="${e(h.id)}">
              <span class="quick-icon">${i(h.icon, 20)}</span>
              <span class="activity-info">
                <strong>${e(h.title)}</strong>
                <p>${e(h.subtitle)} · ${e(h.duration)}</p>
              </span>
              <time>${e(h.time)}</time>
              <span class="status-tag ${h.status === 'cancelled' ? 'cancelled' : ''}">
                ${h.status === 'done' ? '완료' : '중단'}
              </span>
              ${i('chevron', 16)}
            </button>
          `,
        )
        .join('') ||
      html`
        <div class="empty-state">
          ${i('activity', 37)}
          <h3>아직 기록이 없어요</h3>
          <p>로미와 함께한 활동이 여기에 쌓여요.</p>
        </div>
      `}
    </section>
  `;
}
