import { html } from '../html.js';
import { icon as i, floorplan } from '../../art.js';
import { escapeHTML as e } from '../../core.js';
import { renderDelivery } from './delivery.js';

/** Render the task screen from a read-only snapshot. */
export function renderTask(context) {
  const { state, request, heading, button, selected, item, place } = context;
  const t = state.task;
  if (t?.kind === 'transport') return renderDelivery(context);
  if (!t)
    return html`
      ${heading('아직 부탁이 없어요', '로미에게 오늘의 첫 부탁을 건네보세요.')}
      <section class="card empty-state">
        ${i('spark', 44)}
        <h3>어떤 일을 도와드릴까요?</h3>
        <p>물건을 옮기거나, 원하는 곳으로 불러보세요.</p>
        ${button('부탁하러 가기', 'nav', 'btn-primary', 'arrow', 'data-page="request"')}
      </section>
    `;
  const done = t.status === 'done',
    cancelled = t.status === 'cancelled';
  return html`
    ${heading(
      done ? '부탁한 일을 마쳤어요' : cancelled ? '작업을 멈췄어요' : '로미가 도와드리고 있어요',
      done
        ? '작업 결과가 활동 기록에 저장됐어요.'
        : cancelled
          ? '새로운 부탁은 언제든 다시 시작할 수 있어요.'
          : '지금 하고 있는 일을 단계별로 확인하세요.',
    )}
    <div class="two-col">
      <section class="card card-pad">
        ${done
          ? html`
              <div class="complete-panel">
                <div class="success-orb">${i('check', 35)}</div>
                <h2>
                  ${t.kind === 'charge'
                    ? '충전을 시작했어요'
                    : t.kind === 'patrol'
                      ? '우리 집을 잘 둘러봤어요'
                      : t.kind === 'navigate'
                        ? `${e(t.destinationLabel || place(t.dest).name)}에 도착했어요`
                        : `${e(item(t.itemId)?.name)}을<br>${e(place(t.dest).name)}에 옮겼어요`}
                </h2>
                <p>
                  예시 실행 결과가 활동 기록에 저장됐어요.
                  <br />
                  다음 부탁도 함께할게요.
                </p>
                <div class="note blue">
                  ${i('check', 17)} ${e(t.stages[t.stages.length - 1])} · 시뮬레이션 완료
                </div>
                <h3 class="mt">부탁한 결과가 마음에 드나요?</h3>
                <div class="rating-row">
                  <button
                    class="${t.feedback === 'good' ? 'selected' : ''}"
                    data-action="feedback"
                    data-value="good"
                  >
                    ${i('check', 16)} 좋아요
                  </button>
                  <button
                    class="${t.feedback === 'issue' ? 'selected' : ''}"
                    data-action="feedback"
                    data-value="issue"
                  >
                    ${i('help', 16)} 확인이 필요해요
                  </button>
                </div>
                ${button('다른 부탁하기', 'new-request', 'btn-primary', 'spark')}
              </div>
            `
          : cancelled
            ? html`
                <div class="empty-state">
                  ${i('stop', 44)}
                  <h3>${e(t.reason)}</h3>
                  <p>현재 위치에서 멈췄어요. 기록에서 중단 내역을 볼 수 있어요.</p>
                  ${button('새로운 부탁', 'new-request', 'btn-primary', 'spark')}
                </div>
              `
            : html`
                <div class="task-top">
                  <div>
                    <span class="eyebrow">
                      ${t.status === 'running' ? 'IN PROGRESS' : 'PAUSED'}
                    </span>
                    <h2>${e(t.title)}</h2>
                  </div>
                  <span class="status-tag">
                    ${t.status === 'running'
                      ? '진행 중'
                      : t.status === 'blocked'
                        ? '확인 필요'
                        : '일시 정지'}
                  </span>
                </div>
                <div class="row-between tiny muted">
                  <span>${e(t.stages[t.stage])}</span>
                  <strong>${t.progress}%</strong>
                </div>
                <div class="progress-track">
                  <div class="progress-fill" style="width:${t.progress}%"></div>
                </div>
                ${t.status !== 'running'
                  ? html`
                      <div class="note ${t.status === 'blocked' ? 'warn' : 'blue'} mb">
                        ${i(t.status === 'blocked' ? 'alert' : 'pause', 18)}${e(
                          t.reason || '잠시 멈췄어요. 준비되면 이어서 진행하세요.',
                        )}
                      </div>
                    `
                  : ''}
                <div class="steps">
                  ${t.stages
                    .map(
                      (s, j) => html`
                        <div class="step ${j < t.stage ? 'done' : j === t.stage ? 'active' : ''}">
                          <div class="step-number">
                            ${j < t.stage ? i('check', 13) : String(j + 1).padStart(2, '0')}
                          </div>
                          <div class="step-text">
                            <strong>${e(s)}</strong>
                            ${j === t.stage
                              ? html`
                                  <small>
                                    ${t.status === 'running'
                                      ? '로미가 주변을 확인하며 진행하고 있어요.'
                                      : '사용자의 확인을 기다리고 있어요.'}
                                  </small>
                                `
                              : ''}
                          </div>
                        </div>
                      `,
                    )
                    .join('')}
                </div>
                <div class="task-buttons">
                  ${button(
                    t.status === 'running' ? '일시 정지' : '이어서 진행',
                    t.status === 'running' ? 'pause-task' : 'resume-task',
                    'btn-primary',
                    t.status === 'running' ? 'pause' : 'play',
                  )}${button('작업 취소', 'cancel-task', 'btn-outline', 'close')}
                </div>
                <button
                  class="text-btn full mt"
                  data-action="emergency"
                  style="color:var(--danger)"
                >
                  ${i('stop', 15)} 긴급 정지
                </button>
              `}
      </section>
      <div>
        <section class="card map-page">
          <div class="section-top">
            <h2>로미의 이동 경로</h2>
            <span class="demo-badge">시뮬레이션</span>
          </div>
          <div class="map-wrap">
            ${floorplan(state, { path: !done && !cancelled, selected: t.dest })}
          </div>
          <div class="map-legend">
            <span>
              <span class="legend-dot"></span>
              로미
            </span>
            <span>목적지 · ${e(t.destinationLabel || place(t.dest).name)}</span>
          </div>
        </section>
        <div class="note mt">
          ${i('shield', 17)}
          ${t.kind === 'transport'
            ? '대상이 다르거나 길이 막히면, 로미가 멈추고 알려드려요.'
            : '로미는 등록된 공간 안에서만 이동해요.'}
        </div>
        ${!done && !cancelled
          ? html`
              <div class="card card-pad mt">
                <div class="section-top">
                  <h3>상황 대응 체험</h3>
                  <span class="muted tiny">시연용</span>
                </div>
                <div class="row" style="flex-wrap:wrap">
                  ${button('장애물 발견', 'simulate-obstacle', 'btn-outline', 'alert')}${button(
                    '연결 끊김',
                    'disconnect',
                    'btn-outline',
                    'wifi',
                  )}${t.kind === 'transport'
                    ? button('물건을 못 찾음', 'missing-item', 'btn-outline', 'help')
                    : ''}
                </div>
              </div>
            `
          : ''}
      </div>
    </div>
  `;
}
