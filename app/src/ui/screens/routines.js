import { html } from '../html.js';
import { icon as i } from '../../art.js';
import { escapeHTML as e, dayLabel } from '../../core.js';

/** Render the routines screen from a read-only snapshot. */
export function renderRoutines(context) {
  const { state, heading, button, item, place } = context;
  return html`
    ${heading(
      '나의 루틴',
      '반복할 부탁의 시간과 요일을 설정하세요.',
      button('루틴 만들기', 'add-routine', 'btn-primary', 'plus'),
    )}
    <div class="note blue">
      ${i('clock', 18)} 체험에서는 루틴을 저장하고 ‘지금 실행’으로 동작을 확인할 수 있어요. 실제
      시간 예약은 로봇 연결 후 이용해요.
    </div>
    <div class="routine-grid">
      ${state.routines
        .map(
          (r) => html`
            <article class="card routine-detail">
              <div class="routine-detail-top">
                <span class="routine-icon">${i(r.icon, 21)}</span>
                <button
                  class="toggle ${r.enabled ? 'on' : ''}"
                  role="switch"
                  aria-checked="${r.enabled}"
                  aria-label="${e(r.name)} 켜기"
                  data-action="toggle-routine"
                  data-id="${e(r.id)}"
                ></button>
              </div>
              <h2>${e(r.name)}</h2>
              <p>
                ${r.kind === 'charge'
                  ? '충전 스테이션으로 돌아가기'
                  : `${e(item(r.item)?.name || '물건')} → ${e(place(r.dest).name)}`}
              </p>
              <div class="routine-time">${e(r.time)}</div>
              <span class="routine-days">
                ${e(dayLabel(r.days))} ${r.enabled ? '· 사용 중' : '· 꺼짐'}
              </span>
              <div class="routine-detail-footer">
                <button class="text-btn" data-action="edit-routine" data-id="${e(r.id)}">
                  ${i('edit', 13)} 수정
                </button>
                <button class="text-btn" data-action="run-routine" data-id="${e(r.id)}">
                  ${i('play', 13)} 지금 실행
                </button>
              </div>
            </article>
          `,
        )
        .join('')}
    </div>
    ${!state.routines.length
      ? '<section class="card empty-state"><h3>나만의 첫 루틴을 만들어 보세요</h3><p>반복하는 작은 부탁부터 시작해요.</p></section>'
      : ''}
    <section class="card card-pad">
      <div class="section-top">
        <h2>이런 루틴은 어때요?</h2>
        <span class="muted tiny">일상의 작은 힌트</span>
      </div>
      <div class="two-col">
        <button class="example-card" data-action="routine-template" data-id="morning">
          <span class="routine-icon">${i('sun', 21)}</span>
          <span>
            <strong>업무 시작 전, 물 한 병</strong>
            <small>평일 아침 9시 · 물병을 서재 책상으로</small>
          </span>
          ${i('plus', 17)}
        </button>
        <button class="example-card" data-action="routine-template" data-id="night">
          <span class="routine-icon">${i('moon', 21)}</span>
          <span>
            <strong>잠들기 전, 책 한 권</strong>
            <small>매일 저녁 9시 · 읽던 책을 침실로</small>
          </span>
          ${i('plus', 17)}
        </button>
      </div>
    </section>
  `;
}
