import { html } from '../html.js';
import { icon as i, floorplan } from '../../art.js';
import { PLACES, escapeHTML as e } from '../../core.js';
import { REQUEST_EXAMPLES } from '../../data/requests.js';

/** Render the request screen from a read-only snapshot. */
export function renderRequest(context) {
  const { roomIcon, state, request, heading, button, selected, item, liveBanner } = context;
  function requestView() {
    if (request && (request.needsItem || request.needsDest)) return clarifyView();
    return html`
      ${heading(
        '무엇을 도와드릴까요?',
        '말하듯 편하게 부탁해 주세요. 실행 전에 한 번 더 확인할게요.',
      )}${liveBanner()}
      <div class="two-col">
        <section class="card request-intro">
          <div class="request-heading">
            <span class="assistant-orb">${i('spark', 24)}</span>
            <div>
              <h2>로미에게 부탁하기</h2>
              <p>물건과 놓을 곳을 함께 알려주세요.</p>
            </div>
          </div>
          <form id="request-form">
            <div class="prompt-wrap">
              <textarea
                id="request-text"
                name="prompt"
                maxlength="300"
                aria-label="로미에게 부탁할 내용"
                placeholder="거실 테이블에 있는 검은 머그컵을&#10;침실 협탁으로 옮겨줘"
              >
${e(request?.text || '')}</textarea
              >
              <div class="prompt-actions">
                <span class="prompt-hint">한 번에 물건 하나씩 부탁해 주세요.</span>
                <button class="btn btn-primary" type="submit">${i('send', 15)} 부탁하기</button>
              </div>
            </div>
            <div id="request-error" class="field-error" role="alert"></div>
          </form>
          <div class="suggestions">
            ${Object.entries(REQUEST_EXAMPLES)
              .map(
                ([id, example]) => html`
                  <button class="chip" data-action="request-example" data-id="${id}">
                    ${i(example.icon, 13)} ${example.label}
                  </button>
                `,
              )
              .join('')}
          </div>
          <div class="request-footer">
            ${i('shield', 14)} 요청을 이해한 뒤 물건과 이동 경로를 보여드려요.
            <br />
            확인하고 시작하기 전까지 로미는 움직이지 않아요.
          </div>
        </section>
        <section class="card card-pad">
          <div class="section-top"><h2>이런 부탁을 할 수 있어요</h2></div>
          <div class="example-list">
            <button class="example-card" data-action="request-example" data-id="remote">
              <span class="quick-icon">${i('remote', 20)}</span>
              <span>
                <strong>누운 채로 리모컨 가져오기</strong>
                <small>“거실의 리모컨을 침실로 가져다줘”</small>
              </span>
              ${i('chevron', 15)}
            </button>
            <button class="example-card" data-action="request-example" data-id="cup">
              <span class="quick-icon">${i('cup', 20)}</span>
              <span>
                <strong>손이 닿지 않는 물건 옮기기</strong>
                <small>“거실의 머그컵을 침실로 옮겨줘”</small>
              </span>
              ${i('chevron', 15)}
            </button>
            <button class="example-card" data-action="come">
              <span class="quick-icon">${i('locate', 20)}</span>
              <span>
                <strong>필요한 곳으로 부르기</strong>
                <small>공간을 골라 로미를 불러보세요.</small>
              </span>
              ${i('chevron', 15)}
            </button>
            <button class="example-card" data-action="patrol">
              <span class="quick-icon">${i('scan', 20)}</span>
              <span>
                <strong>우리 집 한 바퀴 둘러보기</strong>
                <small>등록된 공간을 순서대로 살펴봐요.</small>
              </span>
              ${i('chevron', 15)}
            </button>
          </div>
          <div class="note blue mt">
            ${i('info', 16)} 체험에서는 예시 물건과 공간으로 부탁을 실행해요. 로봇과 실제로
            연결되지는 않아요.
          </div>
        </section>
      </div>
    `;
  }
  function clarifyView() {
    return html`
      ${heading(
        '조금만 더 알려주세요',
        '같은 이름의 물건이나 공간이 있으면, 먼저 확인하고 움직여요.',
      )}
      <div class="two-col">
        <section class="card card-pad">
          <div class="note blue mb">${i('spark', 18)} “${e(request.text)}”</div>
          <h2>
            ${request.kind !== 'transport'
              ? '어디로 이동할까요?'
              : request.needsItem
                ? request.ambiguous
                  ? '어떤 컵을 옮길까요?'
                  : '어떤 물건을 옮길까요?'
                : '물건을 확인했어요'}
          </h2>
          <p class="muted tiny mt">
            ${request.needsItem
              ? '최근 기억한 물건 중 하나를 선택해 주세요.'
              : '놓을 공간을 선택해 주세요.'}
          </p>
          ${request.kind === 'transport'
            ? html`
                <div class="object-grid mt">
                  ${state.items
                    .filter((x) => !request.ambiguous || x.id.includes('cup'))
                    .map(
                      (x) => html`
                        <button
                          class="object-option ${request.itemId === x.id ? 'selected' : ''}"
                          data-action="choose-item"
                          data-id="${e(x.id)}"
                        >
                          <span class="object-drawing">${i(x.icon, 34)}</span>
                          <strong>${e(x.name)}</strong>
                          <small>${e(x.surface)} · 예시 기록</small>
                        </button>
                      `,
                    )
                    .join('')}
                </div>
              `
            : ''}
          <h3 class="mt mb">
            ${request.kind === 'transport' ? '어디에 놓을까요?' : '목적지를 선택해 주세요'}
          </h3>
          <div class="room-options">
            ${PLACES.filter((x) => x.id !== 'dock')
              .map(
                (p) => html`
                  <button
                    class="room-option ${request.dest === p.id ? 'selected' : ''}"
                    data-action="choose-dest"
                    data-id="${p.id}"
                  >
                    ${i(roomIcon(p.id), 20)}${p.name}
                  </button>
                `,
              )
              .join('')}
          </div>
          <div class="modal-footer">
            ${button('다시 입력', 'request-reset', 'btn-outline', 'back')}${button(
              '부탁 확인하기',
              'confirm-clarify',
              'btn-primary',
              'arrow',
              (request.kind === 'transport' && !request.itemId) || !request.dest ? 'disabled' : '',
            )}
          </div>
        </section>
        <section class="card card-pad">
          <h2>기억하고 있는 공간</h2>
          <div class="map-wrap mt">${floorplan(state, { selected: request.dest })}</div>
          <div class="note">
            ${i('info', 17)} 기억한 위치는 마지막 관찰 기준이에요. 물건을 집기 전에 다시 확인해요.
          </div>
        </section>
      </div>
    `;
  }

  return requestView();
}
