import { html } from '../html.js';
import { icon, floorplan } from '../../art.js';
import { escapeHTML as escape, toward } from '../../core.js';
import { transportFrame } from '../../transport.js';
import { mapModeSwitch } from '../map-mode.js';

const routePath = (route) =>
  route?.length ? `M${route.map((p) => `${p.x} ${p.y}`).join(' L')}` : '';

export function deliveryCopy(task, frame) {
  const name = task.itemName || '물건';
  const destination = task.destinationLabel || '목적지';
  const copies = {
    planning: ['갈 길을 확인하고 있어요', '기억한 물건 위치와 안전한 경로를 확인해요.'],
    approaching: [`${name} 찾으러 가요`, `${task.source}에 있는 물건을 찾으러 이동해요.`],
    checking: [`${name}을 찾았어요`, '맞는 물건인지 가까이에서 다시 확인해요.'],
    picking: ['조심스럽게 집고 있어요', '물건을 잡고, 안정적으로 들었는지 확인해요.'],
    carrying: [`${toward(destination)} 가져가고 있어요`, `${name}을 안전하게 잡고 이동해요.`],
    placing: ['살며시 내려놓고 있어요', '도착한 위치에 물건을 놓고 결과를 확인해요.'],
    delivered: [`${destination}에 전달했어요`, `${name}을 내려놓았어요. 편하게 사용하세요.`],
  };
  if (task.status === 'cancelled')
    return ['부탁을 멈췄어요', '이동을 멈추고 중단 내역을 기록했어요.'];
  if (task.status === 'blocked') return ['주변을 확인해 주세요', task.reason];
  if (task.status === 'paused')
    return ['여기서 잠시 기다릴게요', task.reason || '준비되면 이어서 진행해 주세요.'];
  return copies[frame.phase];
}

function deliveryOverlay(task) {
  const frame = transportFrame(task);
  return html`
    <g class="delivery-route-layer" aria-hidden="true">
      <path class="delivery-route-ghost" d="${routePath(task.sourceRoute)}" />
      <path class="delivery-route-ghost" d="${routePath(task.route)}" />
      <path
        class="delivery-route-ink"
        data-source-route
        pathLength="100"
        d="${routePath(task.sourceRoute)}"
      />
      <path
        class="delivery-route-ink"
        data-carry-route
        pathLength="100"
        d="${routePath(task.route)}"
      />
    </g>
    <g
      class="delivery-station delivery-source"
      transform="translate(${frame.source.x} ${frame.source.y})"
      aria-hidden="true"
    >
      <circle r="6" />
      <text x="-13" y="4" text-anchor="end">집는 곳</text>
    </g>
    <g
      class="delivery-station delivery-destination"
      transform="translate(${frame.destination.x} ${frame.destination.y})"
      aria-hidden="true"
    >
      <circle class="delivery-arrival-ring" r="18" />
      <circle r="5" />
      <text x="-13" y="4" text-anchor="end">${escape(task.destinationLabel || '도착')}</text>
    </g>
    <g
      class="delivery-robot"
      data-delivery-robot
      aria-label="로미 위치"
      transform="translate(${frame.robot.x} ${frame.robot.y})"
    >
      <circle class="delivery-robot-halo" r="24" />
      <circle class="delivery-robot-body" r="13" />
      <path d="m-4 3 4-8 4 8-4-2Z" fill="white" />
      <text y="35" text-anchor="middle">로미</text>
    </g>
    <g class="delivery-grip" data-delivery-grip aria-hidden="true">
      <path data-grip-left d="M-11 -11h-5v22h5" />
      <path data-grip-right d="M11 -11h5v22h-5" />
    </g>
    <g
      class="delivery-cargo"
      data-delivery-cargo
      aria-label="${escape(task.itemName || '물건')} 위치"
      transform="translate(${frame.cargo.x} ${frame.cargo.y})"
    >
      <circle r="18" />
      <g transform="translate(-12 -12)">${icon(task.itemIcon || 'remote', 24)}</g>
      <g class="delivery-drop-check" transform="translate(13 -15)">
        <circle r="8" />
        <path d="m-3 0 2 2 4-4" />
      </g>
    </g>
  `;
}

/** One map and one compact control sheet stay in place through the entire handoff. */
export function renderDelivery({ state, item, place }) {
  const task = state.task;
  const object = item(task.itemId);
  const taskView = {
    ...task,
    itemName: task.itemName || object?.name || '물건',
    itemIcon: task.itemIcon || object?.icon || 'remote',
  };
  const [title, detail] = deliveryCopy(taskView, transportFrame(task));
  return html`
    <section
      class="delivery-experience"
      data-delivery-task="${escape(task.id)}"
      aria-label="물건 전달 진행"
    >
      <header class="delivery-heading">
        <div>
          <div class="delivery-eyebrow">
            <span></span>
            로미의 부탁 수행
            <span class="delivery-demo">시연</span>
          </div>
          <h1>${escape(taskView.itemName)} 가져오기</h1>
          <p>${escape(task.source)} ${icon('arrow', 13)} ${escape(place(task.dest).name)}</p>
        </div>
        <button class="glass-icon" data-action="nav" data-page="home" aria-label="홈으로 돌아가기">
          ${icon('close', 19)}
        </button>
      </header>

      <div class="delivery-scene">
        <div class="delivery-view-switch">${mapModeSwitch(state)}</div>
        <div class="delivery-map">
          ${floorplan(state, {
            showRobot: false,
            interactive: false,
            overlay: deliveryOverlay(taskView),
          })}
        </div>
        <div class="delivery-scene-tools">
          <span class="delivery-camera-label">움직임을 따라가는 지도</span>
          <button
            class="glass-icon"
            data-delivery-camera
            aria-label="전체 지도 보기"
            aria-pressed="false"
          >
            ${icon('map', 19)}
          </button>
        </div>
        <div class="delivery-map-note">${icon('shield', 12)}저장된 지도 안에서 이동해요</div>
      </div>

      <section class="delivery-sheet" aria-label="전달 상태와 조작">
        <div class="delivery-sheet-top">
          <span class="delivery-item-tag">
            ${icon(taskView.itemIcon, 16)}${escape(taskView.itemName)}
            <span data-cargo-status>찾는 중</span>
          </span>
          <span class="delivery-time" data-delivery-time>약 25초 남음</span>
        </div>
        <ol class="delivery-milestones" aria-label="전달 단계">
          ${['찾으러 이동', '집기', '전달']
            .map(
              (label, index) => html`
                <li data-milestone="${index}">
                  <div class="delivery-segment"><span></span></div>
                  <span class="delivery-step-label">
                    <b>${index + 1}</b>
                    ${label}
                  </span>
                </li>
              `,
            )
            .join('')}
        </ol>
        <div class="delivery-status-copy" role="status" aria-live="polite" aria-atomic="true">
          <h2 data-delivery-title>${escape(title)}</h2>
          <p data-delivery-detail>${escape(detail)}</p>
        </div>
        <div class="delivery-action-slot" data-delivery-actions></div>
        <button
          class="delivery-issue"
          data-delivery-feedback
          data-action="feedback"
          data-value="issue"
          hidden
        >
          ${icon('help', 13)}확인이 필요해요
        </button>
        <details class="delivery-options">
          <summary>도움이 필요한가요? ${icon('down', 13)}</summary>
          <div>
            <button data-action="simulate-obstacle">${icon('alert', 15)}장애물 발견 체험</button>
            <button data-action="disconnect">${icon('wifi', 15)}연결 끊김 체험</button>
            <button data-action="missing-item" data-missing-example>
              ${icon('help', 15)}물건을 못 찾음 체험
            </button>
            <button data-action="cancel-task">${icon('close', 15)}이 부탁 취소</button>
          </div>
        </details>
      </section>
    </section>
  `;
}
