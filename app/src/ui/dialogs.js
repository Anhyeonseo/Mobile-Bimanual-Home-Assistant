import { html } from './html.js';
import { icon as i, floorplan } from '../art.js';
import { PLACES, escapeHTML as e, toward } from '../core.js';

function modalFrame(title, body, wide = false, subtitle = '') {
  return html`
    <div class="overlay-backdrop" data-action="backdrop">
      <section
        class="modal ${wide ? 'wide' : ''}"
        role="dialog"
        aria-modal="true"
        aria-labelledby="modal-title"
      >
        <header class="modal-header">
          <div>
            <h2 id="modal-title">${title}</h2>
            ${subtitle
              ? html`
                  <p class="modal-subtitle">${subtitle}</p>
                `
              : ''}
          </div>
          <button class="icon-btn" data-action="close-modal" aria-label="닫기">
            ${i('close', 20)}
          </button>
        </header>
        <div class="modal-body">${body}</div>
      </section>
    </div>
  `;
}
export function renderDialog(modal, context) {
  const { state, modalData, routineDraft, button, item, place, roomIcon, mascot, selected } =
    context;
  switch (modal) {
    case 'review': {
      const q = modalData,
        it = item(q.itemId),
        to = place(q.dest);
      const transport = q.kind === 'transport';
      return modalFrame(
        '이렇게 도와드릴게요',
        html`
          <div class="note blue">
            ${i('spark', 19)}${transport
              ? `${e(it?.name)}을 ${e(toward(to.name))} 옮겨요.`
              : q.kind === 'charge'
                ? '충전 스테이션으로 돌아가 충전을 시작해요.'
                : q.kind === 'patrol'
                  ? '거실, 주방, 침실, 서재를 차례로 둘러봐요.'
                  : `${e(toward(to.name))} 이동해요.`}
          </div>
          <div class="plan-route">
            <div class="plan-place">
              <small>출발</small>
              <strong>${e(transport ? it?.surface : state.robot.room)}</strong>
            </div>
            ${i('arrow', 22)}
            <div class="plan-place">
              <small>도착</small>
              <strong>
                ${e(q.stopLabel || to.name)}${transport && q.dest === 'bedroom'
                  ? ' 협탁'
                  : transport && q.dest === 'study'
                    ? ' 책상'
                    : ''}
              </strong>
            </div>
          </div>
          <h3 class="mb">로미가 할 일</h3>
          <div class="steps">
            ${(transport
              ? [
                  '경로와 물건을 확인해요',
                  '안전하게 집어 목적지로 이동해요',
                  '내려놓고 결과를 알려드려요',
                ]
              : q.kind === 'patrol'
                ? [
                    '등록된 네 공간의 경로를 확인해요',
                    '차례대로 이동하며 둘러봐요',
                    '결과를 활동 기록에 저장해요',
                  ]
                : ['이동 경로를 확인해요', '주변을 살피며 천천히 이동해요', '도착하면 알려드려요']
            )
              .map(
                (s, k) => html`
                  <div class="step">
                    <span class="step-number">${k + 1}</span>
                    <div class="step-text"><strong>${s}</strong></div>
                  </div>
                `,
              )
              .join('')}
          </div>
          <div class="note">
            ${i('clock', 16)} 체험에서는 약 25초 걸려요. 언제든 일시 정지할 수
            있어요.${q.angle ? ` 도착 방향은 ${e(q.angle)}°예요.` : ''}
          </div>
          <div class="modal-footer">
            ${button('다시 살펴보기', 'close-modal', 'btn-outline')}${button(
              '확인하고 시작',
              'start-task',
              'btn-primary',
              'play',
            )}
          </div>
        `,
      );
    }
    case 'come':
      return modalFrame(
        '로미를 어디로 부를까요?',
        html`
          <p>
            현재 있는 공간을 선택해 주세요.
            <br />
            이동하기 전에 경로를 확인할 수 있어요.
          </p>
          <div class="room-options">
            ${PLACES.filter((p) => p.id !== 'dock')
              .map(
                (p) => html`
                  <button
                    class="room-option ${modalData.dest === p.id ? 'selected' : ''}"
                    data-action="come-select"
                    data-id="${p.id}"
                  >
                    ${i(roomIcon(p.id), 21)}${p.name}
                  </button>
                `,
              )
              .join('')}
          </div>
          <div class="modal-footer">
            ${button('취소', 'close-modal', 'btn-outline')}${button(
              '이동 확인하기',
              'come-confirm',
              'btn-primary',
              'arrow',
              !modalData.dest ? 'disabled' : '',
            )}
          </div>
        `,
      );
    case 'localize':
      return modalFrame(
        '시작 위치를 맞춰주세요',
        html`
          <p>
            로봇이 있는 곳과 가까운 위치를 선택하세요. 체험에서는 선택한 위치에서 지도 정합 과정을
            보여드려요.
          </p>
          <div class="pose-options">
            ${[
              { id: 'desk', name: '작업 책상 앞' },
              { id: 'bed', name: '침대 옆 공간' },
              { id: 'counter', name: '주방 조리대 앞' },
            ]
              .map(
                (o) => html`
                  <button
                    class="pose-option ${modalData.pose === o.id ? 'selected' : ''}"
                    data-action="pose-select"
                    data-id="${o.id}"
                  >
                    ${i('locate', 18)}
                    <span>${o.name}</span>
                    ${modalData.pose === o.id ? i('check', 16) : ''}
                  </button>
                `,
              )
              .join('')}
          </div>
          <div class="note blue mt">
            ${i('info', 16)} 현재 위치와 방향이 맞는지 확인한 뒤 이동해요. 지금은 예시 지도에서
            체험합니다.
          </div>
          <div class="modal-footer">
            ${button('취소', 'close-modal', 'btn-outline')}${button(
              '위치 확인 시작',
              'start-localize',
              'btn-primary',
              'locate',
              !modalData.pose ? 'disabled' : '',
            )}
          </div>
        `,
      );
    case 'device':
      return modalFrame(
        '나의 로봇',
        html`
          <div class="settings-device">
            ${mascot('device-mascot')}
            <div>
              <h3>ALOHA Mini</h3>
              <p>HOMETEKHU · HK-2026-001</p>
              <span class="online-tag mt" style="margin-top:8px">
                ${state.robot.connected ? '연결됨' : '연결 끊김'}
              </span>
            </div>
          </div>
          <form id="device-form">
            <div class="field">
              <label class="form-label" for="robot-name">로봇 이름</label>
              <input
                name="name"
                id="robot-name"
                value="${e(state.robot.name)}"
                maxlength="15"
                required
              />
            </div>
            <div class="device-meta mb">
              <div>
                <strong>${state.robot.battery}%</strong>
                <small>배터리</small>
              </div>
              <div>
                <strong>${e(state.robot.room)}</strong>
                <small>현재 위치</small>
              </div>
              <div>
                <strong>v${e(state.settings.firmware)}</strong>
                <small>소프트웨어</small>
              </div>
            </div>
            <div class="modal-footer">
              ${button(
                state.robot.connected ? '연결 끊김 체험' : '다시 연결',
                state.robot.connected ? 'disconnect' : 'reconnect',
                'btn-outline',
                'wifi',
              )}
              <button class="btn btn-primary" type="submit">이름 저장 ${i('check', 16)}</button>
            </div>
          </form>
          <p class="storage-notice">실제 로봇의 연결 상태나 사양을 조회하지 않는 체험 기기예요.</p>
        `,
      );
    case 'remote':
      return modalFrame(
        '로미를 직접 움직여볼까요?',
        html`
          <div class="remote-stage">
            ${floorplan(state)}
            <span class="remote-label">${i('locate', 11)} 지도 기반 수동 조작 체험</span>
          </div>
          <div class="row-between tiny muted mt">
            <span>
              위치 X ${Math.round(state.robot.position.x)} · Y ${Math.round(state.robot.position.y)}
            </span>
            <span>저속 이동 · 1회 0.2m</span>
          </div>
          <div class="dpad">
            <button data-action="drive" data-dir="up" aria-label="앞으로 이동">
              <span style="transform:rotate(-90deg)">${i('arrow')}</span>
            </button>
            <button data-action="drive" data-dir="left" aria-label="왼쪽으로 이동">
              ${i('back')}
            </button>
            <button data-action="remote-stop" aria-label="즉시 정지">${i('stop', 20)}</button>
            <button data-action="drive" data-dir="right" aria-label="오른쪽으로 이동">
              ${i('arrow')}
            </button>
            <button data-action="drive" data-dir="down" aria-label="뒤로 이동">
              <span style="transform:rotate(90deg)">${i('arrow')}</span>
            </button>
          </div>
          <div class="note">
            ${i('shield', 17)} 버튼을 누를 때마다 짧게 이동해요. 벽·가구·진입 금지 구역에는 들어가지
            않아요.
          </div>
          ${button('조작 끝내기', 'close-modal', 'btn-primary full mt', 'check')}
        `,
      );
    case 'profile':
      return modalFrame(
        '나의 프로필',
        html`
          <form id="profile-form">
            <div class="field">
              <label class="form-label" for="profile-name">이름</label>
              <input
                id="profile-name"
                name="name"
                maxlength="15"
                value="${e(state.profile.name)}"
                required
              />
            </div>
            <div class="note blue">
              ${i('user', 17)} 우리 집 관리자 · 체험 프로필
              <br />
              이름은 이 브라우저에만 저장돼요.
            </div>
            <button type="submit" class="btn btn-primary full mt">
              저장하기 ${i('check', 16)}
            </button>
          </form>
        `,
      );
    case 'home':
      return modalFrame(
        '우리 집 이름',
        html`
          <form id="home-form">
            <p class="muted tiny mb">매일 돌아오는 공간에 이름을 지어주세요.</p>
            <div class="field">
              <label class="form-label" for="home-name">집 이름</label>
              <input
                id="home-name"
                name="name"
                maxlength="24"
                value="${e(state.profile.home)}"
                required
              />
            </div>
            <button type="submit" class="btn btn-primary full">
              이 이름으로 저장 ${i('check', 16)}
            </button>
          </form>
        `,
      );
    case 'wifi':
      return modalFrame(
        'Wi-Fi 연결',
        html`
          <p>
            예시 네트워크로 연결 흐름을 확인해요.
            <br />
            실제 Wi-Fi 설정은 변경하지 않아요.
          </p>
          <form id="wifi-form">
            <div class="field">
              <label class="form-label" for="wifi-ssid">네트워크</label>
              <select id="wifi-ssid">
                <option>HOMETEKHU_Home · 2.4GHz</option>
                <option>HOMETEKHU_Guest · 2.4GHz</option>
              </select>
            </div>
            <div class="field">
              <label class="form-label" for="wifi-password">체험 비밀번호</label>
              <input id="wifi-password" type="password" minlength="8" value="12345678" required />
              <small>예시 비밀번호: 12345678 · 입력값은 저장하지 않아요.</small>
            </div>
            <button type="submit" class="btn btn-primary full">
              이 네트워크에 연결 ${i('wifi', 17)}
            </button>
          </form>
        `,
      );
    case 'mapping':
      return modalFrame(
        '우리 집을 천천히 알아가요',
        html`
          <p>
            방문을 열고 바닥을 정리해 주세요.
            <br />
            중간에 멈춰도 저장한 탐색부터 이어갈 수 있어요.
          </p>
          <div class="map-wrap">${floorplan(state, { scan: true })}</div>
          <div class="row-between tiny muted mt">
            <span>
              ${state.map.scan === 100
                ? '공간을 모두 둘러봤어요'
                : state.map.scanning
                  ? '우리 집을 둘러보고 있어요'
                  : state.map.scan > 0
                    ? '탐색을 잠시 멈췄어요'
                    : '준비가 됐어요'}
            </span>
            <strong>${state.map.scan}%</strong>
          </div>
          <div class="progress-track">
            <div class="progress-fill" style="width:${state.map.scan}%"></div>
          </div>
          ${state.map.scan === 100
            ? html`
                <div class="note blue">
                  ${i('check', 16)} 주방, 작업 공간, 침실의 가구 관측을 확인했어요. 저장한 뒤 공간
                  이름을 편집할 수 있어요.
                </div>
                <div class="modal-footer">
                  ${button('다시 탐색', 'restart-mapping', 'btn-outline', 'refresh')}${button(
                    '지도 저장',
                    'finish-mapping',
                    'btn-primary',
                    'check',
                  )}
                </div>
              `
            : html`
                <div class="modal-footer">
                  ${state.map.scanning
                    ? button('일시 정지', 'pause-mapping', 'btn-secondary', 'pause')
                    : button(
                        state.map.scan > 0 ? '이어서 탐색' : '탐색 시작',
                        'start-mapping',
                        'btn-primary',
                        'scan',
                      )}${button(
                    '저장하고 나중에',
                    'save-draft',
                    'btn-outline',
                    'clock',
                    state.map.scan === 0 ? 'disabled' : '',
                  )}
                </div>
              `}
          <p class="storage-notice">
            예시 지도로 탐색 흐름을 체험해요. 실제 공간을 촬영하거나 스캔하지 않아요.
          </p>
        `,
      );
    case 'furniture': {
      const f = modalData.id
        ? state.furniture.find((x) => x.id === modalData.id)
        : { name: '', room: '거실', alias: '', icon: 'table' };
      return modalFrame(
        modalData.id ? '익숙한 이름을 알려주세요' : '새로운 가구 이름 등록',
        html`
          <form id="furniture-form">
            <div class="field">
              <label class="form-label" for="furniture-name">가구 이름</label>
              <input
                id="furniture-name"
                name="name"
                value="${e(f.name)}"
                maxlength="30"
                placeholder="예: 거실 테이블"
                required
              />
            </div>
            <div class="field">
              <label class="form-label" for="furniture-room">어느 공간에 있나요?</label>
              <select id="furniture-room" name="room">
                ${[
                  ...new Set([f.room, ...PLACES.filter((x) => x.id !== 'dock').map((p) => p.name)]),
                ]
                  .map((room) => html`<option${selected(room, f.room)}>${e(room)}</option>`)
                  .join('')}
              </select>
            </div>
            <div class="field">
              <label class="form-label" for="furniture-alias">이렇게 불러도 알아들어요</label>
              <input
                id="furniture-alias"
                name="alias"
                value="${e(f.alias)}"
                maxlength="100"
                placeholder="예: 소파 테이블, 낮은 탁자"
              />
              <small>여러 이름은 쉼표로 나눠주세요.</small>
            </div>
            <div class="note blue">
              ${i('spark', 17)} 저장한 별칭은 부탁할 때 목적지를 이해하는 데 사용돼요.
            </div>
            <div class="modal-footer">
              ${modalData.id
                ? button('삭제', 'delete-furniture', 'btn-outline', 'trash')
                : button('취소', 'close-modal', 'btn-outline')}
              <button type="submit" class="btn btn-primary">이름 저장 ${i('check', 16)}</button>
            </div>
          </form>
        `,
      );
    }
    case 'zone':
      return modalFrame(
        state.map.zone ? '진입 금지 구역을 해제할까요?' : '이곳은 로미가 들어가지 않아요',
        html`
          <div class="map-wrap">${floorplan({ ...state, map: { ...state.map, zone: true } })}</div>
          <p class="mt">
            침실 아래 운동기구 주변 · 예시 금지 구역
            <br />
            화면의 표시된 공간을 진입 금지 구역으로
            ${state.map.zone ? '설정해 두었어요.' : '설정해요.'}
          </p>
          <div class="note">
            ${i('shield', 16)} 보호가 필요한 공간을 비워둘 수 있어요. 수동 조작에서도 이 영역으로
            들어가지 않아요.
          </div>
          <div class="modal-footer">
            ${button('취소', 'close-modal', 'btn-outline')}${button(
              state.map.zone ? '금지 구역 해제' : '이 구역 설정',
              'confirm-zone',
              'btn-primary',
              'check',
            )}
          </div>
        `,
      );
    case 'routine': {
      const r = routineDraft;
      return modalFrame(
        r.id ? '나의 루틴 편집' : '새로운 루틴 만들기',
        html`
          <form id="routine-form">
            <div class="field">
              <label class="form-label" for="routine-name">루틴 이름</label>
              <input
                id="routine-name"
                name="name"
                value="${e(r.name)}"
                maxlength="30"
                placeholder="예: 가볍게 시작하는 아침"
                required
              />
            </div>
            <div class="form-row">
              <div class="field">
                <label class="form-label" for="routine-time">실행 시간</label>
                <input type="time" id="routine-time" name="time" value="${e(r.time)}" required />
              </div>
              <div class="field">
                <label class="form-label" for="routine-kind">할 일</label>
                <select id="routine-kind" name="kind">
                  <option value="transport" ${selected(r.kind, 'transport')}>물건 옮기기</option>
                  <option value="charge" ${selected(r.kind, 'charge')}>충전하기</option>
                </select>
              </div>
            </div>
            <div class="field">
              <label class="form-label">반복 요일</label>
              <div class="day-picker">
                ${['일', '월', '화', '수', '목', '금', '토']
                  .map(
                    (d, k) => html`
                      <button
                        type="button"
                        class="day-button ${r.days.includes(k) ? 'selected' : ''}"
                        data-action="routine-day"
                        data-id="${k}"
                        aria-pressed="${r.days.includes(k)}"
                      >
                        ${d}
                      </button>
                    `,
                  )
                  .join('')}
              </div>
              <small id="days-error" role="alert"></small>
            </div>
            <div
              id="routine-transport-fields"
              class="form-row ${r.kind === 'charge' ? 'hidden' : ''}"
            >
              <div class="field">
                <label class="form-label" for="routine-item">옮길 물건</label>
                <select id="routine-item" name="item">
                  ${state.items
                    .map(
                      (x) => html`
                        <option value="${e(x.id)}" ${selected(r.item, x.id)}>${e(x.name)}</option>
                      `,
                    )
                    .join('')}
                </select>
              </div>
              <div class="field">
                <label class="form-label" for="routine-dest">목적지</label>
                <select id="routine-dest" name="dest">
                  ${PLACES.filter((x) => x.id !== 'dock')
                    .map(
                      (p) => html`
                        <option value="${p.id}" ${selected(r.dest, p.id)}>${p.name}</option>
                      `,
                    )
                    .join('')}
                </select>
              </div>
            </div>
            <div class="note blue">
              ${i('clock', 16)} 저장 후 ‘지금 실행’으로 체험해 보세요. 예약 시간에 자동 실행되지는
              않아요.
            </div>
            <div class="modal-footer">
              ${r.id
                ? button('루틴 삭제', 'delete-routine', 'btn-outline', 'trash')
                : button('취소', 'close-modal', 'btn-outline')}
              <button type="submit" class="btn btn-primary">루틴 저장 ${i('check', 16)}</button>
            </div>
          </form>
        `,
      );
    }
    case 'history': {
      const h = state.history.find((x) => x.id === modalData.id);
      if (!h) return '';
      return modalFrame(
        '함께한 순간의 기록',
        html`
          <div class="row mb">
            <span class="assistant-orb">${i(h.icon, 24)}</span>
            <div>
              <h3>${e(h.title)}</h3>
              <p class="muted tiny" style="margin-top:5px">${e(h.time)} · ${e(h.duration)}</p>
            </div>
          </div>
          <div class="plan-route">
            <div class="plan-place">
              <small>활동 내용</small>
              <strong>${e(h.subtitle)}</strong>
            </div>
            <span class="status-tag ${h.status === 'cancelled' ? 'cancelled' : ''}">
              ${h.status === 'done' ? '완료' : '중단'}
            </span>
          </div>
          ${h.feedback
            ? html`
                <div class="note blue mb">${i('help', 17)} 남긴 의견: ${e(h.feedback)}</div>
              `
            : ''}
          <div class="timeline-mini">
            요청 확인
            <br />
            ${h.status === 'done'
              ? '이동·상태 확인<br>결과 기록 저장'
              : '사용자 또는 연결 상태에 따라 중단<br>현재 위치에서 정지'}
          </div>
          <div class="note">
            ${i('info', 17)} 체험 기록이에요. 실제 로봇의 사진이나 센서 결과는 포함하지 않아요.
          </div>
          <div class="modal-footer">
            ${button('닫기', 'close-modal', 'btn-outline')}${h.kind !== 'map'
              ? button('같은 부탁 다시 하기', 'repeat-task', 'btn-primary', 'refresh')
              : button('지도 보기', 'history-map', 'btn-primary', 'map')}
          </div>
        `,
      );
    }
    case 'notifications':
      return modalFrame(
        '로미가 전하는 소식',
        `${
          state.notifications.length
            ? state.notifications
                .slice(0, 20)
                .map(
                  (n) => html`
                    <div class="notification-item">
                      <span class="quick-icon">${i('bell', 18)}</span>
                      <div>
                        <strong>${e(n.title)}</strong>
                        <p>${e(n.text)}</p>
                      </div>
                    </div>
                  `,
                )
                .join('')
            : '<div class="empty-state">새로운 소식이 없어요.</div>'
        }${button('모두 확인했어요', 'read-notifications', 'btn-primary full mt', 'check')}`,
      );
    case 'firmware':
      return modalFrame(
        '로미는 최신 상태예요',
        html`
          <div class="success-orb">${i('check', 34)}</div>
          <h2 class="center">소프트웨어 v${e(state.settings.firmware)}</h2>
          <p class="center mt">
            체험 기기에 적용된 예시 버전이에요.
            <br />
            업데이트가 준비되면 이곳에서 확인할 수 있어요.
          </p>
          <div class="timeline-mini">
            공간 이름과 별칭 이해 개선
            <br />
            이동 중 상태 안내 개선
            <br />
            반복 루틴 관리 추가
          </div>
          ${button('업데이트 확인', 'check-firmware', 'btn-primary full', 'refresh')}
        `,
      );
    case 'family':
      return modalFrame(
        '가족과 함께 쓰는 로미',
        html`
          <p>
            누가 로미를 사용하는지 한눈에 확인해요.
            <br />
            체험 멤버를 추가해 관리 흐름을 살펴보세요.
          </p>
          ${state.family
            .map(
              (f, k) => html`
                <div class="toggle-row">
                  <div class="row">
                    <span class="avatar">${e(f.name.slice(0, 1))}</span>
                    <div>
                      <strong>${e(f.name)}</strong>
                      <small>${e(f.role)}</small>
                    </div>
                  </div>
                  ${k
                    ? html`
                        <button
                          class="icon-btn"
                          data-action="remove-member"
                          data-id="${k}"
                          aria-label="${e(f.name)} 멤버 삭제"
                        >
                          ${i('trash', 17)}
                        </button>
                      `
                    : '<span class="status-tag">나</span>'}
                </div>
              `,
            )
            .join('')}
          <form id="family-form" class="mt">
            <div class="field">
              <label class="form-label" for="family-name">체험 멤버 이름</label>
              <input id="family-name" name="name" maxlength="15" placeholder="예: 민서" required />
            </div>
            <button type="submit" class="btn btn-primary full">
              체험 멤버 추가 ${i('plus', 17)}
            </button>
          </form>
          <p class="storage-notice">
            실제 초대나 알림을 보내지 않아요. 가족 계정 연동은 실제 로봇 서비스 연결 후 제공할 수
            있어요.
          </p>
        `,
      );
    case 'privacy':
      return modalFrame(
        '우리 집의 정보는 소중하니까',
        html`
          <h3>이 브라우저에만 저장해요</h3>
          <p class="muted tiny mt">
            이름, 가구 별칭, 루틴, 예시 활동 기록은 브라우저의 로컬 저장소에 보관돼요. 같은
            브라우저에서 다시 열면 이어서 사용할 수 있어요.
          </p>
          <h3 class="mt">실내를 촬영하지 않아요</h3>
          <p class="muted tiny mt">
            지도와 물건은 시연용 데이터예요. 카메라·마이크·Bluetooth 권한을 요청하지 않으며, 실제
            Wi-Fi 비밀번호를 수집하거나 저장하지 않아요.
          </p>
          <h3 class="mt">기록은 언제든 초기화할 수 있어요</h3>
          <p class="muted tiny mt">
            설정의 ‘체험 데이터 초기화’에서 이름, 지도 설정, 루틴, 활동을 처음 상태로 되돌릴 수
            있어요.
          </p>
          ${button('확인했어요', 'close-modal', 'btn-primary full mt', 'check')}
        `,
      );
    case 'help':
      return modalFrame(
        '로미와 더 편하게 지내기',
        html`
          <details class="help-item" open>
            <summary>어떤 부탁을 할 수 있나요?</summary>
            <p>
              예시 물건인 머그컵, 물병, 리모컨, 책을 등록된 공간으로 옮길 수 있어요. 원하는 공간으로
              부르기, 집 둘러보기, 충전 스테이션 복귀도 체험할 수 있어요.
            </p>
          </details>
          <details class="help-item">
            <summary>로미가 연결되지 않으면 어떻게 하나요?</summary>
            <p>
              기기 관리에서 ‘다시 연결’을 누르세요. 실제 서비스에서는 로봇 전원과 같은 2.4GHz Wi-Fi
              연결을 확인하는 흐름이에요. 체험에서는 재연결 상태만 바뀝니다.
            </p>
          </details>
          <details class="help-item">
            <summary>부탁을 잘못 이해했어요.</summary>
            <p>
              실행 확인 화면에서 ‘다시 살펴보기’를 누르고 요청을 고쳐 주세요. 컵처럼 대상이 모호하면
              직접 선택할 수 있어요. 지도에서 가구 별칭을 등록하면 목적지 이해에 반영돼요.
            </p>
          </details>
          <details class="help-item">
            <summary>작업 중 로미를 멈추려면요?</summary>
            <p>
              진행 화면의 ‘일시 정지’로 잠깐 멈추거나, ‘작업 취소’로 종료할 수 있어요. ‘긴급 정지’를
              누르면 즉시 멈추며, 해제해도 자동으로 다시 움직이지 않아요.
            </p>
          </details>
          <details class="help-item">
            <summary>실제 로봇이 움직이는 앱인가요?</summary>
            <p>
              현재는 시연용 앱이에요. 조작, 지도, 물체 확인, Wi-Fi 연결, 루틴은 예시 데이터로
              동작해요. 실제 앱으로 출시하려면 로봇 통신, 인증, 센서, 지도, 알림 서비스를 연결해야
              해요.
            </p>
          </details>
          <a class="download-link mt" href="guide.html" target="_blank" rel="noopener">
            ${i('book', 16)} 전체 시연 시나리오 보기
          </a>
        `,
      );
    case 'feedback':
      return modalFrame(
        '어떤 점을 확인할까요?',
        html`
          <form id="feedback-form">
            <p class="muted tiny mb">남겨주신 내용은 이 작업의 기록에 함께 저장해요.</p>
            <div class="field">
              <label class="form-label" for="feedback-text">확인이 필요한 내용</label>
              <textarea
                id="feedback-text"
                name="feedback"
                maxlength="400"
                placeholder="예: 원하는 물건과 달라요"
                required
              ></textarea>
            </div>
            <button class="btn btn-primary full" type="submit">
              의견 저장하기 ${i('check', 16)}
            </button>
          </form>
        `,
      );
    case 'cancel':
      return modalFrame(
        '이 부탁을 그만할까요?',
        html`
          <p>
            지금 진행하던 ${e(state.task?.title)} 작업을 종료해요.
            <br />
            취소한 내역은 활동 기록에 남아요.
          </p>
          <div class="modal-footer">
            ${button('계속하기', 'close-modal', 'btn-outline')}${button(
              '작업 취소',
              'confirm-cancel',
              'btn-danger',
              'stop',
            )}
          </div>
        `,
      );
    case 'reset':
      return modalFrame(
        '체험을 처음 상태로 되돌릴까요?',
        html`
          <p>
            이 브라우저에서 수정한 이름, 가구 별칭, 루틴, 활동 기록이 사라지고 처음의 예시 데이터로
            돌아가요.
          </p>
          <div class="note warn">
            ${i('info', 17)} 필요한 기록은 ‘활동 기록 → 기록 내보내기’에서 먼저 저장할 수 있어요.
          </div>
          <div class="modal-footer">
            ${button('유지하기', 'close-modal', 'btn-outline')}${button(
              '체험 데이터 초기화',
              'confirm-reset',
              'btn-danger',
              'refresh',
            )}
          </div>
        `,
      );
    case 'release':
      return modalFrame(
        '주변이 안전한가요?',
        html`
          <p>
            로미 주변의 장애물과 상태를 확인한 뒤 정지를 해제해 주세요. 해제 후에도 진행하던 작업은
            일시 정지 상태로 남아요.
          </p>
          <div class="modal-footer">
            ${button('정지 유지', 'close-modal', 'btn-outline')}${button(
              '확인하고 해제',
              'confirm-release',
              'btn-primary',
              'check',
            )}
          </div>
        `,
      );
    default:
      return '';
  }
}
