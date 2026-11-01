'use strict';
const $ = id => document.getElementById(id);
let token = '', state = null, map = null, request = null, preview = null, scanId = null;
let busy = false, generation = 0, timer = null, lastRegions = '', lastRequestKey = '';
const spaceLabels = {living_room:'거실', bedroom:'침실', kitchen:'주방', bathroom:'욕실', study:'서재'};
const terminal = new Set(['SUCCEEDED', 'FAILED', 'CANCELLED', 'INTERRUPTED']);
const labels = {
  READY_TO_PLAN: '대상과 목적지를 확인했어요', NEEDS_CLARIFICATION: '조금 더 알려주세요',
  SEARCH_REQUIRED: '최근 기억에서 찾지 못했어요', REOBSERVE_REQUIRED: '물체를 다시 확인해야 해요',
  EXECUTION_REQUESTED: '동작을 준비하고 있어요', WAITING: '다음 동작을 준비하고 있어요', RUNNING: '요청한 일을 진행하고 있어요',
  SUCCEEDED: '소프트웨어 작업을 완료했어요', FAILED: '작업을 완료하지 못했어요', CANCELLED: '취소했어요',
  INTERRUPTED: '서버 재시작으로 작업이 중단됐어요', STOPPING: '정지를 확인하고 있어요',
  STOP_UNCONFIRMED: '정지 여부를 아직 확인하지 못했어요', CANCEL_REQUEST: '진행 중인 작업 취소를 요청했어요',
  NO_ACTIVE_TASK: '현재 진행 중인 작업이 없어요', SCANNING: '스캔 기록을 모으고 있어요',
  PAUSED: '스캔을 잠시 멈췄어요', REVIEW: '공간 이름을 확인하세요', SAVED: '스캔을 저장했어요', DISCARDED: '스캔을 종료했어요'
};
const reasons = {
  ambiguous_object: '조건에 맞는 물체가 여러 개예요. 하나를 선택해 주세요.',
  destination_required: '어디에 놓을까요?', unknown_destination: '놓을 장소를 찾지 못했어요. 아래에서 선택해 주세요.',
  ambiguous_destination: '목적지가 여러 곳이에요. 장소를 선택해 주세요.',
  placement_surface_required: '방 안에서 물체를 놓을 자리를 선택해 주세요.',
  unknown_source: '출발 장소를 찾지 못했어요.', ambiguous_source: '어느 장소에 있는 물건인지 선택해 주세요.',
  unsupported_action: '지금은 “물체를 찾아줘”, “물체를 장소에 놓아줘” 요청을 지원해요.',
  object_category_required: '마커펜, 컵, 리모컨처럼 물체 종류를 알려주세요.',
  multiple_object_categories: '한 번에 한 종류의 물체를 요청해 주세요.',
  unsupported_negation_or_multi_action: '조건이 복잡해요. 한 번에 옮길 물체와 목적지를 하나씩 알려주세요.',
  unsupported_object_constraint: '이 설명은 아직 구분하지 못해요. 물체 종류와 색, 장소로 알려주세요.',
  ambiguous_color: '물체의 색을 하나로 알려주세요.', unauthorized: '접속 키를 확인해 주세요.',
  source_label_unconfirmed: '출발 구역의 이름이 아직 자동 제안 상태예요. 스캔 검토에서 확인해 주세요.',
  destination_label_unconfirmed: '목적지의 이름이 아직 자동 제안 상태예요. 스캔 검토에서 확인해 주세요.',
  localization_required: '예시 시작 위치 확인을 누른 뒤 다시 계획해 주세요.',
  reobserve_required: '예시 관측 갱신 후 새 요청을 입력해 주세요.', scan_active: '스캔을 저장하거나 종료한 뒤 실행해 주세요.',
  robot_busy: '진행 중인 작업을 먼저 마치거나 취소해 주세요.', robot_or_scan_busy: '작업이나 스캔이 진행 중이에요.',
  preview_invalidated: '관측이나 위치가 바뀌었어요. 계획을 다시 확인해 주세요.',
  physical_execution_not_commissioned: '실물 보정과 장치 검증이 필요한 데이터예요. 지금은 실행할 수 없어요.',
  payload_recovery_required: '이전 작업의 물체 상태 확인이 필요해요.',
  fresh_request_permit_required: '실행 요청 시간이 지났어요. 상태를 확인한 뒤 다시 눌러 주세요.',
  selected_object_does_not_match_request: '현재 요청 조건과 맞지 않는 물체예요.',
  scan_not_scanning: '스캔을 시작하거나 이어한 뒤 기록을 넣어 주세요.',
  gateway_fault_latched: '서버에서 문제가 확인됐어요. 상태와 로그를 확인해야 해요.'
};
function el(tag, content, className) { const node = document.createElement(tag); if (content !== undefined) node.textContent = content; if (className) node.className = className; return node; }
function message(error) { $('error').textContent = error ? (reasons[error.message] || (error.name === 'TimeoutError' ? '응답을 기다리는 중이에요. 실행 결과를 먼저 확인해 주세요.' : '요청을 처리하지 못했어요. 입력과 연결 상태를 확인해 주세요.')) : ''; }
async function api(path, body, permit) {
  const response = await fetch(path, {method: body === undefined ? 'GET' : 'POST',
    headers: {Authorization: 'Bearer ' + token, ...(body === undefined ? {} : {'Content-Type': 'application/json'}), ...(permit ? {'X-Request-Permit': permit} : {})},
    body: body === undefined ? undefined : JSON.stringify(body), cache: 'no-store', signal: AbortSignal.timeout(5000)});
  const data = await response.json(); if (!response.ok) throw new Error(data.error || 'request_failed'); return data;
}
function selectedScan() { return state?.scans.find(s => s.session_id === scanId) || null; }
function buttons() {
  const active = !!state?.active_request;
  $('understand').disabled = busy; $('preview').disabled = busy || active || request?.status !== 'READY_TO_PLAN' || request?.intent.operation !== 'fetch_object';
  $('execute').disabled = busy || active || !preview || preview.placement.status !== 'CANDIDATES_READY' || !!request?.task_request_id;
  $('cancel').disabled = busy || (!active && (!request || terminal.has(request.status)));
  $('refresh-demo').disabled = busy || active; $('localize-demo').disabled = busy || active;
  const scan = selectedScan(), s = scan?.status;
  $('start-scan').disabled = busy || active || state?.scans.some(v => ['SCANNING', 'PAUSED', 'REVIEW'].includes(v.status));
  $('scan-demo').disabled = busy || s !== 'SCANNING'; $('pause-scan').disabled = busy || s !== 'SCANNING';
  $('resume-scan').disabled = busy || s !== 'PAUSED'; $('review-scan').disabled = busy || !['SCANNING', 'PAUSED'].includes(s) || !scan?.map_document;
  $('save-scan').disabled = busy || s !== 'REVIEW'; $('discard-scan').disabled = busy || !['SCANNING', 'PAUSED', 'REVIEW'].includes(s);
  $('map-file').disabled = busy || s !== 'SCANNING'; $('observation-file').disabled = busy || s !== 'SCANNING';
}
async function action(fn) { if (busy) return; busy = true; buttons(); try { await fn(); message(null); await refresh(); } catch (e) { message(e); } finally { busy = false; buttons(); } }
async function refresh() {
  const epoch = generation;
  const [next, nav] = await Promise.all([api('/v1/assistant/state'), api('/v1/map')]);
  if (epoch !== generation || !token) return;
  state = next; map = nav;
  if (request) request = state.requests.find(r => r.request_id === request.request_id) || request;
  else if (state.active_request) request = state.requests.find(r => r.task_request_id === state.active_request) || null;
  if (!scanId) scanId = state.scans.find(s => ['SCANNING', 'PAUSED', 'REVIEW'].includes(s.status))?.session_id || state.scans.at(-1)?.session_id || null;
  $('connection-text').textContent = '연결됨'; $('connection-dot').style.background = '#4c8664';
  render();
}
function renderRequest() {
  // Polling must not replace the button a person is focusing or clicking.
  const renderKey = JSON.stringify([request, state.entities]);
  if (renderKey === lastRequestKey) return;
  lastRequestKey = renderKey;
  $('request-result').hidden = !request; $('choices').replaceChildren();
  if (!request) return;
  $('request-result').textContent = labels[request.status] || '상태를 확인하고 있어요';
  const g = request.grounding;
  if (g && !request.task_request_id) {
    for (const reason of g.issues) $('choices').append(el('p', reasons[reason] || '요청을 더 구체적으로 알려주세요.', 'small'));
    if (g.issues.includes('ambiguous_object')) {
      for (const match of g.object_candidates) {
        const row = el('div', undefined, 'choice'); row.append(el('span', match.entity.label));
        const choose = el('button', '이 물체', 'secondary'); choose.onclick = () => clarify({object_id: match.entity.entity_id}); row.append(choose); $('choices').append(row);
      }
    }
    if (g.issues.some(r => ['destination_required', 'unknown_destination', 'ambiguous_destination', 'placement_surface_required'].includes(r))) {
      for (const entity of state.entities.filter(v => v.kind === 'surface')) {
        const row = el('div', undefined, 'choice'); row.append(el('span', entity.label + ' · ' + roomLabel(entity.room_id)));
        const choose = el('button', '여기에 놓기', 'secondary'); choose.onclick = () => clarify({destination_id: entity.entity_id}); row.append(choose); $('choices').append(row);
      }
    }
    if (g.issues.some(r => ['unknown_source', 'ambiguous_source'].includes(r))) {
      for (const entity of state.entities.filter(v => v.kind === 'room')) {
        const row = el('div', undefined, 'choice'); row.append(el('span', entity.label));
        const choose = el('button', '이 방', 'secondary'); choose.onclick = () => clarify({source_id: entity.entity_id}); row.append(choose); $('choices').append(row);
      }
    }
    if (request.intent.operation === 'find_object' && !g.issues.length) {
      for (const c of g.object_candidates) $('choices').append(el('p', c.entity.label + ' · ' + roomLabel(c.entity.room_id) + (c.fresh ? '에서 최근 관측했어요.' : '에서 본 기록이 있어요. 다시 확인해야 해요.'), 'small'));
    }
  }
  const execution = request.execution;
  $('task-status').textContent = execution ? `${labels[execution.status] || '처리 중'} · ${execution.completed_steps}/15 단계 확인` : (labels[request.status] || '계획을 확인해 주세요.');
}
function roomLabel(id) { return state.entities.find(v => v.entity_id === id)?.label || '위치 미확인'; }
function render() {
  renderRequest(); $('entities').replaceChildren();
  for (const value of state.entities.filter(e => e.kind === 'object' || e.kind === 'surface')) {
    const item = el('div', undefined, 'entity'), info = el('div'); info.append(el('strong', value.label));
    const location = value.state === 'carried' ? '로봇이 운반 중' : value.state === 'released_unconfirmed' ? '놓기 결과 확인 중' : roomLabel(value.room_id);
    info.append(el('small', location)); item.append(info, el('span', value.kind === 'surface' ? '놓는 자리' : '예시 물체', 'entity-tag')); $('entities').append(item);
  }
  $('map-label').textContent = state.context.map_id;
  $('pending').replaceChildren(...state.pending.map(p => el('span', p)));
  const scan = selectedScan();
  if (scan) {
    let description = `${scan.name} · ${labels[scan.status] || scan.status}`;
    if (scan.coverage) description += ` · 지도 범위 내 ${Math.round(scan.coverage.known_grid_fraction * 100)}% 관측`;
    if (scan.map_document) description += scan.navigation_ready ? ' · 높이 장애물 반영 지도 포함' : ' · 높이 관측 대기 — 주행용 지도 미완성';
    $('scan-status').textContent = description;
    $('scan-hints').replaceChildren(...scan.hints.map(h => el('div', h, 'hint')));
    if (scan.coverage) $('scan-hints').append(el('div', scan.coverage.note, 'hint'));
    const regionKey = JSON.stringify([scan.session_id, scan.status, scan.regions.map(r => r.classification)]);
    if (regionKey !== lastRegions) {
      lastRegions = regionKey; $('region-review').replaceChildren();
      for (const region of scan.regions) {
        const row = el('div', undefined, 'region');
        row.append(el('p', `${region.classification.user_confirmed ? '확인된 이름' : '자동 제안'} · ${spaceLabels[region.classification.label] || region.classification.label || '아직 용도를 모르겠어요'}`));
        if (scan.status === 'REVIEW') {
          const form = el('form', undefined, 'row'), input = el('input'); input.value = spaceLabels[region.classification.label] || region.classification.label || ''; input.required = true; input.maxLength = 80; input.setAttribute('aria-label', '공간 이름');
          form.append(input, el('button', '이름 저장')); form.onsubmit = e => { e.preventDefault(); action(() => api(`/v1/assistant/scans/${scanId}/label`, {region_id: region.region_id, label: input.value})); }; row.append(form);
        }
        $('region-review').append(row);
      }
    }
  }
  drawMap(); buttons();
}
function drawMap() {
  if (!map) return;
  const showingScan = !$('scan-panel').hidden;
  const grid = (showingScan && (selectedScan()?.navigation_map_document || selectedScan()?.map_document)) || map;
  $('map-label').textContent = grid.map_id + (showingScan ? ' · 스캔 기록' : '');
  const canvas = $('home-map'), width = canvas.clientWidth, height = canvas.clientHeight, dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr; canvas.height = height * dpr; const ctx = canvas.getContext('2d'); ctx.scale(dpr, dpr);
  const rows = grid.rows.length, cols = grid.rows[0].length, scale = Math.min((width - 36) / cols, (height - 38) / rows), left = (width - scale * cols) / 2, top = (height - scale * rows) / 2;
  const position = xy => [left + (xy[0] - grid.origin.x) / grid.resolution_m * scale, top + (rows - (xy[1] - grid.origin.y) / grid.resolution_m) * scale];
  grid.rows.forEach((row, y) => [...row].forEach((cell, x) => { ctx.fillStyle = cell === '#' ? '#566b61' : cell === '?' ? '#d5dccf' : '#fcfcf6'; ctx.fillRect(left + x * scale, top + (rows - y - 1) * scale, scale - .5, scale - .5); }));
  const furnitureNames = {bed:'침대',sofa:'소파',desk:'책상',table:'테이블',coffee_table:'거실 탁자',chair:'의자',wardrobe:'옷장',bookshelf:'책장',nightstand:'협탁',cabinet:'수납장'};
  for (const item of state?.furniture || []) {
    if (item.context.map_id !== grid.map_id || item.context.map_revision !== grid.revision) continue;
    const [x1,y1] = position(item.observed_bounds_m.min), [x2,y2] = position(item.observed_bounds_m.max);
    ctx.save(); ctx.strokeStyle = '#7c659b'; ctx.fillStyle = '#7c659b18'; ctx.setLineDash([4,3]);
    ctx.fillRect(x1,y2,x2-x1,y1-y2); ctx.strokeRect(x1,y2,x2-x1,y1-y2); ctx.setLineDash([]);
    ctx.fillStyle = '#624b82'; ctx.font = '11px system-ui'; ctx.textAlign = 'center';
    ctx.fillText((furnitureNames[item.category] || item.category)+' · 관측 부분', (x1+x2)/2, y2-6); ctx.restore();
  }
  for (const candidate of showingScan ? [] : (preview?.placement.candidates.slice(0, 8) || [])) { const [x, y] = position(candidate.base_xy_yaw); ctx.strokeStyle = '#738caf'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.stroke(); }
  if (showingScan) for (const region of selectedScan()?.regions || []) {
    ctx.fillStyle = region.classification.user_confirmed ? '#36786622' : '#d8974022';
    for (const [x,y] of region.cells) ctx.fillRect(left+x*scale, top+(rows-y-1)*scale, scale-.5, scale-.5);
    const [x,y] = position(region.center_xy); ctx.font = '12px system-ui'; ctx.fillStyle = '#234c3e'; ctx.textAlign = 'center';
    ctx.fillText(spaceLabels[region.classification.label] || region.classification.label || '용도 미확인',x,y);
  }
  const plotted = showingScan ? [] : (state?.entities.filter(v => v.position_m && v.kind !== 'room') || []);
  for (const value of plotted) { const [x, y] = position(value.position_m); ctx.fillStyle = value.kind === 'surface' ? '#367866' : '#d89740'; ctx.beginPath(); ctx.arc(x, y, value.kind === 'surface' ? 8 : 4, 0, Math.PI * 2); ctx.fill(); if (value.kind === 'surface') { ctx.font = '11px system-ui'; ctx.fillStyle = '#234c3e'; ctx.textAlign = 'center'; ctx.fillText(value.label, x, y - 13); } }
}
function clarify(selection) { action(async () => { request = await api(`/v1/assistant/requests/${request.request_id}/clarify`, {selection}); preview = null; $('plan').hidden = true; }); }
function scanAction(name) { action(() => api(`/v1/assistant/scans/${scanId}/${name}`, {})); }
async function poll() { const epoch = generation; try { if (!busy) await refresh(); } catch (_) { if (epoch === generation) { $('connection-text').textContent = '연결 확인 중'; $('connection-dot').style.background = '#ba8149'; } } finally { if (token && epoch === generation) timer = setTimeout(poll, 1000); } }
$('login-form').onsubmit = async event => { event.preventDefault(); if (busy) return; token = $('token').value.trim(); busy = true; try { await refresh(); $('token').value = ''; $('login-panel').hidden = true; $('workspace').hidden = false; $('disconnect').hidden = false; drawMap(); message(null); timer = setTimeout(poll, 1000); } catch (e) { message(e); token = ''; } finally { busy = false; buttons(); } };
$('disconnect').onclick = () => { generation++; clearTimeout(timer); token = ''; state = null; request = null; preview = null; scanId = null; lastRegions = ''; $('workspace').hidden = true; $('login-panel').hidden = false; $('disconnect').hidden = true; $('plan').hidden = true; $('token').value = ''; $('error').textContent = '연결 해제는 작업 취소가 아닙니다. 다시 연결하면 진행 상태를 확인합니다.'; };
$('example').onclick = () => { $('utterance').value = '거실 탁자에 있는 검은 마커펜을 침실 협탁에 놓아줘'; };
$('request-form').onsubmit = event => { event.preventDefault(); action(async () => { const id = 'request-' + crypto.randomUUID(); request = await api('/v1/assistant/requests', {request_id: id, utterance: $('utterance').value.trim()}); preview = null; $('plan').hidden = true; }); };
$('preview').onclick = () => action(async () => {
  preview = await api(`/v1/assistant/requests/${request.request_id}/preview`, {}); const box = $('plan'); box.hidden = false; box.replaceChildren();
  box.append(el('strong', `${preview.target.label} → ${preview.destination.label}`));
  const best = preview.placement.candidates[0]; box.append(el('p', best ? `정차와 높이 조합 ${preview.placement.evaluated}개 중 ${preview.placement.candidates.length}개가 예시 조건을 통과했어요. 선택 높이: ${Math.round(best.lift_height_m * 100)}cm.` : '예시 조건에서도 가능한 자세를 찾지 못했어요.'));
  const list = el('ol'); ['이동하고 높이를 맞춘 뒤 다시 보기', '집기 · 들림 확인 · 운반 준비', '목적지 이동 · 놓을 자리 확인', '놓기 · 결과 확인 · 기억 갱신'].forEach(t => list.append(el('li', t))); box.append(list, el('p', preview.note, 'small'));
});
$('execute').onclick = () => action(async () => { const id = request.request_id, previewId = preview.preview_id; const p = await api('/v1/permit'); try { await api(`/v1/assistant/requests/${id}/execute`, {preview_id: previewId}, p.permit); } catch (error) { await refresh(); throw error; } });
$('cancel').onclick = () => action(async () => { if (state.active_request) await api(`/v1/requests/${encodeURIComponent(state.active_request)}/cancel`, {}); else if (request) await api(`/v1/assistant/requests/${request.request_id}/cancel`, {}); });
$('refresh-demo').onclick = () => action(async () => { await api('/v1/assistant/demo/observe', {}); preview = null; $('plan').hidden = true; });
$('localize-demo').onclick = () => action(async () => { await api('/v1/assistant/localize-simulation', {xy_m: [1.5, 1.5]}); preview = null; $('plan').hidden = true; });
for (const tab of ['task', 'scan']) $(tab + '-tab').onclick = () => { for (const other of ['task', 'scan']) { $(other + '-panel').hidden = other !== tab; $(other + '-tab').classList.toggle('selected', other === tab); $(other + '-tab').setAttribute('aria-selected', String(other === tab)); } drawMap(); };
$('scan-form').onsubmit = event => { event.preventDefault(); action(async () => { const result = await api('/v1/assistant/scans', {session_id: 'scan-' + crypto.randomUUID(), name: $('scan-name').value, mode: 'simulation'}); scanId = result.session_id; }); };
for (const [id, name] of [['scan-demo', 'demo'], ['pause-scan', 'pause'], ['resume-scan', 'resume'], ['review-scan', 'review'], ['save-scan', 'save'], ['discard-scan', 'discard']]) $(id).onclick = () => scanAction(name);
for (const [id, endpoint, key] of [['map-file', 'map', 'map_document'], ['observation-file', 'observe', 'observation']]) $(id).onchange = event => action(async () => { const file = event.target.files[0]; if (!file) return; if (file.size > 120000) throw new Error('file_too_large'); const data = JSON.parse(await file.text()); await api(`/v1/assistant/scans/${scanId}/${endpoint}`, {[key]: data}); event.target.value = ''; });
new ResizeObserver(drawMap).observe($('home-map'));
