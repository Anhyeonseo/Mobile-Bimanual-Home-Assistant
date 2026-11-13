const WIDE_LAYOUT = '(min-width: 900px)';
const REDUCED_MOTION = '(prefers-reduced-motion: reduce)';
let expandedByUser = false;
let drag = null;
let suppressHandleClick = false;

export const isWideLayout = () => window.matchMedia(WIDE_LAYOUT).matches;
export const isComposerDragging = () => drag !== null;

function setSheetDepth(sheet, reveal) {
  sheet.closest('.home-space')?.style.setProperty('--sheet-reveal', String(reveal));
}

/** Share one sheet state between touch gestures, keyboard controls and rendering. */
export function setComposerOpen(open, { focus = false, notify = true } = {}) {
  expandedByUser = open;
  const sheet = document.querySelector('.native-composer');
  if (!sheet) return;
  const expanded = isWideLayout() || open;
  sheet.classList.toggle('is-open', expanded);
  sheet.classList.remove('is-dragging');
  sheet.style.removeProperty('--drag-offset');
  setSheetDepth(sheet, expanded ? 1 : 0);
  const body = sheet.querySelector('.composer-body');
  body.inert = !expanded;
  const handle = sheet.querySelector('.sheet-handle');
  handle.setAttribute('aria-expanded', String(expanded));
  handle.setAttribute('aria-label', `부탁 패널 ${expanded ? '접기' : '펼치기'}`);
  if (notify) window.dispatchEvent(new CustomEvent('composer-change', { detail: open }));
  if (focus && expanded)
    window.setTimeout(() => sheet.querySelector('textarea')?.focus({ preventScroll: true }), 240);
}

export function consumeHandleClick() {
  if (!suppressHandleClick) return false;
  suppressHandleClick = false;
  return true;
}

/** Pointer capture keeps a dragged sheet attached to the finger outside its handle. */
export function initNativeMotion() {
  document.addEventListener('pointerdown', (event) => {
    const handle = event.target.closest('.sheet-handle');
    if (!handle || isWideLayout()) return;
    const sheet = handle.closest('.native-composer');
    const closedOffset = sheet.offsetHeight - 128;
    drag = {
      sheet,
      handle,
      pointerId: event.pointerId,
      startY: event.clientY,
      startOffset: sheet.classList.contains('is-open') ? 0 : closedOffset,
      closedOffset,
      lastY: event.clientY,
      lastTime: performance.now(),
      velocity: 0,
      distance: 0,
    };
    handle.setPointerCapture(event.pointerId);
  });

  document.addEventListener('pointermove', (event) => {
    if (!drag || event.pointerId !== drag.pointerId) return;
    const now = performance.now();
    drag.distance = event.clientY - drag.startY;
    drag.velocity = (event.clientY - drag.lastY) / Math.max(1, now - drag.lastTime);
    drag.lastY = event.clientY;
    drag.lastTime = now;
    const rawOffset = drag.startOffset + drag.distance;
    // Beyond either snap point, resistance provides a soft rubber-band feel.
    const offset =
      rawOffset < 0
        ? rawOffset * 0.18
        : rawOffset > drag.closedOffset
          ? drag.closedOffset + (rawOffset - drag.closedOffset) * 0.18
          : rawOffset;
    if (Math.abs(drag.distance) > 4) {
      drag.sheet.classList.add('is-dragging');
      drag.sheet.style.setProperty('--drag-offset', `${offset}px`);
      setSheetDepth(drag.sheet, Math.max(0, Math.min(1, 1 - offset / drag.closedOffset)));
    }
  });

  const endDrag = (event) => {
    if (!drag || event.pointerId !== drag.pointerId) return;
    const current = drag;
    drag = null;
    if (Math.abs(current.distance) <= 4) return;
    suppressHandleClick = true;
    const projected = current.startOffset + current.distance + current.velocity * 120;
    setComposerOpen(projected < current.closedOffset / 2);
    window.setTimeout(() => {
      suppressHandleClick = false;
    }, 100);
  };
  document.addEventListener('pointerup', endDrag);
  document.addEventListener('pointercancel', (event) => {
    if (!drag || event.pointerId !== drag.pointerId) return;
    drag = null;
    setComposerOpen(expandedByUser);
  });
  window.matchMedia(WIDE_LAYOUT).addEventListener('change', () => setComposerOpen(expandedByUser));

  document.addEventListener('pointerup', (event) => {
    if (window.matchMedia(REDUCED_MOTION).matches) return;
    const button = event.target.closest(
      '.glass-icon, .glass-action, .send-orb, .app-tab, .btn, .chip, .composer-shortcuts button, .example-card, .next-routine',
    );
    if (!button || button.disabled) return;
    button.animate([{ scale: '0.94' }, { scale: '1.035', offset: 0.55 }, { scale: '1' }], {
      duration: 360,
      easing: 'cubic-bezier(.22,.8,.3,1)',
    });
  });
}
