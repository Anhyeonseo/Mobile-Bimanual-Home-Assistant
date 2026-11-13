/** Carry visual continuity across the app's small HTML renders. */
const REDUCED_MOTION = '(prefers-reduced-motion: reduce)';
const SPRING = 'cubic-bezier(.2, 1.2, .36, 1)';
const selectionKey = (element) =>
  ['action', 'id', 'value', 'page'].map((key) => element.dataset[key] || '').join(':');
const selectedButtons = (root) => [...root.querySelectorAll('button.selected[data-action]')];

export function captureSelections(root = document) {
  return new Set(selectedButtons(root).map(selectionKey));
}

export function animateSelections(previous, root = document) {
  if (window.matchMedia(REDUCED_MOTION).matches) return;
  for (const button of selectedButtons(root)) {
    if (previous.has(selectionKey(button))) continue;
    button.animate([{ scale: '.95' }, { scale: '1.025', offset: 0.6 }, { scale: '1' }], {
      duration: 380,
      easing: SPRING,
    });
  }
}

export function captureMotionFrame() {
  const fill = document.querySelector('.progress-fill');
  return {
    page: document.querySelector('.app-shell')?.dataset.page,
    mapView: document.querySelector('.main .floorplan')?.dataset.mapView,
    markers: [...document.querySelectorAll('.main .robot-marker')].map(
      (marker) => getComputedStyle(marker).transform,
    ),
    progress: fill
      ? (fill.getBoundingClientRect().width / fill.parentElement.clientWidth) * 100
      : null,
    stage: document.querySelector('.step.active strong')?.textContent,
    complete: !!document.querySelector('.complete-panel'),
    selections: captureSelections(),
  };
}

export function animateRenderedFrame(previous) {
  const page = document.querySelector('.app-shell')?.dataset.page;
  const samePage = previous.page === page;
  const home = document.querySelector('.home-space');
  if (home) {
    const expanded = home.querySelector('.native-composer').classList.contains('is-open');
    home.style.setProperty('--sheet-reveal', expanded ? '1' : '0');
  }

  // A timer update must not replay page and navigation entry animations.
  if (samePage) {
    const lens = document.querySelector('.tab-lens');
    if (lens) lens.style.animation = 'none';
  } else {
    document.querySelector('.main')?.classList.add('page-enter');
  }
  if (window.matchMedia(REDUCED_MOTION).matches) return;

  if (samePage) {
    const mapView = document.querySelector('.main .floorplan')?.dataset.mapView;
    if (previous.mapView && mapView !== previous.mapView) {
      document
        .querySelector('.map-view-thumb')
        ?.animate(
          [
            { transform: `translateX(${previous.mapView === 'home' ? 0 : 100}%)` },
            { transform: `translateX(${mapView === 'home' ? 0 : 100}%)` },
          ],
          { duration: 540, easing: SPRING },
        );
      // Only the map's appearance changes; live actors and routes stay in place.
      document.querySelectorAll('.map-base-layer, .map-observation-layer').forEach((layer) => {
        layer.animate([{ opacity: 0 }, { opacity: 1 }], { duration: 360, easing: 'ease-out' });
      });
    }
    document.querySelectorAll('.main .robot-marker').forEach((marker, index) => {
      const from = previous.markers[index];
      const to = getComputedStyle(marker).transform;
      if (!from || from === to) return;
      marker.animate([{ transform: from }, { transform: to }], {
        duration: marker.closest('.is-moving') ? 850 : 160,
        easing: 'linear',
      });
    });
    const fill = document.querySelector('.progress-fill');
    if (fill && previous.progress !== null) {
      fill.animate([{ width: `${previous.progress}%` }, { width: fill.style.width }], {
        duration: 850,
        easing: 'cubic-bezier(.22,.68,.35,1)',
      });
    }
    animateSelections(previous.selections);
  }

  const stage = document.querySelector('.step.active');
  if (stage && stage.querySelector('strong')?.textContent !== previous.stage) {
    stage.animate(
      [
        { translate: '12px 0', opacity: 0.35 },
        { translate: '0 0', opacity: 1 },
      ],
      {
        duration: 440,
        easing: SPRING,
      },
    );
  }
  if (!previous.complete) document.querySelector('.success-orb')?.classList.add('celebrate');
}
