import { t } from './i18n.js';

const PAD = 8;
const GAP = 8;

function format(joint, amount) {
  if (joint.unit === 'm') return `${(amount * 1000).toFixed(1)} mm`;
  return `${((amount * 180) / Math.PI).toFixed(1)}°`;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function packColumn(items, height) {
  if (!items.length) return;
  const content = items.reduce((sum, entry) => sum + entry.h, 0);
  const gaps = Math.max(0, items.length - 1);
  const available = Math.max(1, height - PAD * 2);
  const gap = gaps ? Math.max(0, Math.min(GAP, (available - content) / gaps)) : 0;
  const blockH = content + gap * gaps;
  let start = clamp((height - blockH) / 2, PAD, Math.max(PAD, height - PAD - blockH));
  items.forEach((entry) => {
    entry.chipY = start;
    start += entry.h + gap;
  });
}

export function createJointCallouts(root, joints, onChange) {
  root.innerHTML = '';
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.classList.add('callout-lines');
  svg.setAttribute('aria-hidden', 'true');
  root.append(svg);

  const chips = {};
  let placedWidth = -1;
  let placedHeight = -1;

  joints.visible.forEach((joint) => {
    const color = joint.color || '#8fd7c1';
    const chip = document.createElement('div');
    chip.className = 'callout-chip is-hidden';
    chip.dataset.joint = joint.name;
    chip.style.setProperty('--callout-color', color);

    const header = document.createElement('div');
    header.className = 'callout-header';
    const name = document.createElement('span');
    name.className = 'callout-name';
    name.textContent = joint.name === 'joint7' ? t('joint.gripper') : joint.label;
    const value = document.createElement('strong');
    const actualEl = document.createElement('span');
    actualEl.className = 'actual';
    const targetEl = document.createElement('span');
    targetEl.className = 'target-readout';
    value.append(actualEl, targetEl);
    header.append(name, value);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(joint.min);
    slider.max = String(joint.max);
    slider.step = joint.unit === 'm' ? '0.0005' : '0.01';
    slider.value = String(joint.home);
    slider.style.accentColor = color;
    slider.addEventListener('input', () => {
      onChange(joint.name, Number(slider.value));
      targetEl.textContent = format(joint, Number(slider.value));
    });
    chip.addEventListener('pointerdown', (event) => event.stopPropagation());
    chip.append(header, slider);
    root.append(chip);

    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('class', 'callout-line');
    line.style.setProperty('--callout-color', color);
    line.style.stroke = color;
    line.style.fill = 'none';
    const dot = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
    dot.setAttribute('class', 'callout-dot');
    dot.setAttribute('r', '3.5');
    dot.style.setProperty('--callout-color', color);
    dot.style.fill = color;
    svg.append(line, dot);

    chips[joint.name] = {
      chip,
      slider,
      actualEl,
      targetEl,
      joint,
      line,
      dot,
      color,
      side: joint.column || 'right',
      attachX: 0,
      attachY: 0
    };
  });

  function writeTarget(item, amount) {
    item.slider.value = String(amount);
    item.targetEl.textContent = format(item.joint, amount);
  }

  function writeActual(item, amount) {
    item.actualEl.textContent = format(item.joint, amount);
  }

  function placeCards() {
    const width = Math.max(1, root.clientWidth);
    const height = Math.max(1, root.clientHeight);
    if (width === placedWidth && height === placedHeight) return;
    placedWidth = width;
    placedHeight = height;

    const left = [];
    const right = [];
    joints.visible.forEach((joint) => {
      const item = chips[joint.name];
      item.chip.classList.remove('is-hidden');
      const w = Math.max(1, item.chip.offsetWidth);
      const h = Math.max(1, item.chip.offsetHeight);
      const twoColumns = width >= w * 2 + PAD * 3;
      const side = twoColumns ? item.side : 'right';
      const chipX = side === 'right' ? width - w - PAD : PAD;
      const entry = { item, chipX, w, h, side };
      (side === 'right' ? right : left).push(entry);
    });

    left.reverse();
    packColumn(left, height);
    packColumn(right, height);

    [...left, ...right].forEach((entry) => {
      const { item, chipX, chipY, w, h, side } = entry;
      item.chip.style.transform = `translate(${chipX}px, ${chipY}px)`;
      item.attachX = side === 'right' ? chipX : chipX + w;
      item.attachY = chipY + h / 2;
      item.line.setAttribute('x2', String(item.attachX));
      item.line.setAttribute('y2', String(item.attachY));
    });
  }

  function layout(projectWorld, data) {
    placeCards();
    joints.visible.forEach((joint) => {
      const item = chips[joint.name];
      const screen = projectWorld(
        data.xanchor[joint.id * 3],
        data.xanchor[joint.id * 3 + 1],
        data.xanchor[joint.id * 3 + 2]
      );
      if (!screen.visible) {
        item.line.setAttribute('visibility', 'hidden');
        item.dot.setAttribute('visibility', 'hidden');
        return;
      }
      item.line.setAttribute('visibility', 'visible');
      item.dot.setAttribute('visibility', 'visible');
      item.line.setAttribute('x1', String(screen.x));
      item.line.setAttribute('y1', String(screen.y));
      item.line.setAttribute('x2', String(item.attachX));
      item.line.setAttribute('y2', String(item.attachY));
      item.dot.setAttribute('cx', String(screen.x));
      item.dot.setAttribute('cy', String(screen.y));
    });
  }

  return {
    layout,
    applyLang() {
      const gripper = chips.joint7;
      if (gripper) {
        gripper.chip.querySelector('.callout-name').textContent = t('joint.gripper');
        placedWidth = -1;
      }
    },
    setTarget(name, amount) {
      const item = chips[name];
      if (!item) return;
      writeTarget(item, amount);
    },
    setActual(name, amount) {
      const item = chips[name];
      if (!item) return;
      writeActual(item, amount);
    }
  };
}
