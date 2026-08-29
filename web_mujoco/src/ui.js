export function createJointPanel(root, joints, onChange) {
  root.innerHTML = '';
  const sliders = {};

  joints.visible.forEach((joint) => {
    const wrap = document.createElement('div');
    wrap.className = 'joint';
    const label = document.createElement('label');
    const name = document.createElement('span');
    name.textContent = joint.label;
    const value = document.createElement('strong');
    const actualEl = document.createElement('span');
    actualEl.className = 'actual';
    const targetEl = document.createElement('span');
    targetEl.className = 'target-readout';
    value.append(actualEl, targetEl);
    label.append(name, value);
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(joint.min);
    slider.max = String(joint.max);
    slider.step = joint.unit === 'm' ? '0.0005' : '0.01';
    slider.value = String(joint.home);
    slider.addEventListener('input', () => {
      onChange(joint.name, Number(slider.value));
      targetEl.textContent = format(joint, Number(slider.value));
    });
    wrap.append(label, slider);
    root.append(wrap);
    sliders[joint.name] = { slider, actualEl, targetEl, joint };
  });

  function format(joint, amount) {
    if (joint.unit === 'm') return `${(amount * 1000).toFixed(1)} mm`;
    return `${((amount * 180) / Math.PI).toFixed(1)}°`;
  }

  function writeTarget(item, amount) {
    item.slider.value = String(amount);
    item.targetEl.textContent = format(item.joint, amount);
  }

  function writeActual(item, amount) {
    item.actualEl.textContent = format(item.joint, amount);
  }

  return {
    setTarget(name, amount) {
      const item = sliders[name];
      if (!item) return;
      writeTarget(item, amount);
    },
    setActual(name, amount) {
      const item = sliders[name];
      if (!item) return;
      writeActual(item, amount);
    }
  };
}
