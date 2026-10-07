/* Native canvas teaching model and interactive diagram of the supplied images. */
(() => {
  'use strict';
  const clamp = value => Math.max(0, Math.min(1, value));
  const toLinear = value => value <= .04045 ? value / 12.92 : ((value + .055) / 1.055) ** 2.4;
  const toSRGB = value => {
    value = clamp(value);
    return value <= .0031308 ? 12.92 * value : 1.055 * value ** (1 / 2.4) - .055;
  };
  const project = (matrix, x, y) => {
    const divisor = matrix[6] * x + matrix[7] * y + matrix[8];
    return [(matrix[0] * x + matrix[1] * y + matrix[2]) / divisor,
      (matrix[3] * x + matrix[4] * y + matrix[5]) / divisor];
  };
  // Unit-square corners, clockwise: top left, top right, bottom right, bottom left.
  function homography(quad) {
    const [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] = quad;
    const dx1 = x1 - x2, dx2 = x3 - x2, dx3 = x0 - x1 + x2 - x3;
    const dy1 = y1 - y2, dy2 = y3 - y2, dy3 = y0 - y1 + y2 - y3;
    const denominator = dx1 * dy2 - dx2 * dy1;
    if (Math.abs(denominator) < 1e-10) throw new Error('Degenerate projection boundary');
    const g = (dx3 * dy2 - dx2 * dy3) / denominator;
    const h = (dx1 * dy3 - dx3 * dy1) / denominator;
    return [x1 - x0 + g * x1, x3 - x0 + h * x3, x0,
      y1 - y0 + g * y1, y3 - y0 + h * y3, y0, g, h, 1];
  }
  function inverse(matrix) {
    const [a, b, c, d, e, f, g, h, i] = matrix;
    const adjugate = [e * i - f * h, c * h - b * i, b * f - c * e,
      f * g - d * i, a * i - c * g, c * d - a * f,
      d * h - e * g, b * g - a * h, a * e - b * d];
    const determinant = a * adjugate[0] + b * adjugate[3] + c * adjugate[6];
    if (Math.abs(determinant) < 1e-10) throw new Error('Singular projection mapping');
    return adjugate.map(value => value / determinant);
  }
  function sample(data, width, height, x, y, channel) {
    if (x < 0 || x > width - 1 || y < 0 || y > height - 1) return 0;
    const ix = Math.floor(x), iy = Math.floor(y), fx = x - ix, fy = y - iy;
    const right = Math.min(ix + 1, width - 1), bottom = Math.min(iy + 1, height - 1);
    const at = (xx, yy) => data[(yy * width + xx) * 3 + channel];
    return (1 - fy) * ((1 - fx) * at(ix, iy) + fx * at(right, iy))
      + fy * ((1 - fx) * at(ix, bottom) + fx * at(right, bottom));
  }
  const compensate = (desired, response, ambient, strength) =>
    (1 - strength) * desired + strength * clamp((desired - ambient) / response);
  const capture = (input, response, ambient) => clamp(ambient + response * input);

  // Keep a small, dependency-free numerical check runnable with Node.js.
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = {clamp, toLinear, toSRGB, project, homography, inverse, sample, compensate, capture};
  }
  if (typeof document === 'undefined') return;
  const demo = document.getElementById('pc-demo');
  if (!demo) return;
  const byId = id => document.getElementById(`pc-${id}`);
  const diagram = byId('formulation');
  const nodes = [...diagram.querySelectorAll('[data-diagram-node]')];
  const svg = byId('wires');
  const disclosure = document.getElementById('interactive-demo');
  const openDemoLink = () => { if (location.hash === '#interactive-demo') disclosure.open = true; };
  openDemoLink();
  window.addEventListener('hashchange', openDemoLink);
  function drawWires() {
    const outer = diagram.getBoundingClientRect();
    if (!outer.width || !outer.height) return;
    svg.setAttribute('viewBox', `0 0 ${outer.width} ${outer.height}`);
    const box = name => {
      const node = nodes.find(node => node.dataset.diagramNode === name);
      const rect = node.getBoundingClientRect();
      return {x: rect.x - outer.x, y: rect.y - outer.y, w: rect.width, h: rect.height};
    };
    for (const path of svg.querySelectorAll('[data-from]')) {
      const vertical = path.classList.contains('pc-wire-inverse') || path.classList.contains('pc-wire-map');
      const training = path.classList.contains('pc-wire-train');
      const from = box(path.dataset.from), to = box(path.dataset.to);
      let start, end;
      if (training) {
        start = [from.x + from.w / 2, from.y - 2];
        end = [to.x + to.w / 2, to.y - 2];
        path.setAttribute('d', `M${start.join(',')} V18 H${end[0]} V${end[1]}`);
        byId('training-label').setAttribute('x', outer.width / 2);
        byId('training-label').setAttribute('y', 15);
        continue;
      } else if (vertical) {
        start = [from.x + from.w / 2, from.y + from.h + 1];
        end = [to.x + to.w / 2, to.y - 2];
        const caption = byId(path.classList.contains('pc-wire-inverse') ? 'compensate-label' : 'target-map-label');
        caption.setAttribute('x', start[0] + 9);
        caption.setAttribute('y', (start[1] + end[1]) / 2 + 3);
      } else {
        const surfacePort = path.classList.contains('pc-wire-comp') ? .65 : .35;
        start = [from.x + from.w + 3, from.y + from.h * (path.dataset.from === 'surface' ? surfacePort : .5)];
        end = [to.x - 5, to.y + to.h * (path.dataset.to === 'surface' ? surfacePort : .5)];
      }
      path.setAttribute('d', `M${start.join(',')} L${end.join(',')}`);
      const label = svg.querySelector(`[data-path-from="${path.dataset.from}"][data-path-to="${path.dataset.to}"]`);
      if (label) label.setAttribute('transform', `translate(${(start[0] + end[0]) / 2},${(start[1] + end[1]) / 2})`);
    }
  }
  function inspect(node) {
    diagram.dataset.path = node.dataset.path;
    demo.dataset.selected = node.dataset.diagramNode;
    for (const item of nodes) item.setAttribute('aria-pressed', String(item === node));
    const original = node.querySelector('img');
    byId('inspect-photo').src = original.src;
    byId('inspect-photo').alt = original.alt;
    byId('inspect-title').textContent = node.querySelector('.pc-node-label').textContent;
    const description = byId('inspect-description');
    const update = () => {
      if (window.MathJax?.Hub) for (const jax of MathJax.Hub.getAllJax(description)) jax.Remove();
      description.textContent = node.dataset.description;
    };
    if (window.MathJax?.Hub) MathJax.Hub.Queue(update, ['Typeset', MathJax.Hub, description]);
    else update(); // MathJax's startup pass typesets any selection made before it loads.
    byId('inspector-details').open = true;
  }
  for (const node of nodes) node.addEventListener('click', () => inspect(node));
  const resize = new ResizeObserver(drawWires);
  resize.observe(diagram);
  disclosure.addEventListener('toggle', () => {
    if (disclosure.open && window.MathJax?.Hub) MathJax.Hub.Queue(['Rerender', MathJax.Hub, demo], drawWires);
    else drawWires();
  });
  for (const image of diagram.querySelectorAll('img')) image.addEventListener('load', drawWires);
  drawWires();

  const width = 320, height = 240, projectorSize = 256;
  const targetRect = {left: 85.5, top: 29.5, size: 189.5};
  // ponytail: an approximate planar warp illustrates alignment; measured dense
  // correspondences would be needed to reproduce the fabric's non-planar geometry.
  const mapping = homography([[54, 11.5], [275, 25], [298, 225.5], [75.5, 234.5]]);
  const unmapping = inverse(mapping);
  const ids = ['photo', 'geo', 'brightness', 'ambient', 'outline', 'clipping'];
  const controls = Object.fromEntries(ids.map(id => [id, byId(id)]));
  const contexts = Object.fromEntries(['target', 'input', 'camera'].map(id =>
    [id, byId(id).getContext('2d', {willReadFrequently: true})]));
  const frames = {
    target: contexts.target.createImageData(width, height),
    input: contexts.input.createImageData(projectorSize, projectorSize),
    camera: contexts.camera.createImageData(width, height)
  };
  const projector = new Float32Array(projectorSize * projectorSize * 3);
  const clipping = new Uint8Array(projectorSize * projectorSize);
  const coordinates = new Float32Array(width * height * 2);
  let surface, target, pending = false;

  async function pixels(photo) {
    photo.loading = 'eager';
    await photo.decode();
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    const context = canvas.getContext('2d', {willReadFrequently: true});
    context.drawImage(photo, 0, 0, width, height);
    const bytes = context.getImageData(0, 0, width, height).data;
    const linear = new Float32Array(width * height * 3);
    for (let i = 0; i < width * height; i++) {
      for (let c = 0; c < 3; c++) linear[i * 3 + c] = toLinear(bytes[i * 4 + c] / 255);
    }
    return linear;
  }
  function writeRGB(frame, pixel, channel, value) {
    frame.data[pixel * 4 + channel] = Math.round(toSRGB(value) * 255);
    frame.data[pixel * 4 + 3] = 255;
  }
  function render() {
    pending = false;
    const photo = +controls.photo.value / 100, geo = +controls.geo.value / 100;
    const corrected = photo > 0 || geo > 0;
    for (const prefix of ['input', 'capture']) {
      byId(`${prefix}-original-symbol`).hidden = corrected;
      byId(`${prefix}-compensated-symbol`).hidden = !corrected;
    }
    const brightness = +controls.brightness.value / 100, ambient = +controls.ambient.value / 100;
    for (const id of ['photo', 'geo', 'brightness', 'ambient']) {
      byId(`${id}-value`).value = id === 'ambient' ? ambient.toFixed(2) : `${controls[id].value}%`;
    }
    for (const button of demo.querySelectorAll('[data-photo]')) {
      button.setAttribute('aria-pressed', String(+button.dataset.photo === photo * 100 && +button.dataset.geo === geo * 100));
    }
    let unreachable = 0, content = 0;
    for (let y = 0; y < projectorSize; y++) {
      for (let x = 0; x < projectorSize; x++) {
        const u = x / (projectorSize - 1), v = y / (projectorSize - 1);
        const [cx, cy] = project(mapping, u, v);
        const tx = (1 - geo) * (targetRect.left + targetRect.size * u) + geo * cx;
        const ty = (1 - geo) * (targetRect.top + targetRect.size * v) + geo * cy;
        const index = y * projectorSize + x;
        let clipped = false, visible = false;
        for (let c = 0; c < 3; c++) {
          const desired = brightness * sample(target, width, height, tx, ty, c);
          const response = .35 + .65 * sample(surface, width, height, cx, cy, c);
          const required = (desired - ambient) / response;
          clipped ||= required < 0 || required > 1;
          visible ||= desired > .003;
          const input = compensate(desired, response, ambient, photo);
          projector[index * 3 + c] = input;
          writeRGB(frames.input, index, c, input);
        }
        clipping[index] = clipped && visible ? 1 : 0;
        if (visible) { content++; if (clipped) unreachable++; }
      }
    }
    for (let index = 0; index < width * height; index++) {
      const u = coordinates[index * 2], v = coordinates[index * 2 + 1];
      const inside = u >= 0 && u <= 1 && v >= 0 && v <= 1;
      const px = u * (projectorSize - 1), py = v * (projectorSize - 1);
      const mark = inside && controls.clipping.checked && clipping[Math.round(py) * projectorSize + Math.round(px)];
      for (let c = 0; c < 3; c++) {
        writeRGB(frames.target, index, c, target[index * 3 + c] * brightness);
        const response = .35 + .65 * surface[index * 3 + c];
        let value = inside ? capture(sample(projector, projectorSize, projectorSize, px, py, c), response, ambient)
          : surface[index * 3 + c] * .08;
        if (mark) value = .35 * value + .65 * [1, .02, .35][c];
        writeRGB(frames.camera, index, c, value);
      }
    }
    for (const id of ['target', 'input', 'camera']) contexts[id].putImageData(frames[id], 0, 0);
    if (controls.outline.checked) {
      const context = contexts.camera;
      context.save();
      context.strokeStyle = '#6ce5ff';
      context.lineWidth = 1;
      context.setLineDash([4, 3]);
      context.strokeRect(targetRect.left, targetRect.top, targetRect.size, targetRect.size);
      context.restore();
    }
    let message = 'Partial correction: watch how intensities and pixel positions change independently.';
    if (photo === 0 && geo === 0) message = 'No correction: the surface alters the colors and the projection distorts the image shape.';
    else if (photo === 1 && geo === 0) message = 'Photometry only: colors approach the target, but the picture is still warped.';
    else if (photo === 0 && geo === 1) message = 'Geometry only: the picture aligns with the target boundary, but the surface still changes its colors.';
    else if (photo === 1 && geo === 1) message = 'Both corrections: pre-warping aligns the picture and intensity correction restores achievable colors.';
    const status = byId('status');
    if (status.textContent !== message) status.textContent = message;
    const percent = content ? 100 * unreachable / content : 0;
    byId('limits').textContent = percent > .1
      ? `In this model, ${percent.toFixed(1)}% of content pixels require at least one channel outside the projector range. Residual errors remain even with both corrections.`
      : 'The target content is within this model’s projector range. The cyan outline shows the desired position.';
    if (!controls.outline.checked && percent <= .1) byId('limits').textContent = 'The target content is within this model’s projector range.';
    demo.dataset.ready = 'true';
    demo.dataset.photo = controls.photo.value;
    demo.dataset.geo = controls.geo.value;
  }
  const requestRender = () => {
    if (!pending) { pending = true; requestAnimationFrame(render); }
  };
  for (const input of Object.values(controls)) input.addEventListener('input', requestRender);
  for (const button of demo.querySelectorAll('[data-photo]')) {
    button.addEventListener('click', () => {
      controls.photo.value = button.dataset.photo;
      controls.geo.value = button.dataset.geo;
      requestRender();
    });
  }
  byId('reset').addEventListener('click', () => {
    for (const id of ['photo', 'geo', 'brightness', 'ambient']) controls[id].value = controls[id].defaultValue;
    controls.outline.checked = true;
    controls.clipping.checked = false;
    requestRender();
  });
  byId('status').textContent = 'Loading the surface and target photographs…';
  Promise.all([pixels(byId('surface-photo')), pixels(byId('target-photo'))]).then(([s, t]) => {
    surface = s;
    target = t;
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) coordinates.set(project(unmapping, x, y), (y * width + x) * 2);
    }
    for (const fieldset of demo.querySelectorAll('fieldset')) fieldset.disabled = false;
    render();
  }).catch(error => {
    byId('status').textContent = 'The simulation could not load its images. You can still inspect the real images above.';
    console.error('Projector compensation demo:', error);
  });
})();
