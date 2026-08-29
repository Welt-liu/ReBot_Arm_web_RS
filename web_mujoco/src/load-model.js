const MODEL_DIR = `${import.meta.env.BASE_URL}models`;
const SCENE_XML = 'rs_grasp_scene.xml';
const DOWNLOAD_CONCURRENCY = 6;

export async function loadMujocoModule() {
  const loadMujoco = (await import('@mujoco/mujoco')).default;
  return loadMujoco();
}

function parseIncludesAndMeshes(xmlText) {
  const files = new Set();
  const includeRe = /<include\s+file="([^"]+)"/gi;
  const meshRe = /\sfile="([^"]+\.(?:STL|stl|obj|msh))"/g;
  let match;
  while ((match = includeRe.exec(xmlText))) files.add(match[1]);
  while ((match = meshRe.exec(xmlText))) files.add(`meshes/${match[1].split('/').pop()}`);
  return [...files];
}

async function fetchBytes(url, onChunk) {
  const response = await fetch(url, { cache: 'default' });
  if (!response.ok) {
    throw new Error(`无法读取 ${url}（HTTP ${response.status}）`);
  }
  const total = Number(response.headers.get('content-length')) || 0;
  const bytes = new Uint8Array(await response.arrayBuffer());
  onChunk?.(bytes.byteLength, total || bytes.byteLength);
  return bytes;
}

function report(onProgress, key, vars = {}) {
  onProgress?.({ key, vars });
}

function addToVfs(vfs, name, bytes) {
  vfs.addBuffer(name, bytes);
}

function createModel(mujoco, xmlBytes, vfs) {
  const xml = new TextDecoder().decode(xmlBytes);
  const errors = [];
  try {
    const model = mujoco.MjModel.from_xml_string(xml, vfs);
    if (model) return model;
  } catch (error) {
    errors.push(error && error.message ? error.message : String(error));
  }
  throw new Error(`加载 MJCF 失败：${errors.join('；') || '没有可用的加载函数'}`);
}

function createModelFromVfs(mujoco, files, vfs) {
  const sceneXml = new TextDecoder().decode(files[SCENE_XML]);
  const model = mujoco.MjModel.from_xml_path(SCENE_XML, vfs);
  if (model) return model;
  try {
    return mujoco.MjModel.from_xml_string(sceneXml, vfs);
  } catch (error) {
    const msg = error && error.message ? error.message : String(error);
        throw new Error(`加载 MJCF 失败：${msg}`);
  }
}

function extractMaterialProps(files) {
  const props = [];

  function parseSegment(text) {
    const re = /<material\s+([^>]*)\/?>/gi;
    let m;
    while ((m = re.exec(text))) {
      const attrs = m[1];
      const name = attrs.match(/name="([^"]+)"/)?.[1] ?? '';
      const met = parseFloat(attrs.match(/metallic="([^"]+)"/)?.[1] ?? '');
      const rou = parseFloat(attrs.match(/roughness="([^"]+)"/)?.[1] ?? '');
      props.push({
        name,
        metallic: Number.isFinite(met) ? met : null,
        roughness: Number.isFinite(rou) ? rou : null
      });
    }
  }

  function parseWithIncludes(xmlText) {
    const includeRe = /<include\s+file="([^"]+)"\s*\/?>/gi;
    let last = 0;
    let m;
    while ((m = includeRe.exec(xmlText))) {
      parseSegment(xmlText.slice(last, m.index));
      const file = m[1];
      if (files[file]) {
        const txt = new TextDecoder().decode(files[file]);
        parseWithIncludes(txt.replace(/<\/?mujoco[^>]*>/g, ''));
      }
      last = m.index + m[0].length;
    }
    parseSegment(xmlText.slice(last));
  }

  if (files[SCENE_XML]) {
    parseWithIncludes(new TextDecoder().decode(files[SCENE_XML]));
  }
  return props;
}

export async function loadRsScene(mujoco, onProgress) {
  const vfs = new mujoco.MjVFS();
  const xmlQueue = [SCENE_XML];
  const seen = new Set();
  const meshFiles = new Set();
  const files = {};
  let loadedBytes = 0;

  try {
    // Resolve the small XML dependency graph first so every mesh is known
    // before starting the parallel download pool.
    while (xmlQueue.length) {
      const relative = xmlQueue.shift();
      if (seen.has(relative)) continue;
      seen.add(relative);
      report(onProgress, 'status.download', { file: relative });
      const bytes = await fetchBytes(`${MODEL_DIR}/${relative}`, (received, total) => {
        const mb = total ? `${(received / 1048576).toFixed(1)} / ${(total / 1048576).toFixed(1)} MB` : '';
        report(onProgress, 'status.downloadProgress', { file: relative, mb });
      });
      files[relative] = bytes;
      addToVfs(vfs, relative, bytes);
      loadedBytes += bytes.byteLength;

      const xml = new TextDecoder().decode(bytes);
      for (const extra of parseIncludesAndMeshes(xml)) {
        if (extra.toLowerCase().endsWith('.xml')) {
          if (!seen.has(extra)) xmlQueue.push(extra);
        } else {
          meshFiles.add(extra);
        }
      }
    }

    const pending = [...meshFiles].filter((relative) => !seen.has(relative));
    const totalAssets = seen.size + pending.length;
    let completedAssets = seen.size;
    const activeBytes = new Map();
    let lastProgressAt = 0;

    function reportAssetProgress(force = false) {
      const now = Date.now();
      if (!force && now - lastProgressAt < 80) return;
      lastProgressAt = now;
      const inFlightBytes = [...activeBytes.values()].reduce((sum, value) => sum + value, 0);
      report(onProgress, 'status.loadingAssets', {
        done: completedAssets,
        total: totalAssets,
        mb: ((loadedBytes + inFlightBytes) / 1048576).toFixed(1)
      });
    }

    let nextIndex = 0;
    async function downloadWorker() {
      while (nextIndex < pending.length) {
        const relative = pending[nextIndex];
        nextIndex += 1;
        seen.add(relative);
        activeBytes.set(relative, 0);
        reportAssetProgress(true);
        const bytes = await fetchBytes(`${MODEL_DIR}/${relative}`, (received) => {
          activeBytes.set(relative, received);
          reportAssetProgress();
        });
        activeBytes.delete(relative);
        files[relative] = bytes;
        addToVfs(vfs, relative, bytes);
        loadedBytes += bytes.byteLength;
        completedAssets += 1;
        reportAssetProgress(true);
      }
    }

    const workerCount = Math.min(DOWNLOAD_CONCURRENCY, pending.length);
    await Promise.all(Array.from({ length: workerCount }, () => downloadWorker()));

    const materialProps = extractMaterialProps(files);
    report(onProgress, 'status.compiling', { mb: (loadedBytes / 1048576).toFixed(1) });
    const model = createModelFromVfs(mujoco, files, vfs);
    const data = new mujoco.MjData(model);
    mujoco.mj_forward(model, data);
    return { model, data, loadedBytes, files: Object.keys(files), materialProps };
  } finally {
    vfs.delete();
  }
}
