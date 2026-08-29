const MODEL_DIR = `${import.meta.env.BASE_URL}models`;
const SCENE_XML = 'rs_grasp_scene.xml';

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
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`无法读取 ${url}（HTTP ${response.status}）`);
  }
  const total = Number(response.headers.get('content-length')) || 0;
  if (!response.body || !onChunk) {
    return new Uint8Array(await response.arrayBuffer());
  }
  const reader = response.body.getReader();
  const chunks = [];
  let received = 0;
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    received += value.byteLength;
    onChunk(received, total);
  }
  const buffer = new Uint8Array(received);
  let offset = 0;
  for (const chunk of chunks) {
    buffer.set(chunk, offset);
    offset += chunk.byteLength;
  }
  return buffer;
}

function addToVfs(vfs, name, bytes) {
  vfs.addBuffer(name, bytes);
}

function createModel(mujoco, xmlBytes, vfs) {
  const xml = new TextDecoder().decode(xmlBytes);
  const attempts = [
    () => mujoco.MjModel.from_xml_string(xml, vfs),
    () => mujoco.MjModel.from_xml_path(SCENE_XML, vfs),
    () => mujoco.mj_loadXML?.(SCENE_XML, vfs)
  ];
  const errors = [];
  for (const attempt of attempts) {
    try {
      const model = attempt();
      if (model) return model;
    } catch (error) {
      errors.push(error && error.message ? error.message : String(error));
    }
  }
  throw new Error(`加载 MJCF 失败：${errors.join('；') || '没有可用的加载函数'}`);
}

export async function loadRsScene(mujoco, onProgress) {
  const vfs = new mujoco.MjVFS();
  const queue = [SCENE_XML];
  const seen = new Set();
  const files = {};
  let loadedBytes = 0;

  try {
    while (queue.length) {
      const relative = queue.shift();
      if (seen.has(relative)) continue;
      seen.add(relative);
      onProgress?.(`正在下载 ${relative}`);
      const bytes = await fetchBytes(`${MODEL_DIR}/${relative}`, (received, total) => {
        const suffix = total ? ` ${(received / 1048576).toFixed(1)} / ${(total / 1048576).toFixed(1)} MB` : '';
        onProgress?.(`正在下载 ${relative}${suffix}`);
      });
      files[relative] = bytes;
      addToVfs(vfs, relative, bytes);
      loadedBytes += bytes.byteLength;
      if (relative.endsWith('.xml')) {
        const xml = new TextDecoder().decode(bytes);
        for (const extra of parseIncludesAndMeshes(xml)) {
          if (!seen.has(extra)) queue.push(extra);
        }
      }
    }

    onProgress?.(`资源已写入 MjVFS（${(loadedBytes / 1048576).toFixed(1)} MB），正在编译模型`);
    const model = createModel(mujoco, files[SCENE_XML], vfs);
    const data = new mujoco.MjData(model);
    mujoco.mj_forward(model, data);
    return { model, data, loadedBytes, files: Object.keys(files) };
  } finally {
    vfs.delete();
  }
}
