import { cpSync, createReadStream, existsSync, mkdirSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';

const root = path.dirname(fileURLToPath(import.meta.url));
const modelsSrc = path.resolve(
  root,
  '../rebotarm_ros2_RS/src/rebotarm_mujoco_rs/models'
);
const modelsSrcAbs = path.resolve(modelsSrc);

function collectFiles(dir) {
  return readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const fullPath = path.join(dir, entry.name);
    return entry.isDirectory() ? collectFiles(fullPath) : [fullPath];
  });
}

function hashModelFiles() {
  if (!existsSync(modelsSrcAbs)) return 'missing';
  const hash = createHash('sha256');
  collectFiles(modelsSrcAbs)
    .sort()
    .forEach((filePath) => {
      hash.update(path.relative(modelsSrcAbs, filePath).replaceAll(path.sep, '/'));
      hash.update(readFileSync(filePath));
    });
  return hash.digest('hex').slice(0, 12);
}

const modelVersion = hashModelFiles();

function pagesBase() {
  const value = process.env.GITHUB_PAGES_BASE;
  if (!value) return '/';
  return value.endsWith('/') ? value : `${value}/`;
}

export default defineConfig({
  base: pagesBase(),
  define: {
    __MODEL_VERSION__: JSON.stringify(modelVersion)
  },
  optimizeDeps: {
    exclude: ['@mujoco/mujoco']
  },
  assetsInclude: ['**/*.wasm'],
  build: {
    assetsInlineLimit: 0,
    chunkSizeWarningLimit: 4000
  },
  plugins: [
    {
      name: 'copy-rs-models',
      closeBundle() {
       if (!existsSync(modelsSrc)) {
          throw new Error(`RS models not found: ${modelsSrc}`);
       }
        const dest = path.resolve(root, 'dist/models');
        mkdirSync(path.resolve(root, 'dist'), { recursive: true });
        cpSync(modelsSrc, dest, { recursive: true, dereference: true });
      },
      configureServer(server) {
        server.middlewares.use((req, res, next) => {
          const base = pagesBase();
          const prefix = `${base}models/`;
          const url = decodeURIComponent(req.url || '');
          if (!url.startsWith(prefix)) return next();
          const rel = url.slice(prefix.length).split('?')[0];
          const filePath = path.join(modelsSrcAbs, rel);
          if (!filePath.startsWith(modelsSrcAbs)) return next();
          try {
            const stat = statSync(filePath);
            if (!stat.isFile()) return next();
            const etag = `W/"${stat.size.toString(16)}-${Math.trunc(stat.mtimeMs).toString(16)}"`;
            const ext = path.extname(filePath).toLowerCase();
            const types = {
              '.xml': 'text/xml',
              '.stl': 'application/octet-stream',
              '.obj': 'text/plain',
              '.msh': 'application/octet-stream'
            };
            res.setHeader('Content-Type', types[ext] || 'application/octet-stream');
            res.setHeader('Content-Length', stat.size);
            // Model meshes are large and rarely change. Reuse them directly on
            // ordinary reloads; a hard refresh still revalidates against ETag.
            res.setHeader('Cache-Control', 'public, max-age=3600, must-revalidate');
            res.setHeader('ETag', etag);
            res.setHeader('Last-Modified', stat.mtime.toUTCString());
            if (req.headers['if-none-match'] === etag) {
              res.statusCode = 304;
              res.removeHeader('Content-Length');
              res.end();
              return;
            }
            createReadStream(filePath).pipe(res);
          } catch {
            next();
          }
        });
      }
    }
  ],
  server: {
    port: 5173,
    strictPort: false,
    headers: {
      'Cache-Control': 'no-cache, no-store, must-revalidate'
    }
  }
});
