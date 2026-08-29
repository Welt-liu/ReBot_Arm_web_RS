import { cpSync, existsSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';

const root = path.dirname(fileURLToPath(import.meta.url));
const modelsSrc = path.resolve(
  root,
  '../rebotarm_ros2/src/rebotarm_mujoco_rs/models'
);

function pagesBase() {
  const value = process.env.GITHUB_PAGES_BASE;
  if (!value) return '/';
  return value.endsWith('/') ? value : `${value}/`;
}

export default defineConfig({
  base: pagesBase(),
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
      }
    }
  ],
  server: {
    port: 5173,
    strictPort: false,
    headers: {
      'Cache-Control': 'public, max-age=3600'
    }
  }
});
