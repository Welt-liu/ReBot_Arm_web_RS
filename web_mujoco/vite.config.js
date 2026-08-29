import { defineConfig } from 'vite';

export default defineConfig({
  optimizeDeps: {
    exclude: ['@mujoco/mujoco']
  },
  assetsInclude: ['**/*.wasm'],
  server: {
    port: 5173,
    strictPort: false,
    headers: {
      'Cache-Control': 'public, max-age=3600'
    }
  }
});
