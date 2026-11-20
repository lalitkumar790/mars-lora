import { defineConfig } from 'vite';
export default defineConfig({
  base: '/',
  publicDir: false,
  build: { target: 'es2022', outDir: 'dist', emptyOutDir: true, assetsInlineLimit: 0 },
  optimizeDeps: { exclude: ['@mujoco/mujoco'] },
});
