import react, { reactCompilerPreset } from '@vitejs/plugin-react'
import babel from '@rolldown/plugin-babel'
import { defineConfig } from 'vite'
import { viteSingleFile } from 'vite-plugin-singlefile'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    react(),
    babel({ presets: [reactCompilerPreset()] }),
    viteSingleFile()
  ],
  // pnpm dev: the page comes from here, the connection setup goes to the Teleoperation server (python server.py --no-browser).
  // The video and the commands then go straight between the browser and that server over WebRTC. The proxy sends the server's
  // own host name (it does that by itself), so it also has to send the server's own origin, which the server checks.
  server: {
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:8000',
        changeOrigin: true,
        configure: (proxy) => {
          proxy.on('proxyReq', (request) => request.setHeader('Origin', 'http://127.0.0.1:8000'))
        },
      },
    },
  },
})
