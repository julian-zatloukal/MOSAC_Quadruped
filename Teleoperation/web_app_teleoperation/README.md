# MOSAC Teleoperation page

The page of [Teleoperation](../README.md): the robot's camera full screen, and the controls to drive it. React 19 + TypeScript +
Vite, with the React Compiler, built to a single HTML file (`vite-plugin-singlefile`) that `../server.py` serves.

```
pnpm install
pnpm build        dist/index.html (type-checks first)
pnpm lint
pnpm dev          http://localhost:5173 with hot reload; needs `python3 ../server.py --no-browser` running (port 8000)
```

The page talks to the server only over WebRTC: the camera is a video track, and the commands (out) and the robot's state (in)
go over the data channel `teleop`. The one HTTP request is the `POST /api/offer` that sets the connection up, which `pnpm dev`
proxies to the server (`vite.config.ts`); the video and the data then go straight between the browser and the server.

```
src/
  teleop/       plain TypeScript, no React
    protocol.ts     the messages, and the timing constants (heartbeat, command lifetime)
    connection.ts   TeleopConnection: the RTCPeerConnection, the data channel, reconnecting
    driveInput.ts   DriveInput: what is held down -> drive/stop commands, repeated while held
    stats.ts        the link quality (latency, bitrate, loss, ...) out of RTCPeerConnection.getStats()
  hooks/
    useTeleop.ts    the connection for as long as the page is visible
    useDrive.ts     DriveInput plus the keyboard (WASD/arrows) and "let go when the page loses focus"
    useLinkStats.ts  reads the statistics once a second while the link details are open
    useVideoPhase.ts  the stream into the <video>, and whether pictures are really arriving
    useView.ts      fill/fit (remembered) and full screen
  components/       Stage (the video), Veil (why there is no picture), Hud (top bar), LinkStatus (the dot and its details),
                    Controls (the two pads), icons
```

Controls, keys and what they do are in the Teleoperation README.
