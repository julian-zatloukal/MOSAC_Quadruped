import { useRef } from 'react'
import { Controls } from './components/Controls.tsx'
import { Hud } from './components/Hud.tsx'
import { Stage } from './components/Stage.tsx'
import { Veil } from './components/Veil.tsx'
import { useDrive } from './hooks/useDrive.ts'
import { useTeleop } from './hooks/useTeleop.ts'
import { useVideoPhase } from './hooks/useVideoPhase.ts'
import { useFit, useFullscreen } from './hooks/useView.ts'
import { canReceiveVideo } from './teleop/connection.ts'

const unsupported = !canReceiveVideo()

export default function App() {
  const { link, robot, stream, send, getStats } = useTeleop()
  const drive = useDrive(send)
  const videoRef = useRef<HTMLVideoElement>(null)
  const phase = useVideoPhase(videoRef, stream)
  const { fit, toggle: toggleFit } = useFit()
  const fullscreen = useFullscreen()

  return (
    <>
      <Stage videoRef={videoRef} fit={fit} />
      <Veil link={link} phase={phase} robotConnected={robot !== null} unsupported={unsupported} />
      <Hud link={link} robot={robot} getStats={getStats} fit={fit} onFit={toggleFit} fullscreen={fullscreen} />
      <Controls held={drive.held} onPress={drive.press} onRelease={drive.release} />
    </>
  )
}
