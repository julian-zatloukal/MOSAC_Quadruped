import type { RefObject } from 'react'
import './Stage.css'

interface StageProps {
  videoRef: RefObject<HTMLVideoElement | null>
  fit: boolean
}

// The camera full screen, with a shade behind the status at the top and the controls at the bottom. The shades are inside the stage:
// Safari 26 colours its bars after the fixed boxes at the edges of the screen, and the stage has to be the only one there.
export function Stage({ videoRef, fit }: StageProps) {
  return (
    <div className="stage" data-fit={fit} aria-hidden="true">
      <video ref={videoRef} muted playsInline autoPlay disablePictureInPicture disableRemotePlayback />
      <div className="shade top" />
      <div className="shade bottom" />
    </div>
  )
}
