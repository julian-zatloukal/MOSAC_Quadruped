import type { Link } from '../teleop/connection.ts'
import type { VideoPhase } from '../hooks/useVideoPhase.ts'
import { NoVideoIcon } from './icons.tsx'
import './Veil.css'

interface VeilProps {
  link: Link
  phase: VideoPhase
  robotConnected: boolean
  unsupported: boolean
}

// Why there is no picture (or a spinner while it is on its way). Gone once the video is live.
export function Veil({ link, phase, robotConnected, unsupported }: VeilProps) {
  const live = phase === 'live'
  let text: string | null = null
  if (!live && link !== 'connecting') {
    if (link === 'down') text = 'Offline'
    else if (unsupported) text = 'Video not supported'
    else if (!robotConnected) text = 'No robot'
    else if (phase === 'stalled') text = 'No video'
  }
  return (
    <div className={live ? 'veil gone' : 'veil'} role="status" aria-live="polite">
      <div className="veil-card panel">
        {text === null ? <div className="spinner" /> : <NoVideoIcon />}
        <span>{live ? '' : (text ?? 'Connecting…')}</span>
      </div>
    </div>
  )
}
