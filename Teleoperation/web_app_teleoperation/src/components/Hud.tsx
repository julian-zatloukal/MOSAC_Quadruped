import type { Link } from '../teleop/connection.ts'
import type { RobotStatus } from '../teleop/protocol.ts'
import type { useFullscreen } from '../hooks/useView.ts'
import { FitIcon, FullscreenIcon } from './icons.tsx'
import { LinkStatus } from './LinkStatus.tsx'
import './Hud.css'

interface HudProps {
  link: Link
  robot: RobotStatus | null
  getStats: () => Promise<RTCStatsReport> | null
  fit: boolean
  onFit: () => void
  fullscreen: ReturnType<typeof useFullscreen>
}

// Top of the screen: the link on the left, the view buttons on the right.
export function Hud({ link, robot, getStats, fit, onFit, fullscreen }: HudProps) {
  return (
    <header className="hud">
      <LinkStatus link={link} robot={robot} getStats={getStats} />
      <div className="tools">
        <button className="tool panel" type="button" onClick={onFit} aria-pressed={fit}
          aria-label={fit ? 'Fill the screen' : 'Show the whole picture'}>
          <FitIcon fit={fit} />
        </button>
        {fullscreen.available && (
          <button className="tool panel" type="button" onClick={fullscreen.toggle}
            aria-label={fullscreen.active ? 'Leave full screen' : 'Full screen'}>
            <FullscreenIcon active={fullscreen.active} />
          </button>
        )}
      </div>
    </header>
  )
}
